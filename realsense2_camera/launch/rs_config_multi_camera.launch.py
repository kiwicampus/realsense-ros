# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch multiple realsense2_camera nodes based on config file."""
import os
import sys
import pathlib
import copy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration

# Add current directory to path to import local modules
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))
import rs_launch
from config.configs import configs as realsense_configs

# Try to import optional dependencies
try:
    import pyrealsense2 as rs
    PYREALSENSE_AVAILABLE = True
except ImportError:
    PYREALSENSE_AVAILABLE = False
    print("Warning: pyrealsense2 not available. Serial detection will be disabled.")

try:
    from python_utils.vision_utils import printlog
except ImportError:
    def printlog(msg, msg_type="INFO"):
        colors = {
            "OKGREEN": "\033[92m",
            "ERROR": "\033[91m", 
            "INFO": "\033[94m",
            "ENDC": "\033[0m"
        }
        color = colors.get(msg_type, colors["INFO"])
        print(f"{color}[{msg_type}] {msg}{colors['ENDC']}")

def get_connected_serials():
    """Get connected device serial numbers."""
    if not PYREALSENSE_AVAILABLE:
        return []
    
    try:
        ctx = rs.context()
        serials = [str(d.get_info(rs.camera_info.serial_number)) for d in ctx.query_devices()]
        return serials
    except Exception as e:
        printlog(f"Error getting connected serials: {e}", "ERROR")
        return []

def setup_multi_camera_launch(context, params):
    """Setup multiple cameras based on configuration."""
    # Get robot type from launch configuration or environment
    robot_type = LaunchConfiguration('robot_type').perform(context)
    if not robot_type or robot_type == "''":
        robot_type = os.getenv('ROBOT_TYPE', 'default')
    
    printlog(f"Using robot configuration: {robot_type}", "INFO")
    
    # Get configuration
    if robot_type not in realsense_configs:
        printlog(f"Robot type '{robot_type}' not found in configs, using 'default'", "ERROR")
        robot_type = 'default'
    
    config = realsense_configs[robot_type]
    cameras_config = config.get('cameras', [])
    
    printlog(f"Found {len(cameras_config)} cameras in configuration", "INFO")
    
    # Get connected serials
    connected_serials = get_connected_serials()
    if connected_serials:
        printlog(f"Connected devices: {connected_serials}", "INFO")
    else:
        printlog("No connected devices found or pyrealsense2 not available", "ERROR")
    
    # Launch nodes for connected cameras
    launch_nodes = []
    launched_count = 0
    
    for i, camera_config in enumerate(cameras_config):
        camera_name = camera_config['name']
        camera_serial = str(camera_config['serial'])
        
        # Check if camera is connected (skip check if pyrealsense2 not available)
        if PYREALSENSE_AVAILABLE and connected_serials and camera_serial not in connected_serials:
            printlog(f"Camera {camera_name} (serial: {camera_serial}) not connected - skipping", "ERROR")
            continue
        
        printlog(f"Launching camera: {camera_name} (serial: {camera_serial})", "OKGREEN")
        
        # Create parameters dictionary with actual values, not LaunchConfiguration objects
        camera_params = {}
        
        # Set all parameters to their default values first, except serial_no which we handle separately
        for param in rs_launch.configurable_parameters:
            param_name = param['name']
            if param_name != 'serial_no':  # Skip serial_no to avoid type conflicts
                camera_params[param_name] = param['default']
        
        # Override with camera-specific values
        camera_params['camera_name'] = camera_name
        camera_params['camera_namespace'] = camera_name  
        # Use lower resolution to avoid USB bandwidth conflicts with multiple cameras
        camera_params['depth_module.depth_profile'] = '640,480,15'
        camera_params['rgb_camera.color_profile'] = '640,480,15'
        # Note: serial_no is handled separately in the parameter list to ensure proper typing
        
        # Apply any launch configuration overrides (convert LaunchConfiguration to actual values)
        for param_name, launch_config in params.items():
            if param_name not in ['camera_name', 'camera_namespace', 'serial_no', 
                                'depth_module.depth_profile', 'rgb_camera.color_profile']:
                # These will be evaluated at launch time
                camera_params[param_name] = launch_config
        
        # Load lifecycle nodes setting
        lifecycle_param_file = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'global_settings.yaml'
        )
        
        try:
            lifecycle_params = rs_launch.yaml_to_dict(lifecycle_param_file)
            use_lifecycle_node = lifecycle_params.get("use_lifecycle_node", False)
        except Exception as e:
            printlog(f"Failed to load lifecycle settings: {e}", "ERROR")
            use_lifecycle_node = False
        
        # Choose node type
        import launch_ros.actions
        node_action = launch_ros.actions.LifecycleNode if use_lifecycle_node else launch_ros.actions.Node
        log_message = f"Launching {camera_name} as {'LifecycleNode' if use_lifecycle_node else 'Normal ROS Node'}"
        
        # Handle ROS Foxy compatibility for output parameter
        output_param = 'screen'
        if os.getenv('ROS_DISTRO') != 'foxy':
            output_param = camera_params.get('output', 'screen')
        
        # Create the camera node with proper namespace and name
        # Use parameter list with explicit string typing for serial_no
        param_list = [
            camera_params,
            {'serial_no': str(camera_serial)}  # Only serial_no in this dict to avoid type conflicts
        ]
        
        # Build ROS arguments with explicit parameter setting to avoid YAML parsing issues
        ros_args = ['--ros-args', '--log-level', camera_params.get('log_level', 'info')]
        
        # Add serial_no as a command line parameter to avoid YAML octal interpretation
        # Force it to be treated as a string by adding quotes
        ros_args.extend(['-p', f'serial_no:="{camera_serial}"'])
        
        camera_node = node_action(
            package='realsense2_camera',
            namespace=camera_name,  # This sets the namespace properly
            name=camera_name,       # This sets the node name properly
            executable='realsense2_camera_node',
            parameters=[camera_params],  # Only use the first parameter dict
            output=output_param,
            arguments=ros_args,
            emulate_tty=True,
        )
        
        camera_launch_nodes = [
            LogInfo(msg=f"🚀 {log_message}"),
            camera_node
        ]
        
        # Add delay for cameras after the first one to avoid device contention
        if launched_count > 0:
            # Wrap subsequent cameras in a TimerAction to delay startup
            delayed_camera = TimerAction(
                period=2.0 + (launched_count * 1.0),  # 2s + 1s per additional camera
                actions=camera_launch_nodes
            )
            launch_nodes.append(delayed_camera)
        else:
            # Launch first camera immediately
            launch_nodes.extend(camera_launch_nodes)
        
        launched_count += 1
    
    if launched_count == 0:
        printlog("No cameras were launched! Check your configuration and device connections.", "ERROR")
        # Launch a dummy info node to prevent empty launch
        launch_nodes.append(
            LogInfo(msg="❌ No cameras available - check configuration and connections")
        )
    else:
        printlog(f"Successfully launched {launched_count} cameras", "OKGREEN")
        
        # Add static transform publishers between cameras if multiple cameras
        if launched_count > 1:
            import launch_ros.actions
            
            # Create a simple chain of transforms (camera1 -> camera2 -> camera3, etc.)
            camera_links = [camera['name'] + '_link' for camera in cameras_config 
                          if str(camera['serial']) in connected_serials or not PYREALSENSE_AVAILABLE or not connected_serials]
            
            for i in range(len(camera_links) - 1):
                tf_node = launch_ros.actions.Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"tf_{camera_links[i]}_to_{camera_links[i+1]}",
                    arguments=["0", "0", "0", "0", "0", "0", camera_links[i], camera_links[i+1]]
                )
                launch_nodes.append(tf_node)
    
    return launch_nodes

# Configuration parameters
config_parameters = [
    {'name': 'robot_type', 'default': os.getenv('ROBOT_TYPE', 'default'), 'description': 'Robot configuration type from configs.py'},
]

def generate_launch_description():
    """Generate the launch description."""
    # Declare configuration parameters
    declared_config_params = [
        DeclareLaunchArgument(
            param['name'], 
            default_value=param['default'], 
            description=param['description']
        ) for param in config_parameters
    ]
    
    # Filter out parameters that we'll set programmatically per camera
    filtered_params = [param for param in rs_launch.configurable_parameters 
                      if param['name'] not in ['camera_name', 'camera_namespace', 'serial_no']]
    
    # Declare filtered realsense parameters (they will be applied to all cameras)
    declared_rs_params = rs_launch.declare_configurable_parameters(filtered_params)
    
    # Set up parameter dictionary for filtered parameters
    rs_params = rs_launch.set_configurable_parameters(filtered_params)
    
    return LaunchDescription(
        declared_config_params +
        declared_rs_params +
        [
            OpaqueFunction(
                function=setup_multi_camera_launch,
                kwargs={'params': rs_params}
            )
        ]
    )
