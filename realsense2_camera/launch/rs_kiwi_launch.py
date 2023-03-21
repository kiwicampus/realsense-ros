# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

'''
Launch realsense2_camera node option for intra-process communication.
This tool allow the user the evaluate the reduction of the frame latency when intra-process communication is used.
Run syntax: ros2 launch realsense2_camera rs_intra_process_demo_launch.py intra_process_comms:=true
Note: 
*   Running this tool require building with build tools flag on (colcon build --cmake-args '-DBUILD_TOOLS=ON')
*   Currently default for color stream only
'''
import os
import sys
import subprocess
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition, UnlessCondition

# Make sure required packages can be found
process = subprocess.run(['ros2','component', 'types'],
                         stdout=subprocess.PIPE, 
                         universal_newlines=True)

rs_node_class=  'RealSenseNodeFactory'

if process.stdout.find(rs_node_class) == -1:
    sys.exit('Cannot locate all required node components (' + rs_node_class + ') on the available component list\n' + process.stdout + \
    '\nplease make sure you run "colcon build --cmake-args \'-DBUILD_TOOLS=ON\'" command before running this launch file')


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},                     
                           {'name': 'rgb_camera.profile',           'default': '640,360,15', 'description': 'color image width'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'unite_imu_method',             'default': "1", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'depth_module.profile',         'default': '848,480,15', 'description': 'depth module profile'},                           
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'tracking_module.profile',      'default': '0,0,0', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1',              'default': 'false', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'false', 'description': 'enable fisheye2 stream'},
                           {'name': 'enable_confidence',            'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'gyro_fps',                     'default': '0', 'description': "''"},                           
                           {'name': 'accel_fps',                    'default': '0', 'description': "''"},                           
                           {'name': 'enable_gyro',                  'default': 'false', 'description': "''"},                           
                           {'name': 'enable_accel',                 'default': 'false', 'description': "''"},                           
                           {'name': 'enable_pose',                  'default': 'false', 'description': "''"},                           
                           {'name': 'pose_fps',                     'default': '200', 'description': "''"},                           
                           {'name': 'pointcloud.enable',            'default': 'true', 'description': ''}, 
                           {'name': 'pointcloud.stream_filter',     'default': '0', 'description': 'texture stream for pointcloud'},
                           {'name': 'pointcloud.stream_index_filter','default': '-1', 'description': 'texture stream index for pointcloud'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': "''"},                           
                           {'name': 'align_depth.enable',           'default': 'true', 'description': "''"},                           
                           {'name': 'colorizer.enable',             'default': 'false', 'description': "''"},
                           {'name': 'clip_distance',                'default': '-2.', 'description': "''"},                           
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},                           
                           {'name': 'initial_reset',                'default': 'true', 'description': "''"},                           
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': "''"},                           
                           {'name': 'pointcloud.ordered_pc',        'default': 'true', 'description': ''},
                           {'name': 'calib_odom_file',              'default': "''", 'description': "''"},
                           {'name': 'topic_odom_in',                'default': "''", 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'diagnostics_period',           'default': '0.2', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
                           {'name': 'decimation_filter.enable',     'default': 'true', 'description': 'Rate of publishing static_tf'},
                           {'name': 'decimation_filter.filter_magnitude',  'default': '4', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'depth_module.exposure.1',     'default': '7500', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.gain.1',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.exposure.2',     'default': '1', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'depth_module.gain.2',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                           {'name': 'color_virtual_cam',            'default': '-1', 'description': 'virtual camera to write color stream'},
                           {'name': 'texture_display_logs',         'default': '1', 'description': 'whether to show texture related logs'},                            
                           {'name': 'robot_base_frame',             'default': 'chassis', 'description': 'base frame for transform between camera and robot'},                           
                           {'name': 'camera_link_x',                'default': '0.21', 'description': 'x translation between base frame and camera'},                           
                           {'name': 'camera_link_y',                'default': '-0.041', 'description': 'y translation between base frame and camera'},                           
                           {'name': 'camera_link_z',                'default': '0.404', 'description': 'z translation between base frame and camera'},   
                           {'name': 'pc_subsample_fct',             'default': '8', 'description': 'Factor used for subsampling the pointcloud. 1 uses the default density'},
                           {'name': 'color_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'confidence_qos',               'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'depth_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'fisheye_qos',                  'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'infra_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},   
                           {'name': 'pointcloud_qos',               'default': 'SENSOR_DATA', 'description': 'QoS profile name'}, 
                            # filters and depth performance
                            {'name': 'rgb_camera.power_line_frequency',               'default': '2', 'description': 'emitter always on'},
                            {'name': 'depth_module.emitter_always_on',                'default': 'true', 'description': 'emitter always on'}, 
                            {'name': 'depth_module.laser_power',                      'default': '360.0', 'description': 'emitter always on'}, 
                            {'name': 'spatial_filter.enable',                         'default': 'true', 'description': 'emitter always on'},
                            {'name': 'spatial_filter.filter_magnitude',               'default': '2', 'description': 'emitter always on'},
                            {'name': 'spatial_filter.filter_smooth_alpha',            'default': '0.41', 'description': 'emitter always on'},
                            {'name': 'spatial_filter.filter_smooth_delta',            'default': '20', 'description': 'emitter always on'},
                            {'name': 'temporal_filter.enable',                        'default': 'true', 'description': 'emitter always on'},
                            {'name': 'temporal_filter.filter_smooth_alpha',           'default': '0.41', 'description': 'emitter always on'},
                            {'name': 'temporal_filter.filter_smooth_delta',           'default': '20', 'description': 'emitter always on'},
                            {'name': 'hole_filling_filter.enable',                    'default': 'true', 'description': 'emitter always on'},
                            # {'name': 'disparity_filter.enable',                     'default': 'true', 'description': 'emitter always on'},
                            {'name': 'disparity_to_depth.enable',                     'default': 'true', 'description': 'emitter always on'},
                            {'name': 'intra_process_comms',         'default': 'true', 'description': "enable intra-process communication"}, 
                            {'name': 'use_composition',             'default': 'true', 'description': "Whether to use node composition"},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])


def generate_launch_description():
    respawn = bool(int(os.getenv(key="RESPAWN_NODES", default=1)))
    respawn_delay = float(os.getenv(key="RESPAWN_DELAY", default=5))

    return LaunchDescription(
        declare_configurable_parameters(configurable_parameters) + [
        GroupAction(
            condition=IfCondition(LaunchConfiguration("use_composition")),
            actions=[
                LoadComposableNodes(
                    target_container="vision_kronos",
                    composable_node_descriptions=[
                        ComposableNode(
                            package='realsense2_camera',
                            namespace=LaunchConfiguration("camera_name"),
                            plugin='realsense2_camera::' + rs_node_class,
                            name=LaunchConfiguration("camera_name"),
                            parameters=[set_configurable_parameters(configurable_parameters)],
                            extra_arguments=[{'use_intra_process_comms': LaunchConfiguration("intra_process_comms")}]
                        )                                                       
                    ]
                ),
            ],
        ),
        GroupAction(
            condition=UnlessCondition(LaunchConfiguration("use_composition")),
            actions=[
                Node(
                    condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                    package='realsense2_camera',
                    namespace=LaunchConfiguration("camera_name"),
                    name=LaunchConfiguration("camera_name"),
                    executable='realsense2_camera_node',
                    parameters=[set_configurable_parameters(configurable_parameters)],
                    output='screen',
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                    emulate_tty=True,
                    respawn=respawn,
                    respawn_delay=respawn_delay,
                ),
                Node(
                    condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                    package='realsense2_camera',
                    namespace=LaunchConfiguration("camera_name"),
                    name=LaunchConfiguration("camera_name"),
                    executable='realsense2_camera_node',
                    parameters=[set_configurable_parameters(configurable_parameters)
                                , PythonExpression([LaunchConfiguration("config_file")])
                                ],
                    output='screen',
                    arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                    emulate_tty=True,
                    respawn=respawn,
                    respawn_delay=respawn_delay,
                ),
            ],
        ),

    ])


