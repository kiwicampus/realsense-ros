# Copyright (c) 2018 Intel Corporation
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

"""Launch realsense2_camera node."""
import os
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition


def parse_bool2string(string):
    try:
        if int(string):
            return 'true'
        else:
            return 'false'
    except:
        return string


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'enable_pointcloud',            'default': 'true', 'description': 'enable pointcloud'},
                           {'name': 'unite_imu_method',             'default': 'copy', 'description': '[copy|linear_interpolation]'},                           
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},                           
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},                           
                           {'name': 'depth_width',                  'default': os.getenv('STEREO_DEPTH_WIDTH','640'), 'description': 'depth image width'},                           
                           {'name': 'depth_height',                 'default': os.getenv('STEREO_DEPTH_HEIGHT','480'), 'description': 'depth image height'},                           
                           {'name': 'enable_depth',                 'default': parse_bool2string(os.getenv('STEREO_DEPTH_ENABLE','true')), 'description': 'enable depth stream'},
                           {'name': 'color_width',                  'default': os.getenv('STEREO_COLOR_WIDTH','640'), 'description': 'color image width'},                           
                           {'name': 'color_height',                 'default': os.getenv('STEREO_COLOR_HEIGHT','480'), 'description': 'color image height'},                           
                           {'name': 'enable_color',                 'default': parse_bool2string(os.getenv('STEREO_COLOR_ENABLE','true')), 'description': 'enable color stream'},
                           {'name': 'infra_width',                  'default': os.getenv('STEREO_INFRA_WIDTH','640'), 'description': 'infra width'},
                           {'name': 'infra_height',                 'default': os.getenv('STEREO_INFRA_HEIGHT','480'), 'description': 'infra width'},
                           {'name': 'enable_infra1',                'default': parse_bool2string(os.getenv('STEREO_INFRA1_ENABLE','false')), 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': parse_bool2string(os.getenv('STEREO_INFRA2_ENABLE','false')), 'description': 'enable infra2 stream'},
                           {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'fisheye_width',                'default': '-1', 'description': 'fisheye width'},
                           {'name': 'fisheye_height',               'default': '-1', 'description': 'fisheye width'},
                           {'name': 'enable_fisheye1',              'default': 'false', 'description': 'enable fisheye1 stream'},
                           {'name': 'enable_fisheye2',              'default': 'false', 'description': 'enable fisheye2 stream'},
                           {'name': 'confidence_width',             'default': '-1', 'description': 'depth image width'},                           
                           {'name': 'confidence_height',            'default': '-1', 'description': 'depth image height'},                           
                           {'name': 'enable_confidence',            'default': 'false', 'description': 'enable depth stream'},
                           {'name': 'fisheye_fps',                  'default': '-1.', 'description': ''},                           
                           {'name': 'depth_fps',                    'default': str(float(os.getenv('STEREO_DEPTH_FPS','30.'))), 'description': ''},                           
                           {'name': 'confidence_fps',               'default': '-1.', 'description': ''},                           
                           {'name': 'infra_fps',                    'default': str(float(os.getenv('STEREO_INFRA_FPS','30.'))), 'description': ''},                           
                           {'name': 'color_fps',                    'default': str(float(os.getenv('STEREO_COLOR_FPS','30.'))), 'description': ''},                           
                           {'name': 'gyro_fps',                     'default': '-1.', 'description': ''},                           
                           {'name': 'accel_fps',                    'default': '-1.', 'description': ''},    
                           {'name': 'color_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'confidence_qos',               'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'depth_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'fisheye_qos',                  'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'infra_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'enable_gyro',                  'default': parse_bool2string(os.getenv('STEREO_ENABLE_GYRO','false')), 'description': ''},                           
                           {'name': 'enable_accel',                 'default': parse_bool2string(os.getenv('STEREO_ENABLE_ACCEL','false')), 'description': ''},                           
                           {'name': 'pointcloud_texture_stream',    'default': 'RS2_STREAM_COLOR', 'description': 'texture stream for pointcloud'},                           
                           {'name': 'pointcloud_texture_index',     'default': '0', 'description': 'texture stream index for pointcloud'},                          
                           {'name': 'enable_sync',                  'default': 'false', 'description': ''},                           
                           {'name': 'align_depth',                  'default': 'true', 'description': ''},                           
                           {'name': 'filters',                      'default': "''", 'description': ''},                           
                           {'name': 'clip_distance',                'default': '-2.', 'description': ''},                           
                           {'name': 'linear_accel_cov',             'default': '0.01', 'description': ''},                           
                           {'name': 'initial_reset',                'default': 'true', 'description': ''},                           
                           {'name': 'allow_no_texture_points',      'default': 'false', 'description': ''},                           
                           {'name': 'ordered_pc',                   'default': parse_bool2string(os.getenv('STEREO_ORDERED_POINTCLOUD','true')), 'description': ''},                           
                           {'name': 'calib_odom_file',              'default': "''", 'description': "''"},                           
                           {'name': 'topic_odom_in',                'default': "''", 'description': 'topic for T265 wheel odometry'},
                           {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'temporal.holes_fill',          'default': '0', 'description': 'Persistency mode'},
                           {'name': 'stereo_module.exposure.1',     'default': '7500', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.1',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.exposure.2',     'default': '1', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'stereo_module.gain.2',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
                           {'name': 'color_virtual_cam',            'default': os.getenv('STEREO_COLOR_VIRTUAL_CAMERA','-1'), 'description': 'virtual camera to write color stream'},                           
                           {'name': 'robot_base_frame',             'default': os.getenv('STEREO_ROBOT_BASE_FRAME','chassis'), 'description': 'base frame for transform between camera and robot'},                           
                           {'name': 'camera_link_x',                'default': os.getenv('STEREO_CAMERA_LINK_X','0.21'), 'description': 'x translation between base frame and camera'},                           
                           {'name': 'camera_link_y',                'default': os.getenv('STEREO_CAMERA_LINK_Y','-0.041'), 'description': 'y translation between base frame and camera'},                           
                           {'name': 'camera_link_z',                'default': os.getenv('STEREO_CAMERA_LINK_Z','0.404'), 'description': 'z translation between base frame and camera'},                           
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():
    # Default nodes to launch
    respawn_nodes = bool(os.getenv(key="RESPAWN_NODES", default=1))
    respawn_delay = float(os.getenv(key="RESPAWN_DELAY", default=5))
    log_level = 'info'
    logger = LaunchConfiguration("log_level")
    if (os.getenv('ROS_DISTRO') == "dashing") or (os.getenv('ROS_DISTRO') == "eloquent"):
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            # Realsense
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera', 
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters = [set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                ),
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera', 
                node_namespace=LaunchConfiguration("camera_name"),
                node_name=LaunchConfiguration("camera_name"),
                node_executable='realsense2_camera_node',
                prefix=['stdbuf -o L'],
                parameters = [set_configurable_parameters(configurable_parameters)
                            ,{LaunchConfiguration("config_file")}
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                ),
            ])
    else:
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            DeclareLaunchArgument(
                "log_level",
                default_value=["info"],
                description="Logging level",
            ),
            # Realsense
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera', 
                namespace=LaunchConfiguration("camera_name"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters = [set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', logger],
                emulate_tty=True,
                respawn=respawn_nodes,
                respawn_delay=respawn_delay
                ),
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera', 
                namespace=LaunchConfiguration("camera_name"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters = [set_configurable_parameters(configurable_parameters)
                            ,{LaunchConfiguration("config_file")}
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', logger],
                emulate_tty=True,
                respawn=respawn_nodes,
                respawn_delay=respawn_delay
                ),
        ])
