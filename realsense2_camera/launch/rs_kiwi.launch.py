# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2022 Intel Corporation. All Rights Reserved.

"""Launch realsense2_camera node."""
import os

import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.descriptions import ComposableNode

configurable_parameters = [{'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
                           {'name': 'camera_namespace',             'default': '', 'description': 'ROS namespace for the camera node; empty = root namespace'},
                           {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'unite_imu_method',             'default': "1", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           # realsense-ros >= 4.55 builds the profile parameter name as
                           # "<module>.<stream>_profile" (profile_manager.cpp registerVideoSensorProfileFormat),
                           # so the pre-4.55 names 'depth_module.profile' / 'rgb_camera.profile' are never read
                           # and every stream silently falls back to the driver default. Verified 2026-09-06:
                           # colour ran at 1280x720x30 instead of the requested 640x360x15.
                           {'name': 'depth_module.depth_profile',   'default': '848,480,15', 'description': 'depth stream profile'},
                           {'name': 'depth_module.infra_profile',   'default': '848,480,15', 'description': 'infra streams (0/1/2) profile'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'rgb_camera.color_profile',     'default': '640,360,15', 'description': 'color stream profile'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
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
                           {'name': 'stereo_color_publish_rate',    'default': '-1.0', 'description': 'Custom publish rate for stereo color stream. -1 means use default FPS'},
                           {'name': 'stereo_depth_publish_rate',    'default': '-1.0', 'description': 'Custom publish rate for stereo depth stream. -1 means use default FPS'},
                           {'name': 'pc_subsample_fct',             'default': '8', 'description': 'Factor used for subsampling the pointcloud. 1 uses the default density'},
                           {'name': 'color_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'confidence_qos',               'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'depth_qos',                    'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'fisheye_qos',                  'default': 'SENSOR_DATA', 'description': 'QoS profile name'},    
                           {'name': 'infra1_qos',                   'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'infra2_qos',                   'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                           {'name': 'pointcloud.pointcloud_qos',  'default': 'SENSOR_DATA', 'description': 'QoS profile name'},
                            # filters and depth performance
                            {'name': 'rgb_camera.power_line_frequency',               'default': '2', 'description': 'emitter always on'},
                            {'name': 'depth_module.emitter_always_on',               'default': 'true', 'description': 'emitter always on'}, 
                            {'name': 'depth_module.laser_power',               'default': '360.0', 'description': 'emitter always on'}, 
                            {'name': 'spatial_filter.enable',               'default': 'true', 'description': 'emitter always on'},
                            {'name': 'spatial_filter.filter_magnitude',               'default': '2', 'description': 'emitter always on'},
                            {'name': 'spatial_filter.filter_smooth_alpha',               'default': '0.41', 'description': 'emitter always on'},
                            {'name': 'spatial_filter.filter_smooth_delta',               'default': '20', 'description': 'emitter always on'},
                            {'name': 'temporal_filter.enable',               'default': 'true', 'description': 'emitter always on'},
                            {'name': 'temporal_filter.filter_smooth_alpha',               'default': '0.41', 'description': 'emitter always on'},
                            {'name': 'temporal_filter.filter_smooth_delta',               'default': '20', 'description': 'emitter always on'},
                            {'name': 'hole_filling_filter.enable',               'default': 'true', 'description': 'emitter always on'},
                            # {'name': 'disparity_filter.enable',               'default': 'true', 'description': 'emitter always on'},
                            {'name': 'disparity_to_depth.enable',               'default': 'true', 'description': 'emitter always on'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    params = dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])
    # realsense-ros derives a filter's parameter prefix from the processing block's NAME, and the
    # pointcloud block is named after the implementation it was compiled with. On arm64 we build
    # with BUILD_WITH_NEON=true, so the block is "Pointcloud (NEON)" and the node declares
    # pointcloud__neon_.* - every plain pointcloud.* override is then silently dropped. That left
    # pointcloud__neon_.enable at its built-in default of false, so the camera published NO cloud
    # at all on JP5, and pointcloud.ordered_pc / stream_filter never took either.
    # Verified on 4U081 (2026-09-09): 'ros2 param list /camera' has no pointcloud.* whatsoever,
    # and setting pointcloud__neon_.enable=true made the cloud appear immediately.
    # x86 (simulator) keeps the plain name, so send BOTH spellings: the node ignores an override
    # for a parameter it does not declare.
    for name in list(params):
        if name.startswith('pointcloud.'):
            params['pointcloud__neon_.' + name[len('pointcloud.'):]] = params[name]
        elif name == 'allow_no_texture_points':
            params['pointcloud__neon_.allow_no_texture_points'] = params[name]
    return params

def generate_launch_description():
    log_level = 'info'
    respawn = bool(int(os.getenv(key="RESPAWN_NODES", default=1)))
    respawn_delay = float(os.getenv(key="RESPAWN_DELAY", default=5))
    use_cpp_stack = int(os.getenv("LAUNCH_VIDEO_MAPPING_CPP", default=0))
    use_composition = int(os.getenv("VISION_USE_COMPOSITION", default=1))
    if use_composition and use_cpp_stack:
        return LaunchDescription(
            declare_configurable_parameters(configurable_parameters)
            + [
                # Realsense
                GroupAction(
                    condition=IfCondition(
                        PythonExpression([LaunchConfiguration("config_file"), " == ''"])
                    ),
                    actions=[
                        launch_ros.actions.LoadComposableNodes(
                            target_container="vision_kronos",
                            composable_node_descriptions=[
                                ComposableNode(
                                    parameters=[
                                        set_configurable_parameters(
                                            configurable_parameters
                                        )
                                    ],
                                    package="realsense2_camera",
                                    plugin="realsense2_camera::RealSenseNodeFactory",
                                    name="camera",
                                    namespace=LaunchConfiguration("camera_namespace"),
                                    extra_arguments=[{"use_intra_process_comms": True}],
                                )
                            ],
                        ),
                    ],
                ),
                GroupAction(
                    condition=IfCondition(
                        PythonExpression([LaunchConfiguration("config_file"), " != ''"])
                    ),
                    actions=[
                        launch_ros.actions.LoadComposableNodes(
                            target_container="vision_kronos",
                            composable_node_descriptions=[
                                ComposableNode(
                                    parameters=[
                                        set_configurable_parameters(
                                            configurable_parameters
                                        ),
                                        PythonExpression(
                                            [LaunchConfiguration("config_file")]
                                        ),
                                    ],
                                    package="realsense2_camera",
                                    plugin="realsense2_camera::RealSenseNodeFactory",
                                    name="camera",
                                    namespace=LaunchConfiguration("camera_namespace"),
                                    extra_arguments=[{"use_intra_process_comms": True}],
                                )
                            ],
                        ),
                    ],
                ),
            ]
        )
    else:
        return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
            # Realsense
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " == ''"])),
                package='realsense2_camera',
                namespace=LaunchConfiguration("camera_namespace"),
                name=LaunchConfiguration("camera_name"),
                executable='realsense2_camera_node',
                parameters=[set_configurable_parameters(configurable_parameters)
                            ],
                output='screen',
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                emulate_tty=True,
                respawn=respawn,
                respawn_delay=respawn_delay,
                ),
            launch_ros.actions.Node(
                condition=IfCondition(PythonExpression([LaunchConfiguration('config_file'), " != ''"])),
                package='realsense2_camera',
                namespace=LaunchConfiguration("camera_namespace"),
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
        ])
