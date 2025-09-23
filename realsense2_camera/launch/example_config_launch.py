#!/usr/bin/env python3

"""
Example launch file showing how to use rs_config_multi_camera_launch.py

This demonstrates different ways to launch cameras based on configuration:

1. Using environment variable ROBOT_TYPE
2. Specifying robot_type as launch argument
3. With different camera parameters

Usage examples:

# Use default robot type from environment or 'default' config
ros2 launch realsense2_camera rs_config_multi_camera_launch.py

# Specify robot type explicitly
ros2 launch realsense2_camera rs_config_multi_camera_launch.py robot_type:=bimanual-i2rt

# With additional parameters (applied to all cameras)
ros2 launch realsense2_camera rs_config_multi_camera_launch.py robot_type:=bimanual-i2rt enable_depth:=true enable_color:=true

# Enable pointcloud for all cameras
ros2 launch realsense2_camera rs_config_multi_camera_launch.py robot_type:=bimanual-i2rt pointcloud.enable:=true

# With custom color and depth profiles
ros2 launch realsense2_camera rs_config_multi_camera_launch.py robot_type:=bimanual-i2rt rgb_camera.color_profile:=640,480,30 depth_module.depth_profile:=640,480,30
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
import os

def generate_launch_description():
    """Generate launch description that includes the multi-camera launch."""
    
    # You can set default parameters here
    default_launch_args = {
        'robot_type': os.getenv('ROBOT_TYPE', 'bimanual-i2rt'),  # Use bimanual-i2rt as example
        'enable_depth': 'true',
        'enable_color': 'true',
        'pointcloud.enable': 'false',  # Change to 'true' if you want pointclouds
        'align_depth.enable': 'false',  # Change to 'true' if you want aligned depth
        'log_level': 'info',
    }
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/rs_config_multi_camera_launch.py']),
            launch_arguments=default_launch_args.items(),
        )
    ])
