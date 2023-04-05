"""Launch the vision stack in a component container."""
import os
import cv2

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LoadComposableNodes  # , Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# -------------- CONFIGURABLE PARAMETERS -----------------------------------
use_composition = "True" if int(os.getenv("VISION_USE_COMPOSITION", True)) else "False"
use_respawn = "True" if int(os.getenv("VISION_USE_RESPAWN", True)) else "False"
params_file = os.path.join(
    get_package_share_directory("vision_bringup"), "params", "vision_params.yaml"
)


def generate_launch_description():

    launch_arguments = {
        "params_file": params_file,
        "use_respawn": use_respawn,
        "use_composition": use_composition,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value=use_composition,
                description="Whether to use composition or not",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value=use_respawn,
                description="Whether to respawn if a node crashes. Applied when composition is disabled.",
            ),
            # -------------- COMPOSITION -------------------------------
            GroupAction(
                condition=IfCondition(use_composition),
                actions=[
                    # Node(
                    #     name="vision_kronos",
                    #     package="rclcpp_components",
                    #     executable="component_container_isolated",
                    #     output="both",
                    # ),
                    LoadComposableNodes(
                        target_container="vision_kronos",
                        composable_node_descriptions=[
                            ComposableNode(
                                parameters=[params_file],
                                package="realsense2_camera",
                                plugin="realsense2_camera::RealSenseNodeFactory",
                                name="camera",
                                namespace="camera",
                                extra_arguments=[{"use_intra_process_comms": True}],
                            )
                        ],
                    ),
                ],
            ),
            # -------------- NO COMPOSITION ----------------------------
            GroupAction(
                condition=UnlessCondition(use_composition),
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [
                                os.path.join(
                                    get_package_share_directory("realsense2_camera"),
                                    "launch",
                                ),
                                "/rs_launch.py",
                            ]
                        ),
                        launch_arguments=launch_arguments.items(),
                    )
                ],
            ),
        ]
    )
