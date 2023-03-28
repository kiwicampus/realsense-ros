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


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    use_respawn = LaunchConfiguration("use_respawn")
    use_composition = LaunchConfiguration("use_composition")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use simulation (Gazebo) clock if True",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    get_package_share_directory("vision_bringup"),
                    "params",
                    "vision_params.yaml",
                ),
                description="Full path to the ROS2 parameters file to use for all launched nodes",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Whether to use composition or not",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="True",
                description="Whether to respawn if a node crashes. Applied when composition is disabled.",
            ),
            # -------------- COMPOSITION -------------------------------
            GroupAction(
                condition=IfCondition(LaunchConfiguration("use_composition")),
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
                condition=UnlessCondition(LaunchConfiguration("use_composition")),
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
                        )
                    )
                ],
            ),
        ]
    )
