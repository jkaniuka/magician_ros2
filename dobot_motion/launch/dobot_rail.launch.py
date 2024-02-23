import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import (
    LogInfo,
    RegisterEventHandler,
    ExecuteProcess,
    DeclareLaunchArgument,
)
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory("dobot_motion"),
        "config",
        "sliding_rail_params.yaml",
    )

    rail = ExecuteProcess(
        cmd=[
            [
                "sleep 5;",
                "ros2 ",
                "param ",
                "load ",
                PathJoinSubstitution(
                    [LaunchConfiguration("robot_name"), "dobot_rail_server "]
                ),
                config,
            ]
        ],
        shell=True,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                name="robot_name",
                default_value="magician1",
                description="Unique name of the manipulator, which will also be its ROS 2 namespace.",
            ),
            Node(
                package="dobot_motion",
                namespace=LaunchConfiguration("robot_name"),
                executable="sliding_rail_server",
                output="screen",
            ),
            rail,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=rail,
                    on_exit=[
                        LogInfo(
                            msg="Sliding rail action server has been launched correctly"
                        )
                    ],
                )
            ),
        ]
    )
