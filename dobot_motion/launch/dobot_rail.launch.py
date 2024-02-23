import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    LogInfo,
    RegisterEventHandler,
    ExecuteProcess,
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
                "load /dobot_rail_server ",
                config,
            ]
        ],
        shell=True,
    )



    return LaunchDescription(
        [
            Node(
                package="dobot_motion",
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