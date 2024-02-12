import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('dobot_motion'),
        'config',
        'sliding_rail_params.yaml'
        )

    rail_node = Node(
        package='dobot_motion',
        executable='sliding_rail_server',
        output='screen',
    )

    param_loader = ExecuteProcess(
        cmd=[[
            'sleep 5;', 'ros2 param load /dobot_rail_server ', config
        ]],
        shell=True
    )

    logger = RegisterEventHandler(
        OnProcessStart(
            target_action=param_loader,
            on_start=[
                LogInfo(msg='Sliding rail initialized successfully.')
            ]
        )
    )

    return LaunchDescription([
        rail_node,
        param_loader,
        logger
    ])