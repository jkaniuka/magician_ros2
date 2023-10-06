import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('dobot_kinematics'),
        'config',
        'axis_limits.yaml'
        )

    return LaunchDescription([
    Node(
        package='dobot_kinematics',
        executable='trajectory_validator_server',
        output='log',
    ),
    ExecuteProcess(
        cmd=[[
            'sleep 5;','ros2 ', 'param ', 'load ', '/dobot_trajectory_validation_server ', config
        ]],
        shell=True
    )
    ])

