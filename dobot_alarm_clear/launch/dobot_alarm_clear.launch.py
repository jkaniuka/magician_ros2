import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('dobot_alarm_clear'),
        'config',
        'alarm_clear_params.yaml'
        )

    return LaunchDescription([
        Node(
            package='dobot_alarm_clear',
            executable='alarm_clear',
            output='screen',
        ),
    ExecuteProcess(
        cmd=[[
            'sleep 5;', 'ros2 ' , 'param ',  'load ', '/dobot_alarm_clear_srv ', config
        ]],
        shell=True
    )
    ])

