from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='dobot_alarm_clear',
            executable='alarm_clear',
            output='screen',
        ),
    ])
