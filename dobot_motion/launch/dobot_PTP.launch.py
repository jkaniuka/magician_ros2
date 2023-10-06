import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('dobot_motion'),
        'config',
        'PTP_motion_params.yaml'
        )

    return LaunchDescription([
    DeclareLaunchArgument(name='robot_name', default_value='magician1',
            description='Unique name of the manipulator, which will also be its ROS 2 namespace.'),
    Node(
        package='dobot_motion',
        namespace=LaunchConfiguration('robot_name'),
        executable='PTP_server',
        output='screen',
    ),
    ExecuteProcess(
        cmd=[[
            'sleep 5;', 'ros2 ' , 'param ',  'load ', PathJoinSubstitution([LaunchConfiguration('robot_name'), 'dobot_PTP_server ']), config
        ]],
        shell=True
    )
    ])
