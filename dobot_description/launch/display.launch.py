from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.substitutions import Command, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node



def generate_launch_description():
    urdf_tutorial_path = get_package_share_path('dobot_description')
    default_model_path = urdf_tutorial_path / 'model/magician_standalone.urdf.xacro'
    default_rviz_config_path = urdf_tutorial_path / 'rviz/urdf_full.rviz'

    gui_arg = DeclareLaunchArgument(name='gui', default_value='false', choices=['true', 'false'],
                                    description='Flag to enable joint_state_publisher_gui.')

    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_model_path),
                                      description='Absolute path to robot urdf file.')

    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                     description='Absolute path to rviz config file.')

    DOF_arg = DeclareLaunchArgument(name = 'DOF', default_value = '4', choices = ['3', '4'],
                                     description = 'Number of manipulator degrees of freedom.') 

    camera_arg = DeclareLaunchArgument(name = 'use_camera', default_value = 'false', choices = ['true', 'false'],
                                     description = 'Add Intel RealSense D435i camera to URDF model.')

    tool_arg = DeclareLaunchArgument(name = 'tool', default_value = 'gripper', 
                                     choices = ['suction_cup', 'gripper', 'extended_gripper', 'pen', 'none'],
                                     description = 'Type of tool.')    
                                  

    config_expression = ["('",LaunchConfiguration('DOF'),"'", " == '3'", 'and', \
                        "'",LaunchConfiguration('use_camera'),"'", " == 'false'", 'and (', \
                        "'",LaunchConfiguration('tool'),"'", " == 'none'", 'or', \
                        "'",LaunchConfiguration('tool'),"'", " == 'pen' ))", 'or', \
                        "('",LaunchConfiguration('DOF'),"'", " == '4'", 'and (', \
                        "'",LaunchConfiguration('tool'),"'", " == 'gripper'", 'or', \
                        "'",LaunchConfiguration('tool'),"'", " == 'extended_gripper'", 'or', \
                        "'",LaunchConfiguration('tool'),"'", " == 'suction_cup' ))" ]

    config_info = LogInfo(msg = '\n\n\n---------------------------------------------------------------------------\n '
'\n         AVAILABLE CONFIGURATIONS FOR DOBOT MAGICIAN:\n\n\n ' +
'If you cannot see the visualization, check if you entered the correct combination of parameters.\n\n ' +
'DOF:=3\n ' +
'   use_camera:=false\n ' +
'       tool:= none OR tool:=pen\n ' +
'DOF:=4\n ' +
'   use_camera:=true OR use_camera:=false\n ' +
'       tool:= gripper OR tool:=extended_gripper OR tool:=suction_cup\n\n' +
'If the real robot is disconnected and you want to see the visualization and be able to move the robot using joint_state_publisher_gui, add an gui:=true argument.\n\n\n' + 
'---------------------------------------------------------------------------')


    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model'),
                                            ' DOF:=', LaunchConfiguration('DOF'),
                                            ' use_camera:=', LaunchConfiguration('use_camera'),
                                            ' tool:=', LaunchConfiguration('tool')])}],
        condition = IfCondition(PythonExpression(config_expression)),
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        condition = IfCondition(PythonExpression(config_expression))
    )

    return LaunchDescription([
        gui_arg,
        model_arg,
        rviz_arg,
        DOF_arg,
        tool_arg,
        camera_arg,
        # joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        config_info
    ])
