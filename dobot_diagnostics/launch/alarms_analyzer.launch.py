"""Launch analyzer loader with parameters from yaml."""

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_path



def generate_launch_description():

    diagnosticsc_package_path = get_package_share_path('dobot_diagnostics')
    analyzer_params_filepath = diagnosticsc_package_path / 'config/analyzer_params.yaml'


    aggregator = launch_ros.actions.Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        output='screen',
        parameters=[analyzer_params_filepath])
    diag_publisher = launch_ros.actions.Node(
        package='dobot_diagnostics',
        executable='alarms_parser')
    return launch.LaunchDescription([
        aggregator,
        diag_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=aggregator,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )),
    ])
