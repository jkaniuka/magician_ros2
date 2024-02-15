import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot


class SetToolNullConfigurator(Node):

    def __init__(self):
        super().__init__('dobot_init_config')
        print("Loading the initial configuration into the robot.")
        # Reset old alarms before startup
        bot.clear_alarms_state()
        bot.set_end_effector_params(0, 0, 0)

def main(args=None):
    rclpy.init(args=args)

    startup_params_configurator = SetToolNullConfigurator()

    startup_params_configurator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()