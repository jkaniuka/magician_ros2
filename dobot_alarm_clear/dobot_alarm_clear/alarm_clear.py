from dobot_msgs.srv import ExecuteAlarmClear
import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import time



class AlarmClearService(Node):

    def __init__(self):
        super().__init__('dobot_alarm_clear_srv')
        self.srv = self.create_service(ExecuteAlarmClear, 'dobot_alarm_clear_service', self.alarm_clear_callback)
        self.instruction = "Alarms cleared."


    def alarm_clear_callback(self, request, response):

        bot.clear_alarms_state()
        time.sleep(0.1)

        response.success = True
        response.instruction = self.instruction
        return response




def main(args=None):
    rclpy.init(args=args)

    minimal_service = AlarmClearService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()