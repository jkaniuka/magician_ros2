from dobot_msgs.srv import SuctionCupControl
import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import time


class SuctionCupService(Node):

    def __init__(self):
        super().__init__('dobot_suction_cup_srv')
        self.srv = self.create_service(SuctionCupControl, 'dobot_suction_cup_service', self.suction_cup_callback)
        self.suction_cup_latency = 0.500


    def suction_cup_callback(self, request, response):


        if request.enable_suction == True:
            bot.set_end_effector_suction_cup(True, True)
            time.sleep(self.suction_cup_latency)
 
        elif request.enable_suction == False:
            bot.set_end_effector_suction_cup(False, False)
            time.sleep(self.suction_cup_latency)

        else:
            response.success = False
            response.message = "Invalid service request"
            return response

        response.success = True
        response.message = "Suction cup state has been changed"
        return response




def main(args=None):
    rclpy.init(args=args)

    minimal_service = SuctionCupService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()