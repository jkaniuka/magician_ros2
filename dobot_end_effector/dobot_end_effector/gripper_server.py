from dobot_msgs.srv import GripperControl
from dobot_msgs.msg import GripperStatus
import rclpy
from rclpy.node import Node
from dobot_driver.dobot_handle import bot
import time


class GripperService(Node):

    def __init__(self):
        super().__init__('dobot_gripper_srv')
        self.srv = self.create_service(GripperControl, 'dobot_gripper_service', self.gripper_callback)
        self.publisher_ = self.create_publisher(GripperStatus, 'gripper_status_rviz', 10)
        self.gripper_latency = 0.500


    def gripper_callback(self, request, response):

        msg = GripperStatus()
        msg.header.stamp = self.get_clock().now().to_msg()

        if request.gripper_state == "open" and request.keep_compressor_running == True:
            bot.set_end_effector_gripper(True, False)
            time.sleep(self.gripper_latency)
            msg.status = "opened"
            self.publisher_.publish(msg)
        elif request.gripper_state == "open" and request.keep_compressor_running == False:
            bot.set_end_effector_gripper(True, False)
            time.sleep(self.gripper_latency)
            bot.set_end_effector_gripper(False, False)
            msg.status = "opened"
            self.publisher_.publish(msg)
        elif request.gripper_state == "close" and request.keep_compressor_running == True:
            bot.set_end_effector_gripper(True, True)
            time.sleep(self.gripper_latency)
            msg.status = "closed"
            self.publisher_.publish(msg)
        elif request.gripper_state == "close" and request.keep_compressor_running == False:
            bot.set_end_effector_gripper(True, True)
            time.sleep(self.gripper_latency)
            bot.set_end_effector_gripper(False, True)
            msg.status = "closed"
            self.publisher_.publish(msg)
        else:
            response.success = False
            response.message = "Invalid service request"
            return response

        response.success = True
        response.message = "Gripper state has been changed"
        return response




def main(args=None):
    rclpy.init(args=args)

    minimal_service = GripperService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()