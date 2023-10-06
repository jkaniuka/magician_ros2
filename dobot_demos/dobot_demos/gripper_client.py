from dobot_msgs.srv import GripperControl
import rclpy
from rclpy.node import Node


class GripperClient(Node):

    def __init__(self):
        super().__init__('dobot_gripper_cli')
        self.cli = self.create_client(GripperControl, 'dobot_gripper_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GripperControl.Request()

    def send_request(self, gripper_state, keep_compressor_running):
        self.req.gripper_state = gripper_state
        self.req.keep_compressor_running = keep_compressor_running
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    minimal_client = GripperClient()
    response = minimal_client.send_request("close", False)
    minimal_client.get_logger().info(
        'Result of calling service: %s' %(response))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


