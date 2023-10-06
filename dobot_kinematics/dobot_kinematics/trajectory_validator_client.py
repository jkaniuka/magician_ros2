from dobot_msgs.srv import EvaluatePTPTrajectory
import rclpy
from rclpy.node import Node

# The example concerns the case where the target of the motion was set as a point in Cartesian space.

class PoseValidatorClient(Node):

    def __init__(self):
        super().__init__('dobot_trajectory_validation_cli')
        self.client_validate_goal = self.create_client(EvaluatePTPTrajectory, 'dobot_PTP_validation_service')
        while not self.client_validate_goal.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = EvaluatePTPTrajectory.Request()

    def send_request_check_trajectory(self):
        self.req_validate = EvaluatePTPTrajectory.Request()
        self.req_validate.target = [10.0, 20.0, 30.0, 40.0]
        self.req_validate.motion_type = 4
        self.future_validate = self.client_validate_goal.call_async(self.req_validate)
        rclpy.spin_until_future_complete(self, self.future_validate)
        return self.future_validate.result()


def main(args=None):
    rclpy.init(args=args)
    minimal_client = PoseValidatorClient()
    validation_response = minimal_client.send_request_check_trajectory()
    minimal_client.get_logger().info(
    'Result of calling service: %s' %(validation_response))
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


