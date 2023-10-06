import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from dobot_msgs.action import PointToPoint
from dobot_msgs.srv import GripperControl


class PickAndPlace(Node):

    def __init__(self):
        super().__init__('pick_and_place_demo')
        self._action_client = ActionClient(self, PointToPoint, 'PTP_action', callback_group=ReentrantCallbackGroup())
        self.cli = self.create_client(srv_type = GripperControl, srv_name = 'dobot_gripper_service', callback_group = ReentrantCallbackGroup())
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GripperControl.Request()

        self.tasks_list = [
            ["move", [125.0, -180.0, 60.0, 0.0], 1],
            ["gripper", "open", False],
            ["move", [125.0, -180.0, 30.0, 0.0], 1],
            ["gripper", "close", False],
            ["move", [125.0, -180.0, 60.0, 0.0], 1],
            ["move", [150.0, 190.0, 50.0, 0.0], 1],
            ["move", [150.0, 190.0, 20.0, 0.0], 1],
            ["gripper", "open", False],
            ["move", [150.0, 190.0, 50.0, 0.0], 1],
            ["gripper", "close", False],
            ["move", [200.0, 0.0, 100.0, 0.0], 1]
        ]

        self.goal_num = 0

    def execute(self):
            if self.goal_num > len(self.tasks_list)-1:
                rclpy.shutdown()
                sys.exit()
            else:
                self.get_logger().info('*** TASK NUM ***: {0}'.format(self.goal_num))

            if self.tasks_list[self.goal_num][0] == "gripper":
                self.send_request(*self.tasks_list[self.goal_num][1:])
                self.timer = self.create_timer(0.1, self.timer_callback, callback_group=ReentrantCallbackGroup())
                self.goal_num = self.goal_num + 1
            elif self.tasks_list[self.goal_num][0] == "move":
                self.send_goal(*self.tasks_list[self.goal_num][1:])
                self.goal_num = self.goal_num + 1


    def timer_callback(self):
        if self.srv_future.done():
            result = self.srv_future.result()
            self.get_logger().info('Result of service call: {0}'.format(result))
            self.timer.cancel()
            self.execute()

    def send_request(self, gripper_state, keep_compressor_running):
        self.req.gripper_state = gripper_state
        self.req.keep_compressor_running = keep_compressor_running
        self.srv_future = self.cli.call_async(self.req)


    def send_goal(self, _target, _type):
        goal_msg = PointToPoint.Goal()
        goal_msg.target_pose = _target
        goal_msg.motion_type = _type

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        result = future.result().result
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Result of action call: {0}'.format(result))
            self.execute()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback))


def main(args=None):
    rclpy.init(args=args)

    action_client = PickAndPlace()

    action_client.execute()

    executor = MultiThreadedExecutor()

    rclpy.spin(action_client, executor=executor)


if __name__ == '__main__':
    main()


