import time
import math
from dobot_msgs.action import SlidingRail
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dobot_driver.dobot_handle import bot
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from dobot_msgs.msg import DobotAlarmCodes
from rclpy.parameter import Parameter
from std_msgs.msg import Float64

class SlidingRailPTPServer(Node):

    def __init__(self):
        super().__init__('dobot_rail_server')

        self._action_server = ActionServer(
            self,
            SlidingRail,
            'move_sliding_rail',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)

        self.subscription_joints = self.create_subscription(
            JointState,
            'dobot_joint_states',
            self.joints_positions_callback,
            10)

        self.subscription_alarms = self.create_subscription(
            DobotAlarmCodes,
            'dobot_alarms',
            self.active_alarms_callback,
            10)
        
        self.subscription_rail_pose = self.create_subscription(
            Float64,
            'dobot_rail_pose',
            self.rail_pose_callback,
            10)

        self.target = 0
        self.pose_arr = []
        self.dobot_pose = [] 
        self.rail_pose = 0
        self.rail_vel = 100 # init value within safe range
        self.rail_acc = 100 # init value within safe range
        self.active_alarms = False

        self.declare_parameter('rail_vel', rclpy.Parameter.Type.INTEGER, 
                               ParameterDescriptor(description='Sliding rail velocity expressed in mm/s.', 
                                                   additional_constraints = "The velocity value must be less than 140."))

        self.declare_parameter('rail_acc', rclpy.Parameter.Type.INTEGER, 
                        ParameterDescriptor(description='Sliding rail acceleration expressed in mm/s^2.', 
                                            additional_constraints = "The acceleration value must be less than 140."))
        
        bot.set_sliding_rail_status(1,1)
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'rail_vel' and param.type_ == Parameter.Type.INTEGER:
                self.rail_vel = param.value
                bot.set_point_to_point_sliding_rail_params(int(self.rail_vel) * 2, int(self.rail_acc) * 2) #BUG
                return SetParametersResult(successful=True)
            elif param.name == 'rail_acc' and param.type_ == Parameter.Type.INTEGER:
                self.rail_acc = param.value
                bot.set_point_to_point_sliding_rail_params(int(self.rail_vel) * 2, int(self.rail_acc) * 2) #BUG
                return SetParametersResult(successful=True)
            else:
                return SetParametersResult(successful=False)

    def rail_pose_callback(self, msg):
        self.rail_pose = msg.data

    def joints_positions_callback(self, msg):
        self.dobot_pose = [math.degrees(msg.position[0]), math.degrees(msg.position[1]), math.degrees(msg.position[2]), math.degrees(msg.position[3])]

    def active_alarms_callback(self, msg):
        if not msg.alarms_list:
            self.active_alarms = False
        else:
            self.active_alarms = True

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    @staticmethod
    # Check if rail vel and acc are within safe range
    def is_param_valid(param):
        if 0.0 < param < 140.0 and int(param) != 0:
            return True
        return False


    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.target = goal_request.target_pose

        # Check if there are active alarms (if the LED diode lights up red) 
        if self.active_alarms:
            self.get_logger().warn("Goal rejected because of active alarms (LED diode in the robot base lights up red)")
            return GoalResponse.REJECT

        if self.target < 1 or self.target > 1000:
            self.get_logger().warn("Goal rejected - sliding rail is only 1m long")
            return GoalResponse.REJECT

        if SlidingRailPTPServer.is_param_valid(self.rail_vel) and SlidingRailPTPServer.is_param_valid(self.rail_acc):
            self.get_logger().info("Rail velocity: {0}".format(self.rail_vel))
            self.get_logger().info("Rail acceleration: {0}".format(self.rail_acc))
        else:
            self.get_logger().warn('Wrong rail acceleration or velocity value')
            return GoalResponse.REJECT

        self.get_logger().info('Goal: {0}'.format(self.target))
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT 


    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    @staticmethod
    def is_goal_reached(target_pose, current_pose, threshold):
            if abs(target_pose-current_pose) > threshold:
                return False
            return True

    @staticmethod
    def is_pose_stable(pose_arr):
        if len(pose_arr) >= 2:
            if pose_arr[-1] == pose_arr[-2]:
                return True
        return False


    async def execute_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')
        bot.set_point_to_point_sliding_rail_command(4,self.dobot_pose[0], 
                                                    self.dobot_pose[1], 
                                                    self.dobot_pose[2], 
                                                    self.dobot_pose[3], 
                                                    self.target)

        feedback_msg = SlidingRail.Feedback()
        feedback_msg.current_pose = 0.0
        self.pose_arr = []

        result = SlidingRail.Result()

        # Start executing the action
        while not (SlidingRailPTPServer.is_goal_reached(self.target, self.rail_pose, 0.2) and SlidingRailPTPServer.is_pose_stable(self.pose_arr)):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                bot.stop_queue(force=True) 
                bot.clear_queue()
                bot.start_queue()
                self.get_logger().info('Goal canceled')
                result.achieved_pose  = self.rail_pose
                return result


            # Update sequence
            feedback_msg.current_pose = self.rail_pose
            self.pose_arr.append(self.rail_pose)

            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.current_pose))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(0.1)


        goal_handle.succeed()


        result.achieved_pose  = self.rail_pose

        self.get_logger().info('Returning result: {0}'.format(result.achieved_pose))

        return result


def main(args=None):
    rclpy.init(args=args)

    minimal_action_server = SlidingRailPTPServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(minimal_action_server, executor=executor)

    minimal_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
