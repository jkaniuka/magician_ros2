from dobot_msgs.srv import EvaluatePTPTrajectory
import rclpy
from rclpy.node import Node
from dobot_kinematics.dobot_inv_kin import calc_inv_kin
from dobot_kinematics.dobot_forward_kin import calc_FwdKin
import math
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
import os.path as path
from geometry_msgs.msg import PoseStamped 
import tf_transformations
from dobot_kinematics.collision_detection_server import PyBulletCollisionServer
import numpy as np
import os




class PoseValidatorService(Node):

    def __init__(self):
        super().__init__('dobot_trajectory_validation_server')
        self.srv_PTP = self.create_service(EvaluatePTPTrajectory, 'dobot_PTP_validation_service', self.PTP_trajectory_callback)
        self.subscription_TCP = self.create_subscription(PoseStamped, 'dobot_TCP', self.tcp_position_callback, 10)
        self.collision_server = PyBulletCollisionServer()


        # TCP pose before motion execution - in order to determine equation of linear trajectory
        self.dobot_pose = [] 

        self.trajectory_points = []
        self.max_velocity = 115 #TODO [mm/s]
        self.axis_1_range = {"min": -125, "max": 125}
        self.axis_2_range = {"min": -5, "max": 90}
        self.axis_3_range = {"min": -15, "max": 70}
        self.axis_4_range = {"min": -150, "max": 150}

        self.path_to_collision_model = None
        self.prevent_collision_with_ground = None

        # User-defined axis limits

        # Axis 1
        self.declare_parameter('axis_1_range', rclpy.Parameter.Type.INTEGER_ARRAY, 
        ParameterDescriptor(description = "The range of allowed values for the joint angle of the first axis.", 
        additional_constraints = "The value must be between -125 and 125 degrees."))

        # Axis 2
        self.declare_parameter('axis_2_range', rclpy.Parameter.Type.INTEGER_ARRAY, 
        ParameterDescriptor(description = "The range of allowed values for the joint angle of the second axis.", 
        additional_constraints = "The value must be between -5 and 90 degrees."))

        # Axis 3
        self.declare_parameter('axis_3_range', rclpy.Parameter.Type.INTEGER_ARRAY, 
        ParameterDescriptor(description = "The range of allowed values for the joint angle of the third axis.", 
        additional_constraints = "The value must be between -15 and 70 degrees."))

        # Axis 4 
        self.declare_parameter('axis_4_range', rclpy.Parameter.Type.INTEGER_ARRAY, 
        ParameterDescriptor(description = "The range of allowed values for the joint angle of the fourth axis.", 
        additional_constraints = "The value must be between -150 and 150 degrees."))

        # Additional parameters used for collision detection


        # Tool type
        self.declare_parameter('use_ground_collision_detection', rclpy.Parameter.Type.BOOL, 
        ParameterDescriptor(description = "If ground collision detection is on, then you cannot command the movement to the point where the end tool hits the ground. This option is especially useful during laboratory classes and for beginners :-)"))


        self.add_on_set_parameters_callback(self.parameters_callback)


    def parameters_callback(self, params):

        for param in params:
            if param.name == 'axis_1_range':
                self.axis_1_range["min"] = param.value[0]
                self.axis_1_range["max"] = param.value[1]
                return SetParametersResult(successful=True)
            elif param.name == 'axis_2_range':
                self.axis_2_range["min"] = param.value[0]
                self.axis_2_range["max"] = param.value[1]
                return SetParametersResult(successful=True)
            elif param.name == 'axis_3_range':
                self.axis_3_range["min"] = param.value[0]
                self.axis_3_range["max"] = param.value[1]
                return SetParametersResult(successful=True)
            elif param.name == 'axis_4_range':
                self.axis_4_range["min"] = param.value[0]
                self.axis_4_range["max"] = param.value[1]
                return SetParametersResult(successful=True)
            elif param.name == "use_ground_collision_detection":
                if param.value == True:
                    self.prevent_collision_with_ground = True
                elif param.value == False:
                    self.prevent_collision_with_ground = False
                return SetParametersResult(successful=True)
            else:
                return SetParametersResult(successful=False)



    def tcp_position_callback(self, msg):
            quat = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
            _, _, y = tf_transformations.euler_from_quaternion(quat)
            self.dobot_pose = [float(msg.pose.position.x)*1000, float(msg.pose.position.y)*1000, float(msg.pose.position.z)*1000, float(math.degrees(y))]


    def PTP_trajectory_callback(self, request, response):
        (response.is_valid, response.message) = self.is_target_valid(request.target, request.motion_type)
        return response



    def are_angles_in_range(self, angles):
        if (self.axis_1_range["min"] < angles[0] < self.axis_1_range["max"]) and \
           (self.axis_2_range["min"] < angles[1] < self.axis_2_range["max"]) and \
           (self.axis_3_range["min"] < angles[2] < self.axis_3_range["max"]) and \
           (self.axis_4_range["min"] < angles[3] < self.axis_4_range["max"]):
           return True
        return False


    def is_target_valid(self, target, target_type):

        # Target expressed in joint coordinates
        if target_type == 4:
            in_limit = self.are_angles_in_range(target)
            if in_limit == False:
                return (False, 'Joint limits violated')
            is_trajectory_safe = self.collision_server.validate_trajectory(motion_type = target_type, current_pose = self.dobot_pose, target_point = target, detect_ground = self.prevent_collision_with_ground)
            if is_trajectory_safe == False:
                return (False, 'A collision was detected during trajectory validation. The movement cannot be executed.')
            else:
                return (True, 'Trajectory is safe and feasible.')
            
        elif target_type == 5:
            xyz = target.tolist()
            cartesian_target = calc_FwdKin(xyz[0], xyz[1], xyz[2])
            cartesian_target_point = [float(cartesian_target[0]), float(cartesian_target[1]), float(cartesian_target[2])]
            waypoints = self.collision_server.linear_trajecory_to_discrete_waypoints(self.dobot_pose, cartesian_target_point)
            cartesian_target_list = [float(cartesian_target[0]), float(cartesian_target[1]), float(cartesian_target[2]), xyz[3]]
            for point in waypoints:
                end_tool_rotation = target.tolist()[3]
                point.append(end_tool_rotation)
                angles = calc_inv_kin(*point)
                if angles == False:
                    return (False, 'Inv Kin solving error!')
                in_limit = self.are_angles_in_range(angles)
                if in_limit == False:
                    return (False, 'Joint limits violated')
            is_trajectory_safe = self.collision_server.validate_trajectory(motion_type = target_type, current_pose = self.dobot_pose, target_point = cartesian_target_list, detect_ground = self.prevent_collision_with_ground)
            if is_trajectory_safe == False:
                return (False, 'A collision was detected during trajectory validation. The movement cannot be executed.')
            else:
                return (True, 'Trajectory is safe and feasible.')


        # Target expressed in cartesian coordinates
        elif target_type == 1:
            angles = calc_inv_kin(*target)
            if angles == False:
                return (False, 'Inv Kin solving error!')
            in_limit = self.are_angles_in_range(angles)
            if in_limit == False:
                return (False, 'Joint limits violated')
            is_trajectory_safe = self.collision_server.validate_trajectory(motion_type = target_type, current_pose = self.dobot_pose, target_point = target, detect_ground = self.prevent_collision_with_ground)
            if is_trajectory_safe == False:
                return (False, 'A collision was detected during trajectory validation. The movement cannot be executed.')
            else:
                return (True, 'Trajectory is safe and feasible.')


        elif target_type == 2:
            waypoints = self.collision_server.linear_trajecory_to_discrete_waypoints(self.dobot_pose, target)
            for point in waypoints:
                end_tool_rotation = target.tolist()[3]
                point.append(end_tool_rotation)
                angles = calc_inv_kin(*point)
                if angles == False:
                    return (False, 'Inv Kin solving error!')
                in_limit = self.are_angles_in_range(angles)
                if in_limit == False:
                    return (False, 'Joint limits violated')
            is_trajectory_safe = self.collision_server.validate_trajectory(motion_type = target_type, current_pose = self.dobot_pose, target_point = target, detect_ground = self.prevent_collision_with_ground)
            if is_trajectory_safe == False:
                return (False, 'A collision was detected during trajectory validation. The movement cannot be executed.')
            else:
                return (True, 'Trajectory is safe and feasible.')

        elif target_type == "continuous_path": 
            #TODO
            pass
        else:
            return (False, 'Wrong trajectory type!')


def main(args=None):
    rclpy.init(args=args)

    minimal_service = PoseValidatorService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()