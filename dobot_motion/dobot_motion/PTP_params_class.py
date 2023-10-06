import rclpy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange


def declare_PTP_params(self):
    
    # JOINT 1 
    self.declare_parameter('JT1_vel', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Velocity of the first joint in deg/s.", 
    additional_constraints = "For a 500g load, the speed value must be less than 300 and for 250g load less than 320.",
    integer_range = [IntegerRange(from_value = 1, to_value = 320, step = 0)]))

    self.declare_parameter('JT1_acc', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Acceleration of the first joint in deg/s^2.", 
    additional_constraints = "The acceleration value must be less than 300.",
    integer_range = [IntegerRange(from_value = 1, to_value = 300, step = 0)]))

    # JOINT 2
    self.declare_parameter('JT2_vel', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Velocity of the second joint in deg/s.", 
    additional_constraints = "For a 500g load, the speed value must be less than 300 and for 250g load iless than 320.",
    integer_range = [IntegerRange(from_value = 1, to_value = 320, step = 0)]))

    self.declare_parameter('JT2_acc', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Acceleration of the second joint in deg/s^2.", 
    additional_constraints = "The acceleration value must be less than 300.",
    integer_range = [IntegerRange(from_value = 1, to_value = 300, step = 0)]))

    # JOINT 3 
    self.declare_parameter('JT3_vel', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Velocity of the third joint in deg/s.", 
    additional_constraints = "For a 500g load, the speed value must be less than 300 and for 250g load less than 320.",
    integer_range = [IntegerRange(from_value = 1, to_value = 320, step = 0)]))

    self.declare_parameter('JT3_acc', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Acceleration of the third joint in deg/s^2.", 
    additional_constraints = "The acceleration value must be less than 300.",
    integer_range = [IntegerRange(from_value = 1, to_value = 300, step = 0)]))

    # JOINT 4
    self.declare_parameter('JT4_vel', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Velocity of the fourth joint in deg/s.", 
    additional_constraints = "For a 500g load, the speed value must be less than 300 and for 250g load less than 480.",
    integer_range = [IntegerRange(from_value = 1, to_value = 480, step = 0)]))

    self.declare_parameter('JT4_acc', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Acceleration of the fourth joint in deg/s^2.", 
    additional_constraints = "The acceleration value must be less than 300.",
    integer_range = [IntegerRange(from_value = 1, to_value = 300, step = 0)]))

    ##########################################################################################################

    # Velocity
    self.declare_parameter('TCP_vel', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Linear velocity of tool center point (TCP) in mm/s.", 
    additional_constraints = "Must be below 300 mm/s.",
    integer_range = [IntegerRange(from_value = 1, to_value = 300, step = 0)]))

    self.declare_parameter('end_tool_rot_vel', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Rotational velocity of end effector in deg/s.", 
    additional_constraints = "Must be below 300 deg/s for 500g workload and below 480 deg/s for 250g workload.",
    integer_range = [IntegerRange(from_value = 1, to_value = 480, step = 0)]))

    # Acceleration
    self.declare_parameter('TCP_acc', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Linear acceleration of tool center point (TCP) in mm/s^2.", 
    additional_constraints = "Must be below 400 mm/s^2.",
    integer_range = [IntegerRange(from_value = 1, to_value = 400, step = 0)]))

    self.declare_parameter('end_tool_rot_acc', rclpy.Parameter.Type.INTEGER, 
    ParameterDescriptor(description="Rotational acceleration of end effector in deg/s^2.", 
    additional_constraints = "Must be below 300 mm/s^2.",
    integer_range = [IntegerRange(from_value = 1, to_value = 300, step = 0)]))

    #########################################################################################################

    self.joint_params_names = ['JT1_vel', 'JT2_vel', 'JT3_vel', 'JT4_vel', 'JT1_acc', 'JT2_acc', 'JT3_acc', 'JT4_acc']
    self.cartesian_params_names = ['TCP_vel', 'end_tool_rot_vel', 'TCP_acc', 'end_tool_rot_acc']

    self.joint_params_dict = {}
    self.cartesian_params_dict = {}


