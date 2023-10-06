from diagnostic_msgs.msg import DiagnosticArray
from dobot_msgs.msg import DobotAlarmCodes

import rclpy
from rclpy.clock import ROSClock
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

from dobot_diagnostics.diagnostics_utils import (
    joints_status,
    motors_status,
    sensors_status,
    links_status,
    MCU_status,
    FPGA_status,
    other_status
)

class DiagnosticTalker(Node):

    def __init__(self):
        super().__init__('dobot_diagnostic_talker')
        self.pub = self.create_publisher(DiagnosticArray,
                                         '/diagnostics',
                                         qos_profile_system_default)
        self.subscription = self.create_subscription(
            DobotAlarmCodes,
            'dobot_alarms',
            self.listener_callback,
            10)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)
        self.alarms_list = []

        self.array = DiagnosticArray()
        self.array.status = []

    def listener_callback(self, msg):
        self.alarms_list = msg.alarms_list

    def timer_callback(self):
        self.array.header.stamp = ROSClock().now().to_msg()
        alalyzed_anarms = [] 
        alalyzed_anarms.extend(joints_status(self.alarms_list))
        alalyzed_anarms.extend(motors_status(self.alarms_list))
        alalyzed_anarms.extend(sensors_status(self.alarms_list))
        alalyzed_anarms.extend(links_status(self.alarms_list))
        alalyzed_anarms.extend(MCU_status(self.alarms_list))
        alalyzed_anarms.extend(FPGA_status(self.alarms_list))
        alalyzed_anarms.extend(other_status(self.alarms_list))

        self.array.status = alalyzed_anarms

        self.pub.publish(self.array)


def main(args=None):
    rclpy.init(args=args)

    node = DiagnosticTalker()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
