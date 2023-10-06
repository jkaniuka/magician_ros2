import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import time

# Resource: https://github.com/DavidB-CMU/rviz_tools_py/blob/master/src/rviz_tools_py/rviz_tools.py

class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_visualization')
        self.marker_pub = self.create_publisher(Marker, 'TCP_trajectory', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        timer_period = 0.05 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.intermediate_points = []
        self.trans_current = [0,0,0]
        self.trans_previous = [0,0,0]
        self.stability_counter = 0

    def erase_if_not_moving(self):
        if self.trans_current == self.trans_previous:
            self.stability_counter = self.stability_counter + 1
        else:
            self.stability_counter = 0

        if self.stability_counter == 40: # 40 x 0.05 sec = 2 sec
            self.stability_counter = 0
            return True
        return False



    def timer_callback(self):

        try:
            trans = self.tf_buffer.lookup_transform(
                'magician_root_link',
                'TCP',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                "Waiting for TF data")
            time.sleep(1)
            return

        self.trans_current = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]


        ######################################################

        marker = Marker()
        marker.header.frame_id = "magician_root_link"
        marker.id = 0

        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        point = Point()
        point.x = trans.transform.translation.x
        point.y = trans.transform.translation.y
        point.z = trans.transform.translation.z

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
      
        self.intermediate_points.append(point)
        marker.points = self.intermediate_points

        is_pose_stable = self.erase_if_not_moving()

        if is_pose_stable == True:
            marker.action = Marker.DELETE
            self.intermediate_points = []

        self.trans_previous = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        self.marker_pub.publish(marker)
    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TrajectoryPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()