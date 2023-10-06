import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import time

class DobotRange(Node):

    def __init__(self):
        super().__init__('dobot_workspace')
        self.marker_pub = self.create_publisher(Marker, 'reachability_range', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):

        from_frame_rel = 'magician_base_link'
        to_frame_rel = 'magician_root_link'

        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                now)
        except TransformException as ex:
            self.get_logger().info(
                "Waiting for TF data")
            time.sleep(1)
            return

        marker = Marker()
        marker.header.frame_id = "magician_root_link"
        marker.id = 0

        marker.type = Marker.MESH_RESOURCE
        package_share_directory = get_package_share_directory('dobot_visualization_tools')
        my_path = "file://" + str(package_share_directory) + "/dae_models/dobot_range.dae"
        marker.mesh_resource = my_path
        marker.action = Marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.a = 0.3
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0


        marker.pose.orientation.w = trans.transform.rotation.w
        marker.pose.orientation.x = trans.transform.rotation.x
        marker.pose.orientation.y = trans.transform.rotation.y
        marker.pose.orientation.z = trans.transform.rotation.z

        marker.pose.position.x = trans.transform.translation.x
        marker.pose.position.y = trans.transform.translation.y
        marker.pose.position.z = trans.transform.translation.z

        self.marker_pub.publish(marker)
    

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = DobotRange()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()