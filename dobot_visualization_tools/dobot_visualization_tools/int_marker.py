import sys
from interactive_markers import InteractiveMarkerServer, MenuHandler
import rclpy
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker, InteractiveMarkerFeedback
from rclpy.node import Node
import tf_transformations
from math import degrees, pi, sqrt, cos, sin, atan
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import subprocess
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class TCPInteractiveMarker(Node):

    def __init__(self):
        super().__init__('TCP_int_marker')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.br = StaticTransformBroadcaster(self)

        self.declare_parameter('TCP_x_offset', 0.0)
        self.declare_parameter('TCP_z_offset', 0.0)
        self.TCP_x_offset = self.get_parameter('TCP_x_offset').value
        self.TCP_z_offset = self.get_parameter('TCP_z_offset').value

    def timer_callback(self):

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'magician_link_4'
        t.child_frame_id = 'tool'
        t.transform.translation.x = self.TCP_x_offset #0.059
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.TCP_z_offset #-0.12
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

        try:
            trans = self.tf_buffer.lookup_transform(
                'magician_base_link',
                'tool',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                "Waiting for TF data")
            time.sleep(1)
            return
        # create an interactive marker server on the namespace TCP_marker
        server = InteractiveMarkerServer(self, 'TCP_int_marker')

        menu_handler = MenuHandler()
        menu_handler.insert('Linear motion', callback=self.processFeedback)
        menu_handler.insert('Joint interpolated motion', callback=self.processFeedback)

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'magician_base_link'
        int_marker.pose.position.x = trans.transform.translation.x
        int_marker.pose.position.y = trans.transform.translation.y
        int_marker.pose.position.z = trans.transform.translation.z

        int_marker.pose.orientation.x = trans.transform.rotation.x
        int_marker.pose.orientation.y = trans.transform.rotation.y
        int_marker.pose.orientation.z = trans.transform.rotation.z
        int_marker.pose.orientation.w = trans.transform.rotation.w
        int_marker.name = 'dobot_TCP_int_marker'

        marker_scale = 0.01
        box_marker = Marker()
        box_marker.type = Marker.SPHERE
        box_marker.scale.x = marker_scale
        box_marker.scale.y = marker_scale
        box_marker.scale.z = marker_scale
        box_marker.color.r = 1.0
        box_marker.color.g = 1.0
        box_marker.color.b = 0.0
        box_marker.color.a = 1.0

        # create a non-interactive control which contains the box
        box_control = InteractiveMarkerControl()
        box_control.always_visible = True
        box_control.markers.append(box_marker)
        # add the control to the interactive marker
        int_marker.controls.append(box_control)



        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        self.normalizeQuaternion(control.orientation)
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        # control.orientation_mode = InteractiveMarkerControl.FIXED


        marker = self.create_control_visualization([0.3, 1.0, 0.0, 0.0], [0.02, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0])
        control.markers.append(marker)
        marker = self.create_control_visualization([0.3, 1.0, 0.0, 0.0], [-0.02, 0.0, 0.0], [0.0, 0.0, -1.0, 0.0])
        control.markers.append(marker)
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        self.normalizeQuaternion(control.orientation)
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        control.orientation_mode = InteractiveMarkerControl.FIXED
        disk = self.create_disk()
        control.markers.append(disk)
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        self.normalizeQuaternion(control.orientation)
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        # control.orientation_mode = InteractiveMarkerControl.FIXED
        marker = self.create_control_visualization([0.3, 0.0, 0.0, 1.0], [0.0, 0.0, 0.02], [sqrt(2), 0.0, -sqrt(2), 0.0])
        control.markers.append(marker)
        marker = self.create_control_visualization([0.3, 0.0, 0.0, 1.0], [0.0, 0.0, -0.02], [sqrt(2), 0.0, sqrt(2), 0.0])
        control.markers.append(marker)
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        self.normalizeQuaternion(control.orientation)
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        # control.orientation_mode = InteractiveMarkerControl.FIXED

        marker = self.create_control_visualization([0.3, 0.0, 1.0, 0.0], [0.0, 0.02, 0.0], [sqrt(2), 0.0, 0.0, sqrt(2)])
        control.markers.append(marker)
        marker = self.create_control_visualization([0.3, 0.0, 1.0, 0.0], [0.0, -0.02, 0.0], [sqrt(2), 0.0, 0.0, -sqrt(2)])
        control.markers.append(marker)
        int_marker.controls.append(control)


        control = InteractiveMarkerControl()
        control.interaction_mode = InteractiveMarkerControl.MENU
        # control.description = 'MOVE TO POINT'
        marker_scale = 0.05
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = marker_scale
        marker.scale.y = marker_scale
        marker.scale.z = marker_scale
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.0
        control.markers.append(marker)
        int_marker.controls.append(control)

        server.insert(int_marker, feedback_callback=self.processFeedback)
        menu_handler.apply(server, int_marker.name)

        # 'commit' changes and send to all clients
        server.applyChanges()
        self.timer.cancel()
        self.timer.destroy()


    def create_control_visualization(self, color, offset, rotation):
        marker = Marker()
        marker.type = Marker.ARROW
        marker.scale.x = 0.08 #0.1
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.color.a = color[0]
        marker.color.r = color[1]
        marker.color.g = color[2]
        marker.color.b = color[3]

        marker.pose.position.x = offset[0]
        marker.pose.position.y = offset[1]
        marker.pose.position.z = offset[2]

        marker.pose.orientation.w = rotation[0]
        marker.pose.orientation.x = rotation[1]
        marker.pose.orientation.y = rotation[2]
        marker.pose.orientation.z = rotation[3]

        return marker

    def processFeedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            self.get_logger().info(
                f'Menu item {feedback.menu_entry_id} has been clicked'
            )

            if feedback.menu_entry_id == 1:
                motion_type = 2
            elif feedback.menu_entry_id == 2:
                motion_type = 1

            ori = feedback.pose.orientation
            quat = (ori.x, ori.y, ori.z, ori.w)
            _, _, yaw = tf_transformations.euler_from_quaternion(quat)
            x = feedback.pose.position.x 
            y = feedback.pose.position.y 
            z = feedback.pose.position.z 
            r = yaw
            #print(degrees(yaw))
            # print(feedback.menu_entry_id)

            alfa = atan(abs(x/y))
            if x >= 0 and y >= 0:
                goal_x = x - sin(alfa)*self.TCP_x_offset
                goal_y = y - cos(alfa)*self.TCP_x_offset
            elif x >= 0 and y < 0:
                goal_x = x - sin(alfa)*self.TCP_x_offset
                goal_y = y + cos(alfa)*self.TCP_x_offset
            elif x < 0 and y >= 0:
                goal_x = x + sin(alfa)*self.TCP_x_offset
                goal_y = y - cos(alfa)*self.TCP_x_offset
            elif x < 0 and y < 0:
                goal_x = x + sin(alfa)*self.TCP_x_offset
                goal_y = y + cos(alfa)*self.TCP_x_offset

            goal_z = z + self.TCP_z_offset*(-1)

            goal_x = goal_x * 1000
            goal_y = goal_y * 1000
            goal_z = goal_z * 1000
            goal_r = degrees(r)
            #print(goal_x, goal_y, goal_z, goal_r)

            
            command = 'ros2 action send_goal /PTP_action  dobot_msgs/action/PointToPoint "{motion_type: ' + str(motion_type) + ', target_pose: [' + str(goal_x) + ',' + str(goal_y) + ',' + str(goal_z) + ',' + str(goal_r) + '], velocity_ratio: 0.5, acceleration_ratio: 0.3}"'
            subprocess.Popen(
                        command, universal_newlines=True, shell=True,
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE).communicate()

    def normalizeQuaternion(self, quaternion_msg):
        norm = quaternion_msg.x**2 + quaternion_msg.y**2 + quaternion_msg.z**2 + quaternion_msg.w**2
        s = norm**(-0.5)
        quaternion_msg.x *= s
        quaternion_msg.y *= s
        quaternion_msg.z *= s
        quaternion_msg.w *= s



    def create_disk(self):
        marker = Marker()

        marker.type = Marker.TRIANGLE_LIST
        scale = 0.2
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = sqrt(2)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = sqrt(2)
        marker.pose.orientation.z = 0.0

        steps = 36
        width = 0.3

        marker.points = [Point()] * 6 * steps

        circle1 = []
        circle2 = []

        for i in range(steps):
            a=i/steps*pi*2
            v1 = Point()
            v2 = Point()
            v1.y = 0.5 * cos(a)
            v1.z = 0.5 * sin(a)

            v2.y = (1 + width) * v1.y
            v2.z = (1 + width) * v1.z

            circle1.append(v1)
            circle2.append(v2)

        color = ColorRGBA()
        color.r = color.g = color.b = color.a = 1.0
        marker.colors = [ColorRGBA()] * 2 * steps 
        base_color = ColorRGBA()
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        base_color = marker.color
    

        for i in range(0,steps-1,2):
            i1 = i
            i2 = (i +1) % steps
            i3 = (i+2) % steps
            p = i * 6
            c = i * 2

            marker.points[p + 0] = circle1[i1]
            marker.points[p + 1] = circle2[i2]
            marker.points[p + 2] = circle1[i2]

            marker.points[p + 3] = circle1[i2]
            marker.points[p + 4] = circle2[i2]
            marker.points[p + 5] = circle1[i3]

            color.r = base_color.r * 0.6
            color.g = base_color.g * 0.6
            color.b = base_color.b * 0.6

            marker.colors[c] = color
            marker.colors[c + 1] = color

            p  = p + 6
            c = c + 2

            marker.points[p + 0] = circle2[i1]
            marker.points[p + 1] = circle2[i2]
            marker.points[p + 2] = circle1[i1]

            marker.points[p + 3] = circle2[i2]
            marker.points[p + 4] = circle2[i3]
            marker.points[p + 5] = circle1[i3]

            marker.colors[c] = base_color
            marker.colors[c + 1] = base_color
            


        return marker

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TCPInteractiveMarker()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
