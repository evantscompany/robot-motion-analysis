import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker

import math


class RobotMarker(Node):

    def __init__(self):
        super().__init__('robot_marker')

        self.marker_pub = self.create_publisher(
            Marker,
            '/robot_marker',
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Robot Marker Started'
        )

    def odom_callback(self, msg):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'robot'
        marker.id = 0

        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.pose = msg.pose.pose

        marker.scale.x = 0.5
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = RobotMarker()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()