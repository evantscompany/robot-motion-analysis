import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker

class HeadingMarker(Node):

    def __init__(self):

        super().__init__('heading_marker')

        self.marker_pub = self.create_publisher(
            Marker,
            '/heading_marker',
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Heading Marker Started'
        )

    def odom_callback(self, msg):

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'heading'

        marker.id = 0

        # =====================================
        # ARROW marker
        # =====================================

        marker.type = Marker.ARROW

        marker.action = Marker.ADD

        # 현재 위치
        marker.pose.position = (
            msg.pose.pose.position
        )

        # 현재 orientation
        marker.pose.orientation = (
            msg.pose.pose.orientation
        )

        # 화살표 크기
        marker.scale.x = 0.8
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # 파란색
        marker.color.a = 1.0

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        self.marker_pub.publish(marker)

def main(args=None):

    rclpy.init(args=args)

    node = HeadingMarker()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()