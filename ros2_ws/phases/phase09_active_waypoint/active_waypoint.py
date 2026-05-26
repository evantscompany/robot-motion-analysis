import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker

class ActiveWaypoint(Node):

    def __init__(self):

        super().__init__('active_waypoint')

        self.marker_pub = self.create_publisher(
            Marker,
            '/active_waypoint_marker',
            10
        )

        self.waypoints = [
            (0.0, 0.0),
            (1.0, 1.0),
            (2.0, 0.0),
            (3.0, 1.0)
        ]

        # 현재 활성 waypoint
        self.current_index = 0

        # timer
        self.create_timer(
            1.0,
            self.publish_markers
        )

        self.get_logger().info(
            'Active Waypoint Started'
        )

    def publish_markers(self):

        for i, (x, y) in enumerate(self.waypoints):

            marker = Marker()

            marker.header.frame_id = 'map'

            marker.header.stamp = (
                self.get_clock().now().to_msg()
            )

            marker.ns = 'active_waypoints'

            marker.id = i

            marker.type = Marker.SPHERE

            marker.action = Marker.ADD

            marker.pose.position.x = x
            marker.pose.position.y = y

            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            marker.color.a = 1.0

            # =====================================
            # 현재 target만 초록색
            # =====================================

            if i == self.current_index:

                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0

            else:

                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            self.marker_pub.publish(marker)

        # 테스트용 자동 waypoint 변경
        self.current_index += 1

        if self.current_index >= len(self.waypoints):

            self.current_index = 0

def main(args=None):

    rclpy.init(args=args)

    node = ActiveWaypoint()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()