import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

# Way point marker

class WaypointMarker(Node):
    def __init__(self):
        super().__init__('waypoint_marker')

        # marker publisher
        self.marker_pub = self.create_publisher(
            Marker,
            '/waypoint_marker',
            10
        )

        # Waypoint list

        self.waypoints = [
            (0.0, 0.0),
            (1.0, 1.0),
            (2.0, 0.0),
            (3.0, 1.0)
        ]

        # timer 생성
        self.create_timer(
            1.0,
            self.publish_markers
        )

        self.get_logger().info(
            'Waypoint Marker started'
        )

    # Marker publish

    def publish_markers(self):

        # waypoint 개수만큼 marker 생성
        for i,(x,y) in enumerate(self.waypoints):

            marker = Marker()

            # Header

            marker.header.frame_id = 'map'
            marker.header.stamp=(
                self.get_clock().now().to_msg()
            )

            # marker 기본 설정
            marker.ns = 'waypoints'
            marker.id = i

            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # 위치 설정

            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0

            # 회전값
            marker.pose.orientation.w = 1.0

            # 크기설정
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2

            # 색상 설정
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            # publish
            self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()