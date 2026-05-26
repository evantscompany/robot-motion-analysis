import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class WaypointRoute(Node):
    def __init__(self):
        super().__init__('waypoint_route')
        self.marker_pub = self.create_publisher(
            Marker,
            '/waypoint_route',
            10
        )

        self.waypoints = [
            (0.0,0.0),
            (1.0,1.0),
            (2.0,0.0),
            (3.0,1.0)

        ]

        self.create_timer(
            1.0,
            self.publish_route
        )

        self.get_logger().info(
            'Waypoint Route started'
        )

    def publish_route(self):
        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'route'
        marker.id = 0

        # Line_Strip

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        marker.color.a = 1.0

        marker.color.r =1.0
        marker.color.g =1.0
        marker.color.b =0.0

        # waypoint 연결

        for x,y in self.waypoints:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0

            marker.points.append(p)
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointRoute()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()