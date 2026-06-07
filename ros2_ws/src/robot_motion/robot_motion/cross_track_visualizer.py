import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import math

class CrossTrackVisualizer(Node):
    def __init__(self):
        super().__init__('cross_track_visualizer')

        # Pure Pursuit 과 동일한 경로
        self.path = [    
            (0.5, 0.0),
            (1.0, 0.0),
            (1.5, 0.0),
            (2.0, 0.0),
            (2.5, 0.0),
            (3.0, 0.0),

            (3.0, 0.5),
            (3.0, 1.0),
            (3.0, 1.5),
            (3.0, 2.0),
            (3.0, 2.5),
            (3.0, 3.0),

            (2.5, 3.0),
            (2.0, 3.0),
            (1.5, 3.0),
            (1.0, 3.0),
            (0.5, 3.0),
            (0.0, 3.0),

            (0.0, 2.5),
            (0.0, 2.0),
            (0.0, 1.5),
            # (0.0, 1.0),
            # (0.0, 0.5),
            # (0.0, 0.0)
        ]

        self.x = 0.0
        self.y = 0.0

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10

        )

        self.marker_pub = self.create_publisher(
            Marker,
            'cross_track_error',
            10
        )

        self.create_timer(
            0.1,
            self.update_visualization
        )

        self.get_logger().info(
            'Cross Track Visualizer Started'
        )

    def odom_callback(self,msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def update_visualization(self):
        closest_distance = float('inf')
        closest_x = 0.0
        closest_y = 0.0

        for px,py in self.path:
            dist = math.sqrt(
                (px - self.x) **2 +
                (py - self.y) **2
            )

            if dist < closest_distance:

                closest_distance = dist
                closest_x = px
                closest_y = py

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp =(
            self.get_clock().now().to_msg()
        )

        marker.ns = 'cross_track_error'
        marker.id = 0

        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.05
        
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        robot_point = Point()

        robot_point.x = self.x
        robot_point.y = self.y

        path_point = Point()
        path_point.x = closest_x
        path_point.y = closest_y

        marker.points.append(robot_point)
        marker.points.append(path_point)

        self.marker_pub.publish(marker)
        
        self.get_logger().info(
            f'Robot=({self.x:.2f},{self.y:.2f}) '
            f'Closest=({closest_x:.2f},{closest_y:.2f}) '
            f'CTE={closest_distance:.2f}'
        )

def main(args=None):

    rclpy.init(args=args)

    node = CrossTrackVisualizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()