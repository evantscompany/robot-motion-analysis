import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import math


class SegmentCTEVisualizer(Node):

    def __init__(self):
        super().__init__('segment_cte_visualizer')

        # Pure Pursuit 와 동일한 Path

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
            (0.0, 1.0),
            (0.0, 0.5),
            (0.0, 0.0)
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
            '/segment_cte',
            10
        )

        self.create_timer(
            0.1,
            self.update_visualization
        )

        self.get_logger().info(
            'Segment CTE Visualizer Started'
        )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def update_visualization(self):

        best_cte = float('inf')

        closest_proj_x = 0.0
        closest_proj_y = 0.0

        # ==========================
        # 모든 선분 검사
        # ==========================

        for i in range(len(self.path) - 1):

            x1, y1 = self.path[i]
            x2, y2 = self.path[i + 1]

            # 선분 벡터

            abx = x2 - x1
            aby = y2 - y1

            # 로봇 벡터

            apx = self.x - x1
            apy = self.y - y1

            denom = (
                abx * abx +
                aby * aby
            )

            if denom < 1e-6:
                continue

            # Projection 계수

            t = (
                apx * abx +
                apy * aby
            ) / denom

            # 선분 내부 제한

            t = max(
                0.0,
                min(1.0, t)
            )

            # Projection Point

            proj_x = (
                x1 +
                t * abx
            )

            proj_y = (
                y1 +
                t * aby
            )

            # CTE 계산

            cte = math.sqrt(
                (self.x - proj_x) ** 2 +
                (self.y - proj_y) ** 2
            )

            if cte < best_cte:

                best_cte = cte

                closest_proj_x = proj_x
                closest_proj_y = proj_y

        # ==========================
        # RViz Marker
        # ==========================

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'segment_cte'
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

        proj_point = Point()
        proj_point.x = closest_proj_x
        proj_point.y = closest_proj_y

        marker.points.append(robot_point)
        marker.points.append(proj_point)

        self.marker_pub.publish(marker)

        self.get_logger().info(
            f'Robot=({self.x:.2f},{self.y:.2f}) '
            f'Proj=({closest_proj_x:.2f},{closest_proj_y:.2f}) '
            f'CTE={best_cte:.3f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SegmentCTEVisualizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()