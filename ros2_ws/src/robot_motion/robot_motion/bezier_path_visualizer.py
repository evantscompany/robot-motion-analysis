import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import math


class BezierPathVisualizer(Node):

    def __init__(self):

        super().__init__(
            'bezier_path_visualizer'
        )

        # ==================================
        # Raw A* Style Path
        # ==================================

        self.path = [

            (0.0, 0.0),

            (0.0, 2.0),

            (2.0, 2.0),

            (2.0, 4.0),

            (5.0, 4.0)

        ]

        self.marker_pub = (
            self.create_publisher(
                Marker,
                'bezier_path',
                10
            )
        )

        self.create_timer(
            1.0,
            self.publish_path
        )

        self.get_logger().info(
            'Bezier Path Visualizer Started'
        )

    # ==================================
    # Quadratic Bezier
    # ==================================

    def quadratic_bezier(
        self,
        p0,
        p1,
        p2,
        samples=30
    ):

        curve = []

        for i in range(samples + 1):

            t = i / samples

            x = (
                (1 - t) ** 2 * p0[0]
                +
                2 * (1 - t) * t * p1[0]
                +
                t ** 2 * p2[0]
            )

            y = (
                (1 - t) ** 2 * p0[1]
                +
                2 * (1 - t) * t * p1[1]
                +
                t ** 2 * p2[1]
            )

            curve.append(
                (x, y)
            )

        return curve

    # ==================================
    # Build Entire Curve
    # ==================================

    def generate_smooth_path(self):

        smooth_path = []

        for i in range(
            len(self.path) - 2
        ):

            p0 = self.path[i]
            p1 = self.path[i + 1]
            p2 = self.path[i + 2]

            segment = (
                self.quadratic_bezier(
                    p0,
                    p1,
                    p2
                )
            )

            smooth_path.extend(
                segment
            )

        return smooth_path

    # ==================================
    # RViz Publish
    # ==================================

    def publish_path(self):

        path = (
            self.generate_smooth_path()
        )

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.ns = (
            'bezier_path'
        )

        marker.id = 0

        marker.type = (
            Marker.LINE_STRIP
        )

        marker.action = (
            Marker.ADD
        )

        marker.scale.x = 0.08

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in path:

            p = Point()

            p.x = x
            p.y = y

            marker.points.append(
                p
            )

        self.marker_pub.publish(
            marker
        )

        self.get_logger().info(
            f'Smooth Points = {len(path)}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = (
        BezierPathVisualizer()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()