import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class CostmapVisualizer(Node):

    def __init__(self):

        super().__init__(
            'costmap_visualizer'
        )

        # ==========================
        # Grid Map
        # ==========================

        self.rows = 10
        self.cols = 10

        self.grid = [

            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,1,1,1,0,0,0,0],
            [0,0,0,1,1,1,0,0,0,0],
            [0,0,0,1,1,1,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,1,1,0],
            [0,0,0,0,0,0,0,1,1,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0]

        ]

        self.marker_pub = (
            self.create_publisher(
                Marker,
                'costmap',
                10
            )
        )

        self.create_timer(
            1.0,
            self.publish_costmap
        )

        self.get_logger().info(
            'Costmap Visualizer Started'
        )

    # ==========================
    # Inflation Layer
    # ==========================

    def build_costmap(self):

        costmap = [

            row.copy()
            for row in self.grid
        ]

        inflation_radius = 1

        for y in range(self.rows):

            for x in range(self.cols):

                if self.grid[y][x] == 1:

                    for dy in range(
                        -inflation_radius,
                        inflation_radius + 1
                    ):

                        for dx in range(
                            -inflation_radius,
                            inflation_radius + 1
                        ):

                            nx = x + dx
                            ny = y + dy

                            if (
                                0 <= nx < self.cols
                                and
                                0 <= ny < self.rows
                            ):

                                if (
                                    costmap[ny][nx]
                                    != 1
                                ):
                                    costmap[ny][nx] = 20

        return costmap

    # ==========================
    # RViz Publish
    # ==========================

    def publish_costmap(self):

        costmap = (
            self.build_costmap()
        )

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.ns = 'costmap'

        marker.id = 0

        marker.type = (
            Marker.CUBE_LIST
        )

        marker.action = (
            Marker.ADD
        )

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.1

        for y in range(self.rows):

            for x in range(self.cols):

                value = costmap[y][x]

                if value == 0:
                    continue

                p = Point()

                p.x = float(x)
                p.y = float(y)

                marker.points.append(
                    p
                )

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(
            marker
        )

        self.get_logger().info(
            'Costmap Published'
        )


def main(args=None):

    rclpy.init(args=args)

    node = (
        CostmapVisualizer()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()