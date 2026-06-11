import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import heapq


class WeightedAStarVisualizer(Node):

    def __init__(self):

        super().__init__(
            'weighted_a_star_visualizer'
        )

        # ==========================
        # Grid
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

        self.start = (0, 0)
        self.goal = (9, 9)

        self.marker_pub = (
            self.create_publisher(
                Marker,
                'weighted_path',
                10
            )
        )

        self.create_timer(
            1.0,
            self.publish_path
        )

        self.get_logger().info(
            'Weighted A* Started'
        )

    # ==========================
    # Heuristic
    # ==========================

    def heuristic(self, a, b):

        return (
            abs(a[0] - b[0]) +
            abs(a[1] - b[1])
        )

    # ==========================
    # Cost Map 생성
    # ==========================

    def build_costmap(self):

        costmap = [

            [0 for _ in range(self.cols)]
            for _ in range(self.rows)

        ]

        inflation_radius = 2

        for y in range(self.rows):

            for x in range(self.cols):

                if self.grid[y][x] == 1:

                    costmap[y][x] = 100

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
                                    < 50
                                    and
                                    self.grid[ny][nx] == 0
                                ):
                                    costmap[ny][nx] = 50

        return costmap

    # ==========================
    # Path Reconstruction
    # ==========================

    def reconstruct_path(
        self,
        came_from,
        current
    ):

        path = [current]

        while current in came_from:

            current = came_from[current]

            path.append(current)

        path.reverse()

        return path

    # ==========================
    # Weighted A*
    # ==========================

    def weighted_a_star(self):

        costmap = self.build_costmap()

        open_set = []

        heapq.heappush(
            open_set,
            (0, self.start)
        )

        came_from = {}

        g_score = {
            self.start: 0
        }

        while open_set:

            current = heapq.heappop(
                open_set
            )[1]

            if current == self.goal:

                return self.reconstruct_path(
                    came_from,
                    current
                )

            x, y = current

            neighbors = [

                (x + 1, y),
                (x - 1, y),

                (x, y + 1),
                (x, y - 1)

            ]

            for nx, ny in neighbors:

                if (
                    nx < 0 or
                    ny < 0 or
                    nx >= self.cols or
                    ny >= self.rows
                ):
                    continue

                if self.grid[ny][nx] == 1:
                    continue

                move_cost = 1

                cell_cost = (
                    costmap[ny][nx] / 10.0
                )

                tentative_g = (

                    g_score[current]

                    + move_cost

                    + cell_cost

                )

                if (
                    (nx, ny) not in g_score
                    or
                    tentative_g <
                    g_score[(nx, ny)]
                ):

                    came_from[(nx, ny)] = current

                    g_score[(nx, ny)] = (
                        tentative_g
                    )

                    f_score = (

                        tentative_g

                        + self.heuristic(
                            (nx, ny),
                            self.goal
                        )

                    )

                    heapq.heappush(

                        open_set,

                        (
                            f_score,
                            (nx, ny)
                        )

                    )

        return []

    # ==========================
    # RViz
    # ==========================

    def publish_path(self):

        path = self.weighted_a_star()

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.ns = 'weighted_path'

        marker.id = 0

        marker.type = (
            Marker.LINE_STRIP
        )

        marker.action = (
            Marker.ADD
        )

        marker.scale.x = 0.15

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in path:

            p = Point()

            p.x = float(x)
            p.y = float(y)

            marker.points.append(p)

        self.marker_pub.publish(
            marker
        )

        self.get_logger().info(
            f'Path Length={len(path)}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = (
        WeightedAStarVisualizer()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()