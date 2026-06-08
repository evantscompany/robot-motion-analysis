import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import heapq


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        # ======================
        # Grid Map
        # 0 = free
        # 1 = obstacle
        # ======================

        self.grid = [

            [0,0,0,0,0,0],
            [0,1,1,1,0,0],
            [0,0,0,1,0,0],
            [0,0,0,1,0,0],
            [0,0,0,0,0,0],
            [0,0,0,0,0,0]

        ]

        self.rows = len(self.grid)
        self.cols = len(self.grid[0])

        self.start = (0, 0)
        self.goal = (5, 5)

        self.marker_pub = self.create_publisher(
            Marker,
            'astar_path',
            10
        )

        self.path = self.a_star()

        self.create_timer(
            0.5,
            self.publish_path
        )

        self.get_logger().info(
            f'Path = {self.path}'
        )

    # ==========================
    # Heuristic
    # ==========================

    def heuristic(self, a, b):

        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    # ==========================
    # A*
    # ==========================

    def a_star(self):

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
                
                # 예외처리1
                if (
                    nx < 0 or
                    ny < 0 or
                    nx >= self.cols or
                    ny >= self.rows
                ):
                    continue
                
                # 예외처리2
                if self.grid[ny][nx] == 1:
                    continue

                tentative_g = (
                    g_score[current] + 1
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
                        tentative_g +
                        self.heuristic(
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
    # RViz Visualization
    # ==========================

    def publish_path(self):

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'astar_path'
        marker.id = 0

        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.08

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in self.path:

            p = Point()

            p.x = float(x)
            p.y = float(y)

            marker.points.append(p)

        self.marker_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = AStarPlanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()