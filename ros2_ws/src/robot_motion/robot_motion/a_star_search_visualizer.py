import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import heapq


class AStarSearchVisualizer(Node):

    def __init__(self):
        super().__init__('a_star_search_visualizer')

        # ==========================
        # Grid Map
        # ==========================

        self.rows = 10
        self.cols = 10

        self.grid = [

            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,1,1,1,1,1,0,0,0],
            [0,0,1,0,0,0,1,0,0,0],
            [0,0,1,0,0,0,1,0,0,0],
            [0,0,1,0,0,0,1,0,0,0],
            [0,0,1,0,0,0,1,0,0,0],
            [0,0,1,1,1,1,1,0,0,0],
            [0,0,0,0,0,0,0,0,0,0],
            [0,0,0,0,0,0,0,0,0,0]

        ]

        self.start = (0, 0)
        self.goal = (9, 9)

        # ==========================
        # A* 상태 변수
        # ==========================

        self.open_set = []

        heapq.heappush(
            self.open_set,
            (0, self.start)
        )

        self.closed_set = set()

        self.came_from = {}

        self.g_score = {
            self.start: 0
        }

        self.path_found = False
        self.final_path = []

        # ==========================
        # Publisher
        # ==========================

        self.marker_pub = self.create_publisher(
            Marker,
            '/a_star_search',
            10
        )

        # ==========================
        # Timer
        # ==========================

        self.create_timer(
            0.2,
            self.step_search
        )

        self.get_logger().info(
            'A* Search Visualizer Started'
        )

    # ==========================
    # Manhattan Distance
    # ==========================

    def heuristic(self, a, b):

        return (
            abs(a[0] - b[0]) +
            abs(a[1] - b[1])
        )

    # ==========================
    # 한 스텝씩 탐색
    # ==========================

    def step_search(self):

        if self.path_found:
            self.publish_markers()
            return

        if len(self.open_set) == 0:

            self.get_logger().info(
                'Path Not Found'
            )

            self.path_found = True
            return

        current = heapq.heappop(
            self.open_set
        )[1]

        self.closed_set.add(current)

        # 목표 도달

        if current == self.goal:

            self.final_path = (
                self.reconstruct_path(
                    self.came_from,
                    current
                )
            )

            self.path_found = True

            self.get_logger().info(
                'Goal Reached'
            )

            self.publish_markers()
            return

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

            if (nx, ny) in self.closed_set:
                continue

            tentative_g = (
                self.g_score[current] + 1
            )

            if (
                (nx, ny) not in self.g_score
                or
                tentative_g <
                self.g_score[(nx, ny)]
            ):

                self.came_from[
                    (nx, ny)
                ] = current

                self.g_score[
                    (nx, ny)
                ] = tentative_g

                f_score = (
                    tentative_g +
                    self.heuristic(
                        (nx, ny),
                        self.goal
                    )
                )

                heapq.heappush(
                    self.open_set,
                    (
                        f_score,
                        (nx, ny)
                    )
                )

        self.publish_markers()

    # ==========================
    # Path 복원
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
    # RViz Marker
    # ==========================

    def publish_markers(self):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'a_star'
        marker.id = 0

        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.05

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.6

        for cell in self.closed_set:

            p = Point()

            p.x = float(cell[0])
            p.y = float(cell[1])

            marker.points.append(p)

        self.marker_pub.publish(marker)

        # 최종 경로

        if len(self.final_path) > 0:

            path_marker = Marker()

            path_marker.header.frame_id = 'map'
            path_marker.header.stamp = (
                self.get_clock().now().to_msg()
            )

            path_marker.ns = 'path'
            path_marker.id = 1

            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD

            path_marker.scale.x = 0.15

            path_marker.color.r = 0.0
            path_marker.color.g = 1.0
            path_marker.color.b = 0.0
            path_marker.color.a = 1.0

            for cell in self.final_path:

                p = Point()

                p.x = float(cell[0])
                p.y = float(cell[1])

                path_marker.points.append(p)

            self.marker_pub.publish(
                path_marker
            )


def main(args=None):

    rclpy.init(args=args)

    node = AStarSearchVisualizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()