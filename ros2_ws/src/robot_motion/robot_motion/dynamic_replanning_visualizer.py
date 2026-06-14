import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import heapq
import math


class DynamicRobotReplanner(Node):

    def __init__(self):

        super().__init__(
            'dynamic_robot_replanner'
        )

        self.rows = 15
        self.cols = 15

        self.grid = [

            [0 for _ in range(self.cols)]

            for _ in range(self.rows)

        ]

        # ==========================
        # 기존 장애물
        # ==========================

        for y in range(4, 11):

            self.grid[y][7] = 1

        # ==========================
        # start / goal
        # ==========================

        self.robot = (1, 1)

        self.goal = (13, 13)

        # ==========================
        # replanning
        # ==========================

        self.dynamic_obstacle_added = False

        self.start_time = (
            self.get_clock().now()
        )

        self.current_path = []

        # ==========================
        # publisher
        # ==========================

        self.path_pub = self.create_publisher(
            Marker,
            'dynamic_path',
            10
        )

        self.obs_pub = self.create_publisher(
            Marker,
            'dynamic_obstacles',
            10
        )

        self.robot_pub = self.create_publisher(
            Marker,
            'robot_marker',
            10
        )

        # 5Hz
        self.create_timer(
            0.2,
            self.update
        )

        self.get_logger().info(
            'Dynamic Robot Replanner Started'
        )

    # ==========================
    # heuristic
    # ==========================

    def heuristic(self, a, b):

        return (

            abs(a[0] - b[0])

            +

            abs(a[1] - b[1])

        )

    # ==========================
    # reconstruct path
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
    # A*
    # ==========================

    def a_star(self):

        start = self.robot

        open_set = []

        heapq.heappush(
            open_set,
            (0, start)
        )

        came_from = {}

        g_score = {
            start: 0
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

                        tentative_g

                        +

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
    # dynamic obstacle
    # ==========================

    def add_dynamic_obstacle(self):

        # 현재 경로를 가로막음

        for y in range(2, 12):

            self.grid[y][11] = 1

        self.get_logger().warn(
            'DYNAMIC OBSTACLE ADDED'
        )

    # ==========================
    # robot movement
    # ==========================

    def move_robot(self):

        if len(self.current_path) < 2:
            return

        # 현재 위치 제거
        self.current_path.pop(0)

        self.robot = (
            self.current_path[0]
        )

    # ==========================
    # obstacle marker
    # ==========================

    def publish_obstacles(self):

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.ns = 'obstacles'

        marker.id = 0

        marker.type = Marker.CUBE_LIST

        marker.action = Marker.ADD

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.2

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for y in range(self.rows):

            for x in range(self.cols):

                if self.grid[y][x] == 1:

                    p = Point()

                    p.x = float(x)
                    p.y = float(y)

                    marker.points.append(p)

        self.obs_pub.publish(
            marker
        )

    # ==========================
    # path marker
    # ==========================

    def publish_path(self):

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.ns = 'path'

        marker.id = 0

        marker.type = Marker.LINE_STRIP

        marker.action = Marker.ADD

        marker.scale.x = 0.15

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in self.current_path:

            p = Point()

            p.x = float(x)
            p.y = float(y)

            marker.points.append(p)

        self.path_pub.publish(
            marker
        )

    # ==========================
    # robot marker
    # ==========================

    def publish_robot(self):

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.ns = 'robot'

        marker.id = 0

        marker.type = Marker.SPHERE

        marker.action = Marker.ADD

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.pose.position.x = float(
            self.robot[0]
        )

        marker.pose.position.y = float(
            self.robot[1]
        )

        self.robot_pub.publish(
            marker
        )

    # ==========================
    # update
    # ==========================

    def update(self):

        elapsed = (

            self.get_clock().now()

            -

            self.start_time

        ).nanoseconds / 1e9

        # 3초 후
        if (
            elapsed > 3.0
            and
            not self.dynamic_obstacle_added
        ):

            self.add_dynamic_obstacle()

            self.dynamic_obstacle_added = True

        # 매 loop마다 재계획
        self.current_path = self.a_star()

        # 이동
        self.move_robot()

        # publish
        self.publish_obstacles()

        self.publish_path()

        self.publish_robot()

        self.get_logger().info(

            f'Robot={self.robot} '

            f'PathLen={len(self.current_path)}'

        )


def main(args=None):

    rclpy.init(args=args)

    node = DynamicRobotReplanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()