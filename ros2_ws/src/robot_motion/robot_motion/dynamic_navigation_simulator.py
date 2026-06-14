import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import heapq
import random


class DynamicNavigationSimulator(Node):

    def __init__(self):

        super().__init__(
            'dynamic_navigation_simulator'
        )

        # ==========================
        # Map
        # ==========================

        self.rows = 50
        self.cols = 50

        self.robot = (2, 2)

        self.goal = (
            self.cols - 3,
            self.rows - 3
        )

        # ==========================
        # Obstacles
        # ==========================

        self.obstacle_size = 5

        self.obstacles = []

        self.generate_obstacles()

        # ==========================
        # Current Path
        # ==========================

        self.current_path = []

        # ==========================
        # Publishers
        # ==========================

        self.robot_pub = (
            self.create_publisher(
                Marker,
                '/robot_marker',
                10
            )
        )

        self.path_pub = (
            self.create_publisher(
                Marker,
                '/path_marker',
                10
            )
        )

        self.obstacle_pub = (
            self.create_publisher(
                Marker,
                '/obstacle_marker',
                10
            )
        )

        # ==========================
        # Timer
        # ==========================

        self.step_count = 0

        self.create_timer(
            0.2,
            self.update
        )

        self.get_logger().info(
            'Dynamic Navigation Simulator Started'
        )

    # ==========================
    # Obstacle Generation
    # ==========================

    def generate_obstacles(self):

        self.obstacles.clear()

        for _ in range(8):

            while True:

                x = random.randint(
                    5,
                    self.cols - 10
                )

                y = random.randint(
                    5,
                    self.rows - 10
                )

                if (
                    abs(x - self.robot[0]) > 10
                    and
                    abs(y - self.robot[1]) > 10
                ):
                    break

            direction = random.choice(

                [

                    (1, 0),
                    (-1, 0),
                    (0, 1),
                    (0, -1)

                ]

            )

            self.obstacles.append(

                {

                    'x': x,
                    'y': y,
                    'dx': direction[0],
                    'dy': direction[1]

                }

            )

    # ==========================
    # Occupancy Check
    # ==========================

    def is_obstacle(self, x, y):

        for obs in self.obstacles:

            ox = obs['x']
            oy = obs['y']

            if (

                ox <= x < ox + self.obstacle_size

                and

                oy <= y < oy + self.obstacle_size

            ):

                return True

        return False

    # ==========================
    # Obstacle Motion
    # ==========================

    def move_obstacles(self):

        for obs in self.obstacles:

            nx = obs['x'] + obs['dx']
            ny = obs['y'] + obs['dy']

            if (

                nx < 0

                or

                nx + self.obstacle_size
                >= self.cols

            ):

                obs['dx'] *= -1

            if (

                ny < 0

                or

                ny + self.obstacle_size
                >= self.rows

            ):

                obs['dy'] *= -1

            obs['x'] += obs['dx']
            obs['y'] += obs['dy']

    # ==========================
    # Heuristic
    # ==========================

    def heuristic(
        self,
        a,
        b
    ):

        return (

            abs(a[0] - b[0])

            +

            abs(a[1] - b[1])

        )

    # ==========================
    # Reconstruct
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

                    nx < 0

                    or

                    ny < 0

                    or

                    nx >= self.cols

                    or

                    ny >= self.rows

                ):

                    continue

                if self.is_obstacle(
                    nx,
                    ny
                ):

                    continue

                tentative_g = (

                    g_score[current]

                    + 1

                )

                if (

                    (nx, ny)
                    not in g_score

                    or

                    tentative_g
                    <
                    g_score[(nx, ny)]

                ):

                    came_from[
                        (nx, ny)
                    ] = current

                    g_score[
                        (nx, ny)
                    ] = tentative_g

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
    # Robot Move
    # ==========================

    def move_robot(self):

        if len(self.current_path) < 2:
            return

        self.robot = (
            self.current_path[1]
        )

    # ==========================
    # Robot Marker
    # ==========================

    def publish_robot(self):

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.ns = 'robot'

        marker.id = 0

        marker.type = Marker.SPHERE

        marker.action = Marker.ADD

        marker.scale.x = 0.8
        marker.scale.y = 0.8
        marker.scale.z = 0.8

        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.pose.position.x = (
            float(self.robot[0])
        )

        marker.pose.position.y = (
            float(self.robot[1])
        )

        self.robot_pub.publish(
            marker
        )

    # ==========================
    # Path Marker
    # ==========================

    def publish_path(self):

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.ns = 'path'

        marker.id = 0

        marker.type = (
            Marker.LINE_STRIP
        )

        marker.action = (
            Marker.ADD
        )

        marker.scale.x = 0.25

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for x, y in self.current_path:

            p = Point()

            p.x = float(x)
            p.y = float(y)

            marker.points.append(
                p
            )

        self.path_pub.publish(
            marker
        )

    # ==========================
    # Obstacle Marker
    # ==========================

    def publish_obstacles(self):

        marker = Marker()

        marker.header.frame_id = 'map'

        marker.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        marker.ns = 'obstacles'

        marker.id = 0

        marker.type = (
            Marker.CUBE_LIST
        )

        marker.action = (
            Marker.ADD
        )

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 0.5

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for obs in self.obstacles:

            for dx in range(
                self.obstacle_size
            ):

                for dy in range(
                    self.obstacle_size
                ):

                    p = Point()

                    p.x = float(
                        obs['x'] + dx
                    )

                    p.y = float(
                        obs['y'] + dy
                    )

                    marker.points.append(
                        p
                    )

        self.obstacle_pub.publish(
            marker
        )

    # ==========================
    # Update Loop
    # ==========================

    def update(self):

        self.step_count += 1

        # 3초마다
        # (0.2초 × 15)

        if self.step_count % 15 == 0:

            self.move_obstacles()

        # replanning

        self.current_path = (
            self.a_star()
        )

        # robot move

        self.move_robot()

        self.publish_robot()

        self.publish_path()

        self.publish_obstacles()

        distance = (

            abs(
                self.robot[0]
                -
                self.goal[0]
            )

            +

            abs(
                self.robot[1]
                -
                self.goal[1]
            )

        )

        self.get_logger().info(

            f'Robot={self.robot} '

            f'Goal={self.goal} '

            f'Path={len(self.current_path)} '

            f'Distance={distance}'

        )

        if distance == 0:

            self.get_logger().info(
                'GOAL REACHED'
            )

    # ==========================
    # Main
    # ==========================


def main(args=None):

    rclpy.init(args=args)

    node = (
        DynamicNavigationSimulator()
    )

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()