import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math


class StanleyController(Node):

    def __init__(self):
        super().__init__('stanley_controller')

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
            # (0.0, 1.5),
            # (0.0, 1.0),
            # (0.0, 0.5),
            # (0.0, 0.0)
        ]

        # ==========================
        # Stanley Parameters
        # ==========================

        self.speed = 0.5
        self.k = 2.0

        # ==========================

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Stanley Controller Started'
        )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.theta = 2.0 * math.atan2(qz, qw)

    def normalize_angle(self, angle):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def control_loop(self):

        best_cte = float('inf')

        best_proj_x = 0.0
        best_proj_y = 0.0

        best_segment_theta = 0.0

        # ==========================
        # Closest Segment Search
        # ==========================

        for i in range(len(self.path) - 1):

            x1, y1 = self.path[i]
            x2, y2 = self.path[i + 1]

            abx = x2 - x1
            aby = y2 - y1

            apx = self.x - x1
            apy = self.y - y1

            denom = (
                abx * abx +
                aby * aby
            )

            if denom < 1e-6:
                continue

            t = (
                apx * abx +
                apy * aby
            ) / denom

            t = max(
                0.0,
                min(1.0, t)
            )

            proj_x = (
                x1 +
                t * abx
            )

            proj_y = (
                y1 +
                t * aby
            )

            cte = math.sqrt(
                (self.x - proj_x) ** 2 +
                (self.y - proj_y) ** 2
            )

            if cte < best_cte:

                best_cte = cte

                best_proj_x = proj_x
                best_proj_y = proj_y

                best_segment_theta = math.atan2(
                    aby,
                    abx
                )

        # ==========================
        # Final Goal Check
        # ==========================

        final_x, final_y = self.path[-1]

        final_distance = math.sqrt(
            (final_x - self.x) ** 2 +
            (final_y - self.y) ** 2
        )

        if final_distance < 0.2:

            cmd = Twist()

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                'Path Completed'
            )

            return

        # ==========================
        # Heading Error
        # ==========================

        heading_error = (
            best_segment_theta -
            self.theta
        )

        heading_error = self.normalize_angle(
            heading_error
        )

        # ==========================
        # Cross Track Error Sign
        # ==========================

        dx = best_proj_x - self.x
        dy = best_proj_y - self.y

        cross = (
            math.cos(self.theta) * dy -
            math.sin(self.theta) * dx
        )

        if cross < 0.0:
            best_cte *= -1.0

        # ==========================
        # Stanley Law
        # ==========================

        steering = (
            heading_error +
            math.atan2(
                self.k * best_cte,
                self.speed
            )
        )

        steering = self.normalize_angle(
            steering
        )

        # ==========================
        # Command
        # ==========================

        cmd = Twist()

        cmd.linear.x = self.speed

        cmd.angular.z = steering

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'CTE={best_cte:.2f} '
            f'HE={heading_error:.2f} '
            f'STEER={steering:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = StanleyController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()