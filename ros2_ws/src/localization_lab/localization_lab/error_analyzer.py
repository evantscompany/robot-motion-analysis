import math

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class ErrorAnalyzer(Node):

    def __init__(self):

        super().__init__(
            'error_analyzer'
        )

        self.truth_x = None
        self.truth_y = None

        self.kf_x = None
        self.kf_y = None

        self.errors = []

        self.create_subscription(
            PoseStamped,
            '/ground_truth_pose',
            self.truth_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/kalman_pose',
            self.kalman_callback,
            10
        )

        self.timer = self.create_timer(
            5.0,
            self.print_statistics
        )

    def truth_callback(self, msg):

        self.truth_x = (
            msg.pose.position.x
        )

        self.truth_y = (
            msg.pose.position.y
        )

    def kalman_callback(self, msg):

        self.kf_x = (
            msg.pose.position.x
        )

        self.kf_y = (
            msg.pose.position.y
        )

        if (
            self.truth_x is None
            or
            self.kf_x is None
        ):
            return

        error = math.sqrt(

            (self.truth_x - self.kf_x) ** 2

            +

            (self.truth_y - self.kf_y) ** 2

        )

        self.errors.append(error)

    def print_statistics(self):

        if len(self.errors) < 10:
            return

        mean_error = (

            sum(self.errors)
            /
            len(self.errors)

        )

        rmse = math.sqrt(

            sum(

                e * e

                for e in self.errors

            )

            /

            len(self.errors)

        )

        max_error = max(
            self.errors
        )

        self.get_logger().info(

            "\n"
            "=====================\n"
            "Localization Report\n"
            "=====================\n"
            f"Samples     : {len(self.errors)}\n"
            f"Mean Error  : {mean_error:.3f} m\n"
            f"RMSE        : {rmse:.3f} m\n"
            f"Max Error   : {max_error:.3f} m\n"
            "=====================\n"

        )


def main(args=None):

    rclpy.init(args=args)

    node = ErrorAnalyzer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()