import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker

import math

import random


class DeadReckoning(Node):

    def __init__(self):

        super().__init__(
            'dead_reckoning'
        )

        self.dt = 0.1

        # ==========================
        # Ground Truth
        # ==========================

        self.true_x = 0.0
        self.true_y = 0.0
        self.true_theta = 0.0

        # ==========================
        # Estimated Pose
        # ==========================

        self.est_x = 0.0
        self.est_y = 0.0
        self.est_theta = 0.0

        # ==========================
        # Command
        # ==========================

        self.linear_velocity = 1.0
        self.angular_velocity = 0.2

        # ==========================
        # Publisher
        # ==========================

        self.truth_pub = (
            self.create_publisher(
                Marker,
                '/ground_truth',
                10
            )
        )

        self.est_pub = (
            self.create_publisher(
                Marker,
                '/dead_reckoning',
                10
            )
        )

        self.create_timer(
            self.dt,
            self.update
        )

        self.get_logger().info(
            'Dead Reckoning Started'
        )

    def update(self):

        # ==========================
        # Ground Truth
        # ==========================

        self.true_theta += (
            self.angular_velocity
            * self.dt
        )

        self.true_x += (
            self.linear_velocity
            *
            math.cos(
                self.true_theta
            )
            *
            self.dt
        )

        self.true_y += (
            self.linear_velocity
            *
            math.sin(
                self.true_theta
            )
            *
            self.dt
        )



        # ==========================
        # Noisy Dead Reckoning
        # ==========================

        measured_v = (

            self.linear_velocity

            +

            random.gauss(
                0.0,
                0.05
            )

        )

        measured_w = (

            self.angular_velocity

            +

            random.gauss(
                0.0,
                0.02
            )

        )

        self.est_theta += (
            measured_w
            * self.dt
        )

        self.est_x += (

            measured_v

            *

            math.cos(
                self.est_theta
            )

            *

            self.dt

        )

        self.est_y += (

            measured_v

            *

            math.sin(
                self.est_theta
            )

            *

            self.dt

        )

        self.publish_markers()

    def publish_markers(self):

        truth = Marker()

        truth.header.frame_id = 'map'

        truth.ns = 'truth'

        truth.id = 0

        truth.type = Marker.SPHERE

        truth.action = Marker.ADD

        truth.scale.x = 0.5
        truth.scale.y = 0.5
        truth.scale.z = 0.5

        truth.color.b = 1.0
        truth.color.a = 1.0

        truth.pose.position.x = (
            self.true_x
        )

        truth.pose.position.y = (
            self.true_y
        )

        self.truth_pub.publish(
            truth
        )

        estimate = Marker()

        estimate.header.frame_id = 'map'

        estimate.ns = 'estimate'

        estimate.id = 0

        estimate.type = Marker.SPHERE

        estimate.action = Marker.ADD

        estimate.scale.x = 0.4
        estimate.scale.y = 0.4
        estimate.scale.z = 0.4

        estimate.color.g = 1.0
        estimate.color.a = 1.0

        estimate.pose.position.x = (
            self.est_x
        )

        estimate.pose.position.y = (
            self.est_y
        )

        self.est_pub.publish(
            estimate
        )


def main(args=None):

    rclpy.init(args=args)

    node = DeadReckoning()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()