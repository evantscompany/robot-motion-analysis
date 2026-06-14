import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker

import math
import random


class FakeGPSLocalization(Node):

    def __init__(self):

        super().__init__(
            'fake_gps_localization'
        )

        self.dt = 0.1

        # ==========================
        # Ground Truth
        # ==========================

        self.true_x = 0.0
        self.true_y = 0.0
        self.true_theta = 0.0

        # ==========================
        # Dead Reckoning
        # ==========================

        self.est_x = 0.0
        self.est_y = 0.0
        self.est_theta = 0.0

        # ==========================
        # GPS
        # ==========================

        self.gps_x = 0.0
        self.gps_y = 0.0

        # ==========================
        # Command
        # ==========================

        self.linear_velocity = 1.0
        self.angular_velocity = 0.2

        # ==========================
        # Publishers
        # ==========================

        self.truth_pub = self.create_publisher(
            Marker,
            '/ground_truth',
            10
        )

        self.dead_pub = self.create_publisher(
            Marker,
            '/dead_reckoning',
            10
        )

        self.gps_pub = self.create_publisher(
            Marker,
            '/gps_measurement',
            10
        )

        # ==========================
        # Timer
        # ==========================

        self.create_timer(
            self.dt,
            self.update
        )

        self.get_logger().info(
            'Fake GPS Localization Started'
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
        # Dead Reckoning
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

        # ==========================
        # Fake GPS
        # ==========================

        self.gps_x = (

            self.true_x

            +

            random.gauss(
                0.0,
                0.5
            )

        )

        self.gps_y = (

            self.true_y

            +

            random.gauss(
                0.0,
                0.5
            )

        )

        self.publish_markers()

        self.get_logger().info(

            f'True=({self.true_x:.2f},'
            f'{self.true_y:.2f}) '

            f'DR=({self.est_x:.2f},'
            f'{self.est_y:.2f}) '

            f'GPS=({self.gps_x:.2f},'
            f'{self.gps_y:.2f})'

        )

    def publish_markers(self):

        # ==========================
        # Ground Truth
        # ==========================

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

        # ==========================
        # Dead Reckoning
        # ==========================

        dead = Marker()

        dead.header.frame_id = 'map'

        dead.ns = 'dead'
        dead.id = 0

        dead.type = Marker.SPHERE
        dead.action = Marker.ADD

        dead.scale.x = 0.4
        dead.scale.y = 0.4
        dead.scale.z = 0.4

        dead.color.g = 1.0
        dead.color.a = 1.0

        dead.pose.position.x = (
            self.est_x
        )

        dead.pose.position.y = (
            self.est_y
        )

        self.dead_pub.publish(
            dead
        )

        # ==========================
        # GPS
        # ==========================

        gps = Marker()

        gps.header.frame_id = 'map'

        gps.ns = 'gps'
        gps.id = 0

        gps.type = Marker.SPHERE
        gps.action = Marker.ADD

        gps.scale.x = 0.4
        gps.scale.y = 0.4
        gps.scale.z = 0.4

        gps.color.r = 1.0
        gps.color.a = 1.0

        gps.pose.position.x = (
            self.gps_x
        )

        gps.pose.position.y = (
            self.gps_y
        )

        self.gps_pub.publish(
            gps )


def main(args=None):

    rclpy.init(args=args)

    node = FakeGPSLocalization()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()