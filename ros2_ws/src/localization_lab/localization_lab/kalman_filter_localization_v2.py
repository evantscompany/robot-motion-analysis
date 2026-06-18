import math
import random

import rclpy

from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import Marker


class KalmanLocalization(Node):

    def __init__(self):

        super().__init__(
            'kalman_filter_localization_v2'
        )

        self.dt = 0.1

        self.linear_velocity = 1.0
        self.angular_velocity = 0.2

        # Ground Truth

        self.true_x = 0.0
        self.true_y = 0.0
        self.true_theta = 0.0

        # Dead Reckoning

        self.dr_x = 0.0
        self.dr_y = 0.0
        self.dr_theta = 0.0

        # GPS

        self.gps_x = 0.0
        self.gps_y = 0.0

        # Kalman

        self.kf_x = 0.0
        self.kf_y = 0.0

        self.k_gain = 0.8

        # ==================
        # Pose Publishers
        # ==================

        self.truth_pose_pub = self.create_publisher(
            PoseStamped,
            '/ground_truth_pose',
            10
        )

        self.dr_pose_pub = self.create_publisher(
            PoseStamped,
            '/dead_reckoning_pose',
            10
        )

        self.gps_pose_pub = self.create_publisher(
            PoseStamped,
            '/gps_pose',
            10
        )

        self.kf_pose_pub = self.create_publisher(
            PoseStamped,
            '/kalman_pose',
            10
        )

        # ==================
        # Marker Publishers
        # ==================

        self.truth_marker_pub = self.create_publisher(
            Marker,
            '/ground_truth_marker',
            10
        )

        self.dr_marker_pub = self.create_publisher(
            Marker,
            '/dead_reckoning_marker',
            10
        )

        self.gps_marker_pub = self.create_publisher(
            Marker,
            '/gps_marker',
            10
        )

        self.kf_marker_pub = self.create_publisher(
            Marker,
            '/kalman_marker',
            10
        )

        self.timer = self.create_timer(
            self.dt,
            self.update
        )

    def update(self):

        # ==================
        # Ground Truth
        # ==================

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

        # ==================
        # Dead Reckoning
        # ==================

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

        self.dr_theta += (
            measured_w
            * self.dt
        )

        self.dr_x += (
            measured_v
            *
            math.cos(
                self.dr_theta
            )
            *
            self.dt
        )

        self.dr_y += (
            measured_v
            *
            math.sin(
                self.dr_theta
            )
            *
            self.dt
        )

        # ==================
        # GPS
        # ==================

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

        # ==================
        # Kalman
        # ==================

        self.kf_x = (
            self.dr_x
            +
            self.k_gain
            *
            (
                self.gps_x
                -
                self.dr_x
            )
        )

        self.kf_y = (
            self.dr_y
            +
            self.k_gain
            *
            (
                self.gps_y
                -
                self.dr_y
            )
        )

        self.publish_pose(
            self.truth_pose_pub,
            self.true_x,
            self.true_y
        )

        self.publish_pose(
            self.dr_pose_pub,
            self.dr_x,
            self.dr_y
        )

        self.publish_pose(
            self.gps_pose_pub,
            self.gps_x,
            self.gps_y
        )

        self.publish_pose(
            self.kf_pose_pub,
            self.kf_x,
            self.kf_y
        )

        self.publish_marker(
            self.truth_marker_pub,
            self.true_x,
            self.true_y,
            0.0,0.0,1.0,
            "truth"
        )

        self.publish_marker(
            self.dr_marker_pub,
            self.dr_x,
            self.dr_y,
            0.0,1.0,0.0,
            "dr"
        )

        self.publish_marker(
            self.gps_marker_pub,
            self.gps_x,
            self.gps_y,
            1.0,0.0,0.0,
            "gps"
        )

        self.publish_marker(
            self.kf_marker_pub,
            self.kf_x,
            self.kf_y,
            1.0,1.0,0.0,
            "kf"
        )

    def publish_pose(
        self,
        publisher,
        x,
        y
    ):

        msg = PoseStamped()

        msg.header.frame_id = "map"

        msg.pose.position.x = x
        msg.pose.position.y = y

        publisher.publish(msg)

    def publish_marker(
        self,
        publisher,
        x,
        y,
        r,
        g,
        b,
        ns
    ):

        marker = Marker()

        marker.header.frame_id = "map"

        marker.ns = ns

        marker.id = 0

        marker.type = Marker.SPHERE

        marker.action = Marker.ADD

        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        marker.pose.position.x = x
        marker.pose.position.y = y

        publisher.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = KalmanLocalization()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()