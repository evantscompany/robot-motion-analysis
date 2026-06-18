import rclpy

from rclpy.node import Node

from visualization_msgs.msg import Marker

import math
import random

class KalmanFilterLocalization(Node):
    def __init__(self):
        super().__init__('kalman_filter_localization')

        self.dt = 0.1

        # ground truth

        self.true_x = 0.0
        self.true_y = 0.0
        self.true_theta = 0.0

        # dead reackoning
        self.est_x = 0.0
        self.est_y = 0.0
        self.est_theta = 0.0

        # gps
        self.gps_x = 0.0
        self.gps_y = 0.0

        # kalman estimate
        self.kf_x = 0.0
        self.kf_y = 0.0

        # kalman gain

        self.k_gain = 0.1

        # motion

        self.linear_velocity = 1.0
        self.angular_velocity = 0.2

        # publisher

        self.truth_pub = self.create_publisher(
            Marker,
            '/dead_reckoning',
            10
        )

        self.gps_pub = self.create_publisher(
            Marker,
            '/gps_measurement',
            10
        )

        self.kf_pub = self.create_publisher(
            Marker,
            '/kalman_estimate',
            10
        )

        self.create_timer(
            self.dt,
            self.update
        )

        self.get_logger().info(
            'Kalman Filter Localization started'
        )

    
    def update(self):

        # ground Truth

        self.true_theta += (
            self.angular_velocity * self.dt
        )