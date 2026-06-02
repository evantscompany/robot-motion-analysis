import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import math
import csv
from datetime import datetime


class ErrorLogger(Node):

    def __init__(self):

        super().__init__('error_logger')

        # =====================================
        # Waypoint 설정
        # =====================================

        self.waypoints = [
            (3.0, 2.0),
            (5.0, 2.0),
            (5.0, 5.0),
            (2.0, 5.0)
        ]

        # 현재는 첫 번째 waypoint 기준으로 오차 계산
        self.current_waypoint = 0

        # =====================================
        # 현재 로봇 상태
        # =====================================

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # =====================================
        # CSV 파일 생성
        # =====================================

        timestamp = datetime.now().strftime(
            "%Y%m%d_%H%M%S"
        )

        filename = (
            f'logs/error_log_{timestamp}.csv'
        )

        self.csv_file = open(
            filename,
            'w',
            newline=''
        )

        self.writer = csv.writer(
            self.csv_file
        )

        # CSV 헤더

        self.writer.writerow([
            'time',
            'waypoint_index',
            'distance_error',
            'angle_error'
        ])

        # =====================================
        # Subscriber
        # =====================================

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # =====================================
        # Timer
        # =====================================

        # 0.1초마다 오차 기록

        self.create_timer(
            0.1,
            self.log_error
        )

        self.get_logger().info(
            f'Error Logger Started : {filename}'
        )

    # =====================================
    # Odom Callback
    # =====================================

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        # yaw 계산

        self.theta = (
            2.0 * math.atan2(qz, qw)
        )

    # =====================================
    # Error Logging
    # =====================================

    def log_error(self):

        # waypoint 가져오기

        goal_x, goal_y = self.waypoints[
            self.current_waypoint
        ]

        # 거리 오차 계산

        dx = goal_x - self.x
        dy = goal_y - self.y

        distance_error = math.sqrt(
            dx**2 + dy**2
        )

        # 목표 방향 계산

        target_theta = math.atan2(
            dy,
            dx
        )

        # 각도 오차 계산

        angle_error = (
            target_theta - self.theta
        )

        # 각도 정규화

        while angle_error > math.pi:
            angle_error -= 2.0 * math.pi

        while angle_error < -math.pi:
            angle_error += 2.0 * math.pi

        # 현재 시간

        current_time = (
            self.get_clock().now().nanoseconds
            / 1e9
        )

        # CSV 저장

        self.writer.writerow([
            current_time,
            self.current_waypoint,
            distance_error,
            angle_error
        ])

        # 즉시 저장

        self.csv_file.flush()

    # =====================================
    # 종료 시 파일 닫기
    # =====================================

    def destroy_node(self):

        self.csv_file.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = ErrorLogger()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()