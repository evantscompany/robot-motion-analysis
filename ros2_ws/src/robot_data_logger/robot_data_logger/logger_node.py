import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import csv
import os
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # csv 파일 생성
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f'robot_log_{timestamp}.csv'

        self.csv_file = open(filename,mode='w',newline='')
        self.csv_write = csv.writer(self.csv_file)

        # 헤더작성
        self.csv_write.writerow([
            'time',
            'cmd_linear',
            'cmd_angular',
            'pos_x',
            'pos_y',
            'pos_z'
        ])

        # 최신 데이터 저장 변수
        self.cmd_linear = 0.0
        self.cmd_angular = 0.0

        # subscriber 
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info('Data Logger Started')

    def cmd_callback(self,msg):
        self.cmd_linear = msg.linear.x
        self.cmd_angular = msg.angular.z

    def odom_callback(self,msg):
        self.get_logger().info('오도메트리 데이터 수신 중')
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y

        yaw_z = msg.pose.pose.position.z

        current_time = self.get_clock().now().nanoseconds / 1e9

        self.csv_write.writerow([
            current_time,
            self.cmd_linear,
            self.cmd_angular,
            pos_x,
            pos_y,
            yaw_z
        ])

        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)

    node = DataLogger()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ =="__main__":
    main()