import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math

class FakeRobotSimulator(Node):
    def __init__(self):
        super().__init__('fake_robot_simulator')

        self.create_subscription(
            Twist(),
            'cmd_vel',
            self.cmd_callback,
            10
        )


        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # 마지막 cmd_vel 수신시간 - phase 15~16을 위한 추가 사항
        self.last_cmd_time = self.get_clock().now()

        # timeout 시간(초)
        self.timeout_sec = 0.5

        # robot state

        self.x = 0.0
        self.y = 0.0

        # heading angle
        self.theta = 0.0

        # 현재 velocity
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        # timer

        # 10Hz 업데이트
        self.dt = 0.1

        self.create_timer(
            self.dt,
            self.update_robot
        )
        self.get_logger().info(
            'Fake Robot SImulator Started'
        )

    # cmd_vel call back

    def cmd_callback(self,msg):
        self.linear_velocity=(
            msg.linear.x
        )
        self.angular_velocity=(
            msg.angular.z
        )

        # phase15~16 기능을 위한 추가설정
        # 마지막 수신 시각 갱신
        self.last_cmd_time = self.get_clock().now()
    
    def update_robot(self):
        
        # ==========================
        # phase15~16을 위한 설정 추가
        # 현재 시간
        now = self.get_clock().now()
        
        # 마지막 명령 이후 경과시간
        elapsed = (
            now - self.last_cmd_time
        ).nanoseconds /1e9

        # timeout 검사
        if elapsed > self.timeout_sec:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0

        # ==========================


        # heading update
        self.theta += (
            self.angular_velocity*self.dt
        )

        # positiono update
        self.x += (
            self.linear_velocity
            *math.cos(self.theta)
            *self.dt
        )
        self.y += (
            self.linear_velocity
            *math.sin(self.theta)
            *self.dt
        )

        # Quaternion 변환
        qz = math.sin(
            self.theta /2.0
        )
        qw = math.cos(
            self.theta / 2.0
        )

        # Odometry 생성

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # 위치
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # orientation
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # velocity
        odom.twist.twist.linear.x = (
            self.linear_velocity
        )

        odom.twist.twist.angular.z =(
            self.angular_velocity
        )

        # publish
        self.odom_pub.publish(odom)

        # debug log

        self.get_logger().info(
            f'x={self.x:.2f},'
            f'y={self.y:.2f},'
            f'theta={self.theta:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = FakeRobotSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()