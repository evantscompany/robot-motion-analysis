# this node subscribes to /cmd_vel
# -> which contains the robot's linear and angular velocity commands.
# computes odometry, and publishes /odom  
# -> estimating the robot's position and orientation over time

# A = OdomPublisher()
#     ↓
# ROS2 node 생성
#     ↓
# publisher/subscriber/timer 등록
#     ↓
# cmd_vel 들어오면
# listener_callback 실행
#     ↓
# v,w 업데이트
#     ↓
# 0.1초마다
# publish_odom 실행
#     ↓
# x,y,theta 적분 계산
#     ↓
# odom publish (현재는 로그만 출력)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.publisher_ = self.create_publisher(Odometry,'odom',10)             #/odom 토픽 publish -> 현재 계산된 로봇위치를 다른 노드들에게 전달.(nav2,rviz,localization 등)
        self.subscription = self.create_subscription(Twist,'cmd_vel',self.listener_callback, 10) # 최대 10개 메세지 임시 버퍼링.10-> queue size, cmd_vel 메세지 들어올때마다 실행 - 이벤트 기반 구조

        # 현재 로봇의 위치(x,y)와 방향(theta)
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        # 현재 로봇 속도 저장
        self.v, self.w = 0.0,0.0
        # 10Hz 주기로 odom 계산 및 publish
        self.timer = self.create_timer(0.1, self.publish_odom)
    
    def listener_callback(self,msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def publish_odom(self): #타이머 주기마다 실행 - 즉 이벤트 기반 구조
        dt = 0.1
        self.x += self.v * math.cos(self.theta) * dt
        self.y += self.v * math.sin(self.theta) * dt
        self.theta += self.w*dt

        # Odomtry 메시지 생성
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # 위치정보 채우기
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y

        # theta 를 쿼터니언으로 변환 (ROS2 표준)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        # 발생
        self.publisher_.publish(odom)

        # Odom 메시지 구성 및 발생 
        self.get_logger().info(f"Pose : x={self.x:.2f},y={self.y:.2f},theta={self.theta:.2f}")

def main(args = None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()