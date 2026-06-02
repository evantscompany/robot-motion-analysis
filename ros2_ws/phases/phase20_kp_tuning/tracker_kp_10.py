#  기존 16phase 에서는 x=3,y=2 고정이었다면
# 이번엔 사각형 형태로 돌아다니도록
# 해당 내용은 phase 15_16 코드를 수정하는 선에서 진행

# 고정 waypoint 하나만 사용.
# goal_x = 3.0 , goal_y = 2.0

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math

class WaypointTracker(Node):
    def __init__(self):
        super().__init__('waypoint_tracker')

        # waypoint 목록
        self.waypoints = [
            (3.0, 2.0),
            (5.0, 2.0),
            (5.0, 5.0),
            (2.0, 5.0)
        ]
        
        # 현재 목표 waypoint 인덱스
        self.current_waypoint = 0

        # 현재 위치
        self.x = 0.0
        self.y = 0.0

        self.theta = 0.0

        # subscriber

        self.create_subscription(
            Odometry,
            '' \
            '/odom',
            self.odom_callback,
            10
        )

        # publisher

        self.cmd_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # Control Loop

        self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Waypoint Tracker Started'
        )

    # Odom callback

    def odom_callback(self,msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w 

        # yaw 계산
        self.theta = 2.0 * math.atan2(qz, qw)

    # controller

    def control_loop(self):
        # 목표까지의 거리
        if self.current_waypoint >=len(self.waypoints):

            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                'All Waypoints Completed'
            )
            return
        
        goal_x, goal_y = self.waypoints[
            self.current_waypoint
        ]


        dx = goal_x - self.x
        dy = goal_y - self.y
        
        distance = math.sqrt(
            dx**2 + dy**2
        )

        # 목표 방향
        target_theta = math.atan2(
            dy,dx
        )

        # 각도 오차

        angle_error = (
            target_theta - self.theta
        )

        # 목표 도착

        if distance <0.2:
            # cmd = Twist()
            # self.cmd_pub.publish(cmd)
            # self.get_logger().info(
            #     'Goal Reached'
            # )

            # return

            self.get_logger().info(
                f'Waypoint {self.current_waypoint} Reached'
            )
            self.current_waypoint += 1
            return
        

        # P controll -> Kp x 오차. 

        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = (
            1.0 * angle_error
        )
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node=WaypointTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()