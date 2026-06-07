import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math


class PurePursuitTracker(Node):

    def __init__(self):
        super().__init__('pure_pursuit_tracker')

        self.path = [

            (0.5, 0.0),
            (1.0, 0.0),
            (1.5, 0.0),
            (2.0, 0.0),
            (2.5, 0.0),
            (3.0, 0.0),

            (3.0, 0.5),
            (3.0, 1.0),
            (3.0, 1.5),
            (3.0, 2.0),
            (3.0, 2.5),
            (3.0, 3.0),

            (2.5, 3.0),
            (2.0, 3.0),
            (1.5, 3.0),
            (1.0, 3.0),
            (0.5, 3.0),
            (0.0, 3.0),

            (0.0, 2.5),
            (0.0, 2.0),
            (0.0, 1.5),
            # (0.0, 1.0),
            # (0.0, 0.5),
            # (0.0, 0.0)
        ]

        # === Pure Pursuit 파라미터 ===
        self.lookahead_offset = 3
        self.linear_speed = 0.5
        # ============================


        # robot state
        self.x = -0.5
        self.y = 0.0
        self.theta = 0.0

        # subscriber
        self.create_subscription(
            Odometry,
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

        # timer
        self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            'Pure Pursuit Tracker Started'
        )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.theta = 2.0 * math.atan2(
            qz,
            qw
        )

    def control_loop(self):

        # ==========================
        # 가장 가까운 path point 찾기
        # ==========================

        closest_index = 0
        closest_distance = float('inf')

        for i, (px, py) in enumerate(self.path):

            dist = math.sqrt(
                (px - self.x) ** 2 +
                (py - self.y) ** 2
            )

            if dist < closest_distance:
                closest_distance = dist
                closest_index = i

        # ==========================
        # lookahead point 선택
        # ==========================

        lookahead_index = min(
            closest_index +
            self.lookahead_offset,
            len(self.path) - 1
        )

        goal_x, goal_y = (
            self.path[lookahead_index]
        )

        # ==========================
        # 마지막 점 도착 검사
        # ==========================

        final_x, final_y = self.path[-1]

        final_distance = math.sqrt(
            (final_x - self.x) ** 2 +
            (final_y - self.y) ** 2
        )

        if (
            lookahead_index == len(self.path) - 1
            and
            final_distance < 0.2
        ):

            cmd = Twist()

            self.cmd_pub.publish(cmd)

            self.get_logger().info(
                f'CLOSE={closest_index}, '
                f'LOOK={lookahead_index}, '
                f'FINAL={final_distance:.2f},'
                'Path Completed'
            )

            return

        # ==========================
        # heading error 계산
        # ==========================

        dx = goal_x - self.x
        dy = goal_y - self.y

        local_x = (
            math.cos(self.theta) * dx +
            math.sin(self.theta) * dy
        )

        local_y = (
            -math.sin(self.theta) * dx +
            math.cos(self.theta) * dy 
        )

        # ==========================
        # Lookahead Distance
        # ==========================

        ld = math.sqrt(
            local_x ** 2 +
            local_y ** 2
        )
        if ld < 0.001:
            return
        
        # pure pursuit

        curvature = (
            2.0 *
            local_y/
            (ld**2)
        )

        # cmd vel 생성
        cmd = Twist()

        cmd.linear.x = self.linear_speed

        cmd.angular.z=(
            self.linear_speed*
            curvature
        )

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'CLOSE = {closest_index}, '
            f'LOOK = {lookahead_index}, '
            f'LD = {ld:.2f}, '
            f'CURV = {curvature:.2f}'
        )



def main(args=None):

    rclpy.init(args=args)

    node = PurePursuitTracker()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()