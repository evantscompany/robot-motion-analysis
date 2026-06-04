import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

import math

class PurePursuitTracker(Node):
    def __init__(self):
        super().__init__('pure_pursuit_tracker')

        # waypoints

        self.path = [

            (0.5,0.0),
            (1.0,0.0),
            (1.5,0.0),
            (2.0,0.0),
            (2.5,0.0),
            (3.0,0.0),

            (3.0,0.5),
            (3.0,1.0),
            (3.0,1.5),
            (3.0,2.0),
            (3.0,2.5),
            (3.0,3.0),

            (2.5,3.0),
            (2.0,3.0),
            (1.5,3.0),
            (1.0,3.0),
            (0.5,3.0),
            (0.0,3.0),

            (0.0,2.5),
            (0.0,2.0),
            (0.0,1.5),
            (0.0,1.0),
            (0.0,0.5),
            (0.0,0.0)
        ]


        # 몇개 앞 waypoint 를 바라볼 것인가?
        self.lookahead_offset = 1

        # robot state

        self.x = 0.0
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

        # Odom callback

    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        self.theta = 2.0 * math.atan2(
            qz,qw
        )

    # controller
    
    def control_loop(self):
        # 모든 waypoint 완료 검사
        if self.current_waypoint >= len(self.waypoints):
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info('All waypoints completed')
            return
        
        # 1. 현재 추적 중인 목표점 가져오기
        current_goal_x, current_goal_y = self.waypoints[self.current_waypoint]

        # 2. 현재 목표점 도달 여부 체크
        current_dx = current_goal_x - self.x
        current_dy = current_goal_y - self.y
        current_distance = math.sqrt(current_dx**2 + current_dy**2)

        # 도달 판정 범위를 필요에 따라 0.2~0.3 정도로 조절
        if current_distance < 0.3:
            self.get_logger().info(f'★ waypoint {self.current_waypoint} Reached ★')
            self.current_waypoint += 1
            
            # 다음 점으로 넘어가자마자 리스트 끝이면 정지
            if self.current_waypoint >= len(self.waypoints):
                cmd = Twist()
                self.cmd_pub.publish(cmd)
                return
            
            # 웨이포인트가 갱신되었으므로 목표점 다시 세팅
            current_goal_x, current_goal_y = self.waypoints[self.current_waypoint]

        # 3. Lookahead 포인트 결정 
        # 코너를 미리 돌아서 판정이 씹히는 걸 막기 위해, 
        # 조향 타겟 index가 현재 웨이포인트보다 앞서되 리스트 범위를 넘지 않게 조절
        lookahead_index = min(
            self.current_waypoint, # <--- 코너 돌 때 씹힌다면 우선 현재 WP를 정직하게 바라보게 self.current_waypoint로 설정하는 것이 안전합니다.
            len(self.waypoints) - 1
        )
        
        goal_x, goal_y = self.waypoints[lookahead_index]
        
        # 4. 제어 입력(오차) 계산
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx**2 + dy**2)

        target_theta = math.atan2(dy, dx)
        angle_error = target_theta - self.theta

        # 각도 정규화 (-pi ~ pi)
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        cmd = Twist()

        # 5. 회전 우선 제어 알고리즘
        # 오차각이 크면 제자리에서 먼저 돌고, 정면을 바라보면 직진
        if abs(angle_error) > 0.15:  # 데드밴드를 0.15(약 8.5도)로 살짝 넓혀주면 안정적입니다.
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 * angle_error # 회전 속도 이득(Gain) 조절
        else:
            cmd.linear.x = 0.4  # 속도를 0.4로 살짝 낮춰 오버슈트를 줄입니다.
            cmd.angular.z = 2.0 * angle_error

        self.cmd_pub.publish(cmd)

        self.get_logger().info(
            f'WP={self.current_waypoint}, Look={lookahead_index}, '
            f'To_WP_Dist={current_distance:.2f}, Angle_Err={angle_error:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()

    