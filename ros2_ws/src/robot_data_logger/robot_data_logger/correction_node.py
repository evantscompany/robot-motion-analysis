# correction_node 
# feedback Control
# 구조 : 오차계산 -> correction 생성 -> cmd_vel 수정

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class CorrectionNode(Node):
    def __init__(self):
        super().__init__('correction_node')

        # 현재 drift 저장 변수
        self.current_y = 0.0

        # subscriber

        # 원본 cmd_vel 수신
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # 현재 위치 odom 수신
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # publisher
        self.corrected_cmd_pub = self.create_publisher(
            Twist,
            'corrected_cmd_vel',
            10
        )

        self.get_logger().info('Correction Node started')

    # 현재 위치 업데이트
    def odom_callback(self,msg):

        # 현재 y 위치 저장
        self.current_y = msg.pose.pose.position.y

    # cmd_vel 보정
    def cmd_callback(self,msg):
        corrected_msg = Twist()

        # 원래 linear 속도 유지
        corrected_msg.linear.x = msg.linear.x

        # drrift 기반 angular correction

        # P-controller 형태
        # y drift 가 크면 반대반향 회전 correction 추가

        kp = -0.5

        correction = kp*self.current_y

        corrected_msg.angular.z=(
            msg.angular.z + correction
        )

        # publish
        self.corrected_cmd_pub.publish(corrected_msg)

        # 디버그 출력
        self.get_logger().info(
            f'Y Drift: {self.current_y:.3f} |'
            f'Correction : {correction:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CorrectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
        