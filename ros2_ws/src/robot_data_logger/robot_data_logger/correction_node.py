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

        
        #===============================
        # PID Variables (PID 변수 추가)
        
        # 현재 오차 
        self.current_error = 0.0

        # 이전 오차 -> Derivative 계산용
        self.previous_error = 0.0

        # 누적 오차 -> Integral 계산용
        self.integral_error = 0.0

        # 시간 저장
        self.previous_time = self.get_clock().now()

        #===============================


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

        # 현재 y 위치 저장 -> y drift 를 오차로 사용
        self.current_error = msg.pose.pose.position.y

    # cmd_vel 보정
    def cmd_callback(self,msg):
        corrected_msg = Twist()

        # 원래 linear 속도 유지
        corrected_msg.linear.x = msg.linear.x

        #===============================
        # 현재 시간 계산
        #===============================

        current_time = self.get_clock().now()

        dt = (
            current_time - self.previous_time
        ).nanoseconds / 1e9

        # dt 가 너무 작으면 division 방지
        if dt == 0:
            dt = 0.0001

        #===============================
        #PID Gain
        #===============================
        
        kp = -0.5
        ki = -0.05
        kd = -0.1

        #===============================
        # P Term
        #===============================

        p_term = kp*self.current_error

        #===============================
        # I Term
        #===============================

        self.integral_error += self.current_error*dt
        i_term = ki*self.integral_error

        #===============================
        # D Term
        #===============================

        derivate = (
            self.current_error - self.previous_error
        ) /dt

        d_term = kd * derivate

        #===============================
        # Total Correction
        #===============================

        correction =(
            p_term +
            i_term +
            d_term
        )

        # angular correction 적용
        corrected_msg.angular.z = (msg.angular.z+correction)

        # publish 
        self.corrected_cmd_pub.publish(corrected_msg)

        #===============================
        # 상태 업데이트
        #===============================

        self.previous_error = self.current_error
        self.previous_time = current_time

        #===============================
        # 다비그 출력
        #===============================


        self.get_logger().info(
            f'Error : {self.current_error:.3f} |'
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
        