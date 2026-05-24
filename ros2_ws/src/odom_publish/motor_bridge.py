# ROS2 to Arduino 모터 제어 브릿지 
# motor_bridge.py
# 이 노드는 cmd_vel을 구독하여 아두이노 시리얼로 명령을 쏘는 통로


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import json

class MotorBridge(Node):
    def __init__(self):
        super().__init__('motor_bridge')
        self.ser = None

        # 1. 시리얼 포트 설정 (아두이노와 연결)
        try:
            self.ser = serial.Serial('/dev/ttyACM0',9600,timeout=1)
        except:
            self.get_logger().error('시리얼 포트를 열수 없음')
        
        # 2. cmd_vel 구독
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.motor_callback,10)

        # 로봇 물리 파라미터 (단위 : 미터)
        self.wheel_diameter = 0.10 
        self.track_width = 0.15

        self.get_logger().info("모터 브릿지 노드 활성화됨")

    def motor_callback(self,msg):
        v=msg.linear.x      #선속도 m/s
        w=msg.angular.z     #각속도 rad/s

        # 수학모델 - 차동 구동 로봇 속도 계산 V=r*w
        # v = (v_right + v_left) /2
        # w = (v_right - v_left) / track_width

        v_right = v+(w*self.track_width / 2.0)
        v_left = v-(w*self.track_width / 2.0)

        # PWM 변환 m/s 속도를 0~255 PWM 값으로 매핑
        # 테스트를 위해 단순 비례상수 100 곱함. 실제 로봇에 맞춰 튜닝 필요

        pwm_right = int(v_right*100)
        pwm_left = int(v_left*100)

        # 제어범위 클리핑 (0~255 제한) but 20~235로 제한
        pwm_right = max(-235,min(235,pwm_right))
        pwm_left = max(-235,min(235,pwm_left))

        # JSON 에 명령 전송
        cmd = {"left":pwm_left, "right":pwm_right}
        self.ser.write((json.dumps(cmd)+"\n").encode('utf-8'))
        self.get_logger().info(f"send to Arduino : {cmd}")

def main(args = None):
    rclpy.init(args=args)
    node = MotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()