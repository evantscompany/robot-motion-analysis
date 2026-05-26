# ============================================================
# WSL 환경용 matplotlib backend 설정
# ============================================================

import matplotlib
matplotlib.use('TkAgg')

# ============================================================
# Import
# ============================================================

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# ============================================================
# Trajectory Visualizer Node
# ============================================================

class TrajectoryVisualizer(Node):

    def __init__(self):

        super().__init__('trajectory_visualizer')

        # trajectory 저장 리스트
        self.x_data = []
        self.y_data = []

        # odom subscriber
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Live Trajectory Visualizer Started'
        )

    # ========================================================
    # odom callback
    # ========================================================

    def odom_callback(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        self.x_data.append(x)
        self.y_data.append(y)


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = TrajectoryVisualizer()

    # ========================================================
    # matplotlib figure 생성
    # ========================================================

    fig, ax = plt.subplots(figsize=(8, 8))

    # 그래프 업데이트 함수
    def update(frame):

        # ROS callback 처리
        rclpy.spin_once(node, timeout_sec=0.01)

        # 그래프 초기화
        ax.clear()

        # trajectory plot
        ax.plot(
            node.x_data,
            node.y_data,
            marker='o'
        )

        # 그래프 설정
        ax.set_title('Live Robot Trajectory')

        ax.set_xlabel('X Position')
        ax.set_ylabel('Y Position')

        ax.grid(True)

        ax.axis('equal')

    # animation 실행
    ani = FuncAnimation(
        fig,
        update,
        interval=100
    )

    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()