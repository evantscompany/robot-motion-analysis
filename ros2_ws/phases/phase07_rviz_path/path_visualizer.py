'''
In this phase, I will use RViz to analyze real-time robot movements.
In the previous phase, the robot's motion mechanisms were programmed, 
but it was difficult to fully conceptualize the behavior through code alone.
'''

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

# path visulizer Node

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        # path_publisher
        self.path_pub=self.create_publisher(
            Path,
            '/robot_path',
            10
        )

        # odom Subscriber
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # path message 생성

        self.path_msg =Path()

        # Rviz frame 기준
        self.path_msg.header.frame_id = 'map'

        self.get_logger().info(
            'path visualizer Started'
        )

    # Odom callback

    def odom_callback(self,msg):
        # poseStamped 생성
        # 객체 안 내용은 아래와 같음. 
        '''
        단순히 pose 만 쓰지 않고 PoseStamped를 사용해야, 시간과 위치 등을 파악하고, 기준점을 갖고 궤적을 그릴수 있다?

        PoseStamped
        ├── header (언제, 어디서 측정했는가?)
        │    ├── stamp : 데이터가 기록된 '정확한 시간' (초, 나노초 단위)
        │    └── frame_id : 이 위치의 기준이 되는 '좌표계 이름' (예: "map", "odom")
        │
        └── pose (정확한 위치와 방향은 어디인가?)
            ├── position : 위치 좌표 (x, y, z)
            └── orientation : 바라보는 방향 (Quaternion 형태: x, y, z, w)
        '''
        
        pose = PoseStamped()

        # 시간 stamp
        pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # frame 설정
        pose.header.frame_id = 'map'

        # 현재 위치 복사
        pose.pose = msg.pose.pose

        # path 에 pose 추가
        self.path_msg.poses.append(pose)

        # path header 갱신
        self.path_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        # publish
        self.path_pub.publish(
            self.path_msg
        )

        self.get_logger().info(
            f'Path Length: '
            f'{len(self.path_msg.poses)}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()