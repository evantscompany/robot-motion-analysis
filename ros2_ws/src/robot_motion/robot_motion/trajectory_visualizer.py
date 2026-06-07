import rclpy

from rclpy.node import Node

from nav_msgs.msg import Odometry
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped

class TrajectoryVisualizer(Node):
    def __init__(self):
        super().__init__('trajectory_visualizer')

        # path publisher

        self.path_pub = (
            self.create_publisher(
                Path,
                '/path',
                10
            )
        )

        # odom subscriber

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # path message

        self.path_msg = Path()
        self.path_msg.header.frame_id =(
            'map'
        )

        self.get_logger().info(
            'Trajectory Visulizer started'
        )

    def odom_callback(self,msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose=(
            msg.pose.pose
        )

        self.path_msg.header.stamp =(
            self.get_clock()
            .now()
            .to_msg()
        )

        self.path_msg.poses.append(
            pose
        )

        self.path_pub.publish(
            self.path_msg
        )
def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()