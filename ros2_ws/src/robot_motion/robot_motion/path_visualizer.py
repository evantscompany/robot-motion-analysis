import rclpy

from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')

        self.path_pub = self.create_publisher(
            Path,
            '/path',
            10
        )

        self.path_points = [
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
        
        self.create_timer(
            0.5,
            self.publish_path
        )

        self.get_logger().info(
            'Path Visualizer Started'
        )

    def publish_path(self):
        path_msg = Path()

        path_msg.header.frame_id = 'map'
        path_msg.header.stamp=(
            self.get_clock().now().to_msg()
        )

        for x,y in self.path_points:
            pose = PoseStamped()
            pose.header.frame_id = 'map'

            pose.pose.position.x = x
            pose.pose.position.y = y

            pose.pose.orientation.x = x
            pose.pose.orientation.y = y

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

def main(args = None):
    rclpy.init(args=args)
    node = PathVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()