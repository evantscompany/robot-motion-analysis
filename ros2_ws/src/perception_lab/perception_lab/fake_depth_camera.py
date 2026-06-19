import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32

import random


class FakeDepthCamera(Node):

    def __init__(self):

        super().__init__(
            'fake_depth_camera'
        )

        self.publisher = (
            self.create_publisher(
                Float32,
                '/camera/depth',
                10
            )
        )

        self.timer = (
            self.create_timer(
                0.1,
                self.publish_depth
            )
        )

    def publish_depth(self):

        msg = Float32()

        msg.data = (
            1.5 +
            random.uniform(
                -0.2,
                0.2
            )
        )

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = FakeDepthCamera()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()