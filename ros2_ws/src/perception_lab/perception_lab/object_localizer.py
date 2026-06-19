import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32

from geometry_msgs.msg import Point


class ObjectLocalizer(Node):

    def __init__(self):

        super().__init__(
            'object_localizer'
        )

        self.depth = None

        self.cx = 320

        self.cy = 240

        self.create_subscription(
            Float32,
            '/camera/depth',
            self.depth_callback,
            10
        )

        self.publisher = (
            self.create_publisher(
                Point,
                '/object_position',
                10
            )
        )

        self.timer = (
            self.create_timer(
                0.1,
                self.publish_position
            )
        )

    def depth_callback(
        self,
        msg
    ):

        self.depth = msg.data

    def publish_position(self):

        if self.depth is None:
            return

        point = Point()

        point.x = self.depth

        point.y = (
            (self.cx - 320)
            * 0.005
        )

        point.z = (
            (self.cy - 240)
            * 0.005
        )

        self.publisher.publish(
            point
        )

        self.get_logger().info(
            f'X={point.x:.2f} '
            f'Y={point.y:.2f} '
            f'Z={point.z:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ObjectLocalizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()