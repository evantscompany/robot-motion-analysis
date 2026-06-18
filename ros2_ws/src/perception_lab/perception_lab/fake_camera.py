import cv2
import numpy as np

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge

class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.bridge = CvBridge()
        self.timer = self.create_timer(
            0.05,
            self.publish_image
        )

        self.ball_x = 50
        self.direction = 5

    def publish_image(self):

        frame = np.zeros(
            (480,640,3),
            dtype = np.uint8
        )

        cv2.circle(
            frame,
            (self.ball_x,240),
            30,
            (0,0,255),
            -1
        )

        self.ball_x += self.direction

        if self.ball_x >600:
            self.direction = -5

        if self.ball_x <40:
            self.direction = 5

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FakeCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()