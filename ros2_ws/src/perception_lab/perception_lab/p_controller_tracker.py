import cv2
import numpy as np

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import Image

from cv_bridge import CvBridge


class PControllerTracker(Node):

    def __init__(self):

        super().__init__(
            'p_controller_tracker'
        )

        self.bridge = CvBridge()

        self.subscription = (
            self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                10
            )
        )

        self.image_center = 320

        self.kp = 0.005

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        lower_red1 = np.array(
            [0,120,70]
        )

        upper_red1 = np.array(
            [10,255,255]
        )

        lower_red2 = np.array(
            [170,120,70]
        )

        upper_red2 = np.array(
            [180,255,255]
        )

        mask1 = cv2.inRange(
            hsv,
            lower_red1,
            upper_red1
        )

        mask2 = cv2.inRange(
            hsv,
            lower_red2,
            upper_red2
        )

        mask = mask1 + mask2

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) > 0:

            largest = max(
                contours,
                key=cv2.contourArea
            )

            x, y, w, h = cv2.boundingRect(
                largest
            )

            cx = x + w // 2
            cy = y + h // 2

            error = (
                cx -
                self.image_center
            )

            angular_z = (
                self.kp *
                error
            )

            cv2.rectangle(
                frame,
                (x, y),
                (x+w, y+h),
                (0,255,0),
                2
            )

            cv2.circle(
                frame,
                (cx, cy),
                5,
                (255,0,0),
                -1
            )

            cv2.putText(
                frame,
                f"CX={cx}",
                (10,30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0,255,0),
                2
            )

            cv2.putText(
                frame,
                f"ERROR={error}",
                (10,70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0,255,255),
                2
            )

            cv2.putText(
                frame,
                f"ANGULAR_Z={angular_z:.3f}",
                (10,110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255,255,255),
                2
            )

        cv2.line(
            frame,
            (320,0),
            (320,480),
            (255,255,0),
            2
        )

        cv2.imshow(
            "P Controller Tracker",
            frame
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = PControllerTracker()

    rclpy.spin(node)

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()