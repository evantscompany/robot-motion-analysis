import cv2
import numpy as np

import rclpy

from rclpy.node import Node


class VisualServoSimulator(Node):

    def __init__(self):

        super().__init__(
            'visual_servo_simulator'
        )

        self.width = 640
        self.height = 480

        self.ball_x = 100
        self.ball_y = 240

        self.ball_velocity = 4

        self.camera_offset = 0.0

        self.kp = 0.03

        self.timer = self.create_timer(
            0.03,
            self.update
        )

    def update(self):

        self.ball_x += (
            self.ball_velocity
        )

        if self.ball_x > 1000:
            self.ball_velocity = -4

        if self.ball_x < 100:
            self.ball_velocity = 4

        frame = np.zeros(
            (
                self.height,
                self.width,
                3
            ),
            dtype=np.uint8
        )

        display_x = int(
            self.ball_x -
            self.camera_offset
        )

        if (
            0 < display_x < self.width
        ):

            cv2.circle(
                frame,
                (
                    display_x,
                    self.ball_y
                ),
                30,
                (0,0,255),
                -1
            )

            error = (
                display_x
                -
                self.width // 2
            )

            angular_z = (
                self.kp *
                error
            )

            self.camera_offset += (
                angular_z
            )

            cv2.putText(
                frame,
                f"Error={error}",
                (10,40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0,255,0),
                2
            )

            cv2.putText(
                frame,
                f"Angular={angular_z:.2f}",
                (10,80),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0,255,255),
                2
            )

            cv2.putText(
                frame,
                f"Offset={self.camera_offset:.1f}",
                (10,120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255,255,255),
                2
            )

        cv2.line(
            frame,
            (
                self.width//2,
                0
            ),
            (
                self.width//2,
                self.height
            ),
            (255,255,0),
            2
        )

        cv2.imshow(
            "Visual Servo Simulator",
            frame
        )

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = VisualServoSimulator()

    rclpy.spin(node)

    cv2.destroyAllWindows()

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()