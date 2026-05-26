# ============================================================
# Critical Points Covered:
# - Waypoint Switching
# - Distance Error Calculation
# - Target Management
# - Trajectory Following (P-Control)
# ============================================================

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math

# ============================================================
# Waypoint Controller Node
# Guides the robot sequentially through a list of predefined targets.
# ============================================================

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        # =====================================================
        # Publishers
        # =====================================================
        # Publishes the error-corrected velocity commands
        self.cmd_pub = self.create_publisher(
            Twist,
            '/corrected_cmd_vel',
            10
        )

        # =====================================================
        # Subscribers
        # =====================================================
        # Subscribes to raw velocity inputs
        self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )

        # Subscribes to robot odometry for position tracking
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # =====================================================
        # Current Robot Status
        # =====================================================
        self.current_x = 0.0
        self.current_y = 0.0

        # Current heading/lateral error
        self.current_error = 0.0

        # Storage for the latest received velocity command
        self.latest_cmd = Twist()

        # =====================================================
        # Controller Gains
        # =====================================================
        # Proportional gain for the P-controller
        self.kp = -1.0

        # =====================================================
        # Waypoint List
        # =====================================================
        self.waypoints = [
            (0.0, 0.0),
            (1.0, 1.0),
            (2.0, 0.0),
            (3.0, 1.0)
        ]

        # Index of the active target waypoint
        self.current_waypoint_index = 0

        # Distance threshold to consider a waypoint reached
        self.goal_tolerance = 0.2

        self.get_logger().info('Waypoint Controller Node Started.')

    # =========================================================
    # Velocity Command Callback
    # =========================================================
    def cmd_callback(self, msg):
        # Save the latest velocity command
        self.latest_cmd = msg

        # Compute steering correction using P-control
        correction = self.kp * self.current_error

        # Generate a new Twist message for corrected output
        corrected_cmd = Twist()

        # Maintain original linear velocity
        corrected_cmd.linear.x = msg.linear.x

        # Apply angular velocity correction
        corrected_cmd.angular.z = msg.angular.z + correction

        # Publish the modified command
        self.cmd_pub.publish(corrected_cmd)

        # Log control states
        self.get_logger().info(
            f'Error: {self.current_error:.3f} | '
            f'Correction: {correction:.3f}'
        )

    # =========================================================
    # Odometry Callback
    # =========================================================
    def odom_callback(self, msg):
        # Update current robot position
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Retrieve current target waypoint coordinates
        target_x, target_y = self.waypoints[self.current_waypoint_index]

        # Calculate coordinate differences and Euclidean distance to goal
        dx = target_x - self.current_x
        dy = target_y - self.current_y

        distance_error = math.sqrt(dx**2 + dy**2)

        # Save the current tracking error
        self.current_error = dy

        # Check if the robot has reached the current waypoint
        if distance_error < self.goal_tolerance:
            self.get_logger().info(
                f'Waypoint Reached: ({target_x:.2f}, {target_y:.2f})'
            )

            # Switch to the next waypoint
            self.current_waypoint_index += 1

            # Handle the final waypoint boundary condition
            if self.current_waypoint_index >= len(self.waypoints):
                self.current_waypoint_index = len(self.waypoints) - 1
                self.get_logger().info('Final Waypoint Reached.')

        # Log current tracking status
        self.get_logger().info(
            f'Target: ({target_x:.2f}, {target_y:.2f}) | '
            f'Current: ({self.current_x:.2f}, {self.current_y:.2f})'
        )

# ============================================================
# Main Execution Block
# ============================================================

def main(args=None):
    rclpy.init(args=args)
    node = WaypointController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()