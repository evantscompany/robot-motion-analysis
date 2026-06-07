import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker


class GridMapVisualizer(Node):

    def __init__(self):
        super().__init__('grid_map_visualizer')

        # ==========================
        # Grid Map
        #
        # 0 = free
        # 1 = obstacle
        # ==========================

        self.grid = [

            [0,0,0,0,0,0,0,0],
            [0,1,1,1,0,0,0,0],
            [0,0,0,1,0,0,0,0],
            [0,0,0,1,0,1,1,0],
            [0,0,0,0,0,0,0,0],
            [0,1,0,0,0,0,0,0],
            [0,1,0,1,1,1,0,0],
            [0,0,0,0,0,0,0,0]

        ]

        self.cell_size = 0.5

        self.marker_pub = self.create_publisher(
            Marker,
            'grid_map',
            10
        )

        self.create_timer(
            0.5,
            self.publish_map
        )

        self.get_logger().info(
            'Grid Map Visualizer Started'
        )

    def publish_map(self):

        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'grid_map'
        marker.id = 0

        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD

        marker.scale.x = self.cell_size
        marker.scale.y = self.cell_size
        marker.scale.z = 0.1

        marker.color.r = 0.3
        marker.color.g = 0.3
        marker.color.b = 0.3
        marker.color.a = 1.0

        for row in range(len(self.grid)):

            for col in range(len(self.grid[row])):

                if self.grid[row][col] == 1:

                    p = Marker().pose.position

                    p.x = (
                        col *
                        self.cell_size
                    )

                    p.y = (
                        row *
                        self.cell_size
                    )

                    p.z = 0.0

                    marker.points.append(p)

        self.marker_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = GridMapVisualizer()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()