from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='perception_lab',
            executable='fake_camera',
            output='screen'
        ),

        Node(
            package='perception_lab',
            executable='image_viewer',
            output='screen'
        ),

    ])