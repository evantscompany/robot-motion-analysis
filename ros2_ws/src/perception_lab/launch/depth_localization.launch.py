from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='perception_lab',
            executable='fake_camera'
        ),

        Node(
            package='perception_lab',
            executable='fake_depth_camera'
        ),

        Node(
            package='perception_lab',
            executable='object_localizer'
        )
    ])