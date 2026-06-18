from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='localization_lab',
            executable='kalman_filter_localization_v2',
            output ='screen'
        )
    ])