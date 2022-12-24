from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_action_server',
            executable='example53',
            output='screen'),
    ])
