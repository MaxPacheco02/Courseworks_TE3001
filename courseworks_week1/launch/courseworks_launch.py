from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='courseworks',
            executable='signal_generator',
        ),
        Node(
            package='courseworks',
            executable='process',
        ),
    ])