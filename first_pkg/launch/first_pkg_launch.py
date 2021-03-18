from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='first_pkg',
            executable='simple_node',
            name='simple_node'
        ),
        Node(
            package='first_pkg',
            executable='simple_loop_node',
            name='simple_loop_node'
        ),
    ])
