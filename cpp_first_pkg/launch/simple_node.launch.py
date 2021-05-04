from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_first_pkg',
            executable='simple_node',
            name='simple_node'
        ),
        Node(
            package='cpp_first_pkg',
            executable='simple_loop_node',
            name='simple_loop_node'
        ),
    ])
