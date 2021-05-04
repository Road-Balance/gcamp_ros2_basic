from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="py_first_pkg",
                executable="hello_ros_node",
                name="hello_ros_node",
                output='screen'
            ),
        ]
    )
