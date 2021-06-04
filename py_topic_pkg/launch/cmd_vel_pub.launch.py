from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='py_topic_pkg',
                executable='cmd_vel_pub_node',
                name='cmd_vel_pub_node',
                output='screen'
            ),
        ]
    )
