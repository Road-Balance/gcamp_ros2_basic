from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    maze_action_srv_node = Node(
        package='py_action_pkg',
        namespace='',
        executable='maze_action_server',
        name='maze_action_server',
        output='screen',
        parameters=[
            {"robot_namespace": "diffbot"},
        ]
    )

    return LaunchDescription(
        [
            maze_action_srv_node,
        ]
    )
