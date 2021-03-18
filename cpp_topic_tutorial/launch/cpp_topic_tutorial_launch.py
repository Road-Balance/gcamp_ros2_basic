from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cpp_topic_tutorial',
            executable='topic_pub_oop_node',
            name='topic_pub_oop_node'
        ),
        Node(
            package='cpp_topic_tutorial',
            executable='topic_sub_oop_node',
            name='topic_sub_oop_node'
        ),
    ])
