import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    cpp_pub_oop_node = Node(
        package="cpp_topic_tutorial",
        executable="topic_pub_oop_node",
        name="topic_pub_oop_node",
        output="screen",
    )

    cpp_sub_oop_node = Node(
        package="cpp_topic_tutorial",
        executable="topic_sub_oop_node",
        name="topic_sub_oop_node",
        output="screen",
    )

    return LaunchDescription([cpp_pub_oop_node, cpp_sub_oop_node])
