#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        parameters=[],
        arguments=[],
        output="screen",
    )

    turtlesim_teleop_node = Node(
        package='turtlesim',
        executable='draw_square',
        parameters=[],
        arguments=[],
    )

    # create and return launch description object
    return LaunchDescription(
        [
            turtlesim_node,
            turtlesim_teleop_node,
        ]
    )