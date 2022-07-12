#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

import launch
from launch import LaunchDescription
from launch.actions import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf = os.path.join(get_package_share_directory("airobo_gazebo"), "urdf", "skidbot_description.urdf")
    rviz = os.path.join(get_package_share_directory("airobo_gazebo"), "rviz", "description.rviz")

    robot_desc = open(urdf, 'r').read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf],
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz],
    )

    return LaunchDescription(
        [
            joint_state_publisher_gui_node,
            robot_state_publisher_node,
            TimerAction(
                period=3.0,
                actions=[rviz_node]
            ),
        ]
    )
