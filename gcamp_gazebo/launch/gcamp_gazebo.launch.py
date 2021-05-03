"""Launch a Gazebo server with an empty world and initialize ROS with command line arguments."""

import os
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


os.chdir(os.path.join(str(Path.home()), "gcamp_ros2_ws", "src", "gcamp_ros2_basic", "gcamp_gazebo"))

def generate_launch_description():

    robot_file = 'skidbot.urdf'
    package_name = 'gcamp_gazebo'
    world_file_name = 'gcamp_world.world'

    world = os.path.join(get_package_share_directory(package_name), 
                        'worlds', world_file_name)
    urdf = os.path.join(get_package_share_directory(package_name), 
                        'urdf', robot_file)

    # TODO(anyone): Forward command line arguments once that's supported, see
    # https://github.com/ros2/launch/issues/107
    gzserver = launch.actions.ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen'
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                arguments=['-entity', 'gcamp_gazebo', '-file', urdf],
                output='screen')

    return LaunchDescription([
        gzserver,
        spawn_entity
    ])