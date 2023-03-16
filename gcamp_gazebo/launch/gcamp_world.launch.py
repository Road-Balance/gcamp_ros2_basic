import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

# this is the function launch  system will look for
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_file = "skidbot.rviz"
    robot_file = "skidbot.urdf"
    package_name = "gcamp_gazebo"
    world_file_name = "gcamp_world.world"

    pkg_path = os.path.join(get_package_share_directory(package_name))
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   

    urdf_file = os.path.join(pkg_path, "urdf", robot_file)
    rviz_config = os.path.join(pkg_path, "rviz", rviz_file)
    world_path = os.path.join(pkg_path, "worlds", world_file_name)

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))
    )

    # Robot State Publisher
    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'skidbot'],
    )

    rviz_start = ExecuteProcess(
        cmd=["ros2", "run", "rviz2", "rviz2", "-d", rviz_config], output="screen"
    )

    # create and return launch description object
    return LaunchDescription(
        [
            TimerAction(
                period=3.0,
                actions=[rviz_start]
            ),
            # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
            # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
            # ExecuteProcess(
            #     cmd=["gazebo", "--verbose", world_path, "-s", "libgazebo_ros_factory.so"],
            #     output="screen",
            # ),
            start_gazebo_server_cmd,
            start_gazebo_client_cmd,
            robot_state_publisher_node,
            joint_state_publisher,
            # tell gazebo to spwan your robot in the world by calling service
            spawn_entity,
        ]
    )
