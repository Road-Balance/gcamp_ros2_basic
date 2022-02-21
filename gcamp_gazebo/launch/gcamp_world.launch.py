import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file]
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
            ExecuteProcess(
                cmd=["gazebo", "--verbose", world_path, "-s", "libgazebo_ros_factory.so"],
                output="screen",
            ),
            robot_state_publisher_node,
            spawn_entity,

            # # tell gazebo to spwan your robot in the world by calling service
            # ExecuteProcess(
            #     cmd=[ "ros2", "service", "call", "/spawn_entity", "gazebo_msgs/SpawnEntity", spwan_args ],
            #     output="screen",
            # ),
        ]
    )
