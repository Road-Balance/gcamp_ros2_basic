import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

# this is the function launch  system will look for

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    rviz_file = "diffbot.rviz"
    robot_file = "diffbot.urdf"
    package_name = "gcamp_gazebo"
    world_file_name = "maze_world.world"

    # full  path to urdf and world file
    world = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )
    urdf_file = os.path.join(get_package_share_directory(package_name), "urdf", robot_file)
    rviz_config = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file)

    # read urdf contents because to spawn an entity in
    # gazebo we need to provide entire urdf as string on  command line
    robot_desc = open(urdf_file, "r").read()

    # double quotes need to be with escape sequence
    xml = robot_desc.replace('"', '\\"')

    # 90 degree rotation for initial robot pose
    orientation = "{ orientation : { x : 0.0, y: 0.0, z: 0.707, w: 0.707 } }"

    # this is argument format for spwan_entity service
    spwan_args = '{name: "diffbot", xml: "' + xml + '", initial_pose :' + orientation + '}'

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
        arguments=[urdf_file],
    )

    # create and return launch description object
    return LaunchDescription(
        [
            # robot state publisher allows robot model spawn in RVIZ
            robot_state_publisher_node,

            # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
            # That is because only libgazebo_ros_factory.so contains the service call to /spawn_entity
            ExecuteProcess(
                cmd=["gazebo", "--verbose", world, "-s", "libgazebo_ros_factory.so"],
                output="screen",
            ),
            # tell gazebo to spwan your robot in the world by calling service
            ExecuteProcess(
                cmd=[ "ros2", "service", "call", "/spawn_entity", "gazebo_msgs/SpawnEntity", spwan_args ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["ros2", "run", "rviz2", "rviz2", "-d", rviz_config], 
                output="screen"
            ),
        ]
    )
