import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# this is the function launch  system will look for


def generate_launch_description():

    rviz_file = "skidbot.rviz"
    robot_file = "skidbot.urdf"
    package_name = "gcamp_gazebo"
    world_file_name = "maze_world.world"

    # full  path to urdf and world file
    world = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )
    urdf = os.path.join(get_package_share_directory(package_name), "urdf", robot_file)
    rviz = os.path.join(get_package_share_directory(package_name), "rviz", rviz_file)

    # read urdf contents because to spawn an entity in
    # gazebo we need to provide entire urdf as string on  command line
    xml = open(urdf, "r").read()

    # double quotes need to be with escape sequence
    xml = xml.replace('"', '\\"')

    # 90 degree rotation for initial robot pose

    orientation = "{ orientation : { x : 0.0, y: 0.0, z: 0.707, w: 0.707 } }"

    # this is argument format for spwan_entity service
    spwan_args = '{name: "skidbot", xml: "' + xml + '", initial_pose :' + orientation + '}'

    # create and return launch description object
    return LaunchDescription(
        [
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
                cmd=["ros2", "run", "rviz2", "rviz2", "-d", rviz], output="screen"
            ),
        ]
    )
