# https://discourse.ros.org/t/spawning-a-robot-entity-using-a-node-with-gazebo-and-ros-2/9985

import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity


def main(args=None):

    rclpy.init()
    node = rclpy.create_node("gazebo_model_spawner")

    print("Creating Service client to connect to `/spawn_entity`")
    client = node.create_client(SpawnEntity, "/spawn_entity")

    print("Connecting to `/spawn_entity` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info("...Connected!")

    # Get path to the turtlebot3 burgerbot
    urdf_file_path = os.path.join(
        get_package_share_directory("gcamp_gazebo"),
        "urdf",
        "skidbot2.urdf",
    )

    # Set data for request
    request = SpawnEntity.Request()
    request.name = "skidbot2"
    request.xml = open(urdf_file_path, "r").read()
    request.robot_namespace = "skidbot2"
    request.initial_pose.position.x = 1.0
    request.initial_pose.position.y = 1.0
    request.initial_pose.position.z = 0.3

    print("Sending service request to `/spawn_entity`")
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print("response: %r" % future.result())
    else:
        raise RuntimeError("exception while calling service: %r" % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
