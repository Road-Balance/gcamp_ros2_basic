#!/usr/bin/env/ python3

import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from nav_msgs.msg import Odometry  # Odometry is the message type

# https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


class OdometrySubscriber(Node):
    """
    Create an OdometrySubscriber class, which is a subclass of the Node class.
    """

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("odom_subscriber")
        self.sub_period = 10  # Hz

        # Create the subscriber. This subscriber will receive an Odometry
        # from the /skidbot/odom topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Odometry,
            "/diffbot/odom",
            self.listener_callback,
            self.sub_period,
        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, data):
        orientation = data.pose.pose.orientation
        _, _, self._yaw = euler_from_quaternion(orientation)

        print(self._yaw)


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    Odometry_subscriber = OdometrySubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(Odometry_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    Odometry_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == "__main__":
    main()
