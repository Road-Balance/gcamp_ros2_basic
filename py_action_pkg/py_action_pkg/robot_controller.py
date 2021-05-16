#!/usr/bin/env/ python3

import sys
import time
import rclpy
import numpy as np

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 

class RobotController(Node):
    """
    Create an RobotController class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__("robot_controller")

        """
        Clockwise Heading Used
        0 : Upper
        1 : Right
        2 : Down
        3 : Left
        """
        self.rate = self.create_rate(5)

        self.ok = False
        self.yaw = 0.0
        self.forward_distance = 0.0
        self.twist_msg = Twist()

        self.sub_period = 10  # Hz
        self.pub_period = 10  # Hz

        # Create the subscriber. This subscriber will receive an LaserScan
        # Data from the /diffbot/scan topic. The queue size is 10 messages.
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/diffbot/scan',
            self.laser_sub_cb,
            self.sub_period
        )

        # Receive an Odometry from the /diffbot/odom topic.
        self.odom_sub = self.create_subscription(
            Odometry,
            "/diffbot/odom",
            self.odom_sub_cb,
            self.sub_period,
        )

        # Create the publisher. This publisher will control robot by
        # /diffbot/cmd_vel topic. The queue size is 10 messages.
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/diffbot/cmd_vel",
            self.pub_period
        )

        # prevent unused variable warning
        self.cmd_vel_pub
        self.laser_sub
        self.odom_sub
        self.get_logger().info("==== Robot Control Node Started ====\n")

    def is_ok(self):
        return self.ok

    def odom_sub_cb(self, data):
        orientation = data.pose.pose.orientation
        _, _, self.yaw = self.euler_from_quaternion(orientation)
        # print(f"yaw : {self.yaw}")
    
    def laser_sub_cb(self, data):
        self.forward_distance = data.ranges[360]
        # print(f"Distance : {self.forward_distance}")

    def move_robot(self, linear_vel=0.0):
        print("==== Move Robot ====")
        self.twist_msg.linear.x = linear_vel
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

    def stop_robot(self):
        print("==== Stop Robot ====")
        self.twist_msg.linear.x  = 0.0
        self.twist_msg.angular.z  = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

    def turn_robot(self, angular_vel=0.0):
        self.twist_msg.linear.x  = 0.0
        self.twist_msg.angular.z  = angular_vel
        self.cmd_vel_pub.publish(self.twist_msg)

    #    https://gist.github.com/salmagro/2e698ad4fbf9dae40244769c5ab74434
    def euler_from_quaternion(self, quaternion):
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


def turn_robot(rclpy, controller, euler_angle):
    print("Robot Turns to Object Angle")

    while rclpy.ok():
        rclpy.spin_once(controller)
        turn_offset = 0.7 * (euler_angle - controller.yaw)
        controller.turn_robot(turn_offset)

        if abs(turn_offset) < 0.005:
            break

    controller.stop_robot()

def parking_robot(rclpy, controller):
    print("Going Forward Until 0.8m Obstacle Detection")
    while rclpy.ok():
        rclpy.spin_once(controller)
        controller.move_robot(0.5)

        if controller.forward_distance < 0.8:
            break

    controller.stop_robot()

def main(args=None):
    rclpy.init(args=args)

    robot_controller = RobotController()

    parking_robot(rclpy, robot_controller)

    turn_robot(rclpy, robot_controller, 0.0)

    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()