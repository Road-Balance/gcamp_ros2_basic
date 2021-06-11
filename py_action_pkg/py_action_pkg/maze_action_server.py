#!/usr/bin/env/ python3

import math
import time

from custom_interfaces.action import Maze
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from py_action_pkg.image_sub import ImageSubscriber
from py_action_pkg.robot_controller import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import LaserScan

direction_dict = {0: (-1 * math.pi / 2), 1: math.pi, 2: math.pi / 2, 3: 0.0}
direction_str_dict = {0: 'Up', 1: 'Right', 2: 'Down', 3: 'Left'}

"""
Maze.action structure

    int32[] turning_sequence
    ---
    bool success
    ---
    string feedback_msg
"""


class MazeActionServer(Node):

    def __init__(self):
        super().__init__('maze_action_server')

        self.yaw = 0.0
        self.forward_distance = 0.0

        self.twist_msg = Twist()
        self.loop_rate = self.create_rate(5, self.get_clock())

        self.laser_sub = self.create_subscription(
            LaserScan, '/diffbot/scan', self.laser_sub_cb, 10
        )

        self.odom_sub = self.create_subscription(
            Odometry, '/diffbot/odom', self.odom_sub_cb, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/diffbot/cmd_vel', 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)

        self._action_server = ActionServer(
            self, Maze, 'maze_action', self.execute_callback
        )
        self.get_logger().info('=== Maze Action Server Started ====')

    def laser_sub_cb(self, data):
        self.forward_distance = data.ranges[360]
        # print(self.forward_distance)

    def odom_sub_cb(self, data):
        orientation = data.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(orientation)
        # print(f'yaw : {self.yaw}')

    def publish_callback(self):
        self.cmd_vel_pub.publish(self.twist_msg)

    def turn_robot(self, euler_angle):
        print(f'Robot Turns to {euler_angle}')

        turn_offset = 100

        while abs(turn_offset) > 0.087:
            turn_offset = 0.7 * (euler_angle - self.yaw)
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = turn_offset
            self.cmd_vel_pub.publish(self.twist_msg)

        self.stop_robot()

    def parking_robot(self):

        while self.forward_distance > 1.0:
            self.twist_msg.linear.x = 0.5
            self.twist_msg.angular.z = 0.0

            self.cmd_vel_pub.publish(self.twist_msg)

        self.stop_robot()

    def stop_robot(self):
        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(self.twist_msg)

        time.sleep(1)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback = Maze.Feedback()
        feedback.feedback_msg = ''

        for _, val in enumerate(goal_handle.request.turning_sequence):
            print(f'Current Cmd: {val}')

            feedback.feedback_msg = f'Turning {direction_str_dict[val]}'

            self.turn_robot(direction_dict[val])
            self.parking_robot()

            goal_handle.publish_feedback(feedback)

        image_sub_node = ImageSubscriber()
        rclpy.spin_once(image_sub_node)
        center_pixel = image_sub_node.center_pixel

        if sum(center_pixel) < 300 and center_pixel[1] > 100:
            goal_handle.succeed()
            self.get_logger().warn('==== Succeed ====')
            result = Maze.Result()
            result.success = True
        else:
            goal_handle.abort()
            self.get_logger().error('==== Fail ====')
            result = Maze.Result()
            result.success = False

        return result


def main(args=None):
    rclpy.init(args=args)

    # Referenced from robotpilot/ros2-seminar-examples
    # https://github.com/robotpilot/ros2-seminar-examples/blob/main/topic_service_action_rclpy_example/topic_service_action_rclpy_example/calculator/main.py
    try:
        maze_action_server = MazeActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(maze_action_server)
        try:
            executor.spin()
        except KeyboardInterrupt:
            maze_action_server.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            maze_action_server._action_server.destroy()
            maze_action_server.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
