#!/usr/bin/env/ python3

# https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html#id4

import math

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry 

from custom_interfaces.action import Maze
from py_action_pkg.robot_controller import euler_from_quaternion
from py_action_pkg.image_sub import ImageSubscriber

from rclpy.executors import MultiThreadedExecutor

direction_dict = {0: (-1 * math.pi / 2), 1: math.pi, 2: math.pi / 2, 3: 0.0}
direction_str_dict = {0: "Up", 1: "Right", 2: "Down", 3: "Left"}

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
        super().__init__("maze_action_server")

        self.yaw = 0.0
        self.forward_distance = 0.0
        self.left_distance = 0.0
        self.right_distance = 0.0

        self.parking_distance = 0.8
        
        self.twist_msg = Twist()

        self.stop_msg = Twist()
        self.stop_msg.linear.x = 0.0
        self.stop_msg.angular.z = 0.0

        self.sub_period = 10  # Hz
        self.pub_period = 5  # Hz

        self.loop_rate = self.create_rate(5, self.get_clock())
        
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/tinybot/scan',
            self.laser_sub_cb,
            self.sub_period
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/tinybot/odom",
            self.odom_sub_cb,
            self.sub_period,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/tinybot/cmd_vel",
            self.pub_period
        )

        self._action_server = ActionServer(
            self, Maze, "maze_action", self.execute_callback
        )
        self.get_logger().info("=== Maze Action Server Started ====")
    
    def laser_sub_cb(self, data):
        self.right_distance = data.ranges[1]
        self.forward_distance = data.ranges[360]
        self.left_distance = data.ranges[719]
        # print(f"Distance : {self.forward_distance}")

    def odom_sub_cb(self, data):
        orientation = data.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion(orientation)
        # print(f"yaw : {self.yaw}")

    def parking_robot(self):
        print(f"Going Forward Until {self.parking_distance}m Obstacle Detection")

        while self.forward_distance > self.parking_distance:
            self.twist_msg.linear.x = 0.5
            self.twist_msg.angular.z = 0.0
            
            if self.left_distance < 0.8:
                self.twist_msg.angular.z = 0.7 * (self.left_distance - 0.8)

            self.cmd_vel_pub.publish(self.twist_msg)
            self.loop_rate.sleep()

    def turn_robot(self, euler_angle):
        print(f"Robot Turns to {euler_angle}")

        turn_offset = 100

        while abs(turn_offset) > 0.087:
            turn_offset = 0.7 * (euler_angle - self.yaw)
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = turn_offset
            self.cmd_vel_pub.publish(self.twist_msg) 
            self.loop_rate.sleep()

    def stop_robot(self):
        clock_now  = self.get_clock().now().to_msg().sec
        start_time = self.get_clock().now().to_msg().sec

        while (clock_now - start_time) < 2:
            self.cmd_vel_pub.publish(self.stop_msg) 
            self.loop_rate.sleep()
            clock_now = self.get_clock().now().to_msg().sec

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        feedback = Maze.Feedback()
        feedback.feedback_msg = ""

        for i, val in enumerate(goal_handle.request.turning_sequence):
            print(f"Current Cmd: {val}")

            feedback.feedback_msg = f"Turning {direction_str_dict[val]}"

            self.turn_robot(direction_dict[val])
            self.stop_robot()

            self.parking_robot()
            self.stop_robot()

            self.loop_rate.sleep()
            goal_handle.publish_feedback(feedback)

        image_sub_node = ImageSubscriber()
        rclpy.spin_once(image_sub_node)
        center_pixel = image_sub_node.center_pixel

        if sum(center_pixel) < 300 and center_pixel[1] > 100:
            goal_handle.succeed()
            self.get_logger().warn("==== Succeed ====")
            result = Maze.Result()
            result.success = True
        else:
            goal_handle.abort()
            self.get_logger().warn("==== Succeed ====")
            result = Maze.Result()
            result.success = False

        return result

def main(args=None):
    rclpy.init(args=args)

    # Referenced from robotpilot/ros2-seminar-examples
    # https://github.com/robotpilot/ros2-seminar-examples/blob/main/topic_service_action_rclpy_example/topic_service_action_rclpy_example/calculator/main.py
    try:
        fibonacci_action_server = MazeActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(fibonacci_action_server)
        try:
            executor.spin()
        except KeyboardInterrupt:
            fibonacci_action_server.get_logger().info('Keyboard Interrupt (SIGINT)')
        finally:
            executor.shutdown()
            fibonacci_action_server._action_server.destroy()
            fibonacci_action_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
