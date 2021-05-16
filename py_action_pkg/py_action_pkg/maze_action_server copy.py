#!/usr/bin/env/ python3

# https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html#id4

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import Maze
from py_action_pkg.robot_controller import RobotController, turn_robot, parking_robot

"""
Maze.action structure

    int32[] turning_sequence
    ---
    bool success
    ---
    string feedback_msg
"""

class MazeActionServer(Node):
    def __init__(self, rclpy_in):
        super().__init__("maze_action_server")

        self._controller = RobotController()
        self._rclpy = rclpy_in

        self._loop_rate = self.create_rate(1, self.get_clock())
        
        self._action_server = ActionServer(
            self, Maze, "maze_action", self.execute_callback
        )
        self.get_logger().info("=== Maze Action Server Started ====")

    def turn_robot(self, euler_angle):
        print(f"Robot Turns to {euler_angle}")

        while self._rclpy.ok():
            self._rclpy.spin_once(self._controller)
            turn_offset = 0.7 * (euler_angle - self._controller.yaw)
            self._controller.turn_robot(turn_offset)

            if abs(turn_offset) < 0.005:
                break

        self._controller.stop_robot() 

    def parking_robot(self):

        print("Going Forward Until 0.8m Obstacle Detection")
        
        while self._rclpy.ok():
            self._loop_rate.sleep()
            self._rclpy.spin_once(self._controller)
            self._controller.move_robot(0.5)
            
            print(f"{self._controller.forward_distance}")
            if self._controller.forward_distance < 0.8:
                break       

        self._controller.stop_robot()

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        feedback = Maze.Feedback()
        feedback.feedback_msg = ""

        # print(goal_handle.request.turning_sequence)

        for i, val in enumerate(goal_handle.request.turning_sequence):
            # feedback.feedback_msg = f"{str(y)}"
            print(f"Feedback: {i}")
            self.parking_robot()
            time.sleep(1)
            # self.turn_robot(3.1415)
            # goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        self.get_logger().warn("==== Succeed ====")

        result = Maze.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = MazeActionServer(rclpy)
    rclpy.spin(fibonacci_action_server)


if __name__ == "__main__":
    main()
