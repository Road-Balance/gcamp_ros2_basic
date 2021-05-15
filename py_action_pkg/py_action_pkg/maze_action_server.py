#!/usr/bin/env/ python3

# https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html#id4

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interfaces.action import Maze

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
        self._action_server = ActionServer(
            self, Maze, "maze_action", self.execute_callback
        )
        self.get_logger().info("=== Maze Action Server Started ====")

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing goal...")

        feedback = Maze.Feedback()
        feedback.feedback_msg = ""

        # print(goal_handle.request.turning_sequence)

        for i, val in enumerate(goal_handle.request.turning_sequence):
            # feedback.feedback_msg = f"{str(y)}"
            print(f"Feedback: {i}")
            # goal_handle.publish_feedback(feedback)
            time.sleep(1)

        goal_handle.succeed()
        self.get_logger().warn("==== Succeed ====")

        result = Maze.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = MazeActionServer()
    rclpy.spin(fibonacci_action_server)


if __name__ == "__main__":
    main()
