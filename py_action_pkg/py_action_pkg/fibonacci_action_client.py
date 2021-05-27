#!/usr/bin/env/ python3

# https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html#id4

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from custom_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__("fibonacci_action_client")
        self.action_client = ActionClient(self, Fibonacci, "fibonacci")
        self.get_logger().info("=== Fibonacci Action Client Started ====")

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        if self.action_client.wait_for_server(10) is False:
            self.get_logger().error("Server Not exists")

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"Received feedback: {feedback.partial_sequence}")

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().warn(f"Action Done !! Result: {result.sequence}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_client = FibonacciActionClient()

    future = fibonacci_action_client.send_goal(5)

    rclpy.spin(fibonacci_action_client)


if __name__ == "__main__":
    main()
