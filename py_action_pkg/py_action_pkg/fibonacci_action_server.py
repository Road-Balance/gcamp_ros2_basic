#!/usr/bin/env/ python3

# https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html#id4

import time

from custom_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback,
            goal_callback=self.goal_callback,
        )

        self.get_logger().info('=== Fibonacci Action Server Started ====')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i - 1]
            )

            print(f'Feedback: {feedback_msg.partial_sequence}')
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        self.get_logger().warn('==== Succeed ====')

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()
    rclpy.spin(fibonacci_action_server)

    fibonacci_action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
