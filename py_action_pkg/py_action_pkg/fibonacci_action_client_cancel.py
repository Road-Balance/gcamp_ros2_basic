# !/usr/bin/env/ python3
#
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Referenced Documents
# https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html#id4

from custom_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self.action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.goal_handle = None
        self.get_logger().info('=== Fibonacci Action Client Started ====')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        if self.action_client.wait_for_server(10) is False:
            self.get_logger().error('Server Not exists')

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.partial_sequence}')

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Start a 2 second timer
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self.goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self.timer.cancel()

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_client = FibonacciActionClient()

    fibonacci_action_client.send_goal(5)

    # future for Future Work
    # future = fibonacci_action_client.send_goal(5)

    rclpy.spin(fibonacci_action_client)


if __name__ == '__main__':
    main()
