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

from turtlesim.action import RotateAbsolute

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


"""ros2 interface show turtlesim/action/RotateAbsolute
# The desired heading in radians
float32 theta
---
# The angular displacement in radians to the starting position
float32 delta
---
# The remaining rotation in radians
float32 remaining
"""

class TurtleRotate(Node):

    def __init__(self):
        super().__init__('turtle_rotate_client')
        
        # Create Action Client
        self.action_client = ActionClient(self, RotateAbsolute, 'turtle1/rotate_absolute')
        self.get_logger().info('=== Turtle Rotate Action Client Started ====')

    def send_goal(self, theta):
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = theta
        
        # Wait for server first
        if self.action_client.wait_for_server(10) is False:
            self.get_logger().error('Server Not exists')

        # Send Goal then receive future
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        # Done callback Add
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.remaining}')

    def goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Add Result cb
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().warn(f'Action Done !! Result: {result.delta}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)

    turtle_rotate_client = TurtleRotate()

    turtle_rotate_client.send_goal(0.785398)

    rclpy.spin(turtle_rotate_client)

if __name__ == '__main__':
    main()
