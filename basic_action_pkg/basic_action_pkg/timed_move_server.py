#!/usr/bin/env/ python3

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

# https://docs.ros.org/en/foxy/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html#id4

import time

from custom_interfaces.action import TimedMove
from geometry_msgs.msg import Twist

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.node import Node

"""
float32 move_time
---
float32 remaining
---
bool succeed
"""

class TimedMoveServer(Node):

    def __init__(self):
        super().__init__('timed_move_action_server')
        self.action_server = ActionServer(
            self,
            TimedMove,
            'timed_move',
            self.execute_callback,
            goal_callback=self.goal_callback,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.time_start = 0.0
        self.move_msg = Twist()
        self.stop_msg = Twist()

        self.get_logger().info('=== TimedMove Action Server Started ====')

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = TimedMove.Feedback()

        self.time_now = self.get_clock().now().to_msg().sec

        remaining = self.time_now - self.time_start

        while remaining < goal_handle.request.move_time:

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return TimedMove.Result()

            feedback_msg.remaining = float(remaining)

            self.cmd_vel_pub.publish(self.move_msg)
            goal_handle.publish_feedback(feedback_msg)
            remaining = self.get_clock().now().to_msg().sec - self.time_start

        self.cmd_vel_pub.publish(self.stop_msg)

        goal_handle.succeed()
        self.get_logger().warn('==== Succeed ====')

        result = TimedMove.Result()
        result.succeed = True
        return result

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')

        # start to count time
        self.time_start = self.get_clock().now().to_msg().sec
        
        # prepare moving cmds
        self.move_msg.linear.x = 2.0
        self.move_msg.angular.z = 0.0

        return GoalResponse.ACCEPT

def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = TimedMoveServer()
    rclpy.spin(fibonacci_action_server)

    fibonacci_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
