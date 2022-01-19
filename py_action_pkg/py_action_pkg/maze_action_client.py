#!/usr/bin/env/ python3
#
# Copyright 2021 Seoul Business Agency Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from custom_interfaces.action import Maze
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

"""
Maze.action structure

int32[] turning_sequence
---
bool success
---
string feedback_msg
"""


class MazeActionClient(Node):

    def __init__(self):
        super().__init__('maze_action_client')

        self.declare_parameter('robot_namespace', 'diffbot')
        self.robot_namespace = self.get_parameter('robot_namespace').value + '/'
        
        self.action_client = ActionClient(self, Maze, self.robot_namespace + 'maze_action')
        self.get_logger().info('=== Maze Action Client Started ====')

    def send_goal(self, turning_list):
        goal_msg = Maze.Goal()
        goal_msg.turning_sequence = turning_list

        if self.action_client.wait_for_server(10) is False:
            self.get_logger().error('Server Not exists')

        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info(f'Received feedback: {feedback.feedback_msg}')

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().warn(f'Action Done !! Result: {result.success}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    maze_action_client = MazeActionClient()
    user_inputs = []

    # Input Logic
    try:
        maze_action_client.get_logger().info('Enter numbers [or stop] : ')

        while True:
            user_inputs.append(int(input()))
    # if the input is not-integer, just print the list
    except Exception:
        maze_action_client.get_logger().info(f'Your sequence list : {user_inputs}')
    maze_action_client.get_logger().info('==== Sending Goal ====')
    maze_action_client.send_goal(user_inputs)

    # You can get Future for additional functoins
    # future = maze_action_client.send_goal(user_inputs)

    rclpy.spin(maze_action_client)


if __name__ == '__main__':
    main()
