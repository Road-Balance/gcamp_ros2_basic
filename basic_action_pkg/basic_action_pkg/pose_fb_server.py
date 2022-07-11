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
from turtlesim.msg import Pose

import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from queue import Queue

"""
float32 move_time
---
float32 remaining
---
bool succeed
"""

pose_q = Queue()

class PoseSubscriber(Node):

    def __init__(self):

        super().__init__('turtle_pose_sub_node')

        self.pose_subscriber = self.create_subscription(
            Pose, 
            'turtle1/pose',
            self.sub_callback, 
            10
        )

        self.pose_x = 0.0
        self.pose_y = 0.0

    def sub_callback(self, msg):
        """Timer will run this function periodically."""
        self.pose_x = msg.x
        self.pose_y = msg.y

        print(self.pose_y, self.pose_x)

        pose_q.put((self.pose_y, self.pose_x))

    @property
    def turtle_pose(self):
        return (self.pose_x, self.pose_y)


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

        if pose_q.empty() is False:
            pose_x, pose_y = pose_q.get()

        while remaining < goal_handle.request.move_time:

            print("=========", pose_q.get())
            if pose_q.empty() is False:
                pose_q.queue.clear()

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

    # fibonacci_action_server = TimedMoveServer()
    # rclpy.spin(fibonacci_action_server)

    # fibonacci_action_server.destroy()
    # rclpy.shutdown()

    # tp_sub_node = PoseSubscriber()

    # rclpy.spin(tp_sub_node)

    try:
        executor = MultiThreadedExecutor()
        maze_action_server = TimedMoveServer()
        pose_sub_node = PoseSubscriber()
        executor.add_node(maze_action_server)
        executor.add_node(pose_sub_node)
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
