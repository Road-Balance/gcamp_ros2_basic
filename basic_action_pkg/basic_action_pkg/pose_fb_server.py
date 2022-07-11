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

import math

from custom_interfaces.action import PointMove
from turtlesim.action import RotateAbsolute
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

import rclpy
from rclpy.action import ActionClient
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from queue import Queue

"""custom_interfaces/action/PointMove
float32 point_x
float32 point_y
---
bool succeed
---
float32 remaining_pose
"""

position_q = Queue()
angle_q = Queue()

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

        if position_q.empty() is False:
            position_q.queue.clear()

        position_q.put((self.pose_y, self.pose_x))

    @property
    def turtle_pose(self):
        return (self.pose_x, self.pose_y)

class PointMoveServer(Node):

    def __init__(self):
        super().__init__('point_move_action_server')

        self.turning_action_client = ActionClient(self, RotateAbsolute, 'turtle1/rotate_absolute')

        self.action_server = ActionServer(
            self,
            PointMove,
            'point_move',
            self.execute_callback,
            goal_callback=self.goal_callback,
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.can_move = False

        self.time_start = 0.0
        self.move_msg = Twist()
        self.stop_msg = Twist()

        self.get_logger().info('=== PointMove Action Server Started ====')

    def send_goal(self, theta):
        goal_msg = RotateAbsolute.Goal()
        goal_msg.theta = theta

        if self.turning_action_client.wait_for_server(10) is False:
            self.get_logger().error('Server Not exists')

        self.send_goal_future = self.turning_action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, self.send_goal_future)
        self.goal_handle = self.send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self.result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.result_future)

        self.get_logger().info(f'rotate done ...')
        self.can_move = True

        return True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def calc_dist(self, cur_point, obj_point):

        distance = math.sqrt(
            math.pow(cur_point[0] - obj_point[0], 2) + 
            math.pow(cur_point[1] - obj_point[1], 2)
        )
        return distance

    def calc_angle(self, cur_point, obj_point):

        theta = math.atan2(
            obj_point[1] - cur_point[1], 
            obj_point[0] - cur_point[0] 
        )
        return theta

    async def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = PointMove.Feedback()

        obj_point = (goal_handle.request.point_x, goal_handle.request.point_y)
        cur_point = position_q.get()
        self.get_logger().info(f'...{obj_point}, {cur_point}...')
        
        distance = self.calc_dist(cur_point=cur_point, obj_point=obj_point)
        angle = self.calc_angle(cur_point=cur_point, obj_point=obj_point)
        self.get_logger().info(f'...{distance}, {angle}...')

        self.send_goal(angle)

        self.time_now = self.get_clock().now().to_msg().sec
        remaining = self.time_now - self.time_start

        print(remaining, distance)

        while self.can_move and (remaining < distance):

            cur_point = position_q.get()
            distance = self.calc_dist(cur_point, obj_point)

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return PointMove.Result()

            feedback_msg.remaining_pose = float(distance)

            self.cmd_vel_pub.publish(self.move_msg)
            goal_handle.publish_feedback(feedback_msg)
            remaining = self.get_clock().now().to_msg().sec - self.time_start

        self.cmd_vel_pub.publish(self.stop_msg)

        goal_handle.succeed()
        self.get_logger().warn('==== Succeed ====')

        result = PointMove.Result()
        result.succeed = True
        return result

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        # This server allows multiple goals in parallel
        self.get_logger().info('Received goal request')

        # start to count time
        self.time_start = self.get_clock().now().to_msg().sec

        # prepare moving cmds
        self.move_msg.linear.x = 1.0
        self.move_msg.angular.z = 0.0

        return GoalResponse.ACCEPT

def main(args=None):
    rclpy.init(args=args)

    try:
        executor = MultiThreadedExecutor()
        maze_action_server = PointMoveServer()
        pose_sub_node = PoseSubscriber()

        executor.add_node(maze_action_server)
        executor.add_node(pose_sub_node)
        try:
            executor.spin()
        except KeyboardInterrupt:
            maze_action_server.get_logger().info('Keyboard Interrupt (SIGINT)')
        except Exception as e:
            maze_action_server.get_logger().info(repr(e))
        finally:
            executor.shutdown()
            maze_action_server._action_server.destroy()
            maze_action_server.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
