# !/usr/bin/env/ python3
#
# Copyright 2021 @RoadBalance
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

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

from custom_interfaces.srv import TurtleJail


class TurtleJailNode(Node):
    """Make jail in turtlesim.

    If turtle escape from virtual jail
    It'll be spawned again in the center of turtlesim
    """

    def __init__(self):
        """Node Initialization.

        You must type name of the node in inheritanced initializer.
        """
        super().__init__('turtlepose_sub_node')

        # Create Turtle teleport client
        self.client = self.create_client(TeleportAbsolute, 'turtle1/teleport_absolute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, Waiting again...')

        self.request_srv = TeleportAbsolute.Request()
        self.get_logger().info('=== [Service Client : Ready to Call Service Request] ===')
        
        # Create Subscriber for turtle1/pose
        queue_size = 10  # Queue Size
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.sub_callback, queue_size
        )

        # Create Service Server for User Interfaces
        self.srv = self.create_service(
            TurtleJail, 'turtle_jail_size', self.turtle_jail_callback
        )
        self.get_logger().info('==== [Service Server : Ready to receive Service Request] ====')

        # Preserve its rotation before teleport
        self.cur_theta = 0.0

        # jail size in rectangular form
        self.jail_width = 6.0
        self.jail_height = 6.0

    def send_request(self):
        """Service Clinet request fuction"""
        self.request_srv.x = 6.0
        self.request_srv.y = 6.0
        self.request_srv.theta = self.cur_theta

        self.future = self.client.call_async(self.request_srv)

        return self.future

    def sub_callback(self, msg):
        """Turtle Pose Subscriber Callback"""

        if abs(msg.x - 6.0) > self.jail_width or abs(msg.y - 6.0) > self.jail_height:
            self.cur_theta = msg.theta
            self.get_logger().warn("You can't go out Turtle! :(")
            self.send_request()

    def turtle_jail_callback(self, request, response):
        """Service Server for jail resizing client request"""
        self.jail_width = request.width
        self.jail_height = request.height

        self.get_logger().info(f"""Jail Size Update to {self.jail_width}/{self.jail_height}""")

        response.success = True

        return response

def main(args=None):
    rclpy.init(args=args)

    turtle_jail_node = TurtleJailNode()

    rclpy.spin(turtle_jail_node)

    turtle_jail_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
