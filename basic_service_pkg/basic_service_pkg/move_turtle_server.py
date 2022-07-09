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

from geometry_msgs.msg import Twist
from example_interfaces.srv import SetBool
import rclpy
from rclpy.node import Node


# example_interfaces/srv/SetBool srv Description.
#
# bool data # e.g. for hardware enabling / disabling
# ---
# bool success   # indicate successful run of triggered service
# string message # informational, e.g. for error messages


class TurtleCircleNode(Node):

    def __init__(self):
        super().__init__('turtle_circle_server')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.srv = self.create_service(
            SetBool, 'turtle_circle', self.turtle_circle_callback
        )

        self.twist_msg = Twist()
        self.get_logger().info('==== Robot Turning Server Started, Waiting for Request ====')

    def turtle_circle(self):
        self.twist_msg.linear.x = 2.0
        self.twist_msg.angular.z = 1.0

        time_start = self.get_clock().now().to_msg().sec
        time_now = self.get_clock().now().to_msg().sec

        moving_time = 7

        while (time_now - time_start) < moving_time:
            self.publisher.publish(self.twist_msg)
            time_now = self.get_clock().now().to_msg().sec

        self.twist_msg.linear.x = 0.0
        self.twist_msg.angular.z = 0.0
        self.publisher.publish(self.twist_msg)

        self.get_logger().info('Turtle Stop')

    def turtle_circle_callback(self, request, response):
        self.start_time = self.get_clock().now().to_msg().sec

        if request.data is True:
            # Move Turtle
            self.turtle_circle()

        response.success = True
        response.message = "Turtle successfully drawed Circle"

        return response


def main(args=None):
    rclpy.init(args=args)

    turtle_circle_server = TurtleCircleNode()

    rclpy.spin(turtle_circle_server)

    turtle_circle_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
