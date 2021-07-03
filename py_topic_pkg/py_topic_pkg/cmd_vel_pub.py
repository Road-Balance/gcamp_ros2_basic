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

# Referenced from Below Link
# https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html#rclpy.create_node

# !/usr/bin/env/ python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CmdVelPublisher(Node):

    def __init__(self):
        super().__init__('cmd_vel_pub_node')
        self.publisher = self.create_publisher(
            Twist, 'skidbot/cmd_vel', 10
        )  # queue size
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)
        self.get_logger().info(
            'DriveForward node Started, move forward during 5 seconds \n'
        )

    def publish_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        twist_msg.angular.z = 1.0
        self.publisher.publish(twist_msg)

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = CmdVelPublisher()
    start_time = cmd_vel_publisher.get_clock().now().to_msg().sec
    clock_now = start_time
    time_delta = 0

    while (clock_now - start_time) < 5:
        rclpy.spin_once(cmd_vel_publisher)
        clock_now = cmd_vel_publisher.get_clock().now().to_msg().sec

        time_delta = clock_now - start_time
        cmd_vel_publisher.get_logger().info(f'{time_delta} seconds passed')

    cmd_vel_publisher.stop_robot()

    cmd_vel_publisher.get_logger().info('\n==== Stop Publishing ====')
    cmd_vel_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
