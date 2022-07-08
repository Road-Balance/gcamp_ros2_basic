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


"""
This is an second example code for ROS 2 rclpy node programming.

Let's learn about those things.

How to run ROS 2 node periodically(=rclpy.spin).
How to create ROS 2 timer.
Problems could occur during Procedural programming.
"""
import rclpy
from rclpy.node import Node


count = 0


def timer_callback():
    """Timer will run this function periodically."""
    global count
    count += 1
    print(f'==== Hello ROS 2 : {count}====')


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    node = Node('node_name')
    node.create_timer(0.2, timer_callback)
    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
