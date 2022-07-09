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
This is an third example code for ROS 2 rclpy node programming.

Let's learn about those things.

How can spin node only once?
Print ROS 2 log console in public function.
"""

import rclpy
from rclpy.node import Node


count = 0


def timer_callback():
    """Timer will run this function periodically."""
    global count
    count += 1
    print(f'==== Hello ROS 2 : {count}====')
    # How can I use logger without globalization ?
    # node.get_logger().info('\n==== Hello ROS 2 ====')


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    node = Node('node_name')
    node.create_timer(0.2, timer_callback)

    while True:
        rclpy.spin_once(node, timeout_sec=10)

    # rclpy.spin_once(node, timeout_sec=10)


    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
