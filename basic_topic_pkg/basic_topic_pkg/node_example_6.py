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

Implement Example 3 with ROS 2 Node Composition.
"""
import rclpy
from rclpy.node import Node


class NodeClass(Node):
    """Third Node Class.

    Print log periodically with Logger Level.
    """

    def __init__(self):
        """Node Initialization.

        You must type name of the node in inheritanced initializer.
        """
        super().__init__('node_name')
        self.create_timer(0.2, self.timer_callback)

        self.count = 1

    def timer_callback(self):
        """Timer will run this function periodically."""
        # There're 5 logger-level in ROS 2 get_logger() System.
        # Try out and watch whats difference.
        self.get_logger().debug(f'==== Hello ROS 2 : {self.count}====')
        self.get_logger().info(f'==== Hello ROS 2 : {self.count}====')
        self.get_logger().warn(f'==== Hello ROS 2 : {self.count}====')
        self.get_logger().error(f'==== Hello ROS 2 : {self.count}====')
        self.get_logger().fatal(f'==== Hello ROS 2 : {self.count}====')

        self.count += 1


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    node = NodeClass()
    rclpy.spin(node)
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
