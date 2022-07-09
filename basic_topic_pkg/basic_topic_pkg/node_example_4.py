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

ROS 2 Node Composition (Node through Class)
"""

import rclpy
from rclpy.node import Node


class NodeClass(Node):
    """Our First Node Class.

    Actually, This Class is just "dummy".
    It doesn't do anything after initialized.
    """

    def __init__(self):
        """Node Initialization.

        You must type name of the node in inheritanced initializer.
        """
        super().__init__('node_name')


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    node = NodeClass()
    # node2 = NodeClass()
    node.get_logger().info('\n==== Hello ROS 2 ====')
    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
