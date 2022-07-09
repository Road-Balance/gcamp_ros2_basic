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
This is first example code for ROS 2 topic publisher.

Let's learn about those things.

Create topic publisher then check the value from that with ros2 command line tools.
Try control turtle in the turtlesim through turtlesim node.
"""

import random

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class TwistPubNode(Node):
    """Twist msg Publisher Node.

    This node will execute periodic topic publish.
    Put random control inputs info geometry_msgs/msg/Twist type variable
    Then publish to Exact topic.
    """

    def __init__(self):
        """Node Initialization.

        You must type name of the node in inheritanced initializer.
        """
        super().__init__('twist_pub_node')

        self.get_logger().info(
            f'TwistPubNode Created at {self.get_clock().now().to_msg().sec}'
        )
        # As its name create_publisher will return specific publisher for its params.
        #
        # self.create_publisher(
        #   msg_type::, topic_name::, queue_size::
        # )
        # self.twist_publisher = self.create_publisher(Twist, "twist_topic", 10)
        self.twist_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        """Timer will run this function periodically."""
        msg = Twist()
        # Fill in msg with compatible values
        msg.linear.x = random.random()
        msg.angular.z = random.uniform(-1.0, 1.0)
        self.get_logger().info(
            f'Linear X velocity : {msg.linear.x} / Angular Z velocity : {msg.angular.z}'
        )

        # publish into "/turtle1/cmd_vel" topic
        self.twist_publisher.publish(msg)


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    twist_pub_node = TwistPubNode()
    rclpy.spin(twist_pub_node)
    twist_pub_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
