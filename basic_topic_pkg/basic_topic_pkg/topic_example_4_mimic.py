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

How can place publisher & subscriber in the same Node.
Make turtle2 following turtle1.
"""
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class MimicNode(Node):
    """turtlesim msg Mimic Node.

    This node will listen cmd_vel topic from turtle1.
    Then just publish same cmd_vel onto turtle2.
    """

    def __init__(self):
        """Node Initialization.

        You must type name of the node in inheritanced initializer.
        """
        super().__init__('mimic_node')
        queue_size = 10  # Queue Size
        # Create publisher & subscriber at the same time.
        # Look carefully at below two lines.
        self.twist_publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', queue_size)
        self.twist_subscriber = self.create_subscription(
            Twist, 'turtle1/cmd_vel', self.sub_callback, queue_size
        )

    def sub_callback(self, msg):
        """Do sth feedback for msg input.

        This function will be Runned whenever turtle1/pose topic publishes sth.
        """
        pub_msg = Twist()
        # Put subscribed msg into publish msg as it is.
        pub_msg.linear.x = msg.linear.x
        pub_msg.angular.z = msg.angular.z

        self.twist_publisher.publish(pub_msg)


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    mimic_node = MimicNode()

    rclpy.spin(mimic_node)

    mimic_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
