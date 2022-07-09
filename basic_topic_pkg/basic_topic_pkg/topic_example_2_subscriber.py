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
This is first example code for ROS 2 topic subscriber.

Let's learn about those things.

Create topic subscriber then check the value from that with ros2 command line tools.
Listen to pose of turtle in the turtlesim.
"""

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose


class TurtlePoseSubNode(Node):
    """turtlesim/Pose msg Subscriber Node.

    This node will listen pose topic from turtlesim.
    Then just print them on terminal.
    """

    def __init__(self):
        """Node Initialization.

        You must type name of the node in inheritanced initializer.
        """
        super().__init__('turtlepose_sub_node')

        queue_size = 10  # Queue Size
        # You can create subscriber with create_subscription function
        # this function get those params
        #
        # msg type, topic name, callback function, queue_size
        #
        # topic name must exists and coincident with exact topic name
        self.pose_subscriber = self.create_subscription(
            Pose, 'turtle1/pose', self.sub_callback, queue_size
        )

    def sub_callback(self, msg):
        """Timer will run this function periodically."""
        self.get_logger().info(f"""x : {msg.x:.3f} / y : {msg.y:.3f} / z : {msg.theta:.3f}
        linear_velocity : {msg.linear_velocity} / angular_velocity : {msg.angular_velocity }""")


def main(args=None):
    """Do enter into this main function first."""
    rclpy.init(args=args)

    laser_subscriber = TurtlePoseSubNode()

    rclpy.spin(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    """main function"""
    main()
