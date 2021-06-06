# !/usr/bin/env python3

import rclpy
from rclpy.node import Node


class HelloROS(Node):

    def __init__(self):
        super().__init__('hello_ros_node')
        self.get_logger().info('Hello ROS!! :D')


def main(args=None):
    rclpy.init(args=args)

    hello_ros_node = HelloROS()
    hello_ros_node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
