#!/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class LaserSubscriber(Node):
    def __init__(self):
        super().__init__("laser_sub_node")
        queue_size = 10  # Hz
        self.subscriber = self.create_subscription(
            LaserScan, "/skidbot/scan", self.sub_callback, queue_size
        )
        self.subscriber  # prevent unused variable warning

    def sub_callback(self, msg):
        print(f"Distance from Front Object : {msg.ranges}")


def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = LaserSubscriber()

    rclpy.spin(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
