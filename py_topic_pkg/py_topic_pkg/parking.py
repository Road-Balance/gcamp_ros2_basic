#!/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ParkingNode(Node):
    def __init__(self):
        super().__init__("laser_sub_node")
        sub_period = 10  # Hz
        self.subscriber = self.create_subscription(
            LaserScan,
            '/skidbot/scan',
            self.sub_callback,
            sub_period
        )
        self.subscriber # prevent unused variable warning

    def sub_callback(self, msg):
        print(f'Distance from Front Object : {msg.ranges[360]}')

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ParkingNode()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()