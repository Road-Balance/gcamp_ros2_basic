#!/usr/bin/env/ python3

import sys
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ParkingNode(Node):
    def __init__(self):
        super().__init__("parking_node")
        pub_period = 10  # Hz
        sub_period = 10  # Hz
        self.publisher = self.create_publisher(
            Twist,
            "/skidbot/cmd_vel",
            pub_period
        )
        self.subscriber = self.create_subscription(
            LaserScan,
            '/skidbot/scan',
            self.sub_callback,
            sub_period
        )
        self.subscriber # prevent unused variable warning
        self.publisher  # prevent unused variable warning
        self.get_logger().info("==== Parking Node Started ====\n")

    def sub_callback(self, msg):
        twist_msg = Twist()
        distance_forward = msg.ranges[360]

        if distance_forward > 0.5:
            print(f'Distance from Front Object : {distance_forward}')
            twist_msg.linear.x  = 0.5
            self.publisher.publish(twist_msg)
        else:
            self.get_logger().info("==== Parking Done!!! ====\n")
            twist_msg.linear.x  = 0.0
            self.publisher.publish(twist_msg)
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)

    parking_node = ParkingNode()

    try:
        rclpy.spin(parking_node)
    except KeyboardInterrupt:
        print('==== Server stopped cleanly ====')
    except BaseException:
        print('!! Exception in server:', file=sys.stderr)
        raise
    finally:
        # (optional - Done automatically when node is garbage collected)
        rclpy.shutdown() 

if __name__ == '__main__':
    main()