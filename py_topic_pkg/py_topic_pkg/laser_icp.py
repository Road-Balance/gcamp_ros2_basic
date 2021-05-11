#!/usr/bin/env/ python3

from numpy.core.multiarray import empty_like

import math
import rclpy
import numpy as np
from rclpy.node import Node
from py_topic_pkg.ICP import icp
from sensor_msgs.msg import LaserScan

class LaserSubscriber(Node):
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
        self.psi = 0.0
        self.prev_laser = None

    def sub_callback(self, msg):
        laser = np.frombuffer(msg.ranges)
        laser = np.expand_dims(laser, axis=0)

        laser_pad = np.pad(laser, ((0,1),(0,0)), 'constant', constant_values=0)

        if self.prev_laser is None:
            self.prev_laser = laser_pad
        else:
            # pass
            loop_transform, _, _ = icp(self.prev_laser, laser_pad, max_iterations=20)
            # R = self.best_fit_transform(self.prev_laser, laser_pad)
            # self.psi += math.atan2(R[0,0],R[1,0])
            print(loop_transform)

def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = LaserSubscriber()

    rclpy.spin(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()