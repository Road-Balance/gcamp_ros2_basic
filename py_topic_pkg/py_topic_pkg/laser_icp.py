#!/usr/bin/env/ python3

from numpy.core.multiarray import empty_like

import math
import rclpy
import numpy as np
from rclpy.node import Node

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
            R = self.best_fit_transform(self.prev_laser, laser_pad)
            self.psi += math.atan2(R[0,0],R[1,0])
            print(self.psi)
    
    # # https://github.com/ClayFlannigan/icp/blob/master/icp.py
    def best_fit_transform(self, A, B):
        '''
        Calculates the least-squares best-fit transform that maps corresponding points A to B in m spatial dimensions
        Input:
        A: Nxm numpy array of corresponding points
        B: Nxm numpy array of corresponding points
        Returns:
        T: (m+1)x(m+1) homogeneous transformation matrix that maps A on to B
        R: mxm rotation matrix
        t: mx1 translation vector
        '''

        assert A.shape == B.shape

        # get number of dimensions

        # translate points to their centroids
        centroid_A = np.mean(A, axis=0)
        centroid_B = np.mean(B, axis=0)
        AA = A - centroid_A
        BB = B - centroid_B

        # rotation matrix
        H = np.dot(AA.T, BB)
        # print(f'H : {H}')

        m = H.shape[1]

        U, S, Vt = np.linalg.svd(H)
        R = np.dot(Vt.T, U.T)
        # print(Vt)

        # special reflection case
        if np.linalg.det(R) < 0:
            Vt[m-1,:] *= -1
            R = np.dot(Vt.T, U.T)

        # # translation
        # t = centroid_B.T - np.dot(R,centroid_A.T)

        # # homogeneous transformation
        # T = np.identity(m+1)
        # T[:m, :m] = R
        # T[:m, m] = t

        return R

def main(args=None):
    rclpy.init(args=args)

    laser_subscriber = LaserSubscriber()

    rclpy.spin(laser_subscriber)

    laser_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()