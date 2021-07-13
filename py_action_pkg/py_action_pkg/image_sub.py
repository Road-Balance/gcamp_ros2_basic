# !/usr/bin/env/ python3
#
# Original code from link below
# https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/

import cv2

from cv_bridge import (
    CvBridge,
    CvBridgeError,
)  # Package to convert between ROS and OpenCV Images

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image  # Image is the message type


class ImageSubscriber(Node):
    """Create an ImageSubscriber class, which is a subclass of the Node class."""

    def __init__(self):
        """Class constructor to set up the node."""
        # Initiate the Node class's constructor and give it a name
        super().__init__('image_subscriber')
        self.sub_period = 10  # Hz

        # Create the subscriber. This subscriber will receive an Image
        # from the diffbot/camera_sensor/image_raw topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(
            Image,
            'diffbot/camera_sensor/image_raw',
            self.listener_callback,
            self.sub_period,
        )
        self.subscription  # prevent unused variable warning

        # Used to convert between ROS and OpenCV images
        self.cv_bridge = CvBridge()

    def listener_callback(self, data):

        # Convert ROS Image message to OpenCV image
        try:
            current_frame = self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().info(e)

        # Display image
        # cv2.imshow('camera', current_frame)
        # cv2.waitKey(1)

        self.center_pixel = current_frame[400, 400]


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
