# https://docs.ros2.org/foxy/api/rclpy/api/init_shutdown.html#rclpy.create_node

#!/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class CmdVelPublisher(Node):
    def __init__(self):
        super().__init__("cmd_vel_pub_node")
        self.publisher = self.create_publisher(
            Twist, "/skidbot/cmd_vel", 10
        )  # queue size
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)
        self.get_logger().info(
            " DriveForward node Started, move forward during 5 seconds \n"
        )

    def publish_callback(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.5
        twist_msg.angular.z = 1.0
        self.publisher.publish(twist_msg)

    def stop_robot(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)

    cmd_vel_publisher = CmdVelPublisher()
    start_time = cmd_vel_publisher.get_clock().now().to_msg().sec
    clock_now = start_time
    time_delta = 0
    
    while (clock_now - start_time) < 5:
        rclpy.spin_once(cmd_vel_publisher)
        clock_now = cmd_vel_publisher.get_clock().now().to_msg().sec

        time_delta = clock_now - start_time
        print(f"{time_delta} seconds passed")

    cmd_vel_publisher.stop_robot()

    cmd_vel_publisher.get_logger().info("\n==== Stop Publishing ====")
    cmd_vel_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
