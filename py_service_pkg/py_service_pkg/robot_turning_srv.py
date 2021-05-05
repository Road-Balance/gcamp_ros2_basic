#!/usr/bin/env/ python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from custom_interfaces.srv import TurningControl  # CHANGE

# uint32 time_duration
# float64 angular_vel_z
# float64 linear_vel_x
# ---
# bool success

class RobotTurnServer(Node):
    def __init__(self):
        super().__init__("robot_turn_server")
        self.publisher = self.create_publisher(Twist, "/skidbot/cmd_vel", 10)
        self.srv = self.create_service(
            TurningControl, "turn_robot", self.robot_turn_callback
        )
        self.twist_msg = Twist()
        self.start_time = self.get_clock().now().to_msg().sec

        print("==== Robot Turning Server Started, Waiting for Request ====")

    def move_robot(self, seconds=1, linear_x=0.0, angular_z=0.0):
        self.twist_msg.linear.x  = linear_x
        self.twist_msg.angular.z = angular_z

        clock_now = self.get_clock().now().to_msg().sec
        self.get_logger().info("Robot Moves")

        while (clock_now - self.start_time) < seconds:
            clock_now = self.get_clock().now().to_msg().sec
            self.publisher.publish(self.twist_msg)

    def stop_robot(self):
        self.twist_msg.linear.x  = 0.0
        self.twist_msg.angular.z = 0.0

        self.publisher.publish(self.twist_msg)
        self.get_logger().info("Robot Stop")

    def robot_turn_callback(self, request, response):
        self.start_time = self.get_clock().now().to_msg().sec

        self.move_robot(request.time_duration, request.linear_vel_x, request.angular_vel_z)
        self.stop_robot()

        response.success = True
        self.get_logger().info("Servie Process Done...")

        return response

def main(args=None):
    rclpy.init(args=args)

    robot_turn_server = RobotTurnServer()

    rclpy.spin(robot_turn_server)

    robot_turn_server.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
