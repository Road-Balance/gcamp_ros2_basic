#!/usr/bin/env/ python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AdditionServer(Node):
    def __init__(self):
        super().__init__("addition_server")
        self.srv = self.create_service(
            AddTwoInts, "add_two_ints", self.add_two_ints_callback
        )
        self.get_logger().info("==== Basic Server Started ====")

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f"==== Incoming request ==== \n Fisrt Operand : {request.a}, Second Operand: {request.b}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)

    addition_server = AdditionServer()

    rclpy.spin(addition_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
