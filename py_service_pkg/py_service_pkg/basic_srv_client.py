#!/usr/bin/env/ python3

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AdditionClient(Node):
    def __init__(self):
        super().__init__("addition_client")
        self.client = self.create_client(AddTwoInts, "add_two_ints")

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.req = AddTwoInts.Request()
        self.get_logger().info("==== Welcome to Two Int Addition Service ====")

    def send_request(self):
        self.req.a = int(input("> Type First  Number : "))
        self.req.b = int(input("> Type Second Number : "))
        self.future = self.client.call_async(self.req)
        return self.future


def main(args=None):
    rclpy.init(args=args)

    addition_client = AdditionClient()
    future = addition_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(addition_client)

        if future.done():
            try:
                response = future.result()
            except Exception as e:
                addition_client.get_logger().info("Service call failed %r" % (e,))
            else:
                addition_client.get_logger().info(
                    "Result of add_two_ints: for %d + %d = %d"
                    % (addition_client.req.a, addition_client.req.b, response.sum)
                )
            break

    addition_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
