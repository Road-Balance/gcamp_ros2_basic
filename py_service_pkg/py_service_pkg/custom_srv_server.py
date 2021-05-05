from custom_interfaces.srv import AddThreeInts  # CHANGE

import rclpy
from rclpy.node import Node


class AddThreeIntServer(Node):
    def __init__(self):
        super().__init__("custom_srv_server")
        self.srv = self.create_service(
            AddThreeInts, "add_three_ints", self.add_three_ints_callback
        )
        self.get_logger().info("==== Addition Server Started, Waiting for Request ====")

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info(
            f"Incoming requests = a: {request.a}, b: {request.b}, c: {request.c}"
        )
        return response


def main(args=None):
    rclpy.init(args=args)

    add_three_ints_node = AddThreeIntServer()

    rclpy.spin(add_three_ints_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
