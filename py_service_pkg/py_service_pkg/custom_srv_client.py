#!/usr/bin/env/ python3

import sys
import rclpy
from rclpy.node import Node

from custom_interfaces.srv import AddThreeInts 
class AddThreeIntClient(Node):
    def __init__(self):
        super().__init__("custom_srv_client")
        self.client = self.create_client(AddThreeInts, "add_three_ints")  # CHANGE
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        
        self.req = AddThreeInts.Request()
        self.get_logger().info("==== Welcome to Three Int Addition Service ====")


    def send_request(self):
        self.req.a = int(input("> Type First  Number : "))
        self.req.b = int(input("> Type Second Number : "))
        self.req.c = int(input("> Type Third  Number : "))
        self.future = self.client.call_async(self.req)

        return self.future

def main(args=None):
    rclpy.init(args=args)

    add_three_client = AddThreeIntClient()
    future = add_three_client.send_request()

    rclpy.spin_until_future_complete(add_three_client, future)

    if future.done():
        try:
            response = future.result()
        except Exception as e:
            raise RuntimeError(
                "exception while calling service: %r" % future.exception()
            )
        else:
            print("==== Service Call Done ====")
            print(f"Status_message : {add_three_client.req.a} + {add_three_client.req.b} + {add_three_client.req.c} = {response.sum}")
        finally:
            add_three_client.get_logger().warn("==== Shutting down node. ====")
            add_three_client.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
