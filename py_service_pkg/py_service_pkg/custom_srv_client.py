# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# !/usr/bin/env/ python3

from custom_interfaces.srv import AddThreeInts
import rclpy
from rclpy.node import Node


class AddThreeIntClient(Node):

    def __init__(self):
        super().__init__('custom_srv_client')
        self.client = self.create_client(AddThreeInts, 'add_three_ints')  # CHANGE
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = AddThreeInts.Request()
        self.get_logger().info('==== Welcome to Three Int Addition Service ====')

    def send_request(self):
        self.req.a = int(input('> Type First  Number : '))
        self.req.b = int(input('> Type Second Number : '))
        self.req.c = int(input('> Type Third  Number : '))
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
        except Exception:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception()
            )
        else:
            add_three_client.get_logger().info('==== Service Call Done ====')
            add_three_client.get_logger().info(f'Status_message : {add_three_client.req.a} + \
                 {add_three_client.req.b} + {add_three_client.req.c} = {response.sum}')
        finally:
            add_three_client.get_logger().warn('==== Shutting down node. ====')
            add_three_client.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
