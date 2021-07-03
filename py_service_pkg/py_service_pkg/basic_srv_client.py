# !/usr/bin/env/ python3
#
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

from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class AdditionClient(Node):

    def __init__(self):
        super().__init__('addition_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = AddTwoInts.Request()
        self.get_logger().info('==== Welcome to Two Int Addition Service ====')

    def send_request(self):
        self.req.a = int(input('> Type First  Number : '))
        self.req.b = int(input('> Type Second Number : '))
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
                addition_client.get_logger().info('Service call failed %r' % (e,))
            else:
                addition_client.get_logger().info(
                    'Result of add_two_ints: for %d + %d = %d'
                    % (addition_client.req.a, addition_client.req.b, response.sum)
                )
            break

    addition_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
