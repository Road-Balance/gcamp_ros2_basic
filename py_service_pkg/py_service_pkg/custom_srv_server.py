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

from custom_interfaces.srv import AddThreeInts  # CHANGE
import rclpy
from rclpy.node import Node


class AddThreeIntServer(Node):

    def __init__(self):
        super().__init__('custom_srv_server')
        self.srv = self.create_service(
            AddThreeInts, 'add_three_ints', self.add_three_ints_callback
        )
        self.get_logger().info('==== Addition Server Started, Waiting for Request ====')

    def add_three_ints_callback(self, request, response):
        response.sum = request.a + request.b + request.c
        self.get_logger().info(
            f'Incoming requests = a: {request.a}, b: {request.b}, c: {request.c}'
        )
        return response


def main(args=None):
    rclpy.init(args=args)

    add_three_ints_node = AddThreeIntServer()

    rclpy.spin(add_three_ints_node)

    add_three_ints_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
