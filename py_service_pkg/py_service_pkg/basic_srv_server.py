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


class AdditionServer(Node):

    def __init__(self):
        super().__init__('addition_server')
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_two_ints_callback
        )
        self.get_logger().info('==== Basic Server Started ====')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'==== Incoming request ==== \n Fisrt Operand : {request.a}, \
             Second Operand: {request.b}')
        return response


def main(args=None):
    rclpy.init(args=args)

    addition_server = AdditionServer()

    rclpy.spin(addition_server)

    addition_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
