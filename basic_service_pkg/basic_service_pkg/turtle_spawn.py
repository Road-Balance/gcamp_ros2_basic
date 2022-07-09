# !/usr/bin/env/ python3
#
# Copyright 2021 @RoadBalance
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


from turtlesim.srv import Spawn
import rclpy
from rclpy.node import Node


class SpawnTurtle(Node):

    def __init__(self):
        super().__init__('spawn_turtle_node')
        self.client = self.create_client(Spawn, 'spawn')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, Waiting again...')

        self.req = Spawn.Request()
        self.get_logger().info('=== [Ready to Call Service Request] ===')

    def send_request(self):
        # example) "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
        self.req.x = float(input('> Turtle X position : '))
        self.req.y = float(input('> Turtle Y position : '))
        self.req.theta = float(input('> Turtle Angle : '))
        self.req.name = str(input('> Turtle Name : '))

        self.future = self.client.call_async(self.req)
        return self.future


def main(args=None):
    rclpy.init(args=args)

    turtle_spawn_client = SpawnTurtle()
    future = turtle_spawn_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(turtle_spawn_client)

        if future.done():
            try:
                response = future.result()
            except Exception as e:
                turtle_spawn_client.get_logger().info('Service call failed %r' % (e,))
            finally:
                turtle_spawn_client.get_logger().info(
                    'Turtle Named : %s Spawned Successfully.'
                    % (response.name)
                )
            break

    turtle_spawn_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
