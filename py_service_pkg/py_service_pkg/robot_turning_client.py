# !/usr/bin/env/ python3
#
# Copyright 2021 Seoul Business Agency Inc.
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


from custom_interfaces.srv import TurningControl
import rclpy
from rclpy.node import Node


class RobotTurnClient(Node):

    def __init__(self):
        super().__init__('robot_turn_client')
        self.client = self.create_client(TurningControl, 'turn_robot')  # CHANGE

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = TurningControl.Request()
        self.get_logger().info('==== Robot Turn Service Client ====')

    def send_request(self):

        while True:
            try:
                td = input('> Type turning time duration: ')
                vel_x = input('> Type turning linear velocity: ')
                vel_z = input('> Type turning angular velocity: ')

                if float(vel_z) > 1.5707 or float(vel_x) > 3:
                    raise ArithmeticError('Velocity too high !!')

                self.req.time_duration = int(td)

                self.req.linear_vel_x = float(vel_x)
                self.req.angular_vel_z = float(vel_z)
                break
            except ArithmeticError as e:
                self.get_logger().warn(e)
            except Exception as e:
                self.get_logger().warn(e)
                self.get_logger().warn('Not a number, PLZ Type number Again')

        self.future = self.client.call_async(self.req)
        self.get_logger().info(
            f'linear_x : {self.req.linear_vel_x} / angular_z : {self.req.angular_vel_z}'
        )
        self.get_logger().info(' Request Sended ')
        return self.future


def main(args=None):
    rclpy.init(args=args)

    robot_turn_client = RobotTurnClient()
    future = robot_turn_client.send_request()

    rclpy.spin_until_future_complete(robot_turn_client, future)

    if future.done():
        try:
            response = future.result()
        except Exception:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception()
            )
        else:
            robot_turn_client.get_logger().info('==== Service Call Done ====')
            robot_turn_client.get_logger().info(
                f"Result Message : {'Success' if response.success == True else 'Fail'}"
            )
        finally:
            robot_turn_client.get_logger().warn('==== Shutting down node ====')
            robot_turn_client.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
