// Copyright 2021 From roboticsbackend.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// https://roboticsbackend.com/write-minimal-ros2-cpp-node/

#include <memory>
#include "rclcpp/rclcpp.hpp"

std::shared_ptr<rclcpp::Node> node = nullptr;

void timerCallback() {
  RCLCPP_INFO(node->get_logger(), "I am Simple OOP Example");
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  node = std::make_shared<rclcpp::Node>("my_node_name");

  auto timer =
      node->create_wall_timer(std::chrono::milliseconds(200), timerCallback);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
