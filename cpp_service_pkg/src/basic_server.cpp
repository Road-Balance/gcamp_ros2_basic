// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class BasicServer : public rclcpp::Node {
private:
  rclcpp::Service<AddTwoInts>::SharedPtr m_service;

public:
  BasicServer() : Node("add_two_ints_server") {
    RCLCPP_WARN(get_logger(), "Add two Ints Server Started");
    m_service = create_service<AddTwoInts>(
        "add_two_ints",
        std::bind(&BasicServer::response, this, std::placeholders::_1,
                  std::placeholders::_2));
  }

  void response(std::shared_ptr<AddTwoInts::Request> request,
                std::shared_ptr<AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(),
                "Incoming request\na: %ld"
                " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(get_logger(), "sending back response: [%ld]",
                (int16_t)response->sum);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BasicServer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
}
