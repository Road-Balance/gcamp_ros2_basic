// Copyright 2021 Seoul Business Agency Inc.
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

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class CounterPub : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr m_publisher;

  rclcpp::TimerBase::SharedPtr m_timer;

  // declaring non-static data members as 'auto'
  // https://stackoverflow.com/questions/11302981/c11-declaring-non-static-data-members-as-auto
  std_msgs::msg::Int32 msg = std_msgs::msg::Int32();

  void timer_callback() {
    msg.data++;
    m_publisher->publish(msg);
  }

public:
  CounterPub() : Node("topic_pub_oop_node") {
    m_publisher = this->create_publisher<std_msgs::msg::Int32>("/counter", 10);
    m_timer =
        this->create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&CounterPub::timer_callback, this));
    msg.data = 0;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CounterPub>());
  rclcpp::shutdown();

  return 0;
}
