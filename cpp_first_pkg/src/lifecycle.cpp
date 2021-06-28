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

class Talker : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr m_timer;
  size_t m_count;

  void timer_callback()
  {
    m_count++;
    RCLCPP_INFO(this->get_logger(), "I am Simple OOP Example, count : %d", m_count);
  }

public:
  Talker() : Node("simple_oop_node"), m_count(0)
  {
    RCLCPP_WARN(this->get_logger(), "Node Constructor");

    m_timer = this->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&Talker::timer_callback, this));
  }

  ~Talker(){
    // publisher
    RCLCPP_WARN(this->get_logger(), "Node Destructor");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto talker = std::make_shared<Talker>();
  rclcpp::spin(talker);

  RCLCPP_INFO(talker->get_logger(), "==== Spin Done ====");

  rclcpp::shutdown();

  std::cout << "==== After Shutdown ====" << std::endl;

  return 0;
}
