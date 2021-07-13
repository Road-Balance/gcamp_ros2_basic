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
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class TwistPub : public rclcpp::Node {
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;
  rclcpp::TimerBase::SharedPtr m_timer;

  geometry_msgs::msg::Twist m_twist_msg;

  void timer_callback() { move_robot(); }

public:
  TwistPub() : Node("cmd_vel_pub_node") {
    RCLCPP_INFO(get_logger(), "Cmd_vel Pub Node Created");

    m_pub = create_publisher<geometry_msgs::msg::Twist>("skidbot/cmd_vel", 10);
    m_timer = create_wall_timer(std::chrono::milliseconds(100),
                                std::bind(&TwistPub::timer_callback, this));
  }

  void move_robot() {
    m_twist_msg.linear.x = 0.5;
    m_twist_msg.angular.z = 1.0;
    m_pub->publish(m_twist_msg);
  }

  void stop_robot() {
    m_twist_msg.linear.x = 0.0;
    m_twist_msg.angular.z = 0.0;
    m_pub->publish(m_twist_msg);

    RCLCPP_INFO(get_logger(), "==== Stop Robot ====");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto twist_pub = std::make_shared<TwistPub>();

  auto t_start = twist_pub->now();
  auto t_now = twist_pub->now();

  auto stop_time = 5.0;

  while ((t_now - t_start).seconds() < stop_time) {
    t_now = twist_pub->now();
    // rclcpp::spin_some(twist_pub);
    twist_pub->move_robot();

    RCLCPP_INFO(twist_pub->get_logger(), "%f Seconds Passed", (t_now - t_start).seconds());
  }

  // publish doesn't require spin
  twist_pub->stop_robot();

  rclcpp::shutdown();

  return 0;
}
