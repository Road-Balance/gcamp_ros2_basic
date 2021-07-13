// Copyright 2019 http://www.theconstruct.ai/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class CounterSub : public rclcpp::Node {
private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr m_subscriber;

public:
  CounterSub() : Node("topic_sub_oop_node") {
    m_subscriber = this->create_subscription<std_msgs::msg::Int32>(
        "/counter", 10,
        std::bind(&CounterSub::sub_callback, this, std::placeholders::_1));
  }

  void sub_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I got %d", msg->data);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CounterSub>());
  rclcpp::shutdown();

  return 0;
}
