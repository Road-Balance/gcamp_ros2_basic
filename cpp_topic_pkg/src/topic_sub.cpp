// Copyright 2019 http://www.theconstruct.ai/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

rclcpp::Node::SharedPtr node = nullptr;

void sub_callback(const std_msgs::msg::Int32::SharedPtr msg)
{
  // RCLCPP_INFO(node->get_logger(), "I got %d", msg->data);
  std::cout << "I got " << msg->data << std::endl;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("topic_sub_node");

  auto subscriber = node->create_subscription<std_msgs::msg::Int32>("counter", 10, sub_callback);

  rclcpp::spin(node);
  rclcpp::shutdown();

  subscriber = nullptr;
  node = nullptr;

  return 0;
}
