// Copyright 2019 http://www.theconstruct.ai/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("topic_pub_node");

  auto publisher = node->create_publisher<std_msgs::msg::Int32>("counter", 10);
  auto msg = std::make_shared<std_msgs::msg::Int32>();

  msg->data = 0;
  rclcpp::WallRate r(1);

  while (rclcpp::ok())
  {
    publisher->publish(*msg);

    msg->data++;
    rclcpp::spin_some(node);
    r.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
