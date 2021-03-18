#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simple_node");

    RCLCPP_INFO(node->get_logger(), "Logger Test");

    rclcpp::shutdown();
    return 0;
}