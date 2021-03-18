// https://roboticsbackend.com/write-minimal-ros2-cpp-node/

#include "rclcpp/rclcpp.hpp"

std::shared_ptr<rclcpp::Node> node = nullptr;

void timerCallback()
{
    RCLCPP_INFO(node->get_logger(), "I am Simple OOP Example");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("my_node_name");

    auto timer = node->create_wall_timer( std::chrono::milliseconds(200), timerCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}