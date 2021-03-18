#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("simple_loop_node");
    rclcpp::WallRate rate(2);

    while ( rclcpp::ok() ){
        RCLCPP_INFO(node->get_logger(), "Simple Loop Node");
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}