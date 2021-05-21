// create_subscription

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LaserSub : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_sub;

public:
  LaserSub() : Node("topic_sub_oop_node")
  {
    m_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/skidbot/scan", 10, std::bind(&LaserSub::sub_callback, this, std::placeholders::_1));
  }

  void sub_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // std::cout << (msg->ranges).size() << std::endl;

    // for (auto e : msg->ranges)
    //   std::cout << e << std::endl;

    RCLCPP_INFO(this->get_logger(), "I got %f", (msg->ranges)[360]);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserSub>());
  rclcpp::shutdown();

  return 0;
}
