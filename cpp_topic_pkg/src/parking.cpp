#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
class ParkingNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<Twist>::SharedPtr m_pub;
  rclcpp::Subscription<LaserScan>::SharedPtr m_sub;

  rclcpp::TimerBase::SharedPtr m_timer;
  Twist m_twist_msg;

public:
  ParkingNode() : Node("robot_parking_node")
  {
    RCLCPP_INFO(get_logger(), "Parking Node Created");

    m_pub = create_publisher<Twist>("/skidbot/cmd_vel", 10);
    m_sub = create_subscription<LaserScan>("/skidbot/scan", 10,
                                           std::bind(&ParkingNode::sub_callback, this, std::placeholders::_1));
  }

  void sub_callback(const LaserScan::SharedPtr msg)
  {
    auto forward_distance = (msg->ranges)[360];

    if (forward_distance > 0.8)
      move_robot();
    else
      stop_robot();
  }

  void move_robot()
  {
    m_twist_msg.linear.x = 0.5;
    m_twist_msg.angular.z = 1.0;
    m_pub->publish(m_twist_msg);

    std::cout << "==== Move Robot ====" << std::endl;
  }

  void stop_robot()
  {
    m_twist_msg.linear.x = 0.0;
    m_twist_msg.angular.z = 0.0;
    m_pub->publish(m_twist_msg);

    std::cout << "==== Stop Robot ====" << std::endl;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto parking_node = std::make_shared<ParkingNode>();
  rclcpp::spin(parking_node);
  rclcpp::shutdown();

  return 0;
}