#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class TwistPub : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;
  rclcpp::TimerBase::SharedPtr m_timer;

  geometry_msgs::msg::Twist twist_msg = geometry_msgs::msg::Twist();

  void timer_callback()
  {
    // std::cout << "ekndf " << std::endl;
    twist_msg.linear.x = 0.5;
    twist_msg.angular.z = 1.0;
    m_pub->publish(twist_msg);
  }

public:
  TwistPub() : Node("cmd_vel_pub_node")
  {
    RCLCPP_INFO(get_logger(), "cmd_vel_pub_node");

    m_pub = create_publisher<geometry_msgs::msg::Twist>("/skidbot/cmd_vel", 10);
    m_timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TwistPub::timer_callback, this));
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistPub>());
  rclcpp::shutdown();

  return 0;
}