#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class TwistPub : public rclcpp::Node
{
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_pub;
  rclcpp::TimerBase::SharedPtr m_timer;

  geometry_msgs::msg::Twist m_twist_msg;

  void timer_callback()
  {
    move_robot();
  }

public:
  TwistPub() : Node("cmd_vel_pub_node")
  {
    RCLCPP_INFO(get_logger(), "Cmd_vel Pub Node Created");

    m_pub = create_publisher<geometry_msgs::msg::Twist>("/skidbot/cmd_vel", 10);
    m_timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&TwistPub::timer_callback, this));
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

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto twist_pub = std::make_shared<TwistPub>();

  auto t_start = twist_pub->now();
  auto t_now = twist_pub->now();
  auto t_delta = 5 * 1e9;

  while ((t_now - t_start).nanoseconds() < t_delta)
  {
    t_now = twist_pub->now();
    // twist_pub->move_robot();
    rclcpp::spin_some(twist_pub);
  }

  // publish doesn't require spin
  twist_pub->stop_robot();

  rclcpp::shutdown();

  return 0;
}