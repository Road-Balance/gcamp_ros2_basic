#include "rclcpp/rclcpp.hpp"

class Talker : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr m_timer;
  size_t m_count;

  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "I am Simple OOP Example");
  }

public:
  Talker() : Node("simple_oop_node")
  {
    m_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Talker::timer_callback, this));
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}