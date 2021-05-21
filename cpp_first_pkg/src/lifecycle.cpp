#include "rclcpp/rclcpp.hpp"

class Talker : public rclcpp::Node
{
private:
  rclcpp::TimerBase::SharedPtr m_timer;
  size_t m_count;

  void timer_callback()
  {
    m_count++;
    RCLCPP_INFO(this->get_logger(), "I am Simple OOP Example, count : %d", m_count);
  }

public:
  Talker() : Node("simple_oop_node"), m_count(0)
  {
    RCLCPP_WARN(this->get_logger(), "Node Constructor");

    m_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Talker::timer_callback, this));
  }

  ~Talker(){
    RCLCPP_WARN(this->get_logger(), "Node Destructor");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  auto talker = std::make_shared<Talker>();
  rclcpp::spin(talker);
  
  RCLCPP_INFO(talker->get_logger(), "==== Spin Done ====");
  rclcpp::shutdown();

  std::cout << "==== After Shutdown ====" << std::endl;

  return 0;
}