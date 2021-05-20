// #include <memory>

#include "custom_interfaces/srv/turning_control.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

using Twist = geometry_msgs::msg::Twist;
using TurningControl = custom_interfaces::srv::TurningControl;

class RobotTurnServer : public rclcpp::Node
{
private:
  rclcpp::Service<TurningControl>::SharedPtr m_service;
  rclcpp::Publisher<Twist>::SharedPtr m_twist_pub;

  Twist m_twist_msg;

public:
  RobotTurnServer() : Node("robot_turn_server")
  {
    RCLCPP_WARN(get_logger(), "Robot Turn Server Started");

    m_twist_pub = create_publisher<Twist>("/skidbot/cmd_vel", 10);
    m_service = create_service<TurningControl>(
        "turn_robot", std::bind(&RobotTurnServer::response, this, std::placeholders::_1, std::placeholders::_2));
  }

  // uint32 time_duration
  // float64 angular_vel_z
  // float64 linear_vel_x
  // ---
  // bool success
  void response(std::shared_ptr<TurningControl::Request> request, std::shared_ptr<TurningControl::Response> response)
  {
    auto t_start = now();
    auto t_now = now();
    auto t_delta = request->time_duration * 1e9;

    std::cout << "Time Duration : " << request->time_duration << std::endl;
    std::cout << "Linear X Cmd : " << request->linear_vel_x << std::endl;
    std::cout << "Angular Z Cmd : " << request->angular_vel_z << std::endl;

    RCLCPP_INFO(get_logger(), "Request Received Robot Starts to Move");

    while ((t_now - t_start).nanoseconds() < t_delta)
    {
      t_now = now();
      move_robot(request->linear_vel_x, request->angular_vel_z);
    }
    stop_robot();

    RCLCPP_INFO(get_logger(), "Request Done Wating for next request...");
    response->success = true;
  }

  void move_robot(const float& linear_x, const float& angular_z)
  {
    m_twist_msg.linear.x = linear_x;
    m_twist_msg.angular.z = angular_z;

    m_twist_pub->publish(m_twist_msg);
  }

  void stop_robot()
  {
    m_twist_msg.linear.x = 0.0;
    m_twist_msg.angular.z = 0.0;
    m_twist_pub->publish(m_twist_msg);

    RCLCPP_INFO(get_logger(), "Stop Robot and make Node FREE!");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotTurnServer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
}