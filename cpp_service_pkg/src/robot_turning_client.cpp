#include <chrono>
#include <cstdlib>
#include <memory>

#include "custom_interfaces/srv/turning_control.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using TurningControl = custom_interfaces::srv::TurningControl;

class RobotTurnClient : public rclcpp::Node
{
private:
  rclcpp::Client<TurningControl>::SharedPtr m_client;
  std::shared_ptr<TurningControl::Request> m_request;

public:
  RobotTurnClient() : Node("robot_turn_client")
  {
    m_client = create_client<TurningControl>("turn_robot");
    m_request = std::make_shared<TurningControl::Request>();

    while (!m_client->wait_for_service(1s))
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");

    RCLCPP_INFO(get_logger(), "service available, waiting serice call");
  }

  // uint32 time_duration
  // float64 angular_vel_z
  // float64 linear_vel_x
  // ---
  // bool success
  auto get_result_future(const int& time_in, const float& linear_x_in, const float& angular_z_in)
  {
    std::cout << "Input Info" << std::endl 
      << "time_duration : " << time_in << std::endl 
      << "linear_vel_x : " << linear_x_in << std::endl
      << "angular_vel_z : " << angular_z_in << std::endl;

    m_request->time_duration = time_in;
    m_request->linear_vel_x = linear_x_in;
    m_request->angular_vel_z = angular_z_in;

    return m_client->async_send_request(m_request);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: robot_turning_client [seconds] [linear_vel_x] [angular_vel_z]");
    return 1;
  }

  auto basic_service_client = std::make_shared<RobotTurnClient>();
  auto result = basic_service_client->get_result_future(atoi(argv[1]), atof(argv[2]), atof(argv[3]));

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(basic_service_client, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Result : %s", result.get()->success ? "True" : "False");
  else
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");

  rclcpp::shutdown();
  return 0;
}