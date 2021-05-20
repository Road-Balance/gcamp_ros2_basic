#include <chrono>
#include <cstdlib>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class BasicClient : public rclcpp::Node
{
private:
  rclcpp::Client<AddTwoInts>::SharedPtr m_client;
  std::shared_ptr<AddTwoInts::Request> m_request;

public:
  BasicClient() : Node("add_two_ints_client")
  {
    m_client = create_client<AddTwoInts>("add_two_ints");
    m_request = std::make_shared<AddTwoInts::Request>();

    while (!m_client->wait_for_service(1s))
      RCLCPP_INFO(get_logger(), "service not available, waiting again...");

    RCLCPP_INFO(get_logger(), "service available, waiting serice call");
  }

  auto get_result_future(const long long &a, const long long &b)
  {
    m_request->a = a;
    m_request->b = b;

    return m_client->async_send_request(m_request);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3)
  {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "usage: add_two_ints_client X Y");
    return 1;
  }

  auto basic_service_client = std::make_shared<BasicClient>();
  auto result = basic_service_client->get_result_future(atoi(argv[1]), atoi(argv[2]));

  // Wait for the result.
  if (rclcpp::spin_until_future_complete(basic_service_client, result) == rclcpp::executor::FutureReturnCode::SUCCESS)
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sum: %ld", result.get()->sum);
  else
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");

  rclcpp::shutdown();
  return 0;
}