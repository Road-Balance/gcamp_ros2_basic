#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

#include <memory>

using AddTwoInts = example_interfaces::srv::AddTwoInts;

class BasicServer : public rclcpp::Node
{
private:
  rclcpp::Service<AddTwoInts>::SharedPtr m_service;

public:
  BasicServer() : Node("add_two_ints_server")
  {
    RCLCPP_WARN(get_logger(), "Add two Ints Server Started");
    m_service = create_service<AddTwoInts>(
        "add_two_ints", std::bind(&BasicServer::response, this, std::placeholders::_1, std::placeholders::_2));
  }

  void response(std::shared_ptr<AddTwoInts::Request> request, std::shared_ptr<AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(),
                "Incoming request\na: %ld"
                " b: %ld",
                request->a, request->b);
    RCLCPP_INFO(get_logger(), "sending back response: [%ld]", (long int)response->sum);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BasicServer>();

  rclcpp::spin(node);
  rclcpp::shutdown();
}