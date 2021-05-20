#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

rclcpp::Node::SharedPtr node = nullptr;
using std_srvs::srv::SetBool;

// bool data # e.g. for hardware enabling / disabling
// ---
// bool success   # indicate successful run of triggered service
// string message # informational, e.g. for error messages

void server_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                     const std::shared_ptr<SetBool::Request> request, const std::shared_ptr<SetBool::Response> response)
{
  (void)request_header;
  auto ready_state = request->data;
  bool success = false;
  std::string message = "";

  if (ready_state)
  {
    RCLCPP_WARN(node->get_logger(), "Ready State True recieved");
    success = true;
    message = "Congraz!! You just completed your first ROS2 Service :D";
  }
  else
  {
    RCLCPP_WARN(node->get_logger(), "Ready State False recieved");
  }
  response->success = success;
  response->message = message;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  node = rclcpp::Node::make_shared("service_server_node");
  auto srv_server = node->create_service<SetBool>("/setbool_srv", server_callback);
  RCLCPP_WARN(node->get_logger(), "Service Server Node Started!!");
  RCLCPP_INFO(node->get_logger(), "Waiting for Client...");

  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;

  return 0;
}