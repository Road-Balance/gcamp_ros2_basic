// Copyright 2019 http://www.theconstruct.ai/

#include <inttypes.h>
#include <memory>

#include "gazebo_msgs/srv/delete_model.hpp"
#include "rclcpp/rclcpp.hpp"

using DeleteModel = gazebo_msgs::srv::DeleteModel;
rclcpp::Node::SharedPtr g_node = nullptr;

void handle_service(const std::shared_ptr<rmw_request_id_t> request_header,
                    const std::shared_ptr<DeleteModel::Request> request,
                    const std::shared_ptr<DeleteModel::Response> response) {
  (void)request_header;
  RCLCPP_INFO(g_node->get_logger(),
              "Incoming request\nModel-To_Delete-Name: %s",
              request->model_name.c_str());

  response->success = true;
  response->status_message =
      "The Model " + request->model_name + " was deleted.";
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  g_node = rclcpp::Node::make_shared("cpp_simple_service_server");
  auto server = g_node->create_service<DeleteModel>("/gazebo/delete_model",
                                                    handle_service);
  rclcpp::spin(g_node);
  rclcpp::shutdown();
  g_node = nullptr;
  return 0;
}
