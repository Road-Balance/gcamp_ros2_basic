// Copyright 2019 http://www.theconstruct.ai/

#include <chrono>
#include <cinttypes>
#include <iostream>
#include <memory>
#include <string>

#include "gazebo_msgs/srv/delete_model.hpp"
#include "rclcpp/rclcpp.hpp"

gazebo_msgs::srv::DeleteModel::Response::SharedPtr
send_request(rclcpp::Node::SharedPtr node,
             rclcpp::Client<gazebo_msgs::srv::DeleteModel>::SharedPtr client,
             gazebo_msgs::srv::DeleteModel::Request::SharedPtr request) {
  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Client request->model_name : %s",
                request->model_name.c_str());
    return result.get();
  } else {
    RCLCPP_ERROR(node->get_logger(), "service call failed :(");
    return NULL;
  }
}

int main(int argc, char **argv) {
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("cpp_simple_service_client");
  auto topic = std::string("/gazebo/delete_model");
  auto client = node->create_client<gazebo_msgs::srv::DeleteModel>(topic);
  auto request = std::make_shared<gazebo_msgs::srv::DeleteModel::Request>();

  //  Fill the variable model_name of this object with the desired value
  request->model_name = "bowl_1";

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
  }

  auto result = send_request(node, client, request);
  if (result) {
    auto result_str = result->success ? "True" : "False";

    RCLCPP_INFO(node->get_logger(), "Result-Success : %s", result_str);
    RCLCPP_INFO(node->get_logger(), "Result-Status: %s",
                result->status_message.c_str());
  } else {
    RCLCPP_ERROR(node->get_logger(),
                 "Interrupted while waiting for response. Exiting.");
  }

  rclcpp::shutdown();
  return 0;
}
