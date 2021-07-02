// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cinttypes>
#include <memory>

// Deduction
#include <typeinfo>

#include "custom_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = custom_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class FBActionClient : public rclcpp::Node {
private:
  rclcpp_action::Client<Fibonacci>::SharedPtr m_action_client;
  GoalHandleFibonacci::SharedPtr goal_handle;
  rclcpp::TimerBase::SharedPtr m_timer;

public:
  FBActionClient() : Node("fb_action_client"), goal_handle(nullptr) {
    m_action_client =
        rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    m_timer = create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&FBActionClient::send_goal, this));

    RCLCPP_INFO(get_logger(), "FB Action Client Node Created");
  }

  bool is_goal_handle_none() {
    bool is_goal_handle = goal_handle == nullptr ? true : false;
    return is_goal_handle;
  }

  const std::shared_future<GoalHandleFibonacci::WrappedResult>
  get_result_future() {
    auto result_future = m_action_client->async_get_result(goal_handle);
    return result_future;
  }

  const std::shared_future<
      std::shared_ptr<action_msgs::srv::CancelGoal_Response>>
  get_cancel_result_future() {
    auto cancel_result_future = m_action_client->async_cancel_goal(goal_handle);
    return cancel_result_future;
  }

  void send_goal() {
    using namespace std::placeholders;

    // timer cancel required for send goal once
    m_timer->cancel();

    if (!m_action_client->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    auto send_goal_options =
        rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    // TODO(Swimming): Cancel Logic
    send_goal_options.goal_response_callback =
        std::bind(&FBActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&FBActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&FBActionClient::result_callback, this, _1);

    m_action_client->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(
      std::shared_future<GoalHandleFibonacci::SharedPtr> future) {
    goal_handle = future.get();

    if (!goal_handle)
      RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    else
      RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
  }

  void
  feedback_callback(GoalHandleFibonacci::SharedPtr,
                    const std::shared_ptr<const Fibonacci::Feedback> feedback) {
    std::cout << "Next number in sequence received: ";

    for (auto number : feedback->partial_sequence)
      std::cout << number << " ";

    std::cout << std::endl;
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal aborted");
      rclcpp::shutdown();
      return;
    case rclcpp_action::ResultCode::CANCELED:
      rclcpp::shutdown();
      RCLCPP_ERROR(get_logger(), "Goal canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
    }

    std::cout << "Result received: ";

    for (const auto number : result.result->sequence)
      std::cout << number << " ";

    std::cout << std::endl;
    rclcpp::shutdown();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto client_node = std::make_shared<FBActionClient>();

  // rclcpp::spin(client_node);

  while (rclcpp::ok()) {
    rclcpp::spin_some(client_node);

    if (!client_node->is_goal_handle_none()) {
      auto result_future = client_node->get_result_future();
      auto wait_result = rclcpp::spin_until_future_complete(
          client_node, result_future, std::chrono::seconds(3));

      if (wait_result == rclcpp::executor::FutureReturnCode::TIMEOUT) {
        RCLCPP_INFO(client_node->get_logger(), "Canceling goal");
        // Cancel the goal since it is taking too long

        auto cancel_result_future = client_node->get_cancel_result_future();
        if (rclcpp::spin_until_future_complete(client_node,
                                               cancel_result_future) !=
            rclcpp::executor::FutureReturnCode::SUCCESS) {
          RCLCPP_ERROR(client_node->get_logger(), "failed to cancel goal");
          rclcpp::shutdown();
          return 1;
        }

        RCLCPP_INFO(client_node->get_logger(), "goal is being canceled");

        while (rclcpp::ok())
          rclcpp::spin_some(client_node);
      } else if (wait_result != rclcpp::executor::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(client_node->get_logger(), "failed to get result");
        rclcpp::shutdown();
        return 1;
      }
    }
  }

  rclcpp::shutdown();
  return 0;
}
