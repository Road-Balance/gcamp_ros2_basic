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
#include <sstream>

#include "custom_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = custom_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

class FBActionClient : public rclcpp::Node
{
private:
  rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr goal_handle;

public:
  FBActionClient() : Node("fibonacci_action_client")
  {
    client_ptr_ = rclcpp_action::create_client<Fibonacci>(this, "fibonacci");
    timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&FBActionClient::send_goal, this));
  }

  const rclcpp_action::ClientGoalHandle<Fibonacci>::SharedPtr get_goal_handle()
  {
    return goal_handle;
  }

  void send_goal()
  {
    using namespace std::placeholders;
    timer_->cancel();

    if (!client_ptr_->wait_for_action_server())
    {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Fibonacci::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();

    // TODO Cancel Logic
    send_goal_options.goal_response_callback = std::bind(&FBActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback = std::bind(&FBActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&FBActionClient::result_callback, this, _1);

    client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
  {
    goal_handle = future.get();

    if (!goal_handle)
    {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleFibonacci::SharedPtr, const std::shared_ptr<const Fibonacci::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback->partial_sequence)
    {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleFibonacci::WrappedResult& result)
  {
    switch (result.code)
    {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }

    std::stringstream ss;

    ss << "Result received: ";
    for (auto number : result.result->sequence)
    {
      ss << number << " ";
    }

    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto g_node = std::make_shared<FBActionClient>();

  rclcpp::spin(g_node);
  // auto wait_result = rclcpp::spin_until_future_complete(g_node, result_future, std::chrono::seconds(3));

  // if (wait_result == rclcpp::executor::FutureReturnCode::TIMEOUT)
  // {
  //   RCLCPP_INFO(g_node->get_logger(), "canceling goal");
  //   // Cancel the goal since it is taking too long
  //   auto cancel_result_future = action_client->async_cancel_goal(goal_handle);
  //   if (rclcpp::spin_until_future_complete(g_node, cancel_result_future) !=
  //   rclcpp::executor::FutureReturnCode::SUCCESS)
  //   {
  //     RCLCPP_ERROR(g_node->get_logger(), "failed to cancel goal");
  //     rclcpp::shutdown();
  //     return 1;
  //   }
  //   RCLCPP_INFO(g_node->get_logger(), "goal is being canceled");
  // }
  // else if (wait_result != rclcpp::executor::FutureReturnCode::SUCCESS)
  // {
  //   RCLCPP_ERROR(g_node->get_logger(), "failed to get result");
  //   rclcpp::shutdown();
  //   return 1;
  // }

  // g_node.reset();
  rclcpp::shutdown();
  return 0;
}