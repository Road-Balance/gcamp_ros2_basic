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

#include <memory>
#include <thread>

#include "custom_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
// TODO(jacobperron): Remove this once it is included as part of 'rclcpp.hpp'
#include "rclcpp_action/rclcpp_action.hpp"

using Fibonacci = custom_interfaces::action::Fibonacci;
using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

class FBActionServer : public rclcpp::Node {
private:
  rclcpp_action::Server<Fibonacci>::SharedPtr m_action_server;

public:
  FBActionServer() : Node("fb_action_server") {
    using namespace std::placeholders;
    // Create an action server with three callbacks
    //   'handle_goal' and 'handle_cancel' are called by the Executor
    //   (rclcpp::spin) 'execute' is called whenever 'handle_goal' returns by
    //   accepting a goal
    //    Calls to 'execute' are made in an available thread from a pool of
    //    four.
    m_action_server = rclcpp_action::create_server<Fibonacci>(
        this, "fibonacci",
        std::bind(&FBActionServer::handle_goal, this, _1, _2),
        std::bind(&FBActionServer::handle_cancel, this, _1),
        std::bind(&FBActionServer::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(),
                "FB Action Server Created Waiting for client... ");
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Fibonacci::Goal> goal) {
    RCLCPP_INFO(get_logger(), "Got goal request with order %d", goal->order);

    (void)uuid;

    // Let's reject sequences that are over 9000
    if (goal->order > 9000) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_WARN(get_logger(), "Got request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    using namespace std::placeholders;
    std::thread{std::bind(&FBActionServer::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(2);  // 0.5 sec

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto result = std::make_shared<Fibonacci::Result>();
    auto &sequence = feedback->partial_sequence;
    sequence.push_back(0);
    sequence.push_back(1);

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_WARN(get_logger(), "Goal Canceled");
        return;
      }
      // Update sequence
      sequence.push_back(sequence[i] + sequence[i - 1]);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Publish Feedback");

      loop_rate.sleep();
    }
    // Check if goal is done
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Goal Succeeded");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto server_node = std::make_shared<FBActionServer>();
  rclcpp::spin(server_node);

  rclcpp::shutdown();
  return 0;
}
