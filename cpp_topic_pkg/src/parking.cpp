// Copyright 2021 Seoul Business Agency Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using Twist = geometry_msgs::msg::Twist;
using LaserScan = sensor_msgs::msg::LaserScan;
class ParkingNode : public rclcpp::Node
{
private:
  rclcpp::Publisher<Twist>::SharedPtr m_pub;
  rclcpp::Subscription<LaserScan>::SharedPtr m_sub;

  Twist m_twist_msg;

public:
  ParkingNode() : Node("robot_parking_node")
  {
    RCLCPP_INFO(get_logger(), "Parking Node Created");

    m_pub = create_publisher<Twist>("/skidbot/cmd_vel", 10);
    m_sub = create_subscription<LaserScan>("/skidbot/scan", 10,
      std::bind(&ParkingNode::sub_callback, this, std::placeholders::_1));
  }

  void sub_callback(const LaserScan::SharedPtr msg)
  {
    auto forward_distance = (msg->ranges)[360];

    if (forward_distance > 0.8){
      move_robot(forward_distance);
    }else{
      stop_robot();
      rclcpp::shutdown();
    }
  }

  void move_robot(const float &forward_distance)
  {
    m_twist_msg.linear.x = 0.5;
    m_twist_msg.angular.z = 0.0;
    m_pub->publish(m_twist_msg);

    std::cout << "Distance from Obstacle ahead : " << forward_distance << std::endl;
  }

  void stop_robot()
  {
    m_twist_msg.linear.x = 0.0;
    m_twist_msg.angular.z = 0.0;
    m_pub->publish(m_twist_msg);

    RCLCPP_WARN(get_logger(), "Stop Robot and make Node FREE!");
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto parking_node = std::make_shared<ParkingNode>();
  rclcpp::spin(parking_node);
  rclcpp::shutdown();

  return 0;
}
