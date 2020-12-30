// Copyright (c) 2020 OUXT Polaris
//
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

#include <joy_to_twist/joy_to_twist_component.hpp>
#include <memory>
#include <vector>

namespace joy_to_twist
{
JoyToTwistComponent::JoyToTwistComponent(const rclcpp::NodeOptions & options)
: Node("joy_to_twist", options)
{
  param_handler_ptr_ = this->add_on_set_parameters_callback(
    std::bind(
      &JoyToTwistComponent::paramCallback, this,
      std::placeholders::_1));

  declare_parameter("lateral_input_ratio", 0.3);
  get_parameter("lateral_input_ratio", lateral_input_ratio_);
  declare_parameter("longitudal_input_ratio", 1.0);
  get_parameter("longitudal_input_ratio", longitudal_input_ratio_);

  joy_sub_ =
    this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 1,
    std::bind(&JoyToTwistComponent::JoyCallback, this, std::placeholders::_1));
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/target_twist", 1);
}

void JoyToTwistComponent::JoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->axes.size() != 8) {
    return;
  }
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = msg->axes[1] * longitudal_input_ratio_;
  cmd_vel.angular.z = msg->axes[3] * lateral_input_ratio_;
  twist_pub_->publish(cmd_vel);
}

rcl_interfaces::msg::SetParametersResult JoyToTwistComponent::paramCallback(
  const std::vector<rclcpp::Parameter> & params)
{
  auto result = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
  try {
    for (auto && param : params) {
      if (param.get_name() == "longitudal_input_ratio") {
        longitudal_input_ratio_ = param.get_value<double>();
      }
      if (param.get_name() == "lateral_input_ratio") {
        lateral_input_ratio_ = param.get_value<double>();
      }
    }
  } catch (const std::exception & e) {
    result->successful = false;
    result->reason = "";
    return *result;
  }
  result->successful = true;
  result->reason = "";
  return *result;
}
}  // namespace joy_to_twist
