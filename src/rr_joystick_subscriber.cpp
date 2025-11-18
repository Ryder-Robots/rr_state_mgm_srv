// Copyright (c) 2025 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "rr_state_mgm_srv/rr_joystick_subscriber.hpp"

using namespace rr_state_manager;

void RrJoystickSubscriber::init()
{
  RCLCPP_INFO(this->get_logger(), "creating subscriptions");
  rclcpp::SubscriptionOptions options;
  auto topic_callback = std::bind(&RrJoystickSubscriber::callback, this, std::placeholders::_1);
  subscription_       = this->create_subscription<sensor_msgs::msg::Joy>(
      rr_constants::TOPIC_JOY, rclcpp::SensorDataQoS(), topic_callback, options);
}

void RrJoystickSubscriber::callback(sensor_msgs::msg::Joy msg)
{
  std::unique_lock<std::shared_mutex> lock(*mutex_);
  RCLCPP_DEBUG(this->get_logger(), "setting joystick state");
  state_frame_->feature_sets.has_joy = true;
  state_frame_->joystick             = msg;
}
