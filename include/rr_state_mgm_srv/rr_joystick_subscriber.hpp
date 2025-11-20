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

#ifndef RR_JOYSTICK_SUBSCRIBER_HPP
#define RR_JOYSTICK_SUBSCRIBER_HPP

#include "rr_common_base/rr_sensor_constants.hpp"
#include "rr_state_mgm_srv/rr_state_subscriber_base.hpp"
#include "sensor_msgs/msg/joy.hpp"


namespace rr_state_manager
{

/**
 * @class RrJoystickSubscriber
 * @brief updates state for joystick events.
 */
class RrJoystickSubscriber : public RrStateSubscriberBase
{
 public:
  explicit RrJoystickSubscriber(std::shared_ptr<std::shared_mutex> mutex,
                                std::shared_ptr<rr_interfaces::msg::StateFrame> state_frame)
      : RrStateSubscriberBase("rr_joystick_state_node", mutex, state_frame)
  {
    
  }

 protected:
  bool pre_check() override;
  void init() override;

 private:
  void callback(sensor_msgs::msg::Joy msg);
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
};
}  // namespace rr_state_manager

#endif  // RR_JOYSTICK_SUBSCRIBER_HPP