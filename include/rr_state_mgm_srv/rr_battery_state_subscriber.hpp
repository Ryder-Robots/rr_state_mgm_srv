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

#ifndef RR_BATTERY_STATE_SUBSCRIBER_HPP
#define RR_BATTERY_STATE_SUBSCRIBER_HPP

#include "rr_common_base/rr_sensor_constants.hpp"
#include "rr_state_mgm_srv/rr_state_subscriber_base.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

namespace rr_state_manager
{
/**
 * @class RrBatteryStateSubscriber
 * @brief on battery state events, will update the state frame.
 */
class RrBatteryStateSubscriber : public RrStateSubscriberBase<sensor_msgs::msg::BatteryState>
{
 public:
  RrBatteryStateSubscriber() : RrStateSubscriberBase("rr_battery_state_subscriber") {}

  const std::string get_topic() override { return rr_constants::TOPIC_BATT_STATE; }

  void some_method();
};

// class RrBatteryStateSubscriber : public
// RrStateSubscriberBase<sensor_msgs::msg::BatteryState::SharedPtr>
// {
//  public:
//   RrBatteryStateSubscriber() : RrStateSubscriberBase("rr_battery_state_subscriber") {}
//   ~RrBatteryStateSubscriber() = default;

//   /**
//    * @fn get_topic
//    * @brief direct reference to the constant topic.
//    */
//   std::string get_topic() override { return rr_constants::TOPIC_BATT_STATE; }

//   /**
//    * @fn callback
//    * @brief called on battery state event.
//    */
//   void callback(const sensor_msgs::msg::BatteryState::SharedPtr message) override;
// };
}  // namespace rr_state_manager

#endif  // RR_BATTERY_STATE_SUBSCRIBER_HPP