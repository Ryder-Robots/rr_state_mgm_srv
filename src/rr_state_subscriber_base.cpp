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

#include "rr_state_mgm_srv/rr_state_subscriber_base.hpp"

using namespace rr_state_manager;

Ros2Lc_CallbackReturn RrStateSubscriberBase::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (previous_state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    RCLCPP_WARN(get_logger(), "on_configure called from unexpected state: %s",
                previous_state.label().c_str());
    return Ros2Lc_CallbackReturn::FAILURE;
  }

  Ros2Lc_CallbackReturn status = Ros2Lc_CallbackReturn::SUCCESS;
  try
  {
    if (!pre_check()) {
        RCLCPP_ERROR(this->get_logger(), "precheck for %s has failed.", this->get_name());
        status = Ros2Lc_CallbackReturn::FAILURE;
    }
    init();
  }
  catch (const std::exception& e)
  {
    RCLCPP_FATAL(this->get_logger(), "unable to configure node: %s, recieved error: %s",
                 this->get_name(), e.what());
    status = Ros2Lc_CallbackReturn::ERROR;
  }
  catch (...)
  {
    RCLCPP_FATAL(this->get_logger(), "unable to configure node: %s, due to unknown error condition",
                 this->get_name());
    status = Ros2Lc_CallbackReturn::ERROR;
  }
  return status;
}
