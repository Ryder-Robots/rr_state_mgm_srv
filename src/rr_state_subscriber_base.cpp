#include "rr_state_mgm_srv/rr_state_subscriber_base.hpp"

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

using namespace rr_state_manager;

/*
 * At this stage this can be overriden, but the policy could be set up to use parameters.
 * Potentially this could be derived from the frame rate.
 */
template <typename T>
rclcpp::QoS RrStateSubscriberBase<T>::configure_qos()
{
  RCLCPP_INFO(this->get_logger(), "configuring QoS policy");
  rclcpp::QoS qos_profile(1);
  builtin_interfaces::msg::Duration lifespan_duration;
  lifespan_duration.sec     = 0;
  lifespan_duration.nanosec = 330 * 1000000;
  qos_profile.lifespan(lifespan_duration);
  return qos_profile;
}

template <typename T>
void RrStateSubscriberBase<T>::init(std::shared_ptr<std::shared_mutex> mutex,
                                    std::shared_ptr<rr_interfaces::msg::BufferResponse> state_frame)
{
  state_frame_       = state_frame;
  mutex_             = mutex;
  rclcpp::QoS policy = configure_qos();

  RCLCPP_INFO(this->get_logger(), "creating subscriber");
  node_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  subscription_  = this->create_subscription<typename T::MsgType>(
      get_topic(), policy, get_callback_binding(), rmw_qos_profile_default, node_cb_group_);
}

template <typename T>
void RrStateSubscriberBase<T>::callback_around(const typename T::UniquePtr message)
{
  RCLCPP_DEBUG(this->get_logger(), "creating subscriber");
  std::unique_lock<std::shared_mutex> lock(*mutex_);
  callback(*message);
}