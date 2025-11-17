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
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
RrStateSubscriberBase<T>::on_configure(const rclcpp_lifecycle::State& state)
{
  RCLCPP_INFO(this->get_logger(), "creating subscriber: %s", this->get_name());

  std::function<void(const typename T::SharedPtr)> cb_binding =
      [this](const typename T::SharedPtr msg) { this->callback_around(msg); };
  auto node_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  subscription_       = this->create_subscription<typename T::SharedPtr>(
      get_topic(), configure_qos(), cb_binding, rmw_qos_profile_default, node_cb_group_);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

// /*
//  * At this stage this can be overriden, but the policy could be set up to use parameters.
//  * Potentially this could be derived from the frame rate.
//  */

// /**
//  * creates a subscription service.
//  */
// template <typename T>
// void RrStateSubscriberBase<T>::init(std::shared_ptr<std::shared_mutex> mutex,
//                                     std::shared_ptr<rr_interfaces::msg::BufferResponse>
//                                     state_frame)
// {
//   state_frame_       = state_frame;
//   mutex_             = mutex;
//   rclcpp::QoS policy = configure_qos();

//   std::function<void(const typename T::SharedPtr)> cb_binding =
//       [this](const typename T::SharedPtr msg)
//   {
//     this->callback_around(msg);
//   };

//   RCLCPP_INFO(this->get_logger(), "creating subscriber");
//   node_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
//   subscription_  = this->create_subscription<typename T::SharedPtr>(
//       get_topic(), policy, cb_binding, rmw_qos_profile_default, node_cb_group_);
// }

// /**
//  * called as the callback, note that this will reference the callback function, which the
//  sub-class
//  * MUST implement.
//  */
// template <typename T>
// void RrStateSubscriberBase<T>::callback_around(T message)
// {
//   RCLCPP_DEBUG(this->get_logger(), "creating subscriber");
//   std::unique_lock<std::shared_mutex> lock(*mutex_);
//   callback(*message);
// }