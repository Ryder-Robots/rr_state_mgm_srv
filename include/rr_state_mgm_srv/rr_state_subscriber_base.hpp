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

#ifndef RR_STATE_SUBSCRIBER_BASE_HPP
#define RR_STATE_SUBSCRIBER_BASE_HPP

#include <memory>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rr_interfaces/msg/state_frame.hpp"

// definitions are defined here, because ROS2 recommends avoiding mangled nodes (see
// http://design.ros2.org/articles/node_lifecycle.html) so therefore standard C++ aliases can not be
// used, but the want to avoid excessive typing is true.
#define Ros2Lc_CallbackReturn \
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn

namespace rr_state_manager
{

/**
 * @class RrStateSubscriberBase
 * @brief base class for state subscribers
 *
 * Each member of the state service provides a callback that can be overridden
 * in to match the specific service that it implements. This base class
 * provides common functionality that can be used to update the buffer_service (state).
 *
 * Provided in locking mechanics, for a common base method.
 */

class RrStateSubscriberBase : public rclcpp_lifecycle::LifecycleNode
{
 public:
  explicit RrStateSubscriberBase(const std::string& node_name,
                                 std::shared_ptr<std::shared_mutex> mutex,
                                 std::shared_ptr<rr_interfaces::msg::StateFrame> state_frame)
      : rclcpp_lifecycle::LifecycleNode(node_name,
                                        rclcpp::NodeOptions().use_intra_process_comms(true)),
        mutex_(mutex),
        state_frame_(state_frame)
  {
  }

 protected:
  /**
   * @fn pre_check
   * @brief perform pre checks, such as what services are needed before starting.
   */
  virtual bool pre_check() = 0;

  /**
   * @fn init()
   * @brief perform any configuration that is needed by the sub class.
   */
  virtual void init() = 0;

  /**
   * @fn on_configure
   * @brief perform inilization. This should configure the node before any work can be done.
   */
  Ros2Lc_CallbackReturn on_configure(const rclcpp_lifecycle::State&) override;

  // shared mutex for reading and writing from state maintater interface.
  std::shared_ptr<std::shared_mutex> mutex_;

  // state maintainer object. This will be set by the node, during initlization time.
  std::shared_ptr<rr_interfaces::msg::StateFrame> state_frame_;

};
}  // namespace rr_state_manager

#endif  // RR_STATE_SUBSCRIBER_BASE_HPP