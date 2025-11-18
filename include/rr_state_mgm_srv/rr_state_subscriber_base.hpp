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
#include "rclcpp/rclcpp.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"

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

class RrStateSubscriberBase : public rclcpp::Node
{
 public:
  explicit RrStateSubscriberBase(const std::string& node_name,
                                 std::shared_ptr<std::shared_mutex> mutex,
                                 std::shared_ptr<rr_interfaces::msg::BufferResponse> state_frame)
      : Node(node_name), mutex_(mutex), state_frame_(state_frame)
  {
  }

 protected:


  // shared mutex for reading and writing from state maintater interface.
  std::shared_ptr<std::shared_mutex> mutex_;

  // state maintainer object. This will be set by the node, during initlization time.
  std::shared_ptr<rr_interfaces::msg::BufferResponse> state_frame_;
};
}  // namespace rr_state_manager

#endif  // RR_STATE_SUBSCRIBER_BASE_HPP