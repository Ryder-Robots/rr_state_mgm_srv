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

#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "rr_common_base/rr_constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rr_interfaces/msg/state_frame.hpp"
#include "rr_interfaces/srv/state.hpp"


namespace lc = rclcpp_lifecycle;
using LNI    = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;

namespace rr_state_manager
{

namespace rr_state_manager_node
{

/**
 * @class RrStateManagerSrv
 * @brief state management services
 *
 * Sends telementry of current state back to listening services.
 */
class RrStateManagerSrv : public lc::LifecycleNode
{
 public:
  /**
   * @fn RrStateManagerSrv
   * @brief class constructor
   */
  explicit RrStateManagerSrv(const rclcpp::NodeOptions& node_opts,
                             std::shared_ptr<std::shared_mutex> mutex,
                             std::shared_ptr<rr_interfaces::msg::StateFrame> state_frame)
      : lc::LifecycleNode("rr_state_buffer_service", node_opts), mutex_(mutex), state_frame_(state_frame)
  {
  }

  ~RrStateManagerSrv() = default;

  LNI::CallbackReturn on_configure(const lc::State&) override;

  LNI::CallbackReturn on_activate(const lc::State&) override;

  // destroy timer explicitly.
  LNI::CallbackReturn on_deactivate(const lc::State&) override;

  LNI::CallbackReturn on_cleanup(const lc::State&) override;

  LNI::CallbackReturn on_shutdown(const lc::State&) override;

  void get_state_frame(const std::shared_ptr<rr_interfaces::srv::State::Request>, const std::shared_ptr<rr_interfaces::srv::State::Response>);

 private:
  rclcpp::QoS configure_qos();

  rclcpp::Service<rr_interfaces::srv::State>::SharedPtr state_service_;

  // shared mutex for reading and writing from state maintater interface.
  std::shared_ptr<std::shared_mutex> mutex_;

  // state maintainer object. This will be set by the node, during initlization time.
  std::shared_ptr<rr_interfaces::msg::StateFrame> state_frame_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;


  long msg_snt_ = 0;
};
}  // namespace rr_state_manager_node

}  // namespace rr_state_manager

#endif