#ifndef RR_STATE_SUBSCRIBER_BASE_HPP
#define RR_STATE_SUBSCRIBER_BASE_HPP

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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
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
template <typename T>
class RrStateSubscriberBase : public rclcpp_lifecycle::LifecycleNode
{
 public:
  explicit RrStateSubscriberBase(const std::string& node_name)
      : rclcpp_lifecycle::LifecycleNode(node_name)
  {
  }

  /**
   * @fn configure_qos
   * @brief configures profile for node, hopefully defaults are sensible, but it can be overridden.
   */
  virtual rclcpp::QoS configure_qos();

  void callback_around(const T message);

  /**
   * @fn on_configure
   * @brief override lifecycle on_configure method
   *
   * Create base subscriber, this will be used for all subscriber nodes.
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& state) override;

  virtual const std::string get_topic() = 0;

 private:
  typename rclcpp::Subscription<T>::SharedPtr subscription_;
};
// template <typename T>
// class RrStateSubscriberBase : public rclcpp::Node
// {
//  public:
//   /**
//    * @fn RrStateSubscriberBase
//    * @brief class constructor, node name is the sub class node.
//    */
//   explicit RrStateSubscriberBase(const std::string& node_name) : rclcpp::Node(node_name) {}

//   /**
//    * @fn  ~RrStateSubscriberBase
//    * @brief class deconstructor.
//    */
//   ~RrStateSubscriberBase() = default;

//   /**
//    * @fn init
//    * @brief initlizes the service
//    * @param mutex to use
//    * @param buffer_response global state object
//    */
//   void init(std::shared_ptr<std::shared_mutex> mutex,
//             std::shared_ptr<rr_interfaces::msg::BufferResponse> state_frame);

//   /**
//    * @fn configure_qos
//    * @brief configures profile for node, hopefully defaults are sensible, but it can be
//    overridden.
//    */
//   virtual rclcpp::QoS configure_qos();

//   /**
//    * @fn get_topic
//    * @brief topic for the callback.
//    */
//   virtual std::string get_topic() = 0;

//   /**
//    * @fn callback_around
//    * @brief called as part of the callback and adds thread safety.
//    */
//   void callback_around(const T message);

//   /**
//    * @fn callback
//    * @brief implemented by sub-class that act on the event.
//    */
//   virtual void callback(const T message) = 0;

//  protected:
//   // shared mutex for reading and writing from state maintater interface.
//   std::shared_ptr<std::shared_mutex> mutex_;

//   // state maintainer object. This will be set by the node, during initlization time.
//   std::shared_ptr<rr_interfaces::msg::BufferResponse> state_frame_;

//   // controls the topic publisher.
//   long msg_snt_     = 0;  // frames that have been recieved
//   long msg_dropped_ = 0;  // frames that have been dropped
//   rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::CallbackGroup::SharedPtr node_cb_group_;
//   typename rclcpp::Subscription<T> subscription_;
// };
}  // namespace rr_state_manager

#endif  // RR_STATE_SUBSCRIBER_BASE_HPP