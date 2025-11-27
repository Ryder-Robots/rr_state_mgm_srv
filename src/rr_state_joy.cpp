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

#include "rr_state_mgm_srv/rr_state_joy_hpp"

namespace rrobot::state_frame
{
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
    using namespace std::placeholders;

    /*
     * not much to here, which isn't done in header definition, just re-return 
     */
    CallbackReturn RRStateJoyNode::on_configure(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "configuring %s", this->get_name());
        return CallbackReturn::SUCCESS;
    }

    /**
     * Create or re-create callback, setup subscriber.
     */
    CallbackReturn RRStateJoyNode::on_activate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "activating %s", this->get_name());
        rclcpp::SubscriptionOptions options;
        auto callback = std::bind(&RRStateJoyNode::update_state, this, _1);
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(rr_constants::TOPIC_JOY, rclcpp::SensorDataQoS(), callback);

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RRStateJoyNode::on_deactivate(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "deactivating %s", this->get_name());
        subscription_.reset();

        return CallbackReturn::SUCCESS;
    }

    CallbackReturn RRStateJoyNode::on_cleanup(const rclcpp_lifecycle::State &state)
    {
        (void)state;
        RCLCPP_INFO(this->get_logger(), "deactivating %s", this->get_name());
        subscription_.reset();
        return CallbackReturn::SUCCESS;
    }

    void RRStateJoyNode::update_state(const sensor_msgs::msg::Joy::UniquePtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "updating state: %s", this->get_name());
        state_frame_.set_joystick(*msg);
    }
} // namespace rrobot::state_frame

RCLCPP_COMPONENTS_REGISTER_NODE(rrobot::state_frame::RRStateJoyNode);