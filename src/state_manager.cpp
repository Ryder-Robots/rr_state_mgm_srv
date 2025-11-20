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

#include "rr_state_mgm_srv/state_manager.hpp"

using namespace rr_state_manager::rr_state_manager_node;

/*
 * During initalization set all features to 'false' assume that nothing is there until it is set
 * at least once.
 */
LNI::CallbackReturn RrStateManagerSrv::on_configure(const lc::State& state)
{
  (void)state;
  RCLCPP_INFO(this->get_logger(), "configuring state manager service");
  state_frame_->feature_sets.has_batt_state = false;
  state_frame_->feature_sets.has_gps        = false;
  state_frame_->feature_sets.has_img        = false;
  state_frame_->feature_sets.has_imu        = false;
  state_frame_->feature_sets.has_joy        = false;
  state_frame_->feature_sets.has_ranges     = false;

  // set range to 3. this is hard limit for now.
  state_frame_->ranges.resize(3);

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  return LNI::CallbackReturn::SUCCESS;
}

// activate the publisher. and create the timer
LNI::CallbackReturn RrStateManagerSrv::on_activate(const lc::State& state)
{
  (void)state;
  auto callback = std::bind(&RrStateManagerSrv::get_state_frame, this, std::placeholders::_1, std::placeholders::_2);
  state_service_ = create_service<rr_interfaces::srv::State>("state_manager/get_state_frame", callback, configure_qos(), callback_group_);
  return LNI::CallbackReturn::SUCCESS;
}

// destroy timer explicitly.
LNI::CallbackReturn RrStateManagerSrv::on_deactivate(const lc::State& state)
{
  (void)state;
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn RrStateManagerSrv::on_cleanup(const lc::State& state)
{
  (void)state;
  state_service_.reset();
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn RrStateManagerSrv::on_shutdown(const lc::State& state)
{
  (void)state;
  return LNI::CallbackReturn::SUCCESS;
}

rclcpp::QoS RrStateManagerSrv::configure_qos()
{
  RCLCPP_INFO(get_logger(), "configuring QoS policy");
  rclcpp::QoS qos_profile(1);
  builtin_interfaces::msg::Duration lifespan_duration;
  lifespan_duration.sec     = 0;
  lifespan_duration.nanosec = 330 * 1000000;
  qos_profile.lifespan(lifespan_duration);
  return qos_profile;
}

void RrStateManagerSrv::get_state_frame(const std::shared_ptr<rr_interfaces::srv::State::Request> req,
               const std::shared_ptr<rr_interfaces::srv::State::Response> res)
{
  std::shared_lock<std::shared_mutex> lock(*mutex_);

  boost::uuids::random_generator generator;
  boost::uuids::uuid b_uuid = generator();
  unique_identifier_msgs::msg::UUID guid;
  std::copy(b_uuid.begin(), b_uuid.end(), guid.uuid.begin());

  // set the header.
  state_frame_->header.frame_id = rr_constants::LINK_STATE;
  state_frame_->header.stamp    = this->now();

  // set trace and sequence variables.
  state_frame_->guid = guid;
  state_frame_->request_id = req->request_id;
  state_frame_->seq  = ++msg_snt_;

  res->state_frame = *state_frame_;
}