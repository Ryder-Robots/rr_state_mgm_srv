#include "rr_state_mgm_srv/state_manager.hpp"

using namespace rr_state_manager;

using std::placeholders::_1;
using std::placeholders::_2;

/*
 * At this stage this can be overriden, but the policy could be set up to use parameters.
* Potentially this could be derived from the frame rate.
 */
rclcpp::QoS RrStateManagerSrv::configure_qos()
{
  RCLCPP_INFO(this->get_logger(), "configuring QoS policy");
  rclcpp::QoS qos_profile(1);
  builtin_interfaces::msg::Duration lifespan_duration;
  lifespan_duration.sec     = 0;
  lifespan_duration.nanosec = 330 * 1000000;
  qos_profile.lifespan(lifespan_duration);
  return qos_profile;
}

/*
 * During initalization set all features to 'false' assume that nothing is there until it is set
 * at least once.
 */
void RrStateManagerSrv::init()
{
  RCLCPP_INFO(this->get_logger(), "creating state_manager - starting with feature list");
  state_frame_->feature_sets.has_batt_state = false;
  state_frame_->feature_sets.has_gps        = false;
  state_frame_->feature_sets.has_img        = false;
  state_frame_->feature_sets.has_imu        = false;
  state_frame_->feature_sets.has_joy        = false;
  state_frame_->feature_sets.has_ranges     = false;

  // set range to 3. this is hard limit for now.
  state_frame_->ranges.resize(0);

  // publish 3 times per second
  this->declare_parameter<int64_t>("frame_rate", 1000 / 3);
  int64_t frame_rate = this->get_parameter("frame_rate").as_int();
  RCLCPP_INFO(this->get_logger(), "creating state_manager - publishing service, publish rate is %ld",
              frame_rate);
  publisher_ = this->create_publisher<rr_interfaces::msg::BufferResponse>(
      rr_constants::TOPIC_STATE_FRAME, frame_rate);
  auto timer_callback = std::bind(&RrStateManagerSrv::publish_callback, this);
  publish_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); 
  auto ticks = std::chrono::duration<int, std::milli>(frame_rate);
  timer_ = this->create_wall_timer(ticks, timer_callback, publish_group_);
}

/*
 * When called publishes messages to topic for later consumption by higher level services.
 */
void RrStateManagerSrv::publish_callback()
{
  boost::uuids::random_generator generator;
  boost::uuids::uuid b_uuid = generator();
  unique_identifier_msgs::msg::UUID guid;
  std::copy(b_uuid.begin(), b_uuid.end(), guid.uuid.begin());

  // set the header.
  state_frame_->header.frame_id = rr_constants::LINK_STATE;
  state_frame_->header.stamp    = this->now();

  // set trace and sequence variables.
  state_frame_->guid = guid;
  state_frame_->seq  = ++msg_snt_;

  RCLCPP_DEBUG(this->get_logger(), "publishing message for frame: %ld", msg_snt_);
  this->publisher_->publish(*state_frame_);
}

// no pre-checks are getting performed at this stage.
bool RrStateManagerSrv::pre_check() {
    return true;
}