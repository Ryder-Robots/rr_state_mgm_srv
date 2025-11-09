#include "rr_state_mgm_srv/state_manager.hpp"

using namespace rr_state_manager;

using std::placeholders::_1;
using std::placeholders::_2;

/*
 * At this stage this can be overriden, but the policy could be set up to use parameters.
 */
rclcpp::QoS RrStateManagerSrv::configure_qos()
{
  RCLCPP_INFO(logger_, "configuring QoS policy");
  rclcpp::QoS qos_profile(1);
  builtin_interfaces::msg::Duration lifespan_duration;
  lifespan_duration.sec     = 0;
  lifespan_duration.nanosec = 330 * 1000000;
  qos_profile.lifespan(lifespan_duration);
  return qos_profile;
}

/*
 * initialize all recieving services. These will be used to construct the frame.
 */
void RrStateManagerSrv::init_services()
{
  rclcpp::QoS policy = configure_qos();

  RCLCPP_INFO(logger_, "creating services");
  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_BAT_REQ.c_str());
  batt_state_group_     = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_batt_state_svr_ = this->create_service<rr_interfaces::srv::BatteryState>(
      rr_constants_state_mgr::STATE_BAT_REQ,
      std::bind(&RrBatteryStateService::set_batt_state,
                std::make_shared<RrBatteryStateService>(mutex_, state_frame_), _1, _2),
      policy, batt_state_group_);

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_GPS_REQ.c_str());
  gps_state_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_gps_svr_   = this->create_service<rr_interfaces::srv::Gps>(
      rr_constants_state_mgr::STATE_GPS_REQ,
      std::bind(&RrrGpsService::set_gps, std::make_shared<RrrGpsService>(mutex_, state_frame_), _1,
                  _2),
      policy, gps_state_group_);

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_IMG_REQ.c_str());
  img_state_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_img_svr_   = this->create_service<rr_interfaces::srv::Image>(
      rr_constants_state_mgr::STATE_IMG_REQ,
      std::bind(&RrImageService::set_image, std::make_shared<RrImageService>(mutex_, state_frame_),
                  _1, _2),
      policy, img_state_group_);

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_IMU_REQ.c_str());
  imu_state_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_imu_svr_   = this->create_service<rr_interfaces::srv::Imu>(
      rr_constants_state_mgr::STATE_IMU_REQ,
      std::bind(&RrImuService::set_imu, std::make_shared<RrImuService>(mutex_, state_frame_), _1,
                  _2),
      policy, imu_state_group_);

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_JOY_REQ.c_str());
  joy_state_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_joy_req_   = this->create_service<rr_interfaces::srv::Joy>(
      rr_constants_state_mgr::STATE_JOY_REQ,
      std::bind(&RrJoystrickService::set_joystick,
                  std::make_shared<RrJoystrickService>(mutex_, state_frame_), _1, _2),
      policy, joy_state_group_);

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_NAV_REQ.c_str());
  nav_state_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_nav_req_   = this->create_service<rr_interfaces::srv::Navigation>(
      rr_constants_state_mgr::STATE_NAV_REQ,
      std::bind(&RrNavigationService::set_nav,
                  std::make_shared<RrNavigationService>(mutex_, state_frame_), _1, _2),
      policy, nav_state_group_);

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_RNG_REQ.c_str());
  range_state_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  state_range_req_   = this->create_service<rr_interfaces::srv::Range>(
      rr_constants_state_mgr::STATE_RNG_REQ,
      std::bind(&RrRangeService::set_range, std::make_shared<RrRangeService>(mutex_, state_frame_),
                  _1, _2),
      policy, range_state_group_);
}

/*
 * During initalization set all features to 'false' assume that nothing is there until it is set
 * at least once.
 */
void RrStateManagerSrv::init()
{
  RCLCPP_INFO(logger_, "creating state_manager - starting with feature list");
  state_frame_->feature_sets.has_batt_state = false;
  state_frame_->feature_sets.has_gps        = false;
  state_frame_->feature_sets.has_img        = false;
  state_frame_->feature_sets.has_imu        = false;
  state_frame_->feature_sets.has_joy        = false;
  state_frame_->feature_sets.has_ranges     = false;

  // set range to 3. this is hard limit for now.
  state_frame_->ranges.resize(0);

  init_services();

  // publish 3 times per second
  this->declare_parameter<int64_t>("frame_rate", 1000 / 3);
  int64_t frame_rate = this->get_parameter("frame_rate").as_int();
  RCLCPP_INFO(logger_, "creating state_manager - publishing service, publish rate is %ld",
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

  RCLCPP_DEBUG(logger_, "publishing message for frame: %ld", msg_snt_);
  this->publisher_->publish(*state_frame_);
}