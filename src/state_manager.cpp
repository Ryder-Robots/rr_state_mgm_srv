#include "rr_state_mgm_srv/state_manager.hpp"

using namespace rr_state_manager;

using std::placeholders::_1;
using std::placeholders::_2;

/*
 * initialize all recieving services. These will be used to construct the frame.
 */
void RrStateManagerSrv::init_services()
{
  RCLCPP_INFO(logger_, "creating services");

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_BAT_REQ.c_str());
  state_batt_state_svr_ = this->create_service<rr_interfaces::srv::BatteryState>(
      rr_constants_state_mgr::STATE_BAT_REQ,
      std::bind(&RrBatteryStateService::set_batt_state,
                std::make_shared<RrBatteryStateService>(mutex_, state_frame_), _1, _2));

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_GPS_REQ.c_str());
  state_gps_svr_ = this->create_service<rr_interfaces::srv::Gps>(
      rr_constants_state_mgr::STATE_GPS_REQ,
      std::bind(&RrrGpsService::set_gps, std::make_shared<RrrGpsService>(mutex_, state_frame_), _1, _2));

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_IMU_REQ.c_str());
  state_imu_svr_ = this->create_service<rr_interfaces::srv::Imu>(
      rr_constants_state_mgr::STATE_IMU_REQ,
      std::bind(&RrImuService::set_imu, std::make_shared<RrImuService>(mutex_, state_frame_), _1, _2));

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_JOY_REQ.c_str());
  state_joy_req_ = this->create_service<rr_interfaces::srv::Joy>(
      rr_constants_state_mgr::STATE_JOY_REQ,
      std::bind(&RrJoystrickService::set_joystick, std::make_shared<RrJoystrickService>(mutex_, state_frame_), _1, _2));

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_NAV_REQ.c_str());
  state_nav_req_ = this->create_service<rr_interfaces::srv::Navigation>(
      rr_constants_state_mgr::STATE_NAV_REQ,
      std::bind(&RrNavigationService::set_nav, std::make_shared<RrNavigationService>(mutex_, state_frame_), _1, _2));

  RCLCPP_INFO(logger_, "creating state_manager::%s", rr_constants_state_mgr::STATE_RNG_REQ.c_str());
  state_range_req_ = this->create_service<rr_interfaces::srv::Range>(
      rr_constants_state_mgr::STATE_RNG_REQ,
      std::bind(&RrRangeService::set_range, std::make_shared<RrRangeService>(mutex_, state_frame_), _1, _2));
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
  this->declare_parameter<int>("frame_rate", 1000/3);
  int frame_rate = this->get_parameter("frame_rate").as_int();
  RCLCPP_INFO(logger_, "creating state_manager - publishing service, publish rate is %d", frame_rate);
  publisher_ = this->create_publisher<rr_interfaces::msg::BufferResponse>(rr_constants::TOPIC_STATE_FRAME, frame_rate);
}

/*
 * When called publishes messages to topic for later consumption by higher level services.
 */
void RrStateManagerSrv::publish_callback()
{

}