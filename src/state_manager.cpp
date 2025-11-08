#include "rr_state_mgm_srv/state_manager.hpp"

using namespace rr_state_manager;

using std::placeholders::_1;
using std::placeholders::_2;

/**
 *  common template that will control locking.
 */
template <typename T, typename R>
void RrStateManagerSrv::set_state(const std::shared_ptr<T>& request, std::shared_ptr<R>& response,
                                  const std::function<void(const std::shared_ptr<T>&)>& cb1,
                                  const std::function<void(std::shared_ptr<R>&)>& cb2,
                                  const bool override_state)
{
  if (override_state)
  {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    cb1(request);
  }
  std::shared_lock<std::shared_mutex> lock(mutex_);
  cb2(response);
}

// gps setter/getter

// provide service for setting, and/or retrieve battery state
void RrStateManagerSrv::set_batt_state(
    const std::shared_ptr<rr_interfaces::srv::BatteryState::Request> request,
    std::shared_ptr<rr_interfaces::srv::BatteryState::Response> response)
{
  auto cb1 = [this](const std::shared_ptr<rr_interfaces::srv::BatteryState::Request>& request)
  {
    buffer_response_.feature_sets.has_batt_state = true;
    buffer_response_.batt_state                  = request->batt_state_tx;
  };

  auto cb2 = [this](std::shared_ptr<rr_interfaces::srv::BatteryState::Response>& response)
  { response->batt_state_rx = buffer_response_.batt_state; };

  set_state<rr_interfaces::srv::BatteryState::Request, rr_interfaces::srv::BatteryState::Response>(
      request, response, cb1, cb2, request->override_state);
}

// provide service for setting, and/or retrieve gps
void RrStateManagerSrv::set_gps(const std::shared_ptr<rr_interfaces::srv::Gps::Request> request,
                                std::shared_ptr<rr_interfaces::srv::Gps::Response> response)
{

   auto cb1 = [this](const std::shared_ptr<rr_interfaces::srv::Gps::Request>& request)
  {
    buffer_response_.feature_sets.has_gps = true;
    buffer_response_.gps                  = request->gps_tx;
  };

  auto cb2 = [this](std::shared_ptr<rr_interfaces::srv::Gps::Response>& response)
  { response->gps_rx = buffer_response_.gps; };
  
  set_state<rr_interfaces::srv::Gps::Request, rr_interfaces::srv::Gps::Response>(
      request, response, cb1, cb2, request->override_state);
}

// void RrStateManagerSrv::set_joystick(
//     const std::shared_ptr<rr_interfaces::srv::StateJoyReq::Request> request,
//     std::shared_ptr<rr_interfaces::srv::StateJoyReq::Response> response)
// {
//   std::unique_lock<std::shared_mutex> lock(mutex_);
//   buffer_response_.feature_sets.has_joy = true;
//   buffer_response_.joystick             = request->joystick;
//   response->buffer_response             = buffer_response_;
// }

// void RrStateManagerSrv::set_image(
//     const std::shared_ptr<rr_interfaces::srv::StateImage::Request> request,
//     std::shared_ptr<rr_interfaces::srv::StateImage::Response> response)
// {
//   std::unique_lock<std::shared_mutex> lock(mutex_);
//   buffer_response_.feature_sets.has_img = true;
//   buffer_response_.img                  = request->img;
//   response->buffer_response             = buffer_response_;
// }

// void RrStateManagerSrv::set_imu(
//     const std::shared_ptr<rr_interfaces::srv::StateImu::Request> request,
//     std::shared_ptr<rr_interfaces::srv::StateImu::Response> response)
// {
//   std::unique_lock<std::shared_mutex> lock(mutex_);
//   buffer_response_.feature_sets.has_imu = true;
//   buffer_response_.imu                  = request->imu;
//   response->buffer_response             = buffer_response_;
// }

// void RrStateManagerSrv::set_range(
//     const std::shared_ptr<rr_interfaces::srv::StateRange::Request> request,
//     std::shared_ptr<rr_interfaces::srv::StateRange::Response> response)
// {
//   std::unique_lock<std::shared_mutex> lock(mutex_);
//   sensor_msgs::msg::Range range = request->range;
//   if (std::find(RANGES_LINKS_.begin(), RANGES_LINKS_.end(), range.header.frame_id) ==
//       RANGES_LINKS_.end())
//   {
//     RCLCPP_ERROR(logger_, "unsupported range sensor, %s not found in supported",
//                  range.header.frame_id.c_str());
//   }

//   buffer_response_.feature_sets.has_ranges = true;
//   for (size_t i = 0; i < buffer_response_.ranges.size(); ++i)
//   {
//     if (buffer_response_.ranges.at(i).header.frame_id == range.header.frame_id)
//     {
//       buffer_response_.ranges[i] = range;
//       response->buffer_response  = buffer_response_;
//       return;
//     }
//   }

//   buffer_response_.ranges.resize(buffer_response_.ranges.size() + 1);
//   buffer_response_.ranges[buffer_response_.ranges.size() - 1] = range;
//   response->buffer_response                                   = buffer_response_;
// }

// /*
//  * Set UUID and a few other variables to identify the buffer. To track the request there is some
//  * details that be recorded such as trace_guid, and span_guid.
//  *
//  * The span GUID can be set here.
//  */
// void RrStateManagerSrv::get_state(
//     const std::shared_ptr<rr_interfaces::srv::StateResponse::Request> request,
//     std::shared_ptr<rr_interfaces::srv::StateResponse::Response> response)
// {
//   if (!validator_.validate_uuid(request->buffer_request.request_id, false))
//   {
//     RCLCPP_WARN(this->logger_, "request recieved that did not have a request id");
//   }

//   std::shared_lock<std::shared_mutex> lock(mutex_);
//   boost::uuids::uuid boost_uuid = boost::uuids::random_generator()();
//   unique_identifier_msgs::msg::UUID uuid_msg;
//   std::copy(boost_uuid.begin(), boost_uuid.end(), uuid_msg.uuid.begin());
//   buffer_response_.guid = uuid_msg;
//   buffer_response_.header.stamp = this->now();
//   buffer_response_.header.frame_id = rr_constants::LINK_STATE;
//   msg_snt_++;
//   RCLCPP_DEBUG(logger_, "sending current state sequence: %ld", msg_snt_);
//   buffer_response_.seq = msg_snt_;
//   response->buffer_response = buffer_response_;
// }

void RrStateManagerSrv::init_services()
{
  //   RCLCPP_INFO(logger_, "creating services");
  //   RCLCPP_INFO(logger_, "creating state_manager::%s",
  //   rr_constants_state_mgr::STATE_GPS_REQ.c_str()); state_gps_req_ =
  //   this->create_service<rr_interfaces::srv::StateGpsReq>(
  //       rr_constants_state_mgr::STATE_GPS_REQ, std::bind(&RrStateManagerSrv::set_gps, this, _1,
  //       _2));

  //   RCLCPP_INFO(logger_, "creating state_manager::%s",
  //   rr_constants_state_mgr::STATE_JOY_REQ.c_str()); state_joy_req_ =
  //   this->create_service<rr_interfaces::srv::StateJoyReq>(
  //       rr_constants_state_mgr::STATE_JOY_REQ,
  //       std::bind(&RrStateManagerSrv::set_joystick, this, _1, _2));

  //   RCLCPP_INFO(logger_, "creating state_manager::%s",
  //   rr_constants_state_mgr::STATE_BAT_REQ.c_str()); state_bat_req_ =
  //   this->create_service<rr_interfaces::srv::StateBattReq>(
  //       rr_constants_state_mgr::STATE_BAT_REQ,
  //       std::bind(&RrStateManagerSrv::set_batt_state, this, _1, _2));

  //   RCLCPP_INFO(logger_, "creating state_manager::%s",
  //   rr_constants_state_mgr::STATE_IMG_REQ.c_str()); state_img_req_ =
  //   this->create_service<rr_interfaces::srv::StateImage>(
  //       rr_constants_state_mgr::STATE_IMG_REQ,
  //       std::bind(&RrStateManagerSrv::set_image, this, _1, _2));
}

/*
 * During initalization set all features to 'false' assume that nothing is there until it is set
 * at least once.
 */
void RrStateManagerSrv::init()
{
  RCLCPP_INFO(logger_, "creating state_manager - starting with feature list");
  buffer_response_.feature_sets.has_batt_state = false;
  buffer_response_.feature_sets.has_gps        = false;
  buffer_response_.feature_sets.has_img        = false;
  buffer_response_.feature_sets.has_imu        = false;
  buffer_response_.feature_sets.has_joy        = false;
  buffer_response_.feature_sets.has_ranges     = false;

  // set range to 3. this is hard limit for now.
  buffer_response_.ranges.resize(0);

  init_services();
}