#include "rr_state_mgm_srv/state_manager.hpp"

using namespace rr_state_manager;

// gps setter/getter
void RrStateManagerSrv::set_gps(
    const std::shared_ptr<rr_interfaces::srv::StateGpsReq::Request> request,
    std::shared_ptr<rr_interfaces::srv::StateGpsReq::Response> response)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  buffer_response_.feature_sets.has_gps = true;
  buffer_response_.gps                  = request->gps;

  response->buffer_response = buffer_response_;
}

void RrStateManagerSrv::set_range(
    const std::shared_ptr<rr_interfaces::srv::StateRange::Request> request,
    std::shared_ptr<rr_interfaces::srv::StateRange::Response> response)
{
  std::unique_lock<std::shared_mutex> lock(mutex_);
  sensor_msgs::msg::Range range = request->range;
  if (std::find(RANGES_LINKS_.begin(), RANGES_LINKS_.end(), range.header.frame_id) ==
      RANGES_LINKS_.end())
  {
    RCLCPP_ERROR(logger_, "unsupported range sensor, %s not found in supported",
                 range.header.frame_id.c_str());
  }

  buffer_response_.feature_sets.has_ranges = true;
  for (size_t i = 0; i < buffer_response_.ranges.size(); ++i)
  {
    if (buffer_response_.ranges.at(i).header.frame_id == range.header.frame_id)
    {
      buffer_response_.ranges[i] = range;
      response->buffer_response  = buffer_response_;
      return;
    }
  }

  buffer_response_.ranges.resize(buffer_response_.ranges.size() + 1);
  buffer_response_.ranges[buffer_response_.ranges.size() - 1] = range;
  response->buffer_response                                   = buffer_response_;
}

/*
 * During initalization set all features to 'false' assume that nothing is there until it is set
 * at least once.
 */
void RrStateManagerSrv::init()
{
  buffer_response_.feature_sets.has_batt_state = false;
  buffer_response_.feature_sets.has_gps        = false;
  buffer_response_.feature_sets.has_img        = false;
  buffer_response_.feature_sets.has_imu        = false;
  buffer_response_.feature_sets.has_joy        = false;
  buffer_response_.feature_sets.has_ranges     = false;

  // set range to 3. this is hard limit for now.
  buffer_response_.ranges.resize(0);
}