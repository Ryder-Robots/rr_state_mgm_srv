#include "rr_state_mgm_srv/rr_range_service.hpp"

using namespace rr_state_manager;

/**
 * algorithm to find specific range sensor.
 */
void RrRangeService::set_range(const std::shared_ptr<rr_interfaces::srv::Range::Request> request,
                               std::shared_ptr<rr_interfaces::srv::Range::Response> response)
{
  sensor_msgs::msg::Range range = request->range_tx;
  if (std::find(RANGES_LINKS_.begin(), RANGES_LINKS_.end(), range.header.frame_id) ==
      RANGES_LINKS_.end())
  {
    RCLCPP_ERROR(logger_, "unsupported range sensor, %s not found in supported",
                 range.header.frame_id.c_str());
  }

  // if it is not an override then attempt to find the range from the frame_id, and return it.
  if (!request->override_state)
  {
    if (!buffer_response_->feature_sets.has_ranges)
    {
      RCLCPP_WARN(logger_, "range not yet set: not looking for %s", range.header.frame_id.c_str());
      return;
    }

    std::string key = range.header.frame_id;
    auto it         = std::find_if(buffer_response_->ranges.begin(), buffer_response_->ranges.end(),
                                   [key](const sensor_msgs::msg::Range& obj)
                                   { return obj.header.frame_id == key; });

    // Not found
    if (it != buffer_response_->ranges.end())
    {
      RCLCPP_WARN(logger_, "could not find range sensor, %s not found in supported",
                  range.header.frame_id.c_str());
      return;
    }

    response->range_rx = *it;
    return;
  }

  // attempt to find the range sensor.
  std::unique_lock<std::shared_mutex> lock(*mutex_);
  buffer_response_->feature_sets.has_ranges = true;
  for (size_t i = 0; i < buffer_response_->ranges.size(); ++i)
  {
    if (buffer_response_->ranges.at(i).header.frame_id == range.header.frame_id)
    {
      buffer_response_->ranges[i] = range;
      response->range_rx          = buffer_response_->ranges[i];
      return;
    }
  }

  // new range sensor added.
  buffer_response_->ranges.resize(buffer_response_->ranges.size() + 1);
  buffer_response_->ranges[buffer_response_->ranges.size() - 1] = range;
  response->range_rx = buffer_response_->ranges[buffer_response_->ranges.size() - 1];
}