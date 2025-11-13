#include "rr_state_mgm_srv/rr_joystick_service.hpp"

using namespace rr_state_manager;

void RrJoystrickService::set_joystick(
    const std::shared_ptr<rr_interfaces::srv::Joy::Request> request,
    std::shared_ptr<rr_interfaces::srv::Joy::Response> response)
{
  if (request->override_state)
  {
    std::unique_lock<std::shared_mutex> lock(*mutex_);
    buffer_response_->feature_sets.has_joy = true;
    buffer_response_->joystick             = request->joystick_tx;
  }

  std::shared_lock<std::shared_mutex> lock(*mutex_);
  response->joystick_rx = buffer_response_->joystick;
}