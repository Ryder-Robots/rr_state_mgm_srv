#include "rr_state_mgm_srv/rr_battery_state_service.hpp"

using namespace rr_state_manager;

// provide service for setting, and/or retrieve battery state
void RrBatteryStateService::set_batt_state(
    const std::shared_ptr<rr_interfaces::srv::BatteryState::Request> request,
    std::shared_ptr<rr_interfaces::srv::BatteryState::Response> response)
{
  if (request->override_state)
  {
    std::unique_lock<std::shared_mutex> lock(*mutex_);
    buffer_response_->feature_sets.has_batt_state = true;
    buffer_response_->batt_state                  = request->batt_state_tx;
  }

  std::shared_lock<std::shared_mutex> lock(*mutex_);
  response->batt_state_rx = buffer_response_->batt_state;
}
