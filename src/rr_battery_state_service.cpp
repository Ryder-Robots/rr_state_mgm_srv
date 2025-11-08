#include "rr_state_mgm_srv/rr_battery_state_service.hpp"

using namespace rr_state_manager;

// provide service for setting, and/or retrieve battery state
void RrBatteryStateService::set_batt_state(
    const std::shared_ptr<rr_interfaces::srv::BatteryState::Request> request,
    std::shared_ptr<rr_interfaces::srv::BatteryState::Response> response)
{
  auto cb1 = [this](const std::shared_ptr<rr_interfaces::srv::BatteryState::Request>& request)
  {
    buffer_response_->feature_sets.has_batt_state = true;
    buffer_response_->batt_state                  = request->batt_state_tx;
  };

  auto cb2 = [this](std::shared_ptr<rr_interfaces::srv::BatteryState::Response>& response)
  { response->batt_state_rx = buffer_response_->batt_state; };

  set_state<rr_interfaces::srv::BatteryState::Request, rr_interfaces::srv::BatteryState::Response>(
      request, response, cb1, cb2, request->override_state);
}