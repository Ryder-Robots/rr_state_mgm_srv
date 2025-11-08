#ifndef RR_BATTERY_STATE_SERVICE_HPP
#define RR_BATTERY_STATE_SERVICE_HPP

#include "rr_state_mgm_srv/rr_state_base.hpp"
#include "rr_interfaces/srv/battery_state.hpp"

namespace rr_state_manager
{
class RrBatteryStateService : public RrStateManagerBase
{
 public:

  /**
   * @fn set_batt_state
   * @brief callback to set battery state
   * @param request inbound request
   * @param response outbound response
   */
  void set_batt_state(const std::shared_ptr<rr_interfaces::srv::BatteryState::Request> request,
                      std::shared_ptr<rr_interfaces::srv::BatteryState::Response> response);
};
}  // namespace rr_state_manager

#endif