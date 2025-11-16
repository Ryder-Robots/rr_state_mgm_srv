#ifndef RR_BATTERY_STATE_SERVICE_HPP
#define RR_BATTERY_STATE_SERVICE_HPP

#include "rr_interfaces/srv/battery_state.hpp"
#include "rr_state_mgm_srv/rr_service_base.hpp"

namespace rr_state_manager
{
/**
 * @deprecated
 */  
class RrBatteryStateService : public RrStateServiceBase
{
 public:
  RrBatteryStateService(std::shared_ptr<std::shared_mutex> mutex,
                        std::shared_ptr<rr_interfaces::msg::BufferResponse> state)
  {
    init(mutex, state);
  }
  ~RrBatteryStateService() = default;

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