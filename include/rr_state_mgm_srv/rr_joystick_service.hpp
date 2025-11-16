#ifndef RR_JOYSTICK_SERVICE_HPP
#define RR_JOYSTICK_SERVICE_HPP

#include "rr_interfaces/srv/joy.hpp"
#include "rr_state_mgm_srv/rr_service_base.hpp"

namespace rr_state_manager
{
/**
 *  @deprecated
 */  
class RrJoystrickService : public RrStateServiceBase
{
 public:
  RrJoystrickService(std::shared_ptr<std::shared_mutex> mutex,
                     std::shared_ptr<rr_interfaces::msg::BufferResponse> state)
  {
    init(mutex, state);
  }

  ~RrJoystrickService() = default;

  void set_joystick(const std::shared_ptr<rr_interfaces::srv::Joy::Request> request,
                    std::shared_ptr<rr_interfaces::srv::Joy::Response> response);
};
}  // namespace rr_state_manager

#endif  // RR_JOYSTICK_SERVICE_HPP