#ifndef RR_NAVIGATION_SERVICE_HPP
#define RR_NAVIGATION_SERVICE_HPP

#include "rr_interfaces/srv/navigation.hpp"
#include "rr_state_mgm_srv/rr_service_base.hpp"

namespace rr_state_manager
{
class RrNavigationService : public RrStateServiceBase
{
 public:
  RrNavigationService(std::shared_ptr<std::shared_mutex> mutex,
                      std::shared_ptr<rr_interfaces::msg::BufferResponse> state)
  {
    init(mutex, state);
  }

  ~RrNavigationService() = default;

  void set_nav(const std::shared_ptr<rr_interfaces::srv::Navigation::Request> request,
               std::shared_ptr<rr_interfaces::srv::Navigation::Response> response);
};
}  // namespace rr_state_manager

#endif  // RR_NAVIGATION_SERVICE_HPP