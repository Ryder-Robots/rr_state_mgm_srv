#ifndef RR_GPS_SERVICE_HPP
#define RR_GPS_SERVICE_HPP

#include "rr_interfaces/srv/gps.hpp"
#include "rr_state_mgm_srv/rr_service_base.hpp"

namespace rr_state_manager
{
/**
 * @class RrrGpsService
 * @brief triggered when GPS is read or written to state maintainer
 */
class RrrGpsService : public RrStateServiceBase
{
 public:
  RrrGpsService(std::shared_ptr<std::shared_mutex> mutex,
                std::shared_ptr<rr_interfaces::msg::BufferResponse> state)
  {
    init(mutex, state);
  }
  ~RrrGpsService() = default;

  /**
   * @fn set_gps
   * @brief callback function for sending/recieving GPS objects
   * @param request inbound request
   * @param response outbound response
   */
  void set_gps(const std::shared_ptr<rr_interfaces::srv::Gps::Request> request,
               std::shared_ptr<rr_interfaces::srv::Gps::Response> response);
};
}  // namespace rr_state_manager

#endif