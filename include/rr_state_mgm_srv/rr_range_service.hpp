#ifndef RR_RANGE_SERVICE_HPP
#define RR_RANGE_SERVICE_HPP

#include <algorithm>

#include "rr_common_base/rr_constants.hpp"
#include "rr_interfaces/srv/range.hpp"
#include "rr_state_mgm_srv/rr_service_base.hpp"

namespace rr_state_manager
{

/**
 * @deprecated
 */  
class RrRangeService : public RrStateServiceBase
{
 public:
  RrRangeService(std::shared_ptr<std::shared_mutex> mutex,
                 std::shared_ptr<rr_interfaces::msg::BufferResponse> state)
  {
    init(mutex, state);
  }

  ~RrRangeService() = default;

  /**
   * @fn set_range
   * @brief set or retrieve range.
   *
   * @param request, the range sensor to look for if override is set to false, or it will update or
   * create a new range
   * @param response, the range found in the request, or the range that has just been set.
   *
   * When called depending on what state_override is set too (either true or false), the algorithm
   * will check for existing ranges with the same frame id as the one in the request. note that only
   * one range sensor with a specific frame_id may be set,  if a request with the same frame_id is
   * sent again and state_override is set to true, then that range will be overridden.
   */
  void set_range(const std::shared_ptr<rr_interfaces::srv::Range::Request> request,
                 std::shared_ptr<rr_interfaces::srv::Range::Response> response);

 private:
  const std::array<std::string, 3> RANGES_LINKS_ = {rr_constants::LINK_ULTRA_SONIC_CENTER,
                                                    rr_constants::LINK_ULTRA_SONIC_LEFT,
                                                    rr_constants::LINK_ULTRA_SONIC_RIGHT};
};
}  // namespace rr_state_manager

#endif  // RR_RANGE_SERVICE_HPP