#ifndef RR_IMAGE_SERVICE_HPP
#define RR_IMAGE_SERVICE_HPP

#include "rr_interfaces/srv/image.hpp"
#include "rr_state_mgm_srv/rr_service_base.hpp"

namespace rr_state_manager
{
class RrImageService : public RrStateServiceBase
{
 public:
  RrImageService(std::shared_ptr<std::shared_mutex> mutex,
                 std::shared_ptr<rr_interfaces::msg::BufferResponse> state)
  {
    init(mutex, state);
  }

  ~RrImageService() = default;

  /**
   * @fn set_gps
   * @brief callback function for sending/recieving Image objects
   * @param request inbound request
   * @param response outbound response
   */
  void set_image(const std::shared_ptr<rr_interfaces::srv::Image::Request> request,
                 std::shared_ptr<rr_interfaces::srv::Image::Response> response);
};
}  // namespace rr_state_manager

#endif  // RR_IMAGE_SERVICE_HPP