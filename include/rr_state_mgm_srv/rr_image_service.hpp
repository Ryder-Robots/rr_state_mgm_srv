#ifndef RR_IMAGE_SERVICE_HPP
#define RR_IMAGE_SERVICE_HPP


#include "rr_state_mgm_srv/rr_service_base.hpp"
#include "rr_interfaces/srv/image.hpp"


namespace rr_state_manager
{
class RrImageService : public RrStateServiceBase
{

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