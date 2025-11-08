#ifndef RR_IMAGE_SERVICE_HPP
#define RR_IMAGE_SERVICE_HPP


#include <shared_mutex>
#include "rclcpp/rclcpp.hpp"
#include "rr_interfaces/srv/image.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"

namespace rr_state_manager
{
class RrImageService
{

  /**
   * @fn set_gps
   * @brief callback function for sending/recieving Image objects
   * @param request inbound request
   * @param response outbound response
   */
  void set_image(const std::shared_ptr<rr_interfaces::srv::Image::Request> request,
               std::shared_ptr<rr_interfaces::srv::Image::Response> response);

  /**
   * @fn init
   * @brief initlizes the service
   * @param mutex to use
   * @param buffer_response global state object
   */
  void init(std::shared_ptr<std::shared_mutex> mutex,
            std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response);

 private:
  // shared mutex for reading and writing from state maintater interface.
  std::shared_ptr<std::shared_mutex> mutex_;

  // logging object for the service, note this shoudl be overriden by service object, during
  // construction time.
  rclcpp::Logger logger_ = rclcpp::get_logger("state_maintainer");

  // state maintainer object. This will be set by the node, during initlization time.
  std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response_;
};
}  // namespace rr_state_manager

#endif  // RR_IMAGE_SERVICE_HPP