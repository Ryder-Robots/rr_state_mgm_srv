#ifndef RR_SERVICE_BASE_HPP
#define RR_SERVICE_BASE_HPP

#include <shared_mutex>

#include "rclcpp/rclcpp.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"

namespace rr_state_manager
{

/**
 * @class RrStateServiceBase
 *
 * @brief base class for state manager services.
 */
class RrStateServiceBase
{
 public:
  /**
   * @fn init
   * @brief initlizes the service
   * @param mutex to use
   * @param buffer_response global state object
   */
  void init(std::shared_ptr<std::shared_mutex> mutex,
            std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response);

    

 protected:
  // shared mutex for reading and writing from state maintater interface.
  std::shared_ptr<std::shared_mutex> mutex_;

  // logging object for the service, note this shoudl be overriden by service object, during
  // construction time.
  rclcpp::Logger logger_ = rclcpp::get_logger("state_maintainer");

  // state maintainer object. This will be set by the node, during initlization time.
  std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response_;

  // use the following method for concrete class level initialization
  virtual void on_start_up() {}
};
}  // namespace rr_state_manager

#endif