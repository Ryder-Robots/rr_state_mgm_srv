#ifndef RR_STATE_BASE_HPP
#define RR_STATE_BASE_HPP

#include <shared_mutex>
#include <functional>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"

namespace rr_state_manager
{

/**
 * @class RrStateBase
 * @brief
 * provides base class for services provided by the state maintainer
 */

class RrStateManagerBase
{
 public:
  void init(std::shared_ptr<std::shared_mutex> mutex,
            std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response);

 protected:
  /**
   * @fn set_state
   * @brief called by callback method of service, controls the write to state maintainer object.
   */
  template <typename T, typename R>
  void set_state(const std::shared_ptr<T>& request, std::shared_ptr<R>& response,
                 const std::function<void(const std::shared_ptr<T>&)>& cb1,
                 const std::function<void(std::shared_ptr<R>&)>& cb2, const bool override_state);

  // shared mutex for reading and writing from state maintater interface.
  std::shared_ptr<std::shared_mutex> mutex_;

  // logging object for the service, note this shoudl be overriden by service object, during
  // construction time.
  rclcpp::Logger logger_ = rclcpp::get_logger("state_maintainer");

  // state maintainer object. This will be set by the node, during initlization time.
  std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response_;
};
}  // namespace rr_state_manager

#endif