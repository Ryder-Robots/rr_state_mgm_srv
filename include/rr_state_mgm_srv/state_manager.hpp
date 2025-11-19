#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"
#include "rr_state_mgm_srv/rr_state_subscriber_base.hpp"
#include "state_validator.hpp"

namespace rr_state_manager
{

/**
 * @class RrStateManagerSrv
 * @brief state management services
 *
 * Sends telementry of current state back to listening services.
 */
class RrStateManagerSrv : public RrStateSubscriberBase
{
 public:
  /**
   * @fn RrStateManagerSrv
   * @brief class constructor
   */
  RrStateManagerSrv(std::shared_ptr<std::shared_mutex> mutex,
                    std::shared_ptr<rr_interfaces::msg::BufferResponse> state_frame)
      : RrStateSubscriberBase("rr_state_publisher", mutex, state_frame)
  {
  }

  ~RrStateManagerSrv() = default;

 protected:
  /**
   * @fn configure_qos
   * @brief defines the quality of service
   *
   * QOS can be overridden, but does not have to be. This could be modified to use
   * parameters at a later date, but for now it remains something tha can be
   * overloaded, but hopefully sensible.
   */
  virtual rclcpp::QoS configure_qos();

 protected:
  bool pre_check() override;
  void init() override;

  // activate the publisher. and create the timer
  CallbackReturn on_activate(const rclcpp_lifecycle::State&) override;

  // destroy timer explicitly.
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State&) override;

  // publishes state to topic for consumption.
  void publish_callback();

 private:
  rclcpp::CallbackGroup::SharedPtr publish_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<rr_interfaces::msg::BufferResponse>::SharedPtr publisher_;
  long msg_snt_ = 0;
  
  // default frame rate.
  int64_t frame_rate_ = 1000 / 3;
};

}  // namespace rr_state_manager

#endif