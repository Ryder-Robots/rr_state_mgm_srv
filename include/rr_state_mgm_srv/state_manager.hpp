#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <functional>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_base/rr_state_mng_constants.hpp"
#include "rr_interfaces/msg/buffer_response.hpp"
#include "rr_state_mgm_srv/rr_battery_state_service.hpp"
#include "rr_state_mgm_srv/rr_gps_service.hpp"
#include "rr_state_mgm_srv/rr_image_service.hpp"
#include "rr_state_mgm_srv/rr_imu_service.hpp"
#include "rr_state_mgm_srv/rr_joystick_service.hpp"
#include "rr_state_mgm_srv/rr_navigation_service.hpp"
#include "rr_state_mgm_srv/rr_range_service.hpp"
#include "state_validator.hpp"

namespace rr_state_manager
{

/**
 * @class RrStateManagerSrv
 * @brief state management services
 *
 * Creates a state frame for robot, using several frames. Each state frame is published to the
 * state topic. lower level nodes will communicate with the various services this node initlizes.
 *
 * Each state frame based upon a timer for consumption of mid level, and higher level services.
 */
class RrStateManagerSrv : public rclcpp::Node
{
 public:
  RrStateManagerSrv() : Node("rr_state_manager") { init(); }
  ~RrStateManagerSrv() = default;

  const std::shared_ptr<std::shared_mutex> get_mutex(){return mutex_;}
  const std::shared_ptr<rr_interfaces::msg::BufferResponse> get_state_frame() {return state_frame_;}

  // state variables
  rclcpp::Service<rr_interfaces::srv::BatteryState>::SharedPtr state_batt_state_svr_;
  rclcpp::Service<rr_interfaces::srv::Gps>::SharedPtr state_gps_svr_;
  rclcpp::Service<rr_interfaces::srv::Image>::SharedPtr state_img_svr_;
  rclcpp::Service<rr_interfaces::srv::Imu>::SharedPtr state_imu_svr_;
  rclcpp::Service<rr_interfaces::srv::Joy>::SharedPtr state_joy_req_;
  rclcpp::Service<rr_interfaces::srv::Navigation>::SharedPtr state_nav_req_;
  rclcpp::Service<rr_interfaces::srv::Range>::SharedPtr state_range_req_;

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

 private:
  void init();
  void init_services();

  // TODO: these should be moved to there corresponding nodes, to loosen up the coupling.
  // service and publisher groups
  rclcpp::CallbackGroup::SharedPtr batt_state_group_;
  rclcpp::CallbackGroup::SharedPtr gps_state_group_;
  rclcpp::CallbackGroup::SharedPtr img_state_group_;
  rclcpp::CallbackGroup::SharedPtr imu_state_group_;
  rclcpp::CallbackGroup::SharedPtr joy_state_group_;
  rclcpp::CallbackGroup::SharedPtr nav_state_group_;
  rclcpp::CallbackGroup::SharedPtr range_state_group_;
  rclcpp::CallbackGroup::SharedPtr publish_group_;

  // publishes state to topic for consumption.
  void publish_callback();

  rclcpp::Logger logger_ = rclcpp::get_logger("state_maintainer");
  rr_state_validator::RrStateValidator validator_;

  // controls threads for state writing, and reading.
  std::shared_ptr<std::shared_mutex> mutex_ = std::make_shared<std::shared_mutex>();

  // current state frame.
  std::shared_ptr<rr_interfaces::msg::BufferResponse> state_frame_ =
      std::make_shared<rr_interfaces::msg::BufferResponse>();

  // controls the topic publisher.
  long msg_snt_ = 0;  // controls sequence number for each published frame.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<rr_interfaces::msg::BufferResponse>::SharedPtr publisher_;
};

}  // namespace rr_state_manager

#endif