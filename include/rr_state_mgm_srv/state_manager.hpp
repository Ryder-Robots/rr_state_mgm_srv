#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_base/rr_state_mng_constants.hpp"

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

  // state variables
  rclcpp::Service<rr_interfaces::srv::BatteryState>::SharedPtr state_batt_state_svr_;
  rclcpp::Service<rr_interfaces::srv::Gps>::SharedPtr state_gps_svr_;
  rclcpp::Service<rr_interfaces::srv::Image>::SharedPtr state_img_svr_;
  rclcpp::Service<rr_interfaces::srv::Imu>::SharedPtr state_imu_svr_;
  rclcpp::Service<rr_interfaces::srv::Joy>::SharedPtr state_joy_req_;
  rclcpp::Service<rr_interfaces::srv::Navigation>::SharedPtr state_nav_req_;
  rclcpp::Service<rr_interfaces::srv::Range>::SharedPtr state_range_req_;

 private:
  long msg_snt_ = 0;
  void init();
  void init_services();

  rclcpp::Logger logger_ = rclcpp::get_logger("state_maintainer");
  rr_state_validator::RrStateValidator validator_;

  // controls threads for state writing, and reading.
  std::shared_ptr<std::shared_mutex> mutex_ = std::make_shared<std::shared_mutex>();

  // current state frame.
  std::shared_ptr<rr_interfaces::msg::BufferResponse> state_frame_ =
      std::make_shared<rr_interfaces::msg::BufferResponse>();
};

}  // namespace rr_state_manager

#endif