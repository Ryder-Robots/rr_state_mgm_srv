#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <memory>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "rr_common_base/rr_state_mng_constants.hpp"
#include "rr_interfaces/srv/battery_state.hpp"
#include "rr_interfaces/srv/gps.hpp"
#include "rr_interfaces/srv/image.hpp"
#include "rr_interfaces/srv/imu.hpp"
#include "rr_interfaces/srv/joy.hpp"
#include "rr_interfaces/srv/navigation.hpp"
#include "rr_interfaces/srv/range.hpp"
#include "rr_interfaces/srv/state_response.hpp"
#include "state_validator.hpp"

namespace rr_state_manager
{

/**
 * @class RrStateManagerSrv
 * @brief state management services
 *
 * Creates a state frame for robot, using several frames. The intended use is that subscribers will
 * push the current frame to this service, and it will keep it up to date.
 */
class RrStateManagerSrv : public rclcpp::Node
{
 public:
  RrStateManagerSrv() : Node("rr_state_manager") { init(); }

  // public interfaces beneath

  /**
   * @fn set_gps
   * @brief
   * sets GPS state within the state manager.
   */
  void set_gps(const std::shared_ptr<rr_interfaces::srv::Gps::Request> request,
               std::shared_ptr<rr_interfaces::srv::Gps::Response> response);

  // /**
  //  * @fn set_gps
  //  * @brief
  //  * sets Joystick state within the state manager.
  //  */
  // void set_joystick(const std::shared_ptr<rr_interfaces::srv::Joy::Request> request,
  //                   std::shared_ptr<rr_interfaces::srv::Joy::Response> response);

  // /**
  //  * @fn set_batt_state
  //  * @brief
  //  * sets battery state
  //  */
  // void set_batt_state(const std::shared_ptr<rr_interfaces::srv::BatteryState::Request> request,
  //                     std::shared_ptr<rr_interfaces::srv::BatteryState::Response> response);

  // /**
  //  * @fn set_image
  //  * @brief
  //  * sets current image frame in state manager.
  //  */
  // void set_image(const std::shared_ptr<rr_interfaces::srv::Image::Request> request,
  //                std::shared_ptr<rr_interfaces::srv::Image::Response> response);

  // void set_imu(const std::shared_ptr<rr_interfaces::srv::Imu::Request> request,
  //              std::shared_ptr<rr_interfaces::srv::Imu::Response> response);

  // void set_range(const std::shared_ptr<rr_interfaces::srv::StateRange::Request> request,
  //                std::shared_ptr<rr_interfaces::srv::StateRange::Response> response);

  // /**
  //  * @fn get_state
  //  * @brief returns current state
  //  */
  // void get_state(const std::shared_ptr<rr_interfaces::srv::State::Request> request,
  //                std::shared_ptr<rr_interfaces::srv::State::Response> response);

  ~RrStateManagerSrv() = default;

  // state variables
  rclcpp::Service<rr_interfaces::srv::Gps>::SharedPtr state_gps_req_;
  rclcpp::Service<rr_interfaces::srv::Joy>::SharedPtr state_joy_req_;
  rclcpp::Service<rr_interfaces::srv::BatteryState>::SharedPtr state_bat_req_;
  rclcpp::Service<rr_interfaces::srv::Image>::SharedPtr state_img_req_;

 private:
  long msg_snt_ = 0;
  void init();
  void init_services();
  const std::array<std::string, 3> RANGES_LINKS_ = {rr_constants::LINK_ULTRA_SONIC_CENTER,
                                                    rr_constants::LINK_ULTRA_SONIC_LEFT,
                                                    rr_constants::LINK_ULTRA_SONIC_RIGHT};

  rclcpp::Logger logger_ = rclcpp::get_logger("state_maintainer");
  std::shared_mutex mutex_;
  rr_interfaces::msg::BufferResponse buffer_response_;
  rr_state_validator::RrStateValidator validator_;
};

}  // namespace rr_state_manager

#endif