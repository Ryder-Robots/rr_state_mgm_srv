#ifndef STATE_MANAGER_HPP
#define STATE_MANAGER_HPP

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rr_interfaces/srv/state_batt_req.hpp"
#include "rr_interfaces/srv/state_gps_req.hpp"
#include "rr_interfaces/srv/state_image.hpp"
#include "rr_interfaces/srv/state_imu.hpp"
#include "rr_interfaces/srv/state_joy_req.hpp"
#include "rr_interfaces/srv/state_left_front_range.hpp"
#include "rr_interfaces/srv/state_middle_front_range.hpp"
#include "rr_interfaces/srv/state_response.hpp"
#include "rr_interfaces/srv/state_right_front_range.hpp"

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
  void set_gps(const std::shared_ptr<rr_interfaces::srv::StateGpsReq::Request> request,
               std::shared_ptr<rr_interfaces::srv::StateGpsReq::Response> response);

  /**
   * @fn set_gps
   * @brief
   * sets Joystick state within the state manager.
   */
  void set_joystick(const std::shared_ptr<rr_interfaces::srv::StateJoyReq::Request> request,
                    std::shared_ptr<rr_interfaces::srv::StateJoyReq::Response> response);

  /**
   * @fn set_batt_state
   * @brief
   * sets Battery state within the state manager.
   */
  void set_batt_state(const std::shared_ptr<rr_interfaces::srv::StateBattReq::Request> request,
                      std::shared_ptr<rr_interfaces::srv::StateBattReq::Response> response);

  /**
   * @fn set_image
   * @brief
   * sets current image frame in state manager.
   */
  void set_image(const std::shared_ptr<rr_interfaces::srv::StateImage::Request> request,
                 std::shared_ptr<rr_interfaces::srv::StateImage::Response> response);

  void set_imu(const std::shared_ptr<rr_interfaces::srv::StateImu::Request> request,
               std::shared_ptr<rr_interfaces::srv::StateImu::Response> response);

  /**
   * @fn get_state
   * @brief returns current state
   */
  void get_state(const std::shared_ptr<rr_interfaces::srv::StateResponse::Request> request,
                 std::shared_ptr<rr_interfaces::srv::StateResponse::Response> response);

  ~RrStateManagerSrv() = default;

 private:
  void init();
  rclcpp::Logger logger_ = rclcpp::get_logger("state_maintainer");
  std::shared_mutex mutex_;

  rr_interfaces::msg::BufferResponse buffer_response_;
};

}  // namespace rr_state_manager

#endif