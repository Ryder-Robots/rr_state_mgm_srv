#ifndef RR_IMU_SERVICE_HPP
#define RR_IMU_SERVICE_HPP

#include "rr_interfaces/srv/imu.hpp"
#include "rr_state_mgm_srv/rr_service_base.hpp"

namespace rr_state_manager
{

class RrImuService : public RrStateServiceBase
{
 public:
  RrImuService(std::shared_ptr<std::shared_mutex> mutex,
               std::shared_ptr<rr_interfaces::msg::BufferResponse> state)
  {
    init(mutex, state);
  }
  ~RrImuService() = default;

  void set_imu(const std::shared_ptr<rr_interfaces::srv::Imu::Request> request,
               std::shared_ptr<rr_interfaces::srv::Imu::Response> response);
};
}  // namespace rr_state_manager

#endif  // RR_IMU_SERVICE_HPP