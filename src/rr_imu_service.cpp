#include "rr_state_mgm_srv/rr_imu_service.hpp"

using namespace rr_state_manager;

void RrImuService::set_imu(const std::shared_ptr<rr_interfaces::srv::Imu::Request> request,
                           std::shared_ptr<rr_interfaces::srv::Imu::Response> response)
{
  if (request->override_state)
  {
    std::unique_lock<std::shared_mutex> lock(*mutex_);
    buffer_response_->feature_sets.has_imu = true;
    buffer_response_->imu                  = request->imu_tx;
  }

  std::shared_lock<std::shared_mutex> lock(*mutex_);
  response->imu_rx = buffer_response_->imu;
}