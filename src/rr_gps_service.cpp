#include "rr_state_mgm_srv/rr_gps_service.hpp"

using namespace rr_state_manager;

void RrrGpsService::set_gps(const std::shared_ptr<rr_interfaces::srv::Gps::Request> request,
                            std::shared_ptr<rr_interfaces::srv::Gps::Response> response)
{
  if (request->override_state)
  {
    std::unique_lock<std::shared_mutex> lock(*mutex_);
    buffer_response_->feature_sets.has_gps = true;
    buffer_response_->gps                  = request->gps_tx;
  }

  std::shared_lock<std::shared_mutex> lock(*mutex_);
  response->gps_rx = buffer_response_->gps;
}

void RrrGpsService::init(std::shared_ptr<std::shared_mutex> mutex,
                         std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response)
{
  mutex_           = mutex;
  buffer_response_ = buffer_response;
}