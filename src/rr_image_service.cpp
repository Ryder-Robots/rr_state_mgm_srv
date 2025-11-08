#include "rr_state_mgm_srv/rr_image_service.hpp"

using namespace rr_state_manager;

void RrImageService::set_image(const std::shared_ptr<rr_interfaces::srv::Image::Request> request,
                               std::shared_ptr<rr_interfaces::srv::Image::Response> response)
{
  if (request->override_state)
  {
    std::unique_lock<std::shared_mutex> lock(*mutex_);
    buffer_response_->feature_sets.has_img = true;
    buffer_response_->img                  = request->img_tx;
  }

  std::shared_lock<std::shared_mutex> lock(*mutex_);
  response->img_rx = buffer_response_->img;
}

void RrImageService::init(std::shared_ptr<std::shared_mutex> mutex,
                          std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response)
{
  mutex_           = mutex;
  buffer_response_ = buffer_response;
}