#include "rr_state_mgm_srv/rr_navigation_service.hpp"

using namespace rr_state_manager;

void RrNavigationService::set_nav(
    const std::shared_ptr<rr_interfaces::srv::Navigation::Request> request,
    std::shared_ptr<rr_interfaces::srv::Navigation::Response> response)
{
  if (request->override_state)
  {
    std::unique_lock<std::shared_mutex> lock(*mutex_);
    // TODO: need to add nevigation to feature set.
    // buffer_response_->feature_sets.has_nav = true;
    buffer_response_->navigation = request->nav_tx;
  }

  std::shared_lock<std::shared_mutex> lock(*mutex_);
  response->nav_rs = buffer_response_->navigation;
}