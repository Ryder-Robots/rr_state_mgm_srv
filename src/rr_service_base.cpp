#include "rr_state_mgm_srv/rr_service_base.hpp"

using namespace rr_state_manager;

void RrStateServiceBase::init(std::shared_ptr<std::shared_mutex> mutex,
                              std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response)
{
  mutex_           = mutex;
  buffer_response_ = buffer_response;

  on_start_up();
}