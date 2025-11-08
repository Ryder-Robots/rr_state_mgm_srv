#include "rr_state_mgm_srv/rr_state_base.hpp"

using namespace rr_state_manager;

void RrStateManagerBase::init(std::shared_ptr<std::shared_mutex> mutex,
          std::shared_ptr<rr_interfaces::msg::BufferResponse> buffer_response)
{
    mutex_ = mutex;
    buffer_response_ = buffer_response;
}

/**
 *  common template that will control locking.
 */
template <typename T, typename R>
void RrStateManagerBase::set_state(const std::shared_ptr<T>& request, std::shared_ptr<R>& response,
                                  const std::function<void(const std::shared_ptr<T>&)>& cb1,
                                  const std::function<void(std::shared_ptr<R>&)>& cb2,
                                  const bool override_state)
{
  if (override_state)
  {
    std::unique_lock<std::shared_mutex> lock(*mutex_);
    cb1(request);
  }
  std::shared_lock<std::shared_mutex> lock(*mutex_);
  cb2(response);
}