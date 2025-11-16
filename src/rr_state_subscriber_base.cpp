#include "rr_state_mgm_srv/rr_state_subscriber_base.hpp"

using namespace rr_state_manager;

template <typename T>
void RrStateSubscriberBase<T>::init(std::shared_ptr<std::shared_mutex> mutex,
                                    std::shared_ptr<rr_interfaces::msg::BufferResponse> state_frame)
{
  state_frame_       = state_frame;
  mutex_             = mutex;
  rclcpp::QoS policy = configure_qos();

  RCLCPP_INFO(this->get_logger(), "creating subscriber");
  node_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  subscription_  = this->create_subscription<typename T::MsgType>(
      get_topic(), policy, get_callback_binding(), rmw_qos_profile_default, node_cb_group_);
}

template <typename T>
void RrStateSubscriberBase<T>::callback_around(const typename T::UniquePtr message)
{
  RCLCPP_DEBUG(this->get_logger(), "creating subscriber");
  std::unique_lock<std::shared_mutex> lock(*mutex_);
  callback(*message);
}