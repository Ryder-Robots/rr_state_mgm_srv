#include "rclcpp/rclcpp.hpp"
#include "rr_state_mgm_srv/state_manager.hpp"
#include "rr_state_mgm_srv/rr_battery_state_subscriber.hpp"

using namespace rr_state_manager;

/*
 * using multithreaded node to allow different nodes to use the service at once,
 * and so it doesn't interfere with publishing.
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RrStateManagerSrv>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // add subscriber nodes
  auto batt_state_sub = std::make_shared<RrBatteryStateSubscriber>();
  batt_state_sub->init(node->get_mutex(), node->get_state_frame());
  executor.add_node(batt_state_sub->get_node_base_interface());

  executor.spin();
  rclcpp::shutdown();
  return 0;
}