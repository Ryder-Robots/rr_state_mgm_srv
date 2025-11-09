#include "rclcpp/rclcpp.hpp"
#include "rr_state_mgm_srv/state_manager.hpp"

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
  executor.spin();
  rclcpp::shutdown();
  return 0;
}