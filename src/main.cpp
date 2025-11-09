#include "rclcpp/rclcpp.hpp"
#include "rr_state_mgm_srv/state_manager.hpp"

using namespace rr_state_manager;

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RrStateManagerSrv>());
  rclcpp::shutdown();
  return 0;
}