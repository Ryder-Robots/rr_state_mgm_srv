// Copyright (c) 2025 Ryder Robots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "rclcpp/rclcpp.hpp"
#include "rr_state_mgm_srv/rr_battery_state_subscriber.hpp"
#include "rr_state_mgm_srv/rr_gps_subscriber.hpp"
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

  // add subscriber nodes
  auto batt_state_sub = std::make_shared<RrBatteryStateSubscriber>();
  batt_state_sub->init(node->get_mutex(), node->get_state_frame());
  executor.add_node(batt_state_sub->get_node_base_interface());

  auto gps_state_sub = std::make_shared<RrGpsSubscriber>();
  gps_state_sub->init(node->get_mutex(), node->get_state_frame());
  executor.add_node(gps_state_sub->get_node_base_interface());

  executor.spin();
  rclcpp::shutdown();
  return 0;
}