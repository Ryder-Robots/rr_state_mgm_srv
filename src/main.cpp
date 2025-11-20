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
// #include "rr_state_mgm_srv/rr_battery_state_subscriber.hpp"
// #include "rr_state_mgm_srv/rr_gps_subscriber.hpp"
// #include "rr_state_mgm_srv/rr_joystick_subscriber.hpp"
#include "rr_state_mgm_srv/state_manager.hpp"

using namespace rr_state_manager::rr_state_manager_node;

/*
 * using multithreaded node to allow different nodes to use the service at once,
 * and so it doesn't interfere with publishing.
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  std::shared_ptr<std::shared_mutex> mutex = std::make_shared<std::shared_mutex>();
  std::shared_ptr<rr_interfaces::msg::StateFrame> state_frame =
      std::make_shared<rr_interfaces::msg::StateFrame>();

  // auto joy_state_sub = std::make_shared<RrJoystickSubscriber>(mutex, state_frame);
  // executor.add_node(joy_state_sub->get_node_base_interface());
  rclcpp::NodeOptions opts;
  auto state_pub = std::make_shared<RrStateManagerSrv>(opts, mutex, state_frame);
  executor.add_node(state_pub->get_node_base_interface());

  executor.spin();
  rclcpp::shutdown();
  return 0;
}