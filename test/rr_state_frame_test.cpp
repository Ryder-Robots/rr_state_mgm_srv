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

#include "rr_state_mgm_srv/rr_state_frame.hpp"
#include <gtest/gtest.h>

using namespace rrobot::state_frame;

class TestController : public testing::Test
{
  protected:
    TestController() {}
    ~TestController() override
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

     void SetUp() override
  {
    rclcpp::init(0, nullptr);
  }

  RrStateFrameSingleton & object_under_test_ = RrStateFrameSingleton::get_instance();
};

TEST_F(TestController, battery_set) {
{
  rclcpp::Clock clock;
  auto current_time = clock.now();
  sensor_msgs::msg::BatteryState battery_state;
  battery_state.header.frame_id     = rr_constants::LINK_BATT_STATE;
  battery_state.header.stamp        = current_time;
  battery_state.voltage             = 12.4;
  battery_state.temperature         = 20;
  battery_state.current             = -0.01;
  battery_state.charge              = 1;
  battery_state.capacity            = 12;
  battery_state.design_capacity     = 12;
  battery_state.percentage          = 1;
  battery_state.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  battery_state.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  battery_state.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIPO;
  battery_state.cell_voltage.resize(3);
  battery_state.cell_voltage[0] = 1;
  battery_state.cell_voltage[1] = 1;
  battery_state.cell_voltage[2] = 1;
  battery_state.cell_temperature.resize(3);
  battery_state.cell_temperature[0] = 19;
  battery_state.cell_temperature[1] = 21;
  battery_state.cell_temperature[2] = 20;
  battery_state.location            = rr_constants::BATT_LOC_BASE;
  battery_state.serial_number       = "FOOSERIAL1111";
  battery_state.present = true;

  object_under_test_.set_batt_state(battery_state);

  sensor_msgs::msg::BatteryState res = object_under_test_.get_state().batt_state;
  EXPECT_EQ(res.header.frame_id, rr_constants::LINK_BATT_STATE);
  EXPECT_EQ(res.header.stamp, current_time);
  EXPECT_EQ(res.voltage, battery_state.voltage);
  EXPECT_EQ(res.temperature, battery_state.temperature);
  EXPECT_EQ(res.current, battery_state.current);
  EXPECT_EQ(res.charge, battery_state.charge);
  EXPECT_EQ(res.capacity, battery_state.capacity);
  EXPECT_EQ(res.design_capacity, battery_state.design_capacity);
  EXPECT_EQ(res.percentage, battery_state.percentage);
  EXPECT_EQ(res.power_supply_status, battery_state.power_supply_status);
  EXPECT_EQ(res.power_supply_health, battery_state.power_supply_health);
  EXPECT_EQ(res.power_supply_technology, battery_state.power_supply_technology);
  EXPECT_TRUE(res.present);

  // TEST IT
  GTEST_EXPECT_TRUE(true);
}
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}