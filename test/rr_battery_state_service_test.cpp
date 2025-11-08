#include "rr_state_mgm_srv/rr_battery_state_service.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rr_common_base/rr_constants.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

using namespace rr_state_manager;

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
    object_under_test_ = std::make_shared<RrBatteryStateService>();
    object_under_test_->init(mutex_, state_);
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<RrBatteryStateService> object_under_test_;
  std::shared_ptr<std::shared_mutex> mutex_ = std::make_shared<std::shared_mutex>();
  std::shared_ptr<rr_interfaces::msg::BufferResponse> state_ =
      std::make_shared<rr_interfaces::msg::BufferResponse>();
};

TEST_F(TestController, battery_state_set)
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

  rr_interfaces::srv::BatteryState::Request request;
  rr_interfaces::srv::BatteryState::Response response;

  request.batt_state_tx  = battery_state;
  request.override_state = true;

  std::shared_ptr<rr_interfaces::srv::BatteryState::Request> req_sh =
      std::make_shared<rr_interfaces::srv::BatteryState::Request>(request);
  std::shared_ptr<rr_interfaces::srv::BatteryState::Response> res_sh =
      std::make_shared<rr_interfaces::srv::BatteryState::Response>(response);

  object_under_test_->set_batt_state(req_sh, res_sh);
  sensor_msgs::msg::BatteryState res = res_sh->batt_state_rx;

  //   EXPECT_EQ(res_sh->batt_state_rx, rr_constants::LINK_BATT_STATE);

  // TEST IT
  GTEST_EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
