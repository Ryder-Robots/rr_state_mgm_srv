#include "rr_state_mgm_srv/rr_range_service.hpp"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"

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
    range_svr_ = std::make_shared<RrRangeService>();
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::shared_ptr<rr_state_manager::RrRangeService> range_svr_;
};

TEST_F(TestController, range)
{
  rclcpp::Clock clock;
  auto current_time = clock.now();
  sensor_msgs::msg::Range expected1;
  expected1.header.frame_id = rr_constants::LINK_ULTRA_SONIC_CENTER;
  expected1.header.stamp    = current_time;

  // reference: https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304
  expected1.min_range      = 2;
  expected1.max_range      = 500;
  expected1.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  expected1.range          = 80;

  // this value can be calculated as FoV = 2 x arctan(beam radius / distance from sensor), URM09
  //   the beam radius is 60 degrees
  expected1.field_of_view = 2 * atan(2 * (60 / expected1.range));
  rr_interfaces::srv::Range::Request request;
  rr_interfaces::srv::Range::Response response;
  request.range_tx = expected1;
  std::shared_ptr<rr_interfaces::srv::Range::Request> req_sh =
      std::make_shared<rr_interfaces::srv::Range::Request>(request);
  std::shared_ptr<rr_interfaces::srv::Range::Response> res_sh =
      std::make_shared<rr_interfaces::srv::Range::Response>(response);

  range_svr_->set_range(req_sh, res_sh);
  sensor_msgs::msg::Range range = res_sh->range_rx;

  // Note because only one range has been placed in this, we only use the one. However for a real
  // application the link code will need to be checked to see which range was added.
  EXPECT_EQ(rr_constants::LINK_ULTRA_SONIC_CENTER, range.header.frame_id);
  EXPECT_EQ(range.header.stamp, current_time);
  EXPECT_EQ(range.min_range, expected1.min_range);
  EXPECT_EQ(range.max_range, expected1.max_range);
  EXPECT_EQ(range.radiation_type, expected1.radiation_type);
  EXPECT_EQ(range.range, expected1.range);

  // Added a new range sensor.
  sensor_msgs::msg::Range expected2;
  expected2.header.frame_id = rr_constants::LINK_ULTRA_SONIC_LEFT;
  expected2.header.stamp    = current_time;

  // reference: https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304
  expected2.min_range      = 2;
  expected2.max_range      = 500;
  expected2.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  expected2.range          = 40;

  // this value can be calculated as FoV = 2 x arctan(beam radius / distance from sensor), URM09
  //   the beam radius is 60 degrees
  expected2.field_of_view = 2 * atan(2 * (60 / expected2.range));
  req_sh->range_tx        = expected2;
  range_svr_->set_range(req_sh, res_sh);
  range = res_sh->range_rx;

  EXPECT_EQ(rr_constants::LINK_ULTRA_SONIC_LEFT, range.header.frame_id);
  EXPECT_EQ(range.header.stamp, current_time);
  EXPECT_EQ(range.min_range, expected2.min_range);
  EXPECT_EQ(range.max_range, expected2.max_range);
  EXPECT_EQ(range.radiation_type, expected2.radiation_type);
  EXPECT_EQ(range.range, expected2.range);

  // override center
  current_time = clock.now();
  sensor_msgs::msg::Range expected3;
  expected3.header.frame_id = rr_constants::LINK_ULTRA_SONIC_CENTER;
  expected3.header.stamp    = current_time;

  // reference: https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304
  expected3.min_range      = 2;
  expected3.max_range      = 500;
  expected3.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
  expected3.range          = 50;
  req_sh->range_tx         = expected3;
  range_svr_->set_range(req_sh, res_sh);
  range = res_sh->range_rx;

  EXPECT_EQ(rr_constants::LINK_ULTRA_SONIC_CENTER, range.header.frame_id);
  EXPECT_EQ(range.header.stamp, current_time);
  EXPECT_EQ(range.range, expected3.range);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}