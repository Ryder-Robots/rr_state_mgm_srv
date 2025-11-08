#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rr_state_mgm_srv/state_manager.hpp"

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
    state_maintainer_ = std::make_shared<RrStateManagerSrv>();
  }

  void TearDown() override
  {
    state_maintainer_.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rr_state_manager::RrStateManagerSrv> state_maintainer_;
};

// Test setters and getters
TEST_F(TestController, gps)
{
  // rclcpp::Clock clock;
  // auto current_time = clock.now();

  // sensor_msgs::msg::NavSatFix expected;
  // expected.header.stamp    = current_time;
  // expected.header.frame_id = "gps_link";
  // expected.status.status   = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  // expected.status.service  = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // // Set geographic coordinates (latitude, longitude, altitude)
  // expected.latitude  = -33.8688;  // Degrees, e.g., Sydney
  // expected.longitude = 151.2093;  // Degrees, e.g., Sydney
  // expected.altitude  = 58.0;      // In meters above WGS84 ellipsoid

  // // Set position covariance (if known, otherwise leave as default zeros)
  // std::fill(std::begin(expected.position_covariance), std::end(expected.position_covariance), 0.0);
  // expected.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  // rr_interfaces::srv::StateGpsReq::Request request;
  // rr_interfaces::srv::StateGpsReq::Response response;
  // request.gps = expected;
  // std::shared_ptr<rr_interfaces::srv::StateGpsReq::Request> req_sh =
  //     std::make_shared<rr_interfaces::srv::StateGpsReq::Request>(request);
  // std::shared_ptr<rr_interfaces::srv::StateGpsReq::Response> res_sh =
  //     std::make_shared<rr_interfaces::srv::StateGpsReq::Response>(response);

  // state_maintainer_->set_gps(req_sh, res_sh);
  // sensor_msgs::msg::NavSatFix actual = res_sh->buffer_response.gps;

  // EXPECT_EQ(actual.header.stamp, current_time);
  // EXPECT_EQ(actual.header.frame_id, "gps_link");
  // EXPECT_EQ(actual.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  // EXPECT_EQ(actual.status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);

  // // allow a tolerance of around 1 meter.
  // EXPECT_NEAR(actual.latitude, -33.8688, 0.000009);
  // EXPECT_NEAR(actual.longitude, 151.2093, 0.000009);
  // EXPECT_NEAR(actual.altitude, 58.0, 1);

  // GTEST_EXPECT_TRUE(res_sh->buffer_response.feature_sets.has_gps);
}

// TEST_F(TestController, range)
// {
//   rclcpp::Clock clock;
//   auto current_time = clock.now();
//   sensor_msgs::msg::Range expected1;
//   expected1.header.frame_id = rr_constants::LINK_ULTRA_SONIC_CENTER;
//   expected1.header.stamp    = current_time;

//   // reference: https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304
//   expected1.min_range      = 2;
//   expected1.max_range      = 500;
//   expected1.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
//   expected1.range          = 80;

//   // this value can be calculated as FoV = 2 x arctan(beam radius / distance from sensor), URM09 the
//   // beam radius is 60 degrees
//   expected1.field_of_view = 2 * atan(2 * (60 / expected1.range));
//   rr_interfaces::srv::StateRange::Request request;
//   rr_interfaces::srv::StateRange::Response response;
//   request.range = expected1;
//   std::shared_ptr<rr_interfaces::srv::StateRange::Request> req_sh =
//       std::make_shared<rr_interfaces::srv::StateRange::Request>(request);
//   std::shared_ptr<rr_interfaces::srv::StateRange::Response> res_sh =
//       std::make_shared<rr_interfaces::srv::StateRange::Response>(response);

//   state_maintainer_->set_range(req_sh, res_sh);
//   std::vector<sensor_msgs::msg::Range> ranges = res_sh->buffer_response.ranges;

//   // Note because only one range has been placed in this, we only use the one. However for a real application 
//   // the link code will need to be checked to see which range was added.
//   EXPECT_TRUE(res_sh->buffer_response.feature_sets.has_ranges);
//   EXPECT_EQ(rr_constants::LINK_ULTRA_SONIC_CENTER, ranges[0].header.frame_id);
//   EXPECT_EQ(ranges[0].header.stamp, current_time);
//   EXPECT_EQ(ranges[0].min_range, expected1.min_range);
//   EXPECT_EQ(ranges[0].max_range, expected1.max_range);
//   EXPECT_EQ(ranges[0].radiation_type, expected1.radiation_type);
//   EXPECT_EQ(ranges[0].range, expected1.range);

//   // Added a new range sensor.
//   sensor_msgs::msg::Range expected2;
//   expected2.header.frame_id = rr_constants::LINK_ULTRA_SONIC_LEFT;
//   expected2.header.stamp    = current_time;

//   // reference: https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304
//   expected2.min_range      = 2;
//   expected2.max_range      = 500;
//   expected2.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
//   expected2.range          = 40;

//   // this value can be calculated as FoV = 2 x arctan(beam radius / distance from sensor), URM09 the
//   // beam radius is 60 degrees
//   expected2.field_of_view = 2 * atan(2 * (60 / expected2.range));  
//   req_sh->range = expected2;
//   state_maintainer_->set_range(req_sh, res_sh);

//   EXPECT_TRUE(res_sh->buffer_response.feature_sets.has_ranges);
//   EXPECT_EQ(rr_constants::LINK_ULTRA_SONIC_LEFT, res_sh->buffer_response.ranges[1].header.frame_id);
//   EXPECT_EQ(res_sh->buffer_response.ranges[1].header.stamp, current_time);
//   EXPECT_EQ(res_sh->buffer_response.ranges[1].min_range, expected2.min_range);
//   EXPECT_EQ(res_sh->buffer_response.ranges[1].max_range, expected2.max_range);
//   EXPECT_EQ(res_sh->buffer_response.ranges[1].radiation_type, expected2.radiation_type);
//   EXPECT_EQ(res_sh->buffer_response.ranges[1].range, expected2.range);

//   // override center
//   current_time = clock.now();
//   sensor_msgs::msg::Range expected3;
//   expected3.header.frame_id = rr_constants::LINK_ULTRA_SONIC_CENTER;
//   expected3.header.stamp    = current_time;

//   // reference: https://wiki.dfrobot.com/URM09_Ultrasonic_Sensor_(Gravity-I2C)_(V1.0)_SKU_SEN0304
//   expected3.min_range      = 2;
//   expected3.max_range      = 500;
//   expected3.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
//   expected3.range          = 50;
//   req_sh->range = expected3;
//   state_maintainer_->set_range(req_sh, res_sh);

//   EXPECT_EQ(rr_constants::LINK_ULTRA_SONIC_CENTER, res_sh->buffer_response.ranges[0].header.frame_id);
//   EXPECT_EQ(res_sh->buffer_response.ranges[0].header.stamp, current_time);
//   EXPECT_EQ(res_sh->buffer_response.ranges[0].range, expected3.range);
// }

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
