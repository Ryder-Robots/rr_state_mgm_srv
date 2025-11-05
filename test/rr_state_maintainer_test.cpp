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
  rclcpp::Clock clock;
  auto current_time = clock.now();

  sensor_msgs::msg::NavSatFix expected;
  expected.header.stamp    = current_time;
  expected.header.frame_id = "gps_link";
  expected.status.status   = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  expected.status.service  = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

  // Set geographic coordinates (latitude, longitude, altitude)
  expected.latitude  = -33.8688;  // Degrees, e.g., Sydney
  expected.longitude = 151.2093;  // Degrees, e.g., Sydney
  expected.altitude  = 58.0;      // In meters above WGS84 ellipsoid

  // Set position covariance (if known, otherwise leave as default zeros)
  std::fill(std::begin(expected.position_covariance), std::end(expected.position_covariance), 0.0);
  expected.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;

  rr_interfaces::srv::StateGpsReq::Request request;
  rr_interfaces::srv::StateGpsReq::Response response;
  request.gps = expected;
  std::shared_ptr<rr_interfaces::srv::StateGpsReq::Request> req_sh =
      std::make_shared<rr_interfaces::srv::StateGpsReq::Request>(request);
  std::shared_ptr<rr_interfaces::srv::StateGpsReq::Response> res_sh =
      std::make_shared<rr_interfaces::srv::StateGpsReq::Response>(response);

  state_maintainer_->set_gps(req_sh, res_sh);
  sensor_msgs::msg::NavSatFix actual = res_sh->buffer_response.gps;

  EXPECT_EQ(actual.header.stamp, current_time);
  EXPECT_EQ(actual.header.frame_id, "gps_link");
  EXPECT_EQ(actual.status.status, sensor_msgs::msg::NavSatStatus::STATUS_FIX);
  EXPECT_EQ(actual.status.service, sensor_msgs::msg::NavSatStatus::SERVICE_GPS);

  // allow a tolerance of around 1 meter.
  EXPECT_NEAR(actual.latitude, -33.8688, 0.000009);
  EXPECT_NEAR(actual.longitude, 151.2093, 0.000009);
  EXPECT_NEAR(actual.altitude, 58.0, 1);

  GTEST_EXPECT_TRUE(res_sh->buffer_response.feature_sets.has_gps);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
