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
TEST_F(TestController, node_test)
{
  EXPECT_TRUE(true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  auto result = RUN_ALL_TESTS();
  return result;
}
