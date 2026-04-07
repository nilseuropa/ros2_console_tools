#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#define private public
#include "ros2_console_tools/topic_monitor.hpp"
#undef private

namespace ros2_console_tools {
namespace {

class TopicMonitorBackendTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(TopicMonitorBackendTest, ChoosesBestEffortForBestEffortPublishers) {
  TopicMonitorBackend backend;

  std::vector<rclcpp::QoS> publisher_qos_profiles;
  publisher_qos_profiles.emplace_back(rclcpp::SensorDataQoS());

  const auto subscription_qos = backend.compatible_subscription_qos(publisher_qos_profiles);

  EXPECT_EQ(subscription_qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(subscription_qos.durability(), rclcpp::DurabilityPolicy::Volatile);
}

TEST_F(TopicMonitorBackendTest, PreservesReliableTransientLocalWhenAllPublishersOfferIt) {
  TopicMonitorBackend backend;

  rclcpp::QoS latched_qos(rclcpp::KeepLast(25));
  latched_qos.reliable();
  latched_qos.transient_local();

  const auto subscription_qos = backend.compatible_subscription_qos({latched_qos});

  EXPECT_EQ(subscription_qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(subscription_qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
  EXPECT_GE(subscription_qos.depth(), 25u);
}

TEST_F(TopicMonitorBackendTest, FallsBackToCompatibleMixedPublisherProfile) {
  TopicMonitorBackend backend;

  rclcpp::QoS reliable_latched_qos(rclcpp::KeepLast(20));
  reliable_latched_qos.reliable();
  reliable_latched_qos.transient_local();

  rclcpp::QoS sensor_qos = rclcpp::SensorDataQoS();

  const auto subscription_qos =
    backend.compatible_subscription_qos({reliable_latched_qos, sensor_qos});

  EXPECT_EQ(subscription_qos.reliability(), rclcpp::ReliabilityPolicy::BestEffort);
  EXPECT_EQ(subscription_qos.durability(), rclcpp::DurabilityPolicy::Volatile);
  EXPECT_GE(subscription_qos.depth(), 20u);
}

}  // namespace
}  // namespace ros2_console_tools
