#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

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

TEST_F(TopicMonitorBackendTest, FilteredLaunchOnlyKeepsAllowedTopicsAndStartsMonitoring) {
  auto source_node = std::make_shared<rclcpp::Node>("topic_monitor_filtered_source");
  auto allowed_publisher =
    source_node->create_publisher<geometry_msgs::msg::Twist>("/filtered/topic", 10);
  auto other_publisher =
    source_node->create_publisher<geometry_msgs::msg::Twist>("/other/topic", 10);
  (void)allowed_publisher;
  (void)other_publisher;

  TopicMonitorLaunchOptions options;
  options.allowed_topics.push_back("/filtered/topic");
  options.monitor_allowed_topics_on_start = true;
  auto backend = std::make_shared<TopicMonitorBackend>(options);

  rclcpp::executors::SingleThreadedExecutor source_executor;
  rclcpp::executors::SingleThreadedExecutor backend_executor;
  source_executor.add_node(source_node);
  backend_executor.add_node(backend);

  std::thread source_spin_thread([&source_executor]() { source_executor.spin(); });
  std::thread backend_spin_thread([&backend_executor]() { backend_executor.spin(); });

  auto stop_executor = [](rclcpp::executors::SingleThreadedExecutor & executor, std::thread & thread) {
      executor.cancel();
      if (thread.joinable()) {
        thread.join();
      }
    };

  bool filtered_launch_ready = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    backend->refresh_topics();

    const auto filtered_topic = backend->topics_.find("/filtered/topic");
    if (
      backend->topics_.size() == 1 &&
      filtered_topic != backend->topics_.end() &&
      filtered_topic->second.monitored &&
      backend->topics_.find("/other/topic") == backend->topics_.end())
    {
      filtered_launch_ready = true;
      break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(filtered_launch_ready);
  EXPECT_EQ(backend->topics_.size(), 1u);
  EXPECT_NE(backend->topics_.find("/filtered/topic"), backend->topics_.end());
  EXPECT_EQ(backend->topics_.find("/other/topic"), backend->topics_.end());
  EXPECT_TRUE(backend->topics_.at("/filtered/topic").monitored);

  stop_executor(backend_executor, backend_spin_thread);
  stop_executor(source_executor, source_spin_thread);
}

TEST_F(TopicMonitorBackendTest, DirectDetailLaunchCanReturnToEmbeddedCallerOnEscape) {
  TopicMonitorLaunchOptions options;
  options.initial_topic = "/detail/topic";
  options.allowed_topics.push_back("/detail/topic");
  options.open_initial_topic_detail = true;
  options.exit_on_detail_escape = true;

  auto backend = std::make_shared<TopicMonitorBackend>(options);
  TopicEntry entry;
  entry.name = "/detail/topic";
  entry.type = "geometry_msgs/msg/Twist";
  backend->topics_.emplace(entry.name, entry);

  backend->apply_startup_behavior();

  EXPECT_EQ(backend->view_mode_, TopicMonitorViewMode::TopicDetail);
  EXPECT_EQ(backend->detail_topic_name_, "/detail/topic");
  EXPECT_TRUE(backend->topics_.at("/detail/topic").monitored);

  TopicMonitorScreen screen(backend, true);
  EXPECT_FALSE(screen.handle_topic_detail_key(27));
}

}  // namespace
}  // namespace ros2_console_tools
