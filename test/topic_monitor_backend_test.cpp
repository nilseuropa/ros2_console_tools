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

TEST_F(TopicMonitorBackendTest, TopicRowsCommitOutstandingMissesToMissedCount) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/monitored/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.total_missed_messages = 3;
  entry.has_last_message = true;
  const auto now = TopicClock::now();
  entry.last_message_time = now - std::chrono::milliseconds(750);
  entry.intervals.push_back({now - std::chrono::seconds(2), 0.5});
  entry.intervals.push_back({now - std::chrono::milliseconds(1250), 0.5});
  backend.topics_.emplace(entry.name, entry);

  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_TRUE(rows.front().has_expected_frequency);
  EXPECT_EQ(rows.front().current_missed_messages, 1u);
  EXPECT_EQ(rows.front().total_missed_messages, 4u);
  EXPECT_TRUE(rows.front().stale);
}

TEST_F(TopicMonitorBackendTest, TopicRowsOnlyCountRedTransitionOnFirstVisibleStaleSnapshot) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/cadence/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.has_last_message = true;
  const auto now = TopicClock::now();
  entry.last_message_time = now - std::chrono::milliseconds(1260);
  entry.intervals.push_back({now - std::chrono::seconds(2), 0.5});
  entry.intervals.push_back({now - std::chrono::milliseconds(1500), 0.5});
  backend.topics_.emplace(entry.name, entry);

  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().current_missed_messages, 2u);
  EXPECT_EQ(rows.front().total_missed_messages, 1u);
  EXPECT_TRUE(rows.front().stale);
}

TEST_F(TopicMonitorBackendTest, ActiveVisibleGapCountsEachAdditionalMissedPeriod) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/active-gap/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.has_last_message = true;
  const auto now = TopicClock::now();
  entry.last_message_time = now - std::chrono::milliseconds(1260);
  entry.intervals.push_back({now - std::chrono::seconds(2), 0.5});
  entry.intervals.push_back({now - std::chrono::milliseconds(1500), 0.5});
  entry.total_missed_messages = 1;
  entry.counted_current_gap_missed_messages = 1;
  entry.current_gap_visible_active = true;
  entry.current_gap_average_period_seconds = 0.5;
  entry.next_gap_count_time = now - std::chrono::milliseconds(10);
  backend.topics_.emplace(entry.name, entry);

  backend.update_active_gap_counters();
  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().current_missed_messages, 2u);
  EXPECT_EQ(rows.front().total_missed_messages, 2u);
  EXPECT_TRUE(rows.front().stale);
}

TEST_F(TopicMonitorBackendTest, TopicRowsDoNotDoubleCountOutstandingMissesAcrossSnapshots) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/ongoing/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.has_last_message = true;
  const auto now = TopicClock::now();
  entry.last_message_time = now - std::chrono::milliseconds(750);
  entry.intervals.push_back({now - std::chrono::seconds(2), 0.5});
  entry.intervals.push_back({now - std::chrono::milliseconds(1250), 0.5});
  backend.topics_.emplace(entry.name, entry);

  const auto first_rows = backend.topic_rows_snapshot();
  const auto second_rows = backend.topic_rows_snapshot();

  ASSERT_EQ(first_rows.size(), 1u);
  ASSERT_EQ(second_rows.size(), 1u);
  EXPECT_EQ(first_rows.front().current_missed_messages, 1u);
  EXPECT_EQ(first_rows.front().total_missed_messages, 1u);
  EXPECT_EQ(second_rows.front().current_missed_messages, 1u);
  EXPECT_EQ(second_rows.front().total_missed_messages, 1u);
}

TEST_F(TopicMonitorBackendTest, TopicRowsDoNotCountOutstandingMissesBeforeExpectedArrival) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/steady/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.total_missed_messages = 2;
  entry.has_last_message = true;
  const auto now = TopicClock::now();
  entry.last_message_time = now - std::chrono::milliseconds(200);
  entry.intervals.push_back({now - std::chrono::seconds(2), 0.5});
  entry.intervals.push_back({now - std::chrono::milliseconds(1200), 0.5});
  backend.topics_.emplace(entry.name, entry);

  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_TRUE(rows.front().has_expected_frequency);
  EXPECT_EQ(rows.front().current_missed_messages, 0u);
  EXPECT_EQ(rows.front().total_missed_messages, 2u);
  EXPECT_FALSE(rows.front().stale);
}

TEST_F(TopicMonitorBackendTest, TopicRowsApplyTwentyFivePercentGraceBeforeCountingMisses) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/grace/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.total_missed_messages = 2;
  entry.has_last_message = true;
  const auto now = TopicClock::now();
  entry.last_message_time = now - std::chrono::milliseconds(620);
  entry.intervals.push_back({now - std::chrono::seconds(2), 0.5});
  entry.intervals.push_back({now - std::chrono::milliseconds(1200), 0.5});
  backend.topics_.emplace(entry.name, entry);

  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_TRUE(rows.front().has_expected_frequency);
  EXPECT_EQ(rows.front().current_missed_messages, 0u);
  EXPECT_EQ(rows.front().total_missed_messages, 2u);
  EXPECT_FALSE(rows.front().stale);
}

TEST_F(TopicMonitorBackendTest, TopicRowsShowAverageRecoveryDurationForRecoveredOutages) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/recovering/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.total_missed_messages = 5;
  entry.last_recovery_duration_seconds = 0.5;
  entry.recovery_duration_sum_seconds = 0.75;
  entry.recovery_count = 3;
  backend.topics_.emplace(entry.name, entry);

  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().total_missed_messages, 5u);
  EXPECT_EQ(rows.front().last_recovery_time, "500ms");
  EXPECT_EQ(rows.front().avg_recovery_time, "250ms");
}

TEST_F(TopicMonitorBackendTest, TopicRowsIgnoreBurstIntervalsWhenComputingMaxHz) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/burst/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.has_last_message = true;
  const auto now = TopicClock::now();
  entry.last_message_time = now - std::chrono::milliseconds(5);
  entry.intervals.push_back({now - std::chrono::seconds(2), 0.010});
  entry.intervals.push_back({now - std::chrono::milliseconds(1200), 0.000062});
  entry.intervals.push_back({now - std::chrono::milliseconds(900), 0.019938});
  backend.topics_.emplace(entry.name, entry);

  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().avg_hz, "100");
  EXPECT_EQ(rows.front().min_max_hz, "50.2/100");
}

TEST_F(TopicMonitorBackendTest, OnMessageSkipsDetailDecodingOutsideDetailView) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/invalid/topic";
  entry.type = "bad_type/msg/Nope";
  entry.monitored = true;
  backend.topics_.emplace(entry.name, entry);

  rclcpp::SerializedMessage message(32);
  backend.on_message(entry.name, message);

  const auto found = backend.topics_.find(entry.name);
  ASSERT_NE(found, backend.topics_.end());
  EXPECT_TRUE(found->second.detail_error.empty());
  EXPECT_TRUE(found->second.detail_rows.empty());
}

TEST_F(TopicMonitorBackendTest, OnMessageRecordsActualRecoveryGapForSingleMiss) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/single-miss/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.has_last_message = true;
  entry.last_message_time = TopicClock::now() - std::chrono::milliseconds(20);
  entry.has_last_observed_timestamp = true;
  entry.last_observed_timestamp_ns = 1'000'000'000;
  entry.intervals.push_back({TopicClock::now() - std::chrono::seconds(2), 0.01});
  entry.intervals.push_back({TopicClock::now() - std::chrono::milliseconds(1200), 0.01});
  entry.total_missed_messages = 1;
  entry.counted_current_gap_missed_messages = 1;
  entry.current_gap_visible_active = true;
  entry.current_gap_average_period_seconds = 0.01;
  entry.next_gap_count_time = TopicClock::now() + std::chrono::milliseconds(10);
  backend.topics_.emplace(entry.name, entry);

  rclcpp::SerializedMessage message(32);
  rmw_message_info_t rmw_info = rmw_get_zero_initialized_message_info();
  rmw_info.received_timestamp = 1'020'000'000;
  backend.on_message(entry.name, message, rclcpp::MessageInfo(rmw_info));

  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().total_missed_messages, 1u);
  EXPECT_EQ(rows.front().last_recovery_time, "20ms");
  EXPECT_EQ(rows.front().avg_recovery_time, "20ms");
}

TEST_F(TopicMonitorBackendTest, OnMessageRecordsActualRecoveryGapForLongOutage) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/recovery/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.has_last_message = true;
  entry.last_message_time = TopicClock::now() - std::chrono::milliseconds(750);
  entry.has_last_observed_timestamp = true;
  entry.last_observed_timestamp_ns = 1'000'000'000;
  entry.intervals.push_back({TopicClock::now() - std::chrono::seconds(2), 0.5});
  entry.intervals.push_back({TopicClock::now() - std::chrono::milliseconds(1200), 0.5});
  entry.total_missed_messages = 1;
  entry.counted_current_gap_missed_messages = 1;
  entry.current_gap_visible_active = true;
  entry.current_gap_average_period_seconds = 0.5;
  entry.next_gap_count_time = TopicClock::now() + std::chrono::milliseconds(250);
  backend.topics_.emplace(entry.name, entry);

  rclcpp::SerializedMessage message(32);
  rmw_message_info_t rmw_info = rmw_get_zero_initialized_message_info();
  rmw_info.received_timestamp = 1'750'000'000;
  backend.on_message(entry.name, message, rclcpp::MessageInfo(rmw_info));

  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().total_missed_messages, 1u);
  EXPECT_EQ(rows.front().last_recovery_time, "750ms");
  EXPECT_EQ(rows.front().avg_recovery_time, "750ms");
}

TEST_F(TopicMonitorBackendTest, OnMessageRecoveryUsesWallGapWhenMiddlewareTimestampIsShorter) {
  TopicMonitorBackend backend;

  TopicEntry entry;
  entry.name = "/delayed-callback/topic";
  entry.type = "geometry_msgs/msg/Twist";
  entry.monitored = true;
  entry.has_last_message = true;
  entry.last_message_time = TopicClock::now() - std::chrono::milliseconds(120);
  entry.has_last_observed_timestamp = true;
  entry.last_observed_timestamp_ns = 1'000'000'000;
  entry.intervals.push_back({TopicClock::now() - std::chrono::seconds(2), 0.01});
  entry.intervals.push_back({TopicClock::now() - std::chrono::milliseconds(1200), 0.01});
  entry.total_missed_messages = 10;
  entry.counted_current_gap_missed_messages = 10;
  entry.current_gap_visible_active = true;
  entry.current_gap_average_period_seconds = 0.01;
  entry.next_gap_count_time = TopicClock::now() + std::chrono::milliseconds(10);
  backend.topics_.emplace(entry.name, entry);

  rclcpp::SerializedMessage message(32);
  rmw_message_info_t rmw_info = rmw_get_zero_initialized_message_info();
  rmw_info.received_timestamp = 1'027'000'000;
  backend.on_message(entry.name, message, rclcpp::MessageInfo(rmw_info));

  const auto rows = backend.topic_rows_snapshot();

  ASSERT_EQ(rows.size(), 1u);
  EXPECT_EQ(rows.front().total_missed_messages, 10u);
  EXPECT_EQ(rows.front().last_recovery_time, "120ms");
  EXPECT_EQ(rows.front().avg_recovery_time, "120ms");
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
