#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#define private public
#include "ros2_console_tools/log_viewer.hpp"
#include "ros2_console_tools/node_commander.hpp"
#undef private

namespace ros2_console_tools {
namespace {

class NodeCommanderScreenTest : public ::testing::Test {
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

TEST_F(NodeCommanderScreenTest, DetailTreeUsesNodeRootAndActionableNodeScopedFolders) {
  auto target_node = std::make_shared<rclcpp::Node>("node_commander_test_target");
  target_node->declare_parameter("example_parameter", 7);
  auto publisher = target_node->create_publisher<geometry_msgs::msg::Twist>("/node_commander_test/out", 10);
  auto subscriber = target_node->create_subscription<geometry_msgs::msg::Twist>(
    "/node_commander_test/in", 10, [](geometry_msgs::msg::Twist::ConstSharedPtr) {});
  (void)publisher;
  (void)subscriber;

  auto backend = std::make_shared<NodeCommanderBackend>();
  NodeCommanderScreen screen(backend);
  EXPECT_EQ(screen.help_popup_title_, std::string("Node Commander v") + ROS2_CONSOLE_TOOLS_PACKAGE_VERSION);

  rclcpp::executors::SingleThreadedExecutor target_executor;
  rclcpp::executors::SingleThreadedExecutor backend_executor;
  target_executor.add_node(target_node);
  backend_executor.add_node(backend);

  std::thread target_spin_thread([&target_executor]() { target_executor.spin(); });
  std::thread backend_spin_thread([&backend_executor]() { backend_executor.spin(); });

  auto stop_executor = [](rclcpp::executors::SingleThreadedExecutor & executor, std::thread & thread) {
      executor.cancel();
      if (thread.joinable()) {
        thread.join();
      }
    };

  std::vector<DetailLine> lines;
  bool detail_lines_ready = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    backend->refresh_nodes();
    const auto found = std::find(
      backend->node_entries_.begin(),
      backend->node_entries_.end(),
      "/node_commander_test_target");
    if (found != backend->node_entries_.end()) {
      backend->selected_index_ = static_cast<int>(std::distance(backend->node_entries_.begin(), found));
      lines = backend->selected_node_details();
      if (!lines.empty()) {
        detail_lines_ready = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(detail_lines_ready);

  ASSERT_FALSE(lines.empty());
  EXPECT_TRUE(lines.front().is_header);
  EXPECT_EQ(lines.front().text, "/node_commander_test_target");
  EXPECT_EQ(lines.front().depth, 0);
  EXPECT_EQ(lines.front().action, NodeDetailAction::None);
  EXPECT_TRUE(lines.front().section_key.empty());

  const auto node_label = std::find_if(
    std::next(lines.begin()),
    lines.end(),
    [](const DetailLine & line) {
      return !line.is_header && line.text == "/node_commander_test_target";
    });
  EXPECT_EQ(node_label, lines.end());

  const auto parameter_header = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return line.is_header &&
        line.text == "Parameter Services" &&
        line.action == NodeDetailAction::OpenParameters &&
        line.target == "/node_commander_test_target" &&
        line.section_key == "parameters";
    });
  EXPECT_NE(parameter_header, lines.end());

  const auto parameter_child = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return !line.is_header && line.section_key == "parameters";
    });
  EXPECT_EQ(parameter_child, lines.end());

  const auto topics_header = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return line.is_header &&
        line.text == "Topics" &&
        line.action == NodeDetailAction::OpenTopicMonitor &&
        line.target == "/node_commander_test_target" &&
        line.section_key == "topics";
    });
  EXPECT_NE(topics_header, lines.end());

  const auto publishers_header = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return line.is_header &&
        line.text == "Publishers" &&
        line.depth == 1 &&
        line.action == NodeDetailAction::OpenTopicMonitor &&
        line.section_key == "topics/publishers";
    });
  EXPECT_NE(publishers_header, lines.end());

  const auto subscribers_header = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return line.is_header &&
        line.text == "Subscribers" &&
        line.depth == 1 &&
        line.action == NodeDetailAction::OpenTopicMonitor &&
        line.section_key == "topics/subscribers";
    });
  EXPECT_NE(subscribers_header, lines.end());

  const auto services_header = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return line.is_header &&
        line.text == "Services" &&
        line.action == NodeDetailAction::OpenServiceCommander &&
        line.target == "/node_commander_test_target" &&
        line.section_key == "services";
    });
  EXPECT_NE(services_header, lines.end());

  const auto logs_header = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return line.is_header &&
        line.text == "Logs" &&
        line.action == NodeDetailAction::OpenLogViewer &&
        line.target == "/node_commander_test_target" &&
        line.section_key == "logs";
    });
  EXPECT_NE(logs_header, lines.end());

  const auto logs_child = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return !line.is_header && line.section_key == "logs";
    });
  EXPECT_EQ(logs_child, lines.end());

  const auto service_entry = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return !line.is_header &&
        line.section_key == "services" &&
        line.action == NodeDetailAction::OpenServiceCommander;
    });
  EXPECT_NE(service_entry, lines.end());

  const auto publisher_entry = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return !line.is_header &&
        line.section_key == "topics/publishers" &&
        line.action == NodeDetailAction::OpenTopicMonitor &&
        line.depth == 2 &&
        line.target == "/node_commander_test/out";
    });
  EXPECT_NE(publisher_entry, lines.end());

  const auto subscriber_entry = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return !line.is_header &&
        line.section_key == "topics/subscribers" &&
        line.action == NodeDetailAction::OpenTopicMonitor &&
        line.depth == 2 &&
        line.target == "/node_commander_test/in";
    });
  EXPECT_NE(subscriber_entry, lines.end());

  screen.refresh_detail_lines_cache();
  const auto topic_targets =
    screen.detail_targets_for_section("topics", NodeDetailAction::OpenTopicMonitor);
  EXPECT_NE(
    std::find(topic_targets.begin(), topic_targets.end(), "/node_commander_test/out"),
    topic_targets.end());
  EXPECT_NE(
    std::find(topic_targets.begin(), topic_targets.end(), "/node_commander_test/in"),
    topic_targets.end());

  const bool service_child_visible_before = std::any_of(
    screen.detail_lines_cache_.begin(),
    screen.detail_lines_cache_.end(),
    [](const DetailLine & line) {
      return !line.is_header &&
        line.section_key == "services" &&
        line.action == NodeDetailAction::OpenServiceCommander;
    });
  EXPECT_TRUE(service_child_visible_before);

  screen.collapsed_detail_sections_["services"] = true;
  screen.refresh_detail_lines_cache();

  const bool services_header_visible_after = std::any_of(
    screen.detail_lines_cache_.begin(),
    screen.detail_lines_cache_.end(),
    [](const DetailLine & line) {
      return line.is_header && line.section_key == "services";
    });
  const bool service_child_visible_after = std::any_of(
    screen.detail_lines_cache_.begin(),
    screen.detail_lines_cache_.end(),
    [](const DetailLine & line) {
      return !line.is_header &&
        line.section_key == "services" &&
        line.action == NodeDetailAction::OpenServiceCommander;
    });

  EXPECT_TRUE(services_header_visible_after);
  EXPECT_FALSE(service_child_visible_after);

  screen.collapsed_detail_sections_.clear();
  screen.collapsed_detail_sections_["topics"] = true;
  screen.refresh_detail_lines_cache();

  const bool topics_header_visible_after = std::any_of(
    screen.detail_lines_cache_.begin(),
    screen.detail_lines_cache_.end(),
    [](const DetailLine & line) {
      return line.is_header && line.text == "Topics" && line.section_key == "topics";
    });
  const bool topic_child_visible_after = std::any_of(
    screen.detail_lines_cache_.begin(),
    screen.detail_lines_cache_.end(),
    [](const DetailLine & line) {
      return line.section_key.rfind("topics", 0) == 0 && !(line.is_header && line.text == "Topics");
    });

  EXPECT_TRUE(topics_header_visible_after);
  EXPECT_FALSE(topic_child_visible_after);

  screen.collapsed_detail_sections_.clear();
  screen.collapsed_detail_sections_["topics/publishers"] = true;
  screen.refresh_detail_lines_cache();

  const bool publishers_header_visible_after = std::any_of(
    screen.detail_lines_cache_.begin(),
    screen.detail_lines_cache_.end(),
    [](const DetailLine & line) {
      return line.is_header && line.text == "Publishers" && line.section_key == "topics/publishers";
    });
  const bool publisher_child_visible_after = std::any_of(
    screen.detail_lines_cache_.begin(),
    screen.detail_lines_cache_.end(),
    [](const DetailLine & line) {
      return !line.is_header && line.section_key == "topics/publishers";
    });

  EXPECT_TRUE(publishers_header_visible_after);
  EXPECT_FALSE(publisher_child_visible_after);

  stop_executor(backend_executor, backend_spin_thread);
  stop_executor(target_executor, target_spin_thread);
}

TEST_F(NodeCommanderScreenTest, LogViewerLiveLaunchDoesNotPreseedAliasSources) {
  LogViewerLaunchOptions options;
  options.initial_live_source = "attention_layer";
  options.live_source_aliases = {"/attention_layer", "attention_layer"};
  options.open_live_source_on_start = true;

  auto backend = std::make_shared<LogViewerBackend>(options);

  EXPECT_EQ(backend->view_mode_, LogViewerViewMode::SourceLive);
  EXPECT_EQ(backend->live_source_name_, "attention_layer");
  EXPECT_TRUE(backend->source_snapshot().empty());
  EXPECT_TRUE(backend->log_source_matches_live_source("/attention_layer"));
  EXPECT_TRUE(backend->log_source_matches_live_source("attention_layer"));
}

}  // namespace
}  // namespace ros2_console_tools
