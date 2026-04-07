#include <gtest/gtest.h>

#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#define private public
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

TEST_F(NodeCommanderScreenTest, HeaderActionsExposeNodeScopedToolsAndCollapseHidesChildren) {
  auto target_node = std::make_shared<rclcpp::Node>("node_commander_test_target");
  target_node->declare_parameter("example_parameter", 7);
  auto publisher = target_node->create_publisher<geometry_msgs::msg::Twist>("/node_commander_test/out", 10);
  auto subscriber = target_node->create_subscription<geometry_msgs::msg::Twist>(
    "/node_commander_test/in", 10, [](geometry_msgs::msg::Twist::ConstSharedPtr) {});
  (void)publisher;
  (void)subscriber;

  auto backend = std::make_shared<NodeCommanderBackend>();
  NodeCommanderScreen screen(backend);

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

  const auto publishers_header = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return line.is_header &&
        line.text == "Publishers" &&
        line.action == NodeDetailAction::OpenTopicMonitor &&
        line.target == "/node_commander_test_target" &&
        line.section_key == "publishers";
    });
  EXPECT_NE(publishers_header, lines.end());

  const auto subscribers_header = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return line.is_header &&
        line.text == "Subscribers" &&
        line.action == NodeDetailAction::OpenTopicMonitor &&
        line.target == "/node_commander_test_target" &&
        line.section_key == "subscribers";
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
        line.section_key == "publishers" &&
        line.action == NodeDetailAction::OpenTopicMonitor &&
        line.target == "/node_commander_test/out";
    });
  EXPECT_NE(publisher_entry, lines.end());

  const auto subscriber_entry = std::find_if(
    lines.begin(),
    lines.end(),
    [](const DetailLine & line) {
      return !line.is_header &&
        line.section_key == "subscribers" &&
        line.action == NodeDetailAction::OpenTopicMonitor &&
        line.target == "/node_commander_test/in";
    });
  EXPECT_NE(subscriber_entry, lines.end());

  screen.refresh_detail_lines_cache();
  const auto publisher_targets =
    screen.detail_targets_for_section("publishers", NodeDetailAction::OpenTopicMonitor);
  const auto subscriber_targets =
    screen.detail_targets_for_section("subscribers", NodeDetailAction::OpenTopicMonitor);
  EXPECT_NE(
    std::find(publisher_targets.begin(), publisher_targets.end(), "/node_commander_test/out"),
    publisher_targets.end());
  EXPECT_NE(
    std::find(subscriber_targets.begin(), subscriber_targets.end(), "/node_commander_test/in"),
    subscriber_targets.end());

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

  stop_executor(backend_executor, backend_spin_thread);
  stop_executor(target_executor, target_spin_thread);
}

}  // namespace
}  // namespace ros2_console_tools
