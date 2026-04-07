#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

#define private public
#include "ros2_console_tools/node_commander.hpp"
#include "ros2_console_tools/service_commander.hpp"
#undef private

namespace ros2_console_tools {
namespace {

class ServiceCommanderBackendTest : public ::testing::Test {
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

TEST_F(ServiceCommanderBackendTest, CallsParameterServiceWithEmbeddedNodeCommanderAlive) {
  auto target_node = std::make_shared<rclcpp::Node>("service_commander_test_target");
  target_node->declare_parameter("example_parameter", 7);

  auto node_commander_backend = std::make_shared<NodeCommanderBackend>();
  auto service_commander_backend = std::make_shared<ServiceCommanderBackend>();

  rclcpp::executors::SingleThreadedExecutor target_executor;
  rclcpp::executors::SingleThreadedExecutor node_executor;
  rclcpp::executors::SingleThreadedExecutor service_executor;
  target_executor.add_node(target_node);
  node_executor.add_node(node_commander_backend);
  service_executor.add_node(service_commander_backend);

  std::thread target_spin_thread([&target_executor]() { target_executor.spin(); });
  std::thread node_spin_thread([&node_executor]() { node_executor.spin(); });
  std::thread service_spin_thread([&service_executor]() { service_executor.spin(); });

  auto stop_executor = [](rclcpp::executors::SingleThreadedExecutor & executor, std::thread & thread) {
      executor.cancel();
      if (thread.joinable()) {
        thread.join();
      }
    };

  const ServiceEntry selected_service{
    "/service_commander_test_target/get_parameters",
    "rcl_interfaces/srv/GetParameters"
  };
  service_commander_backend->selected_service_ = selected_service;

  bool request_loaded = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    service_commander_backend->reload_selected_service_request();
    if (!service_commander_backend->request_rows_.empty()) {
      request_loaded = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(request_loaded);

  if (request_loaded) {
    service_commander_backend->call_selected_service();

    EXPECT_TRUE(service_commander_backend->response_error_.empty())
      << service_commander_backend->response_error_;
    EXPECT_EQ(
      service_commander_backend->status_line_,
      "Called /service_commander_test_target/get_parameters.");
    EXPECT_FALSE(service_commander_backend->response_rows_.empty());

    service_commander_backend->close_service_detail();
  }

  stop_executor(service_executor, service_spin_thread);
  stop_executor(node_executor, node_spin_thread);
  stop_executor(target_executor, target_spin_thread);
}

TEST_F(ServiceCommanderBackendTest, SelectsNodeAndFiltersServicesToThatNode) {
  auto target_node = std::make_shared<rclcpp::Node>("service_commander_test_target");
  target_node->declare_parameter("example_parameter", 7);

  auto other_node = std::make_shared<rclcpp::Node>("service_commander_test_other");
  other_node->declare_parameter("example_parameter", 11);

  auto service_commander_backend = std::make_shared<ServiceCommanderBackend>();

  rclcpp::executors::SingleThreadedExecutor target_executor;
  rclcpp::executors::SingleThreadedExecutor other_executor;
  rclcpp::executors::SingleThreadedExecutor service_executor;
  target_executor.add_node(target_node);
  other_executor.add_node(other_node);
  service_executor.add_node(service_commander_backend);

  std::thread target_spin_thread([&target_executor]() { target_executor.spin(); });
  std::thread other_spin_thread([&other_executor]() { other_executor.spin(); });
  std::thread service_spin_thread([&service_executor]() { service_executor.spin(); });

  auto stop_executor = [](rclcpp::executors::SingleThreadedExecutor & executor, std::thread & thread) {
      executor.cancel();
      if (thread.joinable()) {
        thread.join();
      }
    };

  bool filtered_services_loaded = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    service_commander_backend->refresh_node_list();
    const auto found = std::find(
      service_commander_backend->node_entries_.begin(),
      service_commander_backend->node_entries_.end(),
      "/service_commander_test_target");
    if (found != service_commander_backend->node_entries_.end()) {
      service_commander_backend->selected_node_index_ = static_cast<int>(
        std::distance(service_commander_backend->node_entries_.begin(), found));
      service_commander_backend->select_current_node();
      if (!service_commander_backend->services_.empty()) {
        filtered_services_loaded = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_EQ(service_commander_backend->target_node_, "/service_commander_test_target");
  EXPECT_EQ(service_commander_backend->view_mode_, ServiceCommanderViewMode::ServiceList);
  EXPECT_TRUE(filtered_services_loaded);

  if (filtered_services_loaded) {
    const auto target_service = std::find_if(
      service_commander_backend->services_.begin(),
      service_commander_backend->services_.end(),
      [](const ServiceEntry & entry) {
        return entry.name == "/service_commander_test_target/get_parameters";
      });
    EXPECT_NE(target_service, service_commander_backend->services_.end());

    const auto other_service = std::find_if(
      service_commander_backend->services_.begin(),
      service_commander_backend->services_.end(),
      [](const ServiceEntry & entry) {
        return entry.name == "/service_commander_test_other/get_parameters";
      });
    EXPECT_EQ(other_service, service_commander_backend->services_.end());
  }

  stop_executor(service_executor, service_spin_thread);
  stop_executor(other_executor, other_spin_thread);
  stop_executor(target_executor, target_spin_thread);
}

TEST_F(ServiceCommanderBackendTest, NodeListRefreshDoesNotForcePreviousTargetSelection) {
  auto target_node = std::make_shared<rclcpp::Node>("service_commander_test_target");
  target_node->declare_parameter("example_parameter", 7);

  auto other_node = std::make_shared<rclcpp::Node>("service_commander_test_other");
  other_node->declare_parameter("example_parameter", 11);

  auto service_commander_backend = std::make_shared<ServiceCommanderBackend>();

  rclcpp::executors::SingleThreadedExecutor target_executor;
  rclcpp::executors::SingleThreadedExecutor other_executor;
  rclcpp::executors::SingleThreadedExecutor service_executor;
  target_executor.add_node(target_node);
  other_executor.add_node(other_node);
  service_executor.add_node(service_commander_backend);

  std::thread target_spin_thread([&target_executor]() { target_executor.spin(); });
  std::thread other_spin_thread([&other_executor]() { other_executor.spin(); });
  std::thread service_spin_thread([&service_executor]() { service_executor.spin(); });

  auto stop_executor = [](rclcpp::executors::SingleThreadedExecutor & executor, std::thread & thread) {
      executor.cancel();
      if (thread.joinable()) {
        thread.join();
      }
    };

  bool switched_back_to_node_list = false;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
    service_commander_backend->refresh_node_list();
    const auto target_it = std::find(
      service_commander_backend->node_entries_.begin(),
      service_commander_backend->node_entries_.end(),
      "/service_commander_test_target");
    const auto other_it = std::find(
      service_commander_backend->node_entries_.begin(),
      service_commander_backend->node_entries_.end(),
      "/service_commander_test_other");
    if (
      target_it != service_commander_backend->node_entries_.end() &&
      other_it != service_commander_backend->node_entries_.end())
    {
      service_commander_backend->selected_node_index_ = static_cast<int>(
        std::distance(service_commander_backend->node_entries_.begin(), target_it));
      service_commander_backend->select_current_node();
      service_commander_backend->switch_to_node_list();
      service_commander_backend->selected_node_index_ = static_cast<int>(
        std::distance(service_commander_backend->node_entries_.begin(), other_it));
      service_commander_backend->refresh_node_list();
      switched_back_to_node_list = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  EXPECT_TRUE(switched_back_to_node_list);
  if (
    switched_back_to_node_list &&
    !service_commander_backend->node_entries_.empty() &&
    service_commander_backend->selected_node_index_ >= 0 &&
    service_commander_backend->selected_node_index_ < static_cast<int>(service_commander_backend->node_entries_.size()))
  {
    EXPECT_EQ(
      service_commander_backend->node_entries_[static_cast<std::size_t>(service_commander_backend->selected_node_index_)],
      "/service_commander_test_other");
  }

  stop_executor(service_executor, service_spin_thread);
  stop_executor(other_executor, other_spin_thread);
  stop_executor(target_executor, target_spin_thread);
}

TEST_F(ServiceCommanderBackendTest, MissingTargetNodeDoesNotThrowDuringServiceRefresh) {
  auto service_commander_backend = std::make_shared<ServiceCommanderBackend>("/service_commander_missing_node");

  EXPECT_NO_THROW(service_commander_backend->refresh_node_list());
  EXPECT_NO_THROW(service_commander_backend->refresh_services());
  EXPECT_TRUE(service_commander_backend->selected_service_.name.empty());
}

}  // namespace
}  // namespace ros2_console_tools
