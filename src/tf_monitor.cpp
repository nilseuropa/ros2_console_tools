#include "ros2_console_tools/tf_monitor.hpp"

#include <thread>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto backend = std::make_shared<ros2_console_tools::TfMonitorBackend>();
  backend->initialize_subscriptions();
  ros2_console_tools::TfMonitorScreen screen(backend);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(backend);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const int result = screen.run();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
