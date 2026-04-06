#include "ros2_console_tools/topic_monitor.hpp"

#include <thread>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto backend = std::make_shared<ros2_console_tools::TopicMonitorBackend>();
  ros2_console_tools::TopicMonitorScreen screen(backend);

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
