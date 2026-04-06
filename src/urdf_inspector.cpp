#include "ros2_console_tools/urdf_inspector.hpp"

#include <thread>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  const std::string target_node = argc > 1 ? argv[1] : "";

  auto backend = std::make_shared<ros2_console_tools::UrdfInspectorBackend>(target_node);
  ros2_console_tools::UrdfInspectorScreen screen(backend);

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
