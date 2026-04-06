#include "ros2_console_tools/parameter_commander.hpp"

#include <thread>
#include <vector>

int main(int argc, char ** argv) {
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  rclcpp::init(argc, argv);
  const std::string target_node = args.size() > 1 ? args[1] : "";

  auto backend = std::make_shared<ros2_console_tools::ParameterCommanderBackend>(target_node);
  ros2_console_tools::ParameterCommanderScreen screen(backend);

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
