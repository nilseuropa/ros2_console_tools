#include "ros2_console_tools/action_commander.hpp"

#include <vector>

int main(int argc, char * argv[]) {
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  rclcpp::init(argc, argv);
  const std::string initial_action = args.size() > 1 ? args[1] : "";
  const int result = ros2_console_tools::run_action_commander_tool(initial_action, false);
  rclcpp::shutdown();
  return result;
}
