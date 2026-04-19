#include "ros2_console_tools/action_commander.hpp"

#include <cstdio>
#include <vector>

int main(int argc, char * argv[]) {
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  bool embedded_mode = false;
  std::string initial_action;

  for (std::size_t index = 1; index < args.size(); ++index) {
    if (args[index] == "--embedded") {
      embedded_mode = true;
      continue;
    }
    if (initial_action.empty()) {
      initial_action = args[index];
      continue;
    }
    std::fprintf(stderr, "action_commander: unknown argument '%s'\n", args[index].c_str());
    return 2;
  }

  rclcpp::init(argc, argv);
  const int result = ros2_console_tools::run_action_commander_tool(initial_action, embedded_mode);
  rclcpp::shutdown();
  return result;
}
