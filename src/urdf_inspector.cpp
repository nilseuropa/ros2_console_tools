#include "ros2_console_tools/urdf_inspector.hpp"
#include <cstdio>
#include <vector>

int main(int argc, char ** argv) {
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  bool embedded_mode = false;
  std::string target_node;

  for (std::size_t index = 1; index < args.size(); ++index) {
    if (args[index] == "--embedded") {
      embedded_mode = true;
      continue;
    }
    if (target_node.empty()) {
      target_node = args[index];
      continue;
    }
    std::fprintf(stderr, "urdf_inspector: unknown argument '%s'\n", args[index].c_str());
    return 2;
  }

  rclcpp::init(argc, argv);

  const int result = ros2_console_tools::run_urdf_inspector_tool(target_node, embedded_mode);
  rclcpp::shutdown();
  return result;
}
