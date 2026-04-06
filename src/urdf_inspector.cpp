#include "ros2_console_tools/urdf_inspector.hpp"
#include <vector>

int main(int argc, char ** argv) {
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  rclcpp::init(argc, argv);
  const std::string target_node = args.size() > 1 ? args[1] : "";

  const int result = ros2_console_tools::run_urdf_inspector_tool(target_node);
  rclcpp::shutdown();
  return result;
}
