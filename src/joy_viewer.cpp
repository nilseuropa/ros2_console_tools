#include "ros2_console_tools/joy_viewer.hpp"

#include <vector>

int main(int argc, char * argv[]) {
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  rclcpp::init(argc, argv);
  const std::string topic = args.size() > 1 ? args[1] : "";
  const int result = ros2_console_tools::run_joy_viewer_tool(topic, false);
  rclcpp::shutdown();
  return result;
}
