#include "ros2_console_tools/log_viewer.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  const int result = ros2_console_tools::run_log_viewer_tool();
  rclcpp::shutdown();
  return result;
}
