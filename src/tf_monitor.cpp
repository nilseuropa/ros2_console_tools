#include "ros2_console_tools/tf_monitor.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  const int result = ros2_console_tools::run_tf_monitor_tool();
  rclcpp::shutdown();
  return result;
}
