#include "ros2_console_tools/topic_monitor.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  const int result = ros2_console_tools::run_topic_monitor_tool();
  rclcpp::shutdown();
  return result;
}
