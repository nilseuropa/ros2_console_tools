#include "ros2_console_tools/service_commander.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  const int result = ros2_console_tools::run_service_commander_tool();
  rclcpp::shutdown();
  return result;
}
