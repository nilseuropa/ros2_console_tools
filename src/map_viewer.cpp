#include "ros2_console_tools/map_viewer.hpp"

#include <cstdio>
#include <vector>

int main(int argc, char * argv[]) {
  const std::vector<std::string> args = rclcpp::remove_ros_arguments(argc, argv);
  bool embedded_mode = false;
  std::string topic;

  for (std::size_t index = 1; index < args.size(); ++index) {
    if (args[index] == "--embedded") {
      embedded_mode = true;
      continue;
    }
    if (topic.empty()) {
      topic = args[index];
      continue;
    }
    std::fprintf(stderr, "map_viewer: unknown argument '%s'\n", args[index].c_str());
    return 2;
  }

  rclcpp::init(argc, argv);
  const int result = ros2_console_tools::run_map_viewer_tool(topic, embedded_mode);
  rclcpp::shutdown();
  return result;
}
