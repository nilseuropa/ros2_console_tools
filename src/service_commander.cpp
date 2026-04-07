#include "ros2_console_tools/service_commander.hpp"

#include <cstdio>
#include <string>

int main(int argc, char ** argv) {
  std::string initial_node;
  std::string initial_service;
  bool embedded_mode = false;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--embedded") {
      embedded_mode = true;
      continue;
    }
    if (argument == "--node") {
      if (index + 1 >= argc) {
        std::fprintf(stderr, "service_commander: --node requires a value\n");
        return 2;
      }
      initial_node = argv[++index];
      continue;
    }
    if (argument == "--service") {
      if (index + 1 >= argc) {
        std::fprintf(stderr, "service_commander: --service requires a value\n");
        return 2;
      }
      initial_service = argv[++index];
      continue;
    }
    std::fprintf(stderr, "service_commander: unknown argument '%s'\n", argument.c_str());
    return 2;
  }

  rclcpp::init(argc, argv);
  const int result =
    ros2_console_tools::run_service_commander_tool(initial_node, initial_service, embedded_mode);
  rclcpp::shutdown();
  return result;
}
