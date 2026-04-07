#include "ros2_console_tools/diagnostics_viewer.hpp"

#include <cstdio>
#include <string>

int main(int argc, char ** argv) {
  bool embedded_mode = false;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--embedded") {
      embedded_mode = true;
      continue;
    }
    std::fprintf(stderr, "diagnostics_viewer: unknown argument '%s'\n", argument.c_str());
    return 2;
  }

  rclcpp::init(argc, argv);
  const int result = ros2_console_tools::run_diagnostics_viewer_tool(embedded_mode);
  rclcpp::shutdown();
  return result;
}
