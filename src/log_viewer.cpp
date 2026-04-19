#include "ros2_console_tools/log_viewer.hpp"

#include <cstdio>
#include <string>

int main(int argc, char ** argv) {
  ros2_console_tools::LogViewerLaunchOptions options;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--embedded") {
      options.embedded_mode = true;
      continue;
    }
    if (argument == "--initial-live-source") {
      if (index + 1 >= argc) {
        std::fprintf(stderr, "log_viewer: --initial-live-source requires a value\n");
        return 2;
      }
      options.initial_live_source = argv[++index];
      continue;
    }
    if (argument == "--live-source-alias") {
      if (index + 1 >= argc) {
        std::fprintf(stderr, "log_viewer: --live-source-alias requires a value\n");
        return 2;
      }
      options.live_source_aliases.push_back(argv[++index]);
      continue;
    }
    if (argument == "--open-live-source") {
      options.open_live_source_on_start = true;
      continue;
    }
    std::fprintf(stderr, "log_viewer: unknown argument '%s'\n", argument.c_str());
    return 2;
  }

  rclcpp::init(argc, argv);
  const int result = ros2_console_tools::run_log_viewer_tool(options);
  rclcpp::shutdown();
  return result;
}
