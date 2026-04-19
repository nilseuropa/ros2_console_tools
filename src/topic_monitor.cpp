#include "ros2_console_tools/topic_monitor.hpp"

#include <cstdio>
#include <string>

int main(int argc, char ** argv) {
  ros2_console_tools::TopicMonitorLaunchOptions options;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--embedded") {
      options.embedded_mode = true;
      continue;
    }
    if (argument == "--topic") {
      if (index + 1 >= argc) {
        std::fprintf(stderr, "topic_monitor: --topic requires a value\n");
        return 2;
      }
      options.initial_topic = argv[++index];
      continue;
    }
    if (argument == "--allowed-topic") {
      if (index + 1 >= argc) {
        std::fprintf(stderr, "topic_monitor: --allowed-topic requires a value\n");
        return 2;
      }
      options.allowed_topics.push_back(argv[++index]);
      continue;
    }
    if (argument == "--open-topic-detail") {
      options.open_initial_topic_detail = true;
      continue;
    }
    if (argument == "--monitor-allowed") {
      options.monitor_allowed_topics_on_start = true;
      continue;
    }
    if (argument == "--exit-on-detail-escape") {
      options.exit_on_detail_escape = true;
      continue;
    }
    std::fprintf(stderr, "topic_monitor: unknown argument '%s'\n", argument.c_str());
    return 2;
  }

  rclcpp::init(argc, argv);
  const int result = ros2_console_tools::run_topic_monitor_tool(options);
  rclcpp::shutdown();
  return result;
}
