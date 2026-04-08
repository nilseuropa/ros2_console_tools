#ifndef ROS2_CONSOLE_TOOLS__PROCESS_RUNNER_HPP_
#define ROS2_CONSOLE_TOOLS__PROCESS_RUNNER_HPP_

#include <string>
#include <vector>

namespace ros2_console_tools {

struct ProcessResult {
  bool started{false};
  bool exited_normally{false};
  int exit_code{-1};
  std::string output;

  bool succeeded() const {
    return started && exited_normally && exit_code == 0;
  }
};

ProcessResult run_process(const std::vector<std::string> & arguments);

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__PROCESS_RUNNER_HPP_
