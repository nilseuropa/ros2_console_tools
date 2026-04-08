#ifndef ROS2_CONSOLE_TOOLS__JOURNAL_CLIENT_HPP_
#define ROS2_CONSOLE_TOOLS__JOURNAL_CLIENT_HPP_

#include <string>
#include <utility>
#include <vector>

namespace ros2_console_tools {

struct JournalEntry {
  std::string timestamp;
  int priority{6};
  std::string unit;
  std::string identifier;
  std::string message;
  std::string pid;
  std::string code_file;
  std::string code_line;
  std::string code_function;
  std::vector<std::pair<std::string, std::string>> fields;
};

std::string journal_priority_label(int priority);
std::vector<JournalEntry> parse_journal_json_lines(const std::string & text);

class JournalClient {
public:
  std::vector<JournalEntry> read_entries(
    const std::string & unit_filter,
    int max_priority,
    int line_count,
    const std::string & text_filter,
    std::string * error = nullptr) const;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__JOURNAL_CLIENT_HPP_
