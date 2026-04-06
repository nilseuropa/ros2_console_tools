#ifndef ROS2_CONSOLE_TOOLS__LOG_VIEWER_HPP_
#define ROS2_CONSOLE_TOOLS__LOG_VIEWER_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <set>
#include <string>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_log_viewer_tool(bool embedded_mode = false);

using LogViewerClock = std::chrono::steady_clock;
using LogMessage = rcl_interfaces::msg::Log;

inline std::string time_string(const builtin_interfaces::msg::Time & stamp) {
  std::ostringstream stream;
  stream << stamp.sec << "." << std::setw(3) << std::setfill('0') << (stamp.nanosec / 1000000U);
  return stream.str();
}

inline std::string level_string(uint8_t level) {
  switch (level) {
    case LogMessage::DEBUG:
      return "DEBUG";
    case LogMessage::INFO:
      return "INFO";
    case LogMessage::WARN:
      return "WARN";
    case LogMessage::ERROR:
      return "ERROR";
    case LogMessage::FATAL:
      return "FATAL";
    default:
      return "LOG";
  }
}

inline int level_color(uint8_t level, bool selected) {
  switch (level) {
    case LogMessage::WARN:
      return selected ? tui::kColorSelection : tui::kColorWarn;
    case LogMessage::ERROR:
      return selected ? tui::kColorSelection : tui::kColorError;
    case LogMessage::FATAL:
      return selected ? tui::kColorFatal : tui::kColorFatal;
    case LogMessage::DEBUG:
    case LogMessage::INFO:
    default:
      return selected ? tui::kColorSelection : 0;
  }
}

struct LogEntry {
  builtin_interfaces::msg::Time stamp;
  uint8_t level{0};
  std::string source;
  std::string message;
  std::string file;
  std::string function_name;
  uint32_t line{0};
};

struct SourceEntry {
  std::string name;
  bool selected{false};
};

struct StyledSpan {
  std::string text;
  int color{0};
};

enum class PaneFocus {
  Sources,
  Logs,
};

enum class LogViewerViewMode {
  Split,
  SourceLive,
  CodeInspect,
};

class LogViewerScreen;

class LogViewerBackend : public rclcpp::Node {
public:
  LogViewerBackend();
  void initialize_subscription();

private:
  friend class LogViewerScreen;

  static constexpr std::size_t kMaxLogs = 2000;

  void cycle_minimum_level();
  void on_log_message(const LogMessage & message);
  void refresh_sources();
  std::vector<SourceEntry> source_snapshot() const;
  std::vector<LogEntry> filtered_logs_snapshot() const;
  std::vector<LogEntry> live_source_logs_snapshot() const;
  void toggle_selected_source();
  void open_detail_popup(const std::vector<LogEntry> & snapshot);
  void open_live_detail_popup(const std::vector<LogEntry> & snapshot);
  void open_live_source();
  void close_live_source();
  void open_code_view();
  void close_code_view();
  void clamp_source_selection(const std::vector<SourceEntry> & snapshot);
  void clamp_log_selection(const std::vector<LogEntry> & snapshot);
  void clamp_live_log_selection(const std::vector<LogEntry> & snapshot);
  void set_status(const std::string & text);
  void set_status_locked(const std::string & text) const;

  mutable std::mutex mutex_;
  rclcpp::Subscription<LogMessage>::SharedPtr log_subscription_;
  std::deque<LogEntry> logs_;
  std::map<std::string, bool> sources_;
  LogViewerViewMode view_mode_{LogViewerViewMode::Split};
  PaneFocus focus_{PaneFocus::Sources};
  int selected_source_index_{0};
  int selected_log_index_{0};
  int selected_live_log_index_{0};
  int source_scroll_{0};
  int log_scroll_{0};
  int live_log_scroll_{0};
  bool hide_unselected_{false};
  uint8_t minimum_level_{LogMessage::INFO};
  std::string text_filter_;
  std::string live_source_name_;
  bool filter_prompt_open_{false};
  std::string filter_buffer_;
  bool detail_popup_open_{false};
  LogEntry detail_entry_;
  std::vector<std::string> code_view_lines_;
  std::string code_view_path_;
  int code_view_top_line_{0};
  int code_view_target_line_{0};
  mutable std::string status_line_{"Waiting for /rosout..."};
  LogViewerClock::time_point last_message_time_{};
};

class LogViewerScreen {
public:
  explicit LogViewerScreen(std::shared_ptr<LogViewerBackend> backend, bool embedded_mode = false);
  int run();

private:
  static bool is_identifier_char(char character);
  static bool is_keyword(const std::string & token);

  bool handle_key(int key);
  bool handle_search_key(int key);
  bool handle_filter_prompt_key(int key);
  bool handle_detail_popup_key(int key);
  bool handle_source_key(int key);
  bool handle_live_source_key(int key);
  bool handle_log_key(int key);
  bool handle_code_view_key(int key);
  int page_step() const;
  void draw();
  std::vector<StyledSpan> highlight_code_line(const std::string & line) const;
  void draw_code_line(int row, int left, int width, int line_number, const std::string & line, bool highlight) const;
  void draw_code_view_pane(int top, int left, int bottom, int right) const;
  void draw_sources_pane(int top, int left, int bottom, int right);
  void draw_logs_pane(int top, int left, int bottom, int right);
  void draw_live_source_pane(int top, int left, int bottom, int right);
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;
  void draw_detail_popup(int rows, int columns) const;

  std::shared_ptr<LogViewerBackend> backend_;
  bool embedded_mode_{false};
  tui::SearchState search_state_;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__LOG_VIEWER_HPP_
