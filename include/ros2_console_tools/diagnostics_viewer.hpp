#ifndef ROS2_CONSOLE_TOOLS__DIAGNOSTICS_VIEWER_HPP_
#define ROS2_CONSOLE_TOOLS__DIAGNOSTICS_VIEWER_HPP_

#ifdef OK
#undef OK
#endif

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_diagnostics_viewer_tool(bool embedded_mode = false);

using DiagnosticsClock = std::chrono::steady_clock;
using DiagnosticArray = diagnostic_msgs::msg::DiagnosticArray;
using DiagnosticStatus = diagnostic_msgs::msg::DiagnosticStatus;
using DiagnosticKeyValue = diagnostic_msgs::msg::KeyValue;

constexpr uint8_t kDiagnosticLevelOk = 0;
constexpr uint8_t kDiagnosticLevelWarn = 1;
constexpr uint8_t kDiagnosticLevelError = 2;
constexpr uint8_t kDiagnosticLevelStale = 3;

inline std::string diagnostic_level_string(uint8_t level) {
  switch (level) {
    case kDiagnosticLevelOk:
      return "OK";
    case kDiagnosticLevelWarn:
      return "WARN";
    case kDiagnosticLevelError:
      return "ERROR";
    case kDiagnosticLevelStale:
      return "STALE";
    default:
      return "UNKNOWN";
  }
}

inline int diagnostic_level_color(uint8_t level, bool selected) {
  switch (level) {
    case kDiagnosticLevelOk:
      return selected ? tui::kColorPositiveSelection : tui::kColorPositive;
    case kDiagnosticLevelWarn:
      return selected ? tui::kColorSelection : tui::kColorWarn;
    case kDiagnosticLevelError:
      return selected ? tui::kColorSelection : tui::kColorError;
    case kDiagnosticLevelStale:
      return selected ? tui::kColorFatal : tui::kColorFatal;
    default:
      return selected ? tui::kColorSelection : 0;
  }
}

inline std::string diagnostic_time_string(const builtin_interfaces::msg::Time & stamp) {
  if (stamp.sec == 0 && stamp.nanosec == 0U) {
    return "-";
  }
  return std::to_string(stamp.sec) + "." + std::to_string(stamp.nanosec / 1000000U);
}

struct DiagnosticEntry {
  std::string key;
  std::string name;
  std::string hardware_id;
  std::string message;
  std::vector<DiagnosticKeyValue> values;
  uint8_t level{kDiagnosticLevelOk};
  builtin_interfaces::msg::Time message_stamp;
  DiagnosticsClock::time_point received_time{};
  std::string source_topic;
};

struct DiagnosticStatusRow {
  std::string key;
  std::string name;
  std::string hardware_id;
  std::string message;
  uint8_t level{kDiagnosticLevelOk};
  std::string age;
};

struct DiagnosticsDetailLine {
  std::string text;
  bool is_header{false};
};

enum class DiagnosticsPaneFocus {
  StatusList,
  Details,
};

class DiagnosticsViewerScreen;

class DiagnosticsViewerBackend : public rclcpp::Node {
public:
  DiagnosticsViewerBackend();
  void initialize_subscriptions();

private:
  friend class DiagnosticsViewerScreen;

  static constexpr auto kEntryStaleAfter = std::chrono::seconds(5);

  void on_diagnostics_message(const DiagnosticArray & message, const std::string & source_topic);
  void clear_entries();
  void cycle_minimum_level();
  void clamp_status_selection(const std::vector<DiagnosticStatusRow> & snapshot);
  void clamp_detail_selection(const std::vector<DiagnosticsDetailLine> & snapshot);
  std::vector<DiagnosticStatusRow> status_snapshot() const;
  std::vector<DiagnosticsDetailLine> detail_snapshot(const std::string & key) const;
  std::string selected_entry_key(const std::vector<DiagnosticStatusRow> & snapshot) const;
  void set_status(const std::string & text);
  static std::string entry_key(const DiagnosticStatus & status);
  static std::string format_age(const DiagnosticsClock::time_point & received_time);

  mutable std::mutex mutex_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_subscription_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diagnostics_agg_subscription_;
  std::map<std::string, DiagnosticEntry> entries_;
  int selected_status_index_{0};
  int selected_detail_index_{0};
  int status_scroll_{0};
  int detail_scroll_{0};
  uint8_t minimum_level_{kDiagnosticLevelOk};
  std::string status_line_{"Waiting for diagnostics..."};
};

class DiagnosticsViewerScreen {
public:
  explicit DiagnosticsViewerScreen(
    std::shared_ptr<DiagnosticsViewerBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  bool handle_search_key(int key);
  bool handle_status_list_key(int key);
  bool handle_detail_key(int key);
  int page_step() const;
  void draw();
  void draw_status_list(int top, int left, int bottom, int right);
  void draw_details(int top, int left, int bottom, int right);
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<DiagnosticsViewerBackend> backend_;
  bool embedded_mode_{false};
  tui::SearchState search_state_;
  DiagnosticsPaneFocus focus_{DiagnosticsPaneFocus::StatusList};
  tui::TerminalPane terminal_pane_;
};

}  // namespace ros2_console_tools

#endif  // ROS2_CONSOLE_TOOLS__DIAGNOSTICS_VIEWER_HPP_
