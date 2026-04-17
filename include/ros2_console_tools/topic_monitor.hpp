#ifndef ROS2_CONSOLE_TOOLS__TOPIC_MONITOR_HPP_
#define ROS2_CONSOLE_TOOLS__TOPIC_MONITOR_HPP_

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <optional>
#include <set>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

struct TopicMonitorLaunchOptions {
  std::string initial_topic;
  std::vector<std::string> allowed_topics;
  bool embedded_mode{false};
  bool open_initial_topic_detail{false};
  bool monitor_allowed_topics_on_start{false};
  bool exit_on_detail_escape{false};
};

int run_topic_monitor_tool(const TopicMonitorLaunchOptions & options);
int run_topic_monitor_tool(const std::string & initial_topic = "", bool embedded_mode = false);

using TopicClock = std::chrono::steady_clock;
using MessageMember = rosidl_typesupport_introspection_cpp::MessageMember;
using MessageMembers = rosidl_typesupport_introspection_cpp::MessageMembers;

inline std::string format_hz(double hz) {
  if (hz <= 0.0) {
    return "-";
  }

  std::ostringstream stream;
  if (hz >= 100.0) {
    stream << std::fixed << std::setprecision(0) << hz;
  } else if (hz >= 10.0) {
    stream << std::fixed << std::setprecision(1) << hz;
  } else {
    stream << std::fixed << std::setprecision(2) << hz;
  }
  return stream.str();
}

inline std::string format_bandwidth(double bytes_per_second) {
  if (bytes_per_second <= 0.0) {
    return "-";
  }

  static const char * kUnits[] = {"B/s", "KB/s", "MB/s", "GB/s"};
  int unit_index = 0;
  while (bytes_per_second >= 1024.0 && unit_index < 3) {
    bytes_per_second /= 1024.0;
    ++unit_index;
  }

  std::ostringstream stream;
  if (bytes_per_second >= 100.0) {
    stream << std::fixed << std::setprecision(0);
  } else if (bytes_per_second >= 10.0) {
    stream << std::fixed << std::setprecision(1);
  } else {
    stream << std::fixed << std::setprecision(2);
  }
  stream << bytes_per_second << kUnits[unit_index];
  return stream.str();
}

inline std::string format_duration(double seconds) {
  if (seconds <= 0.0) {
    return "-";
  }

  std::ostringstream stream;
  if (seconds >= 60.0) {
    const auto minutes = static_cast<int>(seconds / 60.0);
    const double remaining_seconds = seconds - (static_cast<double>(minutes) * 60.0);
    stream << minutes << "m";
    if (remaining_seconds >= 10.0) {
      stream << std::fixed << std::setprecision(0) << remaining_seconds << "s";
    } else {
      stream << std::fixed << std::setprecision(1) << remaining_seconds << "s";
    }
  } else if (seconds >= 10.0) {
    stream << std::fixed << std::setprecision(1) << seconds << "s";
  } else if (seconds >= 1.0) {
    stream << std::fixed << std::setprecision(2) << seconds << "s";
  } else {
    stream << std::fixed << std::setprecision(0) << (seconds * 1000.0) << "ms";
  }
  return stream.str();
}

template<typename T>
inline std::string scalar_to_string(const T & value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

template<>
inline std::string scalar_to_string<bool>(const bool & value) {
  return value ? "true" : "false";
}

template<>
inline std::string scalar_to_string<uint8_t>(const uint8_t & value) {
  return std::to_string(static_cast<unsigned int>(value));
}

template<>
inline std::string scalar_to_string<int8_t>(const int8_t & value) {
  return std::to_string(static_cast<int>(value));
}

struct Sample {
  TopicClock::time_point time;
  std::size_t bytes;
};

struct IntervalSample {
  TopicClock::time_point time;
  double seconds;
};

struct DetailRow {
  int depth{0};
  std::string field;
  std::string value;
  std::string path;
  bool numeric{false};
  double numeric_value{0.0};
};

struct PlotSample {
  TopicClock::time_point time;
  double value{0.0};
};

struct TopicIntrospection {
  std::shared_ptr<rcpputils::SharedLibrary> cpp_library;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const rosidl_message_type_support_t * cpp_typesupport{nullptr};
  const MessageMembers * members{nullptr};
  std::unique_ptr<rclcpp::SerializationBase> serialization;
};

struct TopicEntry {
  std::string name;
  std::string type;
  bool monitored{false};
  std::string monitor_error;
  bool has_last_message{false};
  TopicClock::time_point last_message_time{};
  bool has_last_observed_timestamp{false};
  std::int64_t last_observed_timestamp_ns{0};
  std::deque<Sample> samples;
  std::deque<IntervalSample> intervals;
  std::size_t sample_bytes_sum{0};
  std::size_t total_missed_messages{0};
  std::size_t counted_current_gap_missed_messages{0};
  bool current_gap_visible_active{false};
  TopicClock::time_point next_gap_count_time{};
  double current_gap_average_period_seconds{0.0};
  double last_recovery_duration_seconds{0.0};
  double recovery_duration_sum_seconds{0.0};
  std::size_t recovery_count{0};
  std::vector<DetailRow> detail_rows;
  std::map<std::string, std::deque<PlotSample>> plot_samples;
  std::string detail_error;
};

struct TopicRow {
  std::string name;
  std::string type;
  bool monitored{false};
  bool stale{false};
  std::string avg_hz;
  std::string min_max_hz;
  std::string bandwidth;
  std::string last_recovery_time;
  std::string avg_recovery_time;
  bool has_expected_frequency{false};
  std::size_t current_missed_messages{0};
  std::size_t total_missed_messages{0};
};

struct TopicListItem {
  bool is_namespace{false};
  std::string label;
  std::string namespace_path;
  int depth{0};
  TopicRow row;
};

enum class TopicMonitorViewMode {
  TopicList,
  TopicDetail,
};

class TopicMonitorScreen;

class TopicMonitorBackend : public rclcpp::Node {
public:
  explicit TopicMonitorBackend(const TopicMonitorLaunchOptions & options = {});

private:
  friend class TopicMonitorScreen;

  static constexpr auto kWindowDuration = std::chrono::seconds(5);
  static constexpr auto kStaleAfter = std::chrono::milliseconds(1500);
  static constexpr double kMissGraceFraction = 0.25;

  void refresh_topics();
  void apply_startup_behavior();
  void warm_up_topic_list();
  void maybe_refresh_topics();
  std::vector<TopicRow> topic_rows_snapshot();
  std::vector<DetailRow> detail_rows_snapshot(const std::string & topic_name) const;
  std::vector<DetailRow> visible_detail_rows_snapshot(const std::string & topic_name) const;
  std::vector<PlotSample> plot_samples_snapshot(
    const std::string & topic_name, const std::string & field_path) const;
  std::string detail_error_snapshot(const std::string & topic_name) const;
  static std::optional<double> average_period_seconds(const TopicEntry & entry);
  static std::size_t missed_messages_for_gap(double seconds_since_last, double average_period);
  static void advance_visible_gap_counts(TopicEntry & entry, TopicClock::time_point now);
  void update_active_gap_counters();
  static void compute_stats(TopicEntry & entry, TopicClock::time_point now, TopicRow & row);
  void toggle_selected_topic_monitoring();
  void open_selected_topic_detail();
  void close_topic_detail();
  rclcpp::QoS subscription_qos_for_topic(const std::string & topic_name) const;
  rclcpp::QoS compatible_subscription_qos(const std::vector<rclcpp::QoS> & publisher_qos_profiles) const;
  void start_monitoring(const TopicEntry & entry);
  void stop_monitoring(const std::string & topic_name);
  void on_message(const std::string & topic_name, const rclcpp::SerializedMessage & message);
  void on_message(
    const std::string & topic_name, const rclcpp::SerializedMessage & message,
    const rclcpp::MessageInfo & message_info);
  void on_message(
    const std::string & topic_name, const rclcpp::SerializedMessage & message,
    const rclcpp::MessageInfo * message_info);
  TopicIntrospection & get_or_create_introspection(const std::string & type);
  std::vector<DetailRow> decode_detail_rows(
    const std::string & type, const rclcpp::SerializedMessage & message);
  void append_message_members(
    std::vector<DetailRow> & rows, int depth, const std::string & parent_path,
    const MessageMembers * members, const void * message_memory) const;
  std::string normalize_member_label(
    const MessageMembers & parent_members, const MessageMember & member) const;
  void append_member(
    std::vector<DetailRow> & rows, int depth, const std::string & parent_path, const std::string & label,
    const MessageMember & member, const void * field_memory) const;
  std::string array_element_to_string(
    const MessageMember & member, const void * field_memory, std::size_t index) const;
  std::string scalar_field_to_string(uint8_t type_id, const void * field_memory) const;
  std::optional<double> scalar_field_to_double(uint8_t type_id, const void * field_memory) const;
  void clamp_topic_selection(const std::vector<TopicListItem> & items);
  std::string topic_namespace(const std::string & topic_name) const;
  std::string topic_leaf_name(const std::string & topic_name) const;
  bool is_topic_namespace_expanded(const std::string & namespace_path) const;
  std::vector<TopicListItem> visible_topic_items();
  void expand_selected_namespace();
  void collapse_selected_namespace();
  void clamp_detail_selection(const std::vector<DetailRow> & rows);
  void clamp_visible_detail_selection();
  void expand_selected_detail_row();
  void collapse_selected_detail_row();
  bool selected_detail_row_can_plot() const;
  std::optional<DetailRow> selected_visible_detail_row() const;
  bool detail_row_has_children(const std::vector<DetailRow> & rows, std::size_t index) const;
  bool topic_is_allowed(const std::string & topic_name) const;
  bool select_initial_topic_if_present();
  void expand_topic_namespace_path(const std::string & namespace_path);

  mutable std::mutex mutex_;
  std::map<std::string, TopicEntry> topics_;
  std::map<std::string, TopicIntrospection> introspection_cache_;
  std::map<std::string, bool> collapsed_topic_namespaces_;
  std::vector<std::pair<std::string, rclcpp::GenericSubscription::SharedPtr>> monitored_subscriptions_;
  TopicMonitorViewMode view_mode_{TopicMonitorViewMode::TopicList};
  std::set<std::string> allowed_topic_names_;
  std::string initial_topic_name_;
  bool open_initial_topic_detail_{false};
  bool monitor_allowed_topics_on_start_{false};
  bool exit_on_detail_escape_{false};
  bool pending_namespace_expansion_{false};
  int selected_index_{0};
  int list_scroll_{0};
  int selected_detail_index_{0};
  int detail_scroll_{0};
  std::map<std::string, bool> collapsed_detail_paths_;
  bool show_only_monitored_{false};
  std::string detail_topic_name_;
  std::string status_line_{"Loading topics..."};
  TopicClock::time_point last_refresh_time_{};
  rclcpp::TimerBase::SharedPtr gap_counter_timer_;
};

class TopicMonitorScreen {
public:
  explicit TopicMonitorScreen(std::shared_ptr<TopicMonitorBackend> backend, bool embedded_mode = false);
  int run();

private:
  bool handle_key(int key);
  bool handle_topic_list_key(int key);
  bool handle_search_key(int key);
  bool handle_topic_detail_key(int key);
  bool launch_selected_visualizer();
  bool launch_selected_plot();
  int page_step() const;
  void draw();
  void draw_topic_list(int top, int left, int bottom, int right);
  void draw_topic_detail(int top, int left, int bottom, int right);
  void draw_plot_popup(int rows, int columns) const;
  void draw_status_line(int row, int columns) const;
  void draw_help_line(int row, int columns) const;

  std::shared_ptr<TopicMonitorBackend> backend_;
  bool embedded_mode_{false};
  tui::SearchState search_state_;
  bool plot_popup_open_{false};
  std::string plot_topic_name_;
  std::string plot_field_name_;
  std::string plot_field_path_;
  tui::TerminalPane terminal_pane_;
};

}  // namespace ros2_console_tools

#endif
