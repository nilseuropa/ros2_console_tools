#include "ros2_console_tools/diagnostics_viewer.hpp"

#include <algorithm>
#include <cstdio>
#include <utility>

namespace ros2_console_tools {

DiagnosticsViewerBackend::DiagnosticsViewerBackend()
: Node("diagnostics_viewer")
{
  const std::string theme_config_path =
    this->declare_parameter<std::string>("theme_config_path", tui::default_theme_config_path());
  std::string theme_error;
  if (!tui::load_theme_from_file(theme_config_path, &theme_error)) {
    if (theme_config_path != tui::default_theme_config_path()) {
      RCLCPP_WARN(this->get_logger(), "%s", theme_error.c_str());
    }
  }
}

void DiagnosticsViewerBackend::initialize_subscriptions() {
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(50));
  diagnostics_subscription_ = this->create_subscription<DiagnosticArray>(
    "/diagnostics", qos,
    [this](const DiagnosticArray & message) { on_diagnostics_message(message, "/diagnostics"); });
  diagnostics_agg_subscription_ = this->create_subscription<DiagnosticArray>(
    "/diagnostics_agg", qos,
    [this](const DiagnosticArray & message) { on_diagnostics_message(message, "/diagnostics_agg"); });
}

void DiagnosticsViewerBackend::on_diagnostics_message(
  const DiagnosticArray & message, const std::string & source_topic)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (const auto & status : message.status) {
    DiagnosticEntry entry;
    entry.key = entry_key(status);
    entry.name = status.name.empty() ? "<unnamed>" : status.name;
    entry.hardware_id = status.hardware_id;
    entry.message = status.message;
    entry.values = status.values;
    entry.level = status.level;
    entry.message_stamp = message.header.stamp;
    entry.received_time = DiagnosticsClock::now();
    entry.source_topic = source_topic;
    entries_[entry.key] = std::move(entry);
  }

  status_line_ = entries_.empty()
    ? "Waiting for diagnostics..."
    : "Loaded " + std::to_string(entries_.size()) + " diagnostic statuses.";
}

void DiagnosticsViewerBackend::clear_entries() {
  std::lock_guard<std::mutex> lock(mutex_);
  entries_.clear();
  selected_status_index_ = 0;
  selected_detail_index_ = 0;
  status_scroll_ = 0;
  detail_scroll_ = 0;
  status_line_ = "Cleared diagnostics cache.";
}

void DiagnosticsViewerBackend::cycle_minimum_level() {
  switch (minimum_level_) {
    case kDiagnosticLevelOk:
      minimum_level_ = kDiagnosticLevelWarn;
      break;
    case kDiagnosticLevelWarn:
      minimum_level_ = kDiagnosticLevelError;
      break;
    case kDiagnosticLevelError:
      minimum_level_ = kDiagnosticLevelStale;
      break;
    case kDiagnosticLevelStale:
    default:
      minimum_level_ = kDiagnosticLevelOk;
      break;
  }
  selected_status_index_ = 0;
  status_scroll_ = 0;
  set_status("Minimum level: " + diagnostic_level_string(minimum_level_) + ".");
}

void DiagnosticsViewerBackend::clamp_status_selection(const std::vector<DiagnosticStatusRow> & snapshot) {
  if (snapshot.empty()) {
    selected_status_index_ = 0;
    status_scroll_ = 0;
    return;
  }
  selected_status_index_ = std::clamp(selected_status_index_, 0, static_cast<int>(snapshot.size()) - 1);
}

void DiagnosticsViewerBackend::clamp_detail_selection(const std::vector<DiagnosticsDetailLine> & snapshot) {
  if (snapshot.empty()) {
    selected_detail_index_ = 0;
    detail_scroll_ = 0;
    return;
  }
  selected_detail_index_ = std::clamp(selected_detail_index_, 0, static_cast<int>(snapshot.size()) - 1);
}

std::vector<DiagnosticStatusRow> DiagnosticsViewerBackend::status_snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<DiagnosticStatusRow> snapshot;
  snapshot.reserve(entries_.size());

  for (const auto & [key, entry] : entries_) {
    if (entry.level < minimum_level_) {
      continue;
    }
    snapshot.push_back({
      key,
      entry.name,
      entry.hardware_id,
      entry.message,
      entry.level,
      format_age(entry.received_time)});
  }

  std::sort(
    snapshot.begin(), snapshot.end(),
    [](const DiagnosticStatusRow & lhs, const DiagnosticStatusRow & rhs) {
      if (lhs.level != rhs.level) {
        return lhs.level > rhs.level;
      }
      if (lhs.name != rhs.name) {
        return lhs.name < rhs.name;
      }
      return lhs.hardware_id < rhs.hardware_id;
    });
  return snapshot;
}

std::vector<DiagnosticsDetailLine> DiagnosticsViewerBackend::detail_snapshot(const std::string & key) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = entries_.find(key);
  if (found == entries_.end()) {
    return {{"No diagnostic status selected.", false}};
  }

  const auto & entry = found->second;
  std::vector<DiagnosticsDetailLine> lines;
  lines.reserve(12 + entry.values.size());

  lines.push_back({"Status", true});
  lines.push_back({"Name: " + entry.name, false});
  lines.push_back({"Level: " + diagnostic_level_string(entry.level), false});
  lines.push_back({"Message: " + (entry.message.empty() ? "-" : entry.message), false});
  lines.push_back({"Hardware ID: " + (entry.hardware_id.empty() ? "-" : entry.hardware_id), false});
  lines.push_back({"Source topic: " + entry.source_topic, false});
  lines.push_back({"Message stamp: " + diagnostic_time_string(entry.message_stamp), false});
  lines.push_back({"Last update: " + format_age(entry.received_time) + " ago", false});

  lines.push_back({"Values", true});
  if (entry.values.empty()) {
    lines.push_back({"-", false});
  } else {
    for (const auto & value : entry.values) {
      const std::string key_text = value.key.empty() ? "<unnamed>" : value.key;
      lines.push_back({key_text + ": " + value.value, false});
    }
  }
  return lines;
}

std::string DiagnosticsViewerBackend::selected_entry_key(
  const std::vector<DiagnosticStatusRow> & snapshot) const
{
  if (snapshot.empty() ||
    selected_status_index_ < 0 ||
    selected_status_index_ >= static_cast<int>(snapshot.size()))
  {
    return "";
  }
  return snapshot[static_cast<std::size_t>(selected_status_index_)].key;
}

void DiagnosticsViewerBackend::set_status(const std::string & text) {
  std::lock_guard<std::mutex> lock(mutex_);
  status_line_ = text;
}

std::string DiagnosticsViewerBackend::entry_key(const DiagnosticStatus & status) {
  return status.name + "\n" + status.hardware_id;
}

std::string DiagnosticsViewerBackend::format_age(const DiagnosticsClock::time_point & received_time) {
  if (received_time.time_since_epoch().count() == 0) {
    return "-";
  }

  const auto age_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(DiagnosticsClock::now() - received_time);
  const double age_seconds = static_cast<double>(age_ms.count()) / 1000.0;
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.1fs", age_seconds);
  return buffer;
}

}  // namespace ros2_console_tools
