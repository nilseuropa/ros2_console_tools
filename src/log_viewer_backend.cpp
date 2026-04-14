#include "ros2_console_tools/log_viewer.hpp"

#include <algorithm>
#include <fstream>
#include <utility>

namespace ros2_console_tools {

LogViewerBackend::LogViewerBackend()
: Node("log_viewer")
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

void LogViewerBackend::initialize_subscription() {
  log_subscription_ = this->create_subscription<LogMessage>(
    "/rosout", rclcpp::QoS(rclcpp::KeepLast(200)),
    [this](const LogMessage & message) { on_log_message(message); });
}

void LogViewerBackend::cycle_minimum_level() {
  switch (minimum_level_) {
    case LogMessage::DEBUG:
      minimum_level_ = LogMessage::INFO;
      break;
    case LogMessage::INFO:
      minimum_level_ = LogMessage::WARN;
      break;
    case LogMessage::WARN:
      minimum_level_ = LogMessage::ERROR;
      break;
    case LogMessage::ERROR:
      minimum_level_ = LogMessage::FATAL;
      break;
    case LogMessage::FATAL:
    default:
      minimum_level_ = LogMessage::DEBUG;
      break;
  }
  selected_log_index_ = 0;
  log_scroll_ = 0;
  selected_live_log_index_ = 0;
  live_log_scroll_ = 0;
  live_log_follow_newest_ = true;
  set_status("Minimum level: " + level_string(minimum_level_) + ".");
}

void LogViewerBackend::on_log_message(const LogMessage & message) {
  std::lock_guard<std::mutex> lock(mutex_);

  LogEntry entry;
  entry.stamp = message.stamp;
  entry.level = message.level;
  entry.source = message.name.empty() ? "<unknown>" : message.name;
  entry.message = message.msg;
  entry.file = message.file;
  entry.function_name = message.function;
  entry.line = message.line;
  logs_.push_back(std::move(entry));
  if (logs_.size() > kMaxLogs) {
    logs_.pop_front();
  }

  if (sources_.find(message.name) == sources_.end()) {
    sources_[message.name] = false;
  }
  last_message_time_ = LogViewerClock::now();
}

void LogViewerBackend::refresh_sources() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & log : logs_) {
    if (sources_.find(log.source) == sources_.end()) {
      sources_[log.source] = false;
    }
  }
  set_status_locked("Loaded " + std::to_string(sources_.size()) + " log sources.");
}

std::vector<SourceEntry> LogViewerBackend::source_snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<SourceEntry> snapshot;
  snapshot.reserve(sources_.size());
  for (const auto & [name, selected] : sources_) {
    snapshot.push_back({name, selected});
  }
  return snapshot;
}

std::vector<LogEntry> LogViewerBackend::filtered_logs_snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<LogEntry> snapshot;
  snapshot.reserve(logs_.size());
  const bool any_source_selected = std::any_of(
    sources_.begin(), sources_.end(),
    [](const auto & item) { return item.second; });

  for (const auto & log : logs_) {
    if (log.level < minimum_level_) {
      continue;
    }
    if (hide_unselected_ && any_source_selected) {
      const auto found = sources_.find(log.source);
      if (found == sources_.end() || !found->second) {
        continue;
      }
    }
    if (!text_filter_.empty()) {
      if (log.message.find(text_filter_) == std::string::npos &&
        log.source.find(text_filter_) == std::string::npos &&
        log.file.find(text_filter_) == std::string::npos &&
        log.function_name.find(text_filter_) == std::string::npos)
      {
        continue;
      }
    }
    snapshot.push_back(log);
  }
  return snapshot;
}

std::vector<LogEntry> LogViewerBackend::live_source_logs_snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<LogEntry> snapshot;
  snapshot.reserve(logs_.size());

  for (auto it = logs_.rbegin(); it != logs_.rend(); ++it) {
    const auto & log = *it;
    if (log.source != live_source_name_) {
      continue;
    }
    if (log.level < minimum_level_) {
      continue;
    }
    if (!text_filter_.empty()) {
      if (log.message.find(text_filter_) == std::string::npos &&
        log.source.find(text_filter_) == std::string::npos &&
        log.file.find(text_filter_) == std::string::npos &&
        log.function_name.find(text_filter_) == std::string::npos)
      {
        continue;
      }
    }
    snapshot.push_back(log);
  }

  return snapshot;
}

void LogViewerBackend::toggle_selected_source() {
  auto snapshot = source_snapshot();
  clamp_source_selection(snapshot);
  if (snapshot.empty() || selected_source_index_ < 0 || selected_source_index_ >= static_cast<int>(snapshot.size())) {
    set_status("No source selected.");
    return;
  }

  const std::string name = snapshot[static_cast<std::size_t>(selected_source_index_)].name;
  std::lock_guard<std::mutex> lock(mutex_);
  auto found = sources_.find(name);
  if (found == sources_.end()) {
    return;
  }
  found->second = !found->second;
  set_status_locked((found->second ? "Selected " : "Deselected ") + name + ".");
}

void LogViewerBackend::open_detail_popup(const std::vector<LogEntry> & snapshot) {
  clamp_log_selection(snapshot);
  if (snapshot.empty() || selected_log_index_ < 0 || selected_log_index_ >= static_cast<int>(snapshot.size())) {
    set_status("No log line selected.");
    return;
  }

  detail_entry_ = snapshot[static_cast<std::size_t>(selected_log_index_)];
  detail_popup_open_ = true;
}

void LogViewerBackend::open_live_detail_popup(const std::vector<LogEntry> & snapshot) {
  clamp_live_log_selection(snapshot);
  if (snapshot.empty() || selected_live_log_index_ < 0 ||
    selected_live_log_index_ >= static_cast<int>(snapshot.size()))
  {
    set_status("No log line selected.");
    return;
  }

  detail_entry_ = snapshot[static_cast<std::size_t>(selected_live_log_index_)];
  detail_popup_open_ = true;
}

void LogViewerBackend::open_live_source() {
  auto snapshot = source_snapshot();
  clamp_source_selection(snapshot);
  if (snapshot.empty() || selected_source_index_ < 0 || selected_source_index_ >= static_cast<int>(snapshot.size())) {
    set_status("No source selected.");
    return;
  }

  live_source_name_ = snapshot[static_cast<std::size_t>(selected_source_index_)].name;
  view_mode_ = LogViewerViewMode::SourceLive;
  live_log_scroll_ = 0;
  selected_live_log_index_ = 0;
  live_log_follow_newest_ = true;
  set_status("Live view: " + live_source_name_ + ".");
}

void LogViewerBackend::close_live_source() {
  view_mode_ = LogViewerViewMode::Split;
  live_source_name_.clear();
  live_log_scroll_ = 0;
  selected_live_log_index_ = 0;
  live_log_follow_newest_ = true;
  set_status("Returned to split view.");
}

void LogViewerBackend::open_code_view() {
  if (detail_entry_.file.empty()) {
    set_status("No source file available for this log entry.");
    return;
  }

  const std::filesystem::path path(detail_entry_.file);
  if (!std::filesystem::exists(path) || !std::filesystem::is_regular_file(path)) {
    set_status("Source file not available locally: " + detail_entry_.file);
    return;
  }

  std::ifstream input(path);
  if (!input.is_open()) {
    set_status("Failed to open source file: " + detail_entry_.file);
    return;
  }

  code_view_lines_.clear();
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    code_view_lines_.push_back(line);
  }

  code_view_path_ = detail_entry_.file;
  code_view_target_line_ = detail_entry_.line == 0 ? 1 : static_cast<int>(detail_entry_.line);
  code_view_target_line_ = std::clamp(
    code_view_target_line_, 1, std::max(1, static_cast<int>(code_view_lines_.size())));

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int visible_rows = std::max(1, rows - 4);
  code_view_top_line_ = std::max(0, code_view_target_line_ - 1 - visible_rows / 2);
  const int max_top_line = std::max(0, static_cast<int>(code_view_lines_.size()) - visible_rows);
  code_view_top_line_ = std::min(code_view_top_line_, max_top_line);

  detail_popup_open_ = false;
  view_mode_ = LogViewerViewMode::CodeInspect;
  set_status("Inspecting " + code_view_path_ + ":" + std::to_string(code_view_target_line_) + ".");
}

void LogViewerBackend::close_code_view() {
  view_mode_ = live_source_name_.empty() ? LogViewerViewMode::Split : LogViewerViewMode::SourceLive;
  code_view_lines_.clear();
  code_view_path_.clear();
  code_view_top_line_ = 0;
  code_view_target_line_ = 0;
  set_status("Closed code inspection.");
}

void LogViewerBackend::clamp_source_selection(const std::vector<SourceEntry> & snapshot) {
  if (snapshot.empty()) {
    selected_source_index_ = 0;
    source_scroll_ = 0;
    return;
  }
  selected_source_index_ = std::clamp(selected_source_index_, 0, static_cast<int>(snapshot.size()) - 1);
}

void LogViewerBackend::clamp_log_selection(const std::vector<LogEntry> & snapshot) {
  if (snapshot.empty()) {
    selected_log_index_ = 0;
    log_scroll_ = 0;
    return;
  }
  selected_log_index_ = std::clamp(selected_log_index_, 0, static_cast<int>(snapshot.size()) - 1);
}

void LogViewerBackend::clamp_live_log_selection(const std::vector<LogEntry> & snapshot) {
  if (snapshot.empty()) {
    selected_live_log_index_ = 0;
    live_log_scroll_ = 0;
    return;
  }
  selected_live_log_index_ = std::clamp(
    selected_live_log_index_, 0, static_cast<int>(snapshot.size()) - 1);
}

void LogViewerBackend::set_status(const std::string & text) {
  std::lock_guard<std::mutex> lock(mutex_);
  set_status_locked(text);
}

void LogViewerBackend::set_status_locked(const std::string & text) const {
  status_line_ = text;
}

}  // namespace ros2_console_tools
