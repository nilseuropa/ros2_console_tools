#include "ros2_console_tools/journal_viewer.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

namespace ros2_console_tools {

namespace {

std::string entry_identity(const JournalEntry & entry) {
  return entry.timestamp + "|" + entry.unit + "|" + entry.identifier + "|" + entry.message;
}

std::string build_status_line(
  std::size_t entry_count,
  const std::string & unit_filter,
  int max_priority,
  const std::string & text_filter,
  bool live_mode,
  bool full_history)
{
  std::string status = std::string(live_mode ? "Live" : "Snapshot");
  status += "  " + std::to_string(entry_count);
  status += full_history ? " matching journal entries" : " recent journal entries";
  if (!unit_filter.empty()) {
    status += " for " + unit_filter;
  }
  status += "  priority<=" + journal_priority_label(max_priority);
  if (!text_filter.empty()) {
    status += "  filter=" + text_filter;
  }
  return status;
}

}  // namespace

JournalViewerBackend::JournalViewerBackend(const std::string & initial_unit)
: unit_filter_(initial_unit) {
  std::string theme_error;
  (void)tui::load_theme_from_file(tui::default_theme_config_path(), &theme_error);
  refresh_entries();
}

void JournalViewerBackend::refresh_entries() {
  std::string previous_identity;
  bool live_mode = true;
  int requested_line_count = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!entries_.empty() && selected_index_ >= 0 && selected_index_ < static_cast<int>(entries_.size())) {
      previous_identity = entry_identity(entries_[static_cast<std::size_t>(selected_index_)]);
    }
    live_mode = live_mode_;
    requested_line_count = live_mode_ ? line_count_ : 0;
  }

  std::string error;
  std::vector<JournalEntry> entries =
    client_.read_entries(unit_filter_, max_priority_, requested_line_count, text_filter_, &error);

  std::lock_guard<std::mutex> lock(mutex_);
  if (entries.empty() && !error.empty()) {
    entries_.clear();
    selected_index_ = 0;
    entry_scroll_ = 0;
    status_line_ = error;
    return;
  }

  entries_ = std::move(entries);
  last_live_refresh_time_ = std::chrono::steady_clock::now();
  if (!previous_identity.empty()) {
    const auto found = std::find_if(
      entries_.begin(), entries_.end(),
      [&previous_identity](const JournalEntry & entry) {
        return entry_identity(entry) == previous_identity;
      });
    if (found != entries_.end()) {
      selected_index_ = static_cast<int>(std::distance(entries_.begin(), found));
    }
  }
  clamp_selection();
  status_line_ = build_status_line(
    entries_.size(), unit_filter_, max_priority_, text_filter_, live_mode, requested_line_count <= 0);
}

void JournalViewerBackend::maybe_poll_live_updates() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!live_mode_) {
      return;
    }
    const auto now = std::chrono::steady_clock::now();
    if (last_live_refresh_time_ != std::chrono::steady_clock::time_point::min() &&
      now - last_live_refresh_time_ < std::chrono::seconds(1))
    {
      return;
    }
  }

  refresh_entries();
}

void JournalViewerBackend::clamp_selection() {
  if (entries_.empty()) {
    selected_index_ = 0;
    entry_scroll_ = 0;
    return;
  }

  selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(entries_.size()) - 1);
  entry_scroll_ = std::max(0, std::min(entry_scroll_, selected_index_));
}

void JournalViewerBackend::cycle_priority_filter() {
  static const int kPriorityOrder[] = {3, 4, 5, 6, 7};
  auto found = std::find(std::begin(kPriorityOrder), std::end(kPriorityOrder), max_priority_);
  if (found == std::end(kPriorityOrder) || ++found == std::end(kPriorityOrder)) {
    max_priority_ = kPriorityOrder[0];
  } else {
    max_priority_ = *found;
  }
  refresh_entries();
}

void JournalViewerBackend::set_text_filter(const std::string & filter_text) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    text_filter_ = filter_text;
  }
  refresh_entries();
}

void JournalViewerBackend::toggle_live_mode() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    live_mode_ = !live_mode_;
    last_live_refresh_time_ = std::chrono::steady_clock::time_point::min();
  }

  refresh_entries();
}

bool JournalViewerBackend::live_mode_enabled() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return live_mode_;
}

std::string JournalViewerBackend::priority_filter_label() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return journal_priority_label(max_priority_);
}

std::vector<JournalDetailRow> JournalViewerBackend::detail_rows_snapshot() const {
  std::vector<JournalDetailRow> rows;

  std::lock_guard<std::mutex> lock(mutex_);
  if (entries_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(entries_.size())) {
    rows.push_back({"No journal entry selected.", false});
    return rows;
  }

  const auto & entry = entries_[static_cast<std::size_t>(selected_index_)];
  rows.push_back({"Entry", true});
  rows.push_back({"Timestamp: " + entry.timestamp, false});
  rows.push_back({"Priority: " + journal_priority_label(entry.priority), false});
  if (!entry.unit.empty()) {
    rows.push_back({"Unit: " + entry.unit, false});
  }
  if (!entry.identifier.empty()) {
    rows.push_back({"Identifier: " + entry.identifier, false});
  }
  if (!entry.pid.empty()) {
    rows.push_back({"PID: " + entry.pid, false});
  }

  rows.push_back({"Message", true});
  if (entry.message.empty()) {
    rows.push_back({"<empty>", false});
  } else {
    std::istringstream message_stream(entry.message);
    std::string line;
    while (std::getline(message_stream, line)) {
      rows.push_back({line, false});
    }
  }

  if (!entry.code_file.empty() || !entry.code_line.empty() || !entry.code_function.empty()) {
    rows.push_back({"Code", true});
    if (!entry.code_file.empty()) {
      rows.push_back({"File: " + entry.code_file, false});
    }
    if (!entry.code_line.empty()) {
      rows.push_back({"Line: " + entry.code_line, false});
    }
    if (!entry.code_function.empty()) {
      rows.push_back({"Function: " + entry.code_function, false});
    }
  }

  return rows;
}

}  // namespace ros2_console_tools
