#include <ncursesw/ncurses.h>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

namespace {

using Clock = std::chrono::steady_clock;
using LogMessage = rcl_interfaces::msg::Log;

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorStatus = tui::kColorStatus,
  kColorHelp = tui::kColorHelp,
  kColorWarn = tui::kColorWarn,
  kColorError = tui::kColorError,
  kColorFatal = tui::kColorFatal,
  kColorPositive = tui::kColorPositive,
  kColorSource = tui::kColorAccent,
  kColorPopup = tui::kColorPopup,
};

enum class PaneFocus {
  Sources,
  Logs,
};

enum class ViewMode {
  Split,
  SourceLive,
  CodeInspect,
};

using tui::Session;
using tui::truncate_text;
using tui::draw_box;
using tui::draw_box_char;
using tui::draw_search_box;
using tui::draw_status_bar;
using tui::draw_help_bar;
using tui::draw_help_bar_region;
using tui::draw_text_vline;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::SearchState;
using tui::start_search;

std::string time_string(const builtin_interfaces::msg::Time & stamp) {
  std::ostringstream stream;
  stream << stamp.sec << "." << std::setw(3) << std::setfill('0') << (stamp.nanosec / 1000000U);
  return stream.str();
}

std::string level_string(uint8_t level) {
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

int level_color(uint8_t level, bool selected) {
  switch (level) {
    case LogMessage::WARN:
      return selected ? kColorSelection : kColorWarn;
    case LogMessage::ERROR:
      return selected ? kColorSelection : kColorError;
    case LogMessage::FATAL:
      return selected ? kColorFatal : kColorFatal;
    case LogMessage::DEBUG:
    case LogMessage::INFO:
    default:
      return selected ? kColorSelection : 0;
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

class LogViewerNode : public rclcpp::Node {
public:
  LogViewerNode()
  : Node("log_viewer") {}

  int run() {
    Session ncurses_session;
    refresh_sources();

    bool running = true;
    while (running && rclcpp::ok()) {
      draw();
      const int key = getch();
      if (key == ERR) {
        continue;
      }
      running = handle_key(key);
    }

    return 0;
  }

private:
  static constexpr std::size_t kMaxLogs = 2000;

  bool handle_key(int key) {
    if (filter_prompt_open_) {
      return handle_filter_prompt_key(key);
    }
    if (search_state_.active) {
      return handle_search_key(key);
    }
    if (view_mode_ == ViewMode::CodeInspect) {
      return handle_code_view_key(key);
    }
    if (detail_popup_open_) {
      return handle_detail_popup_key(key);
    }

    switch (key) {
      case KEY_F(10):
        return false;
      case 27:
        if (is_alt_binding(key, 's')) {
          start_search(search_state_);
          set_status("Search.");
          return true;
        }
        if (view_mode_ == ViewMode::SourceLive) {
          close_live_source();
          return true;
        }
        break;
      case '\t':
        if (view_mode_ == ViewMode::Split) {
          focus_ = focus_ == PaneFocus::Sources ? PaneFocus::Logs : PaneFocus::Sources;
        }
        return true;
      case KEY_F(4):
        refresh_sources();
        return true;
      case KEY_F(5):
        hide_unselected_ = !hide_unselected_;
        log_scroll_ = 0;
        selected_log_index_ = 0;
        set_status(hide_unselected_ ? "Showing selected sources only." : "Showing all sources.");
        return true;
      case KEY_F(6):
        cycle_minimum_level();
        return true;
      case '/':
        filter_prompt_open_ = true;
        filter_buffer_ = text_filter_;
        set_status("Filter text: " + filter_buffer_);
        return true;
      default:
        break;
    }

    if (view_mode_ == ViewMode::SourceLive) {
      return handle_live_source_key(key);
    }
    if (focus_ == PaneFocus::Sources) {
      return handle_source_key(key);
    }
    return handle_log_key(key);
  }

  bool handle_search_key(int key) {
    const SearchInputResult result = handle_search_input(search_state_, key);
    if (result == SearchInputResult::Cancelled) {
      set_status("Search cancelled.");
      return true;
    }
    if (result == SearchInputResult::Accepted) {
      set_status(search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query);
      return true;
    }
    if (result != SearchInputResult::Changed) {
      return true;
    }

    if (view_mode_ == ViewMode::SourceLive) {
      const auto snapshot = live_source_logs_snapshot();
      std::vector<std::string> labels;
      labels.reserve(snapshot.size());
      for (const auto & entry : snapshot) {
        labels.push_back(level_string(entry.level) + " " + entry.message);
      }
      const int match = find_best_match(labels, search_state_.query, selected_live_log_index_);
      if (match >= 0) {
        selected_live_log_index_ = match;
      }
      set_status("Search: " + search_state_.query);
      return true;
    }

    if (focus_ == PaneFocus::Sources) {
      const auto snapshot = source_snapshot();
      std::vector<std::string> labels;
      labels.reserve(snapshot.size());
      for (const auto & entry : snapshot) {
        labels.push_back(entry.name);
      }
      const int match = find_best_match(labels, search_state_.query, selected_source_index_);
      if (match >= 0) {
        selected_source_index_ = match;
      }
    } else {
      const auto snapshot = filtered_logs_snapshot();
      std::vector<std::string> labels;
      labels.reserve(snapshot.size());
      for (const auto & entry : snapshot) {
        labels.push_back(entry.source + " " + entry.message);
      }
      const int match = find_best_match(labels, search_state_.query, selected_log_index_);
      if (match >= 0) {
        selected_log_index_ = match;
      }
    }
    set_status("Search: " + search_state_.query);
    return true;
  }

  bool handle_filter_prompt_key(int key) {
    switch (key) {
      case 27:
        filter_prompt_open_ = false;
        filter_buffer_.clear();
        set_status("Filter unchanged.");
        return true;
      case '\n':
      case KEY_ENTER:
        text_filter_ = filter_buffer_;
        filter_prompt_open_ = false;
        filter_buffer_.clear();
        selected_log_index_ = 0;
        log_scroll_ = 0;
        set_status(text_filter_.empty() ? "Text filter cleared." : "Text filter applied.");
        return true;
      case KEY_BACKSPACE:
      case 127:
      case '\b':
        if (!filter_buffer_.empty()) {
          filter_buffer_.pop_back();
        }
        set_status("Filter text: " + filter_buffer_);
        return true;
      default:
        if (key >= 32 && key <= 126) {
          filter_buffer_.push_back(static_cast<char>(key));
          set_status("Filter text: " + filter_buffer_);
        }
        return true;
    }
  }

  bool handle_detail_popup_key(int key) {
    switch (key) {
      case KEY_F(10):
        return false;
      case KEY_F(2):
        open_code_view();
        return true;
      case 27:
      case '\n':
      case KEY_ENTER:
        detail_popup_open_ = false;
        return true;
      default:
        return true;
    }
  }

  bool handle_source_key(int key) {
    const auto snapshot = source_snapshot();
    clamp_source_selection(snapshot);

    switch (key) {
      case KEY_UP:
      case 'k':
        if (selected_source_index_ > 0) {
          --selected_source_index_;
        }
        return true;
      case KEY_DOWN:
      case 'j':
        if (selected_source_index_ + 1 < static_cast<int>(snapshot.size())) {
          ++selected_source_index_;
        }
        return true;
      case KEY_PPAGE:
        selected_source_index_ = std::max(0, selected_source_index_ - page_step());
        return true;
      case KEY_NPAGE:
        if (!snapshot.empty()) {
          selected_source_index_ = std::min(static_cast<int>(snapshot.size()) - 1, selected_source_index_ + page_step());
        }
        return true;
      case ' ':
      case KEY_IC:
        toggle_selected_source();
        return true;
      case '\n':
      case KEY_ENTER:
        open_live_source();
        return true;
      default:
        return true;
    }
  }

  bool handle_live_source_key(int key) {
    const auto snapshot = live_source_logs_snapshot();
    clamp_live_log_selection(snapshot);

    switch (key) {
      case KEY_UP:
      case 'k':
        if (selected_live_log_index_ > 0) {
          --selected_live_log_index_;
        }
        return true;
      case KEY_DOWN:
      case 'j':
        if (selected_live_log_index_ + 1 < static_cast<int>(snapshot.size())) {
          ++selected_live_log_index_;
        }
        return true;
      case KEY_PPAGE:
        selected_live_log_index_ = std::max(0, selected_live_log_index_ - page_step());
        return true;
      case KEY_NPAGE:
        if (!snapshot.empty()) {
          selected_live_log_index_ = std::min(
            static_cast<int>(snapshot.size()) - 1,
            selected_live_log_index_ + page_step());
        }
        return true;
      case '\n':
      case KEY_ENTER:
        open_live_detail_popup(snapshot);
        return true;
      default:
        return true;
    }
  }

  bool handle_log_key(int key) {
    const auto snapshot = filtered_logs_snapshot();
    clamp_log_selection(snapshot);

    switch (key) {
      case KEY_UP:
      case 'k':
        if (selected_log_index_ > 0) {
          --selected_log_index_;
        }
        return true;
      case KEY_DOWN:
      case 'j':
        if (selected_log_index_ + 1 < static_cast<int>(snapshot.size())) {
          ++selected_log_index_;
        }
        return true;
      case KEY_PPAGE:
        selected_log_index_ = std::max(0, selected_log_index_ - page_step());
        return true;
      case KEY_NPAGE:
        if (!snapshot.empty()) {
          selected_log_index_ = std::min(static_cast<int>(snapshot.size()) - 1, selected_log_index_ + page_step());
        }
        return true;
      case '\n':
      case KEY_ENTER:
        open_detail_popup(snapshot);
        return true;
      default:
        return true;
    }
  }

  bool handle_code_view_key(int key) {
    switch (key) {
      case KEY_F(10):
        return false;
      case 27:
      case KEY_ENTER:
      case '\n':
        close_code_view();
        return true;
      case KEY_UP:
      case 'k':
        if (code_view_top_line_ > 0) {
          --code_view_top_line_;
        }
        return true;
      case KEY_DOWN:
      case 'j':
        if (code_view_top_line_ + 1 < static_cast<int>(code_view_lines_.size())) {
          ++code_view_top_line_;
        }
        return true;
      case KEY_PPAGE:
        code_view_top_line_ = std::max(0, code_view_top_line_ - page_step());
        return true;
      case KEY_NPAGE:
        if (!code_view_lines_.empty()) {
          code_view_top_line_ = std::min(
            std::max(0, static_cast<int>(code_view_lines_.size()) - 1),
            code_view_top_line_ + page_step());
        }
        return true;
      case KEY_HOME:
      case 'g':
        code_view_top_line_ = 0;
        return true;
      case KEY_END:
      case 'G':
        if (!code_view_lines_.empty()) {
          code_view_top_line_ = std::max(0, static_cast<int>(code_view_lines_.size()) - 1);
        }
        return true;
      default:
        return true;
    }
  }

  void cycle_minimum_level() {
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
    set_status("Minimum level: " + level_string(minimum_level_) + ".");
  }

  int page_step() const {
    int rows = 0;
    int columns = 0;
    getmaxyx(stdscr, rows, columns);
    (void)columns;
    return std::max(5, rows - 8);
  }

  void on_log_message(const LogMessage & message) {
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
    last_message_time_ = Clock::now();
  }

  void refresh_sources() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & log : logs_) {
      if (sources_.find(log.source) == sources_.end()) {
        sources_[log.source] = false;
      }
    }
    set_status_locked("Loaded " + std::to_string(sources_.size()) + " log sources.");
  }

  std::vector<SourceEntry> source_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SourceEntry> snapshot;
    snapshot.reserve(sources_.size());
    for (const auto & [name, selected] : sources_) {
      snapshot.push_back({name, selected});
    }
    return snapshot;
  }

  std::vector<LogEntry> filtered_logs_snapshot() const {
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

  std::vector<LogEntry> live_source_logs_snapshot() const {
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

  void toggle_selected_source() {
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

  void open_detail_popup(const std::vector<LogEntry> & snapshot) {
    clamp_log_selection(snapshot);
    if (snapshot.empty() || selected_log_index_ < 0 || selected_log_index_ >= static_cast<int>(snapshot.size())) {
      set_status("No log line selected.");
      return;
    }

    detail_entry_ = snapshot[static_cast<std::size_t>(selected_log_index_)];
    detail_popup_open_ = true;
  }

  void open_live_detail_popup(const std::vector<LogEntry> & snapshot) {
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

  void open_live_source() {
    auto snapshot = source_snapshot();
    clamp_source_selection(snapshot);
    if (snapshot.empty() || selected_source_index_ < 0 || selected_source_index_ >= static_cast<int>(snapshot.size())) {
      set_status("No source selected.");
      return;
    }

    live_source_name_ = snapshot[static_cast<std::size_t>(selected_source_index_)].name;
    view_mode_ = ViewMode::SourceLive;
    live_log_scroll_ = 0;
    selected_live_log_index_ = 0;
    set_status("Live view: " + live_source_name_ + ".");
  }

  void close_live_source() {
    view_mode_ = ViewMode::Split;
    live_source_name_.clear();
    live_log_scroll_ = 0;
    selected_live_log_index_ = 0;
    set_status("Returned to split view.");
  }

  void open_code_view() {
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
    view_mode_ = ViewMode::CodeInspect;
    set_status("Inspecting " + code_view_path_ + ":" + std::to_string(code_view_target_line_) + ".");
  }

  void close_code_view() {
    view_mode_ = live_source_name_.empty() ? ViewMode::Split : ViewMode::SourceLive;
    code_view_lines_.clear();
    code_view_path_.clear();
    code_view_top_line_ = 0;
    code_view_target_line_ = 0;
    set_status("Closed code inspection.");
  }

  void draw() {
    erase();

    int rows = 0;
    int columns = 0;
    getmaxyx(stdscr, rows, columns);
    const int help_row = rows - 1;
    const int status_row = rows - 2;
    const int content_bottom = std::max(1, status_row - 1);

    draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
    mvprintw(0, 1, "Log Viewer ");
    if (view_mode_ == ViewMode::CodeInspect) {
      draw_code_view_pane(1, 1, content_bottom - 1, columns - 2);
    } else if (view_mode_ == ViewMode::Split) {
      const int left_width = std::max(24, (columns - 2) / 4);
      const int separator_x = 1 + left_width;
      draw_sources_pane(1, 1, content_bottom - 1, separator_x - 1);
      attron(COLOR_PAIR(kColorFrame));
      draw_text_vline(1, separator_x, content_bottom - 1);
      attroff(COLOR_PAIR(kColorFrame));
      draw_logs_pane(1, separator_x + 1, content_bottom - 1, columns - 2);
    } else {
      draw_live_source_pane(1, 1, content_bottom - 1, columns - 2);
    }
    draw_status_line(status_row, columns);
    draw_help_line(help_row, columns);
    if (detail_popup_open_) {
      draw_detail_popup(rows, columns);
    }
    draw_search_box(rows, columns, search_state_);

    refresh();
  }

  static bool is_identifier_char(char character) {
    return std::isalnum(static_cast<unsigned char>(character)) || character == '_';
  }

  static bool is_keyword(const std::string & token) {
    static const std::set<std::string> keywords = {
      "auto", "bool", "break", "case", "catch", "char", "class", "const", "constexpr",
      "continue", "default", "delete", "do", "double", "else", "enum", "explicit",
      "false", "float", "for", "if", "inline", "int", "long", "namespace", "new",
      "noexcept", "nullptr", "operator", "private", "protected", "public", "return",
      "short", "signed", "sizeof", "static", "struct", "switch", "template", "this",
      "throw", "true", "try", "typename", "uint8_t", "uint16_t", "uint32_t", "uint64_t",
      "int8_t", "int16_t", "int32_t", "int64_t", "using", "virtual", "void", "volatile",
      "while"
    };
    return keywords.find(token) != keywords.end();
  }

  std::vector<StyledSpan> highlight_code_line(const std::string & line) const {
    std::vector<StyledSpan> spans;
    std::size_t index = 0;

    auto push_span = [&](const std::string & text, int color) {
      if (text.empty()) {
        return;
      }
      if (!spans.empty() && spans.back().color == color) {
        spans.back().text += text;
      } else {
        spans.push_back({text, color});
      }
    };

    while (index < line.size()) {
      if (index + 1 < line.size() && line[index] == '/' && line[index + 1] == '/') {
        push_span(line.substr(index), kColorSource);
        break;
      }
      if (line[index] == '"') {
        std::size_t end = index + 1;
        bool escaped = false;
        while (end < line.size()) {
          if (line[end] == '"' && !escaped) {
            ++end;
            break;
          }
          escaped = (line[end] == '\\' && !escaped);
          if (line[end] != '\\') {
            escaped = false;
          }
          ++end;
        }
        push_span(line.substr(index, end - index), kColorPositive);
        index = end;
        continue;
      }
      if (std::isdigit(static_cast<unsigned char>(line[index]))) {
        std::size_t end = index + 1;
        while (end < line.size() &&
          (std::isalnum(static_cast<unsigned char>(line[end])) || line[end] == '.' || line[end] == '_'))
        {
          ++end;
        }
        push_span(line.substr(index, end - index), kColorWarn);
        index = end;
        continue;
      }
      if (line[index] == '#') {
        push_span(line.substr(index), kColorError);
        break;
      }
      if (is_identifier_char(line[index])) {
        std::size_t end = index + 1;
        while (end < line.size() && is_identifier_char(line[end])) {
          ++end;
        }
        const std::string token = line.substr(index, end - index);
        push_span(token, is_keyword(token) ? kColorHeader : 0);
        index = end;
        continue;
      }

      push_span(std::string(1, line[index]), 0);
      ++index;
    }

    return spans;
  }

  void draw_code_line(int row, int left, int width, int line_number, const std::string & line, bool highlight) const {
    mvhline(row, left, ' ', width);
    if (highlight) {
      mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
    }

    const int line_number_width = std::max(4, static_cast<int>(std::to_string(
      std::max(1, static_cast<int>(code_view_lines_.size()))).size()));
    const std::string line_prefix =
      (highlight ? ">" : " ") + std::string(line_number_width - static_cast<int>(std::to_string(line_number).size()), ' ')
      + std::to_string(line_number) + " ";

    if (highlight) {
      mvaddnstr(row, left, line_prefix.c_str(), width);
      mvaddnstr(row, left + static_cast<int>(line_prefix.size()), truncate_text(line, width - static_cast<int>(line_prefix.size())).c_str(), width - static_cast<int>(line_prefix.size()));
      mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      return;
    }

    attron(COLOR_PAIR(kColorHeader));
    mvaddnstr(row, left, line_prefix.c_str(), width);
    attroff(COLOR_PAIR(kColorHeader));

    int column = left + static_cast<int>(line_prefix.size());
    const int remaining = std::max(0, width - static_cast<int>(line_prefix.size()));
    int drawn = 0;
    for (const auto & span : highlight_code_line(line)) {
      if (drawn >= remaining) {
        break;
      }
      const std::string text = truncate_text(span.text, remaining - drawn);
      if (span.color != 0) {
        attron(COLOR_PAIR(span.color));
      }
      mvaddnstr(row, column, text.c_str(), remaining - drawn);
      if (span.color != 0) {
        attroff(COLOR_PAIR(span.color));
      }
      column += static_cast<int>(text.size());
      drawn += static_cast<int>(text.size());
    }
  }

  void draw_code_view_pane(int top, int left, int bottom, int right) const {
    const int width = right - left + 1;
    const int visible_rows = std::max(1, bottom - top + 1);
    const std::string title =
      truncate_text("Code: " + code_view_path_ + ":" + std::to_string(code_view_target_line_), width);

    attron(COLOR_PAIR(kColorHeader));
    mvprintw(top, left, "%-*s", width, title.c_str());
    attroff(COLOR_PAIR(kColorHeader));

    const int max_top_line = std::max(0, static_cast<int>(code_view_lines_.size()) - (visible_rows - 1));
    const int top_line = std::clamp(code_view_top_line_, 0, max_top_line);
    for (int row = top + 1; row <= bottom; ++row) {
      const int line_index = top_line + (row - top - 1);
      if (line_index >= static_cast<int>(code_view_lines_.size())) {
        mvhline(row, left, ' ', width);
        continue;
      }
      draw_code_line(
        row, left, width, line_index + 1, code_view_lines_[static_cast<std::size_t>(line_index)],
        line_index + 1 == code_view_target_line_);
    }
  }

  void draw_sources_pane(int top, int left, int bottom, int right) {
    const auto snapshot = source_snapshot();
    clamp_source_selection(snapshot);

    const int width = right - left + 1;
    const int visible_rows = std::max(1, bottom - top + 1);
    if (selected_source_index_ < source_scroll_) {
      source_scroll_ = selected_source_index_;
    }
    if (selected_source_index_ >= source_scroll_ + visible_rows - 1) {
      source_scroll_ = std::max(0, selected_source_index_ - visible_rows + 2);
    }

    attron(COLOR_PAIR(kColorHeader));
    mvprintw(top, left, "%-*s", width, "Sources");
    attroff(COLOR_PAIR(kColorHeader));

    const int first_row = source_scroll_;
    const int last_row = std::min(static_cast<int>(snapshot.size()), first_row + visible_rows - 1);
    for (int row = top + 1; row <= bottom; ++row) {
      const bool has_item = first_row + (row - top - 1) < last_row;
      const bool selected = has_item && (first_row + (row - top - 1) == selected_source_index_);
      mvhline(row, left, ' ', width);
      if (selected && focus_ == PaneFocus::Sources) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      }
      if (!has_item) {
        continue;
      }

      const auto & entry = snapshot[static_cast<std::size_t>(first_row + (row - top - 1))];
      const std::string label = std::string(entry.selected ? "* " : "  ") + entry.name;
      const int color = selected && focus_ == PaneFocus::Sources ? kColorSelection : kColorSource;
      attron(COLOR_PAIR(color));
      mvprintw(row, left, "%-*s", width, truncate_text(label, width).c_str());
      attroff(COLOR_PAIR(color));
      if (selected && focus_ == PaneFocus::Sources) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
        mvchgat(row, left, std::min(width, static_cast<int>(label.size())), A_NORMAL, color, nullptr);
      }
    }
  }

  void draw_logs_pane(int top, int left, int bottom, int right) {
    const auto snapshot = filtered_logs_snapshot();
    clamp_log_selection(snapshot);

    const int width = right - left + 1;
    const int visible_rows = std::max(1, bottom - top + 1);
    const int time_width = 12;
    const int level_width = 6;
    const int source_width = std::max(16, width / 4);
    const int message_width = std::max(10, width - time_width - level_width - source_width - 3);
    const int sep_one_x = left + time_width;
    const int sep_two_x = sep_one_x + 1 + level_width;
    const int sep_three_x = sep_two_x + 1 + source_width;

    if (selected_log_index_ < log_scroll_) {
      log_scroll_ = selected_log_index_;
    }
    if (selected_log_index_ >= log_scroll_ + visible_rows - 1) {
      log_scroll_ = std::max(0, selected_log_index_ - visible_rows + 2);
    }

    attron(COLOR_PAIR(kColorHeader));
    mvprintw(top, left, "%-*s", time_width, "Time");
    draw_box_char(top, sep_one_x, WACS_VLINE, '|');
    mvprintw(top, sep_one_x + 1, "%-*s", level_width, "Level");
    draw_box_char(top, sep_two_x, WACS_VLINE, '|');
    mvprintw(top, sep_two_x + 1, "%-*s", source_width, "Source");
    draw_box_char(top, sep_three_x, WACS_VLINE, '|');
    mvprintw(top, sep_three_x + 1, "%-*s", message_width, "Message");
    attroff(COLOR_PAIR(kColorHeader));

    const int first_row = log_scroll_;
    const int last_row = std::min(static_cast<int>(snapshot.size()), first_row + visible_rows - 1);
    for (int row = top + 1; row <= bottom; ++row) {
      const bool has_item = first_row + (row - top - 1) < last_row;
      const bool selected = has_item && (first_row + (row - top - 1) == selected_log_index_);
      mvhline(row, left, ' ', width);
      if (selected && focus_ == PaneFocus::Logs) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      }

      draw_box_char(row, sep_one_x, WACS_VLINE, '|');
      draw_box_char(row, sep_two_x, WACS_VLINE, '|');
      draw_box_char(row, sep_three_x, WACS_VLINE, '|');
      if (!has_item) {
        continue;
      }

      const auto & entry = snapshot[static_cast<std::size_t>(first_row + (row - top - 1))];
      const int color = level_color(entry.level, selected && focus_ == PaneFocus::Logs);
      if (color != 0) {
        attron(COLOR_PAIR(color));
      }
      mvprintw(row, left, "%-*s", time_width, truncate_text(time_string(entry.stamp), time_width).c_str());
      mvprintw(row, sep_one_x + 1, "%-*s", level_width, truncate_text(level_string(entry.level), level_width).c_str());
      mvprintw(row, sep_two_x + 1, "%-*s", source_width, truncate_text(entry.source, source_width).c_str());
      mvprintw(row, sep_three_x + 1, "%-*s", message_width, truncate_text(entry.message, message_width).c_str());
      if (color != 0) {
        attroff(COLOR_PAIR(color));
      }
      if (selected && focus_ == PaneFocus::Logs) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
        draw_box_char(row, sep_one_x, WACS_VLINE, '|');
        draw_box_char(row, sep_two_x, WACS_VLINE, '|');
        draw_box_char(row, sep_three_x, WACS_VLINE, '|');
        if (color != 0) {
          mvchgat(row, left, time_width, A_NORMAL, color, nullptr);
          mvchgat(row, sep_one_x + 1, level_width, A_NORMAL, color, nullptr);
          mvchgat(row, sep_two_x + 1, source_width, A_NORMAL, color, nullptr);
          mvchgat(row, sep_three_x + 1, message_width, A_NORMAL, color, nullptr);
        }
      }
    }
  }

  void draw_live_source_pane(int top, int left, int bottom, int right) {
    const auto snapshot = live_source_logs_snapshot();
    clamp_live_log_selection(snapshot);

    const int width = right - left + 1;
    const int visible_rows = std::max(1, bottom - top + 1);
    if (selected_live_log_index_ < live_log_scroll_) {
      live_log_scroll_ = selected_live_log_index_;
    }
    if (selected_live_log_index_ >= live_log_scroll_ + visible_rows - 1) {
      live_log_scroll_ = std::max(0, selected_live_log_index_ - visible_rows + 2);
    }

    attron(COLOR_PAIR(kColorHeader));
    mvprintw(top, left, "%-*s", width, truncate_text("Live: " + live_source_name_, width).c_str());
    attroff(COLOR_PAIR(kColorHeader));

    const int first_row = live_log_scroll_;
    const int last_row = std::min(static_cast<int>(snapshot.size()), first_row + visible_rows - 1);
    for (int row = top + 1; row <= bottom; ++row) {
      const bool has_item = first_row + (row - top - 1) < last_row;
      const bool selected = has_item && (first_row + (row - top - 1) == selected_live_log_index_);
      mvhline(row, left, ' ', width);
      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      }
      if (!has_item) {
        continue;
      }

      const auto & entry = snapshot[static_cast<std::size_t>(first_row + (row - top - 1))];
      const int color = level_color(entry.level, selected);
      const std::string line =
        time_string(entry.stamp) + " " + level_string(entry.level) + " " + entry.message;
      const std::string rendered = truncate_text(line, width);
      if (color != 0) {
        attron(COLOR_PAIR(color));
      }
      mvprintw(row, left, "%-*s", width, rendered.c_str());
      if (color != 0) {
        attroff(COLOR_PAIR(color));
      }
      if (selected && color != 0) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
        mvchgat(row, left, static_cast<int>(rendered.size()), A_NORMAL, color, nullptr);
      }
    }
  }

  void draw_status_line(int row, int columns) const {
    std::string line = status_line_;
    if (filter_prompt_open_) {
      line = "Filter text: " + filter_buffer_;
    } else {
      line += "  level>=" + level_string(minimum_level_);
      if (!text_filter_.empty()) {
        line += "  filter=\"" + text_filter_ + "\"";
      }
      line += "  follow=" + std::string(hide_unselected_ ? "selected" : "all");
    }
    draw_status_bar(row, columns, line);
  }

  void draw_help_line(int row, int columns) const {
    const std::string help =
      view_mode_ == ViewMode::CodeInspect
      ? "Esc Back  Up Down Scroll  PgUp PgDn Page  Home Top  End Bottom  F10 Exit"
      : view_mode_ == ViewMode::SourceLive
      ? "Enter Details  Alt+S Search  Esc Back  F4 Refresh  F6 Level  / Filter  F10 Exit"
      : "Tab Pane  Space Mark  Enter Live  Alt+S Search  F4 Refresh  F5 Hide Unselected  F6 Level  / Filter  F10 Exit";
    draw_help_bar(row, columns, help);
  }

  void draw_detail_popup(int rows, int columns) const {
    const int popup_width = std::min(columns - 6, 96);
    const int popup_height = std::min(rows - 6, 12);
    const int left = std::max(2, (columns - popup_width) / 2);
    const int top = std::max(2, (rows - popup_height) / 2);
    const int right = left + popup_width - 1;
    const int bottom = top + popup_height - 1;

    for (int row = top; row <= bottom; ++row) {
      mvhline(row, left, ' ', popup_width);
      mvchgat(row, left, popup_width, A_NORMAL, kColorPopup, nullptr);
    }
    draw_box(top, left, bottom, right, kColorFrame);

    const auto print_line = [&](int row, const std::string & label, const std::string & value) {
      const std::string text = label + value;
      mvprintw(row, left + 2, "%s", truncate_text(text, popup_width - 4).c_str());
    };

    print_line(top + 1, "time: ", time_string(detail_entry_.stamp));
    print_line(top + 2, "level: ", level_string(detail_entry_.level));
    print_line(top + 3, "source: ", detail_entry_.source);
    print_line(top + 4, "file: ", detail_entry_.file);
    print_line(top + 5, "function: ", detail_entry_.function_name);
    print_line(top + 6, "line: ", std::to_string(detail_entry_.line));
    print_line(top + 8, "message: ", detail_entry_.message);
    draw_help_bar_region(bottom - 1, left + 2, popup_width - 4, "F2 Inspect Code  Esc Close  Enter Close  F10 Exit");
  }

  void clamp_source_selection(const std::vector<SourceEntry> & snapshot) {
    if (snapshot.empty()) {
      selected_source_index_ = 0;
      source_scroll_ = 0;
      return;
    }
    selected_source_index_ = std::clamp(selected_source_index_, 0, static_cast<int>(snapshot.size()) - 1);
  }

  void clamp_log_selection(const std::vector<LogEntry> & snapshot) {
    if (snapshot.empty()) {
      selected_log_index_ = 0;
      log_scroll_ = 0;
      return;
    }
    selected_log_index_ = std::clamp(selected_log_index_, 0, static_cast<int>(snapshot.size()) - 1);
  }

  void clamp_live_log_selection(const std::vector<LogEntry> & snapshot) {
    if (snapshot.empty()) {
      selected_live_log_index_ = 0;
      live_log_scroll_ = 0;
      return;
    }
    selected_live_log_index_ = std::clamp(
      selected_live_log_index_, 0, static_cast<int>(snapshot.size()) - 1);
  }

  void set_status(const std::string & text) {
    std::lock_guard<std::mutex> lock(mutex_);
    set_status_locked(text);
  }

  void set_status_locked(const std::string & text) const {
    status_line_ = text;
  }

  mutable std::mutex mutex_;
  rclcpp::Subscription<LogMessage>::SharedPtr log_subscription_;
  std::deque<LogEntry> logs_;
  std::map<std::string, bool> sources_;
  ViewMode view_mode_{ViewMode::Split};
  PaneFocus focus_{PaneFocus::Sources};
  int selected_source_index_{0};
  int selected_log_index_{0};
  int selected_live_log_index_{0};
  int source_scroll_{0};
  int log_scroll_{0};
  int live_log_scroll_{0};
  bool hide_unselected_{false};
  uint8_t minimum_level_{LogMessage::DEBUG};
  std::string text_filter_;
  std::string live_source_name_;
  bool filter_prompt_open_{false};
  std::string filter_buffer_;
  bool detail_popup_open_{false};
  LogEntry detail_entry_;
  SearchState search_state_;
  std::vector<std::string> code_view_lines_;
  std::string code_view_path_;
  int code_view_top_line_{0};
  int code_view_target_line_{0};
  mutable std::string status_line_{"Waiting for /rosout..."};
  Clock::time_point last_message_time_{};

public:
  void initialize_subscription() {
    log_subscription_ = this->create_subscription<LogMessage>(
      "/rosout", rclcpp::QoS(rclcpp::KeepLast(200)),
      [this](const LogMessage & message) { on_log_message(message); });
  }
};

}  // namespace

}  // namespace ros2_console_tools

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_console_tools::LogViewerNode>();
  node->initialize_subscription();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const int result = node->run();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
