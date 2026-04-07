#include "ros2_console_tools/log_viewer.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <cctype>
#include <set>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_log_viewer_tool(bool embedded_mode) {
  auto backend = std::make_shared<LogViewerBackend>();
  backend->initialize_subscription();
  LogViewerScreen screen(backend, embedded_mode);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(backend);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const int result = screen.run();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return result;
}

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorWarn = tui::kColorWarn,
  kColorError = tui::kColorError,
  kColorFatal = tui::kColorFatal,
  kColorPositive = tui::kColorPositive,
  kColorSource = tui::kColorAccent,
  kColorPopup = tui::kColorPopup,
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
using tui::apply_role_chgat;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::start_search;
using tui::theme_attr;

}  // namespace

LogViewerScreen::LogViewerScreen(std::shared_ptr<LogViewerBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int LogViewerScreen::run() {
  Session ncurses_session;
  backend_->refresh_sources();

  bool running = true;
  while (running && rclcpp::ok()) {
    terminal_pane_.update();
    draw();
    const int key = getch();
    if (key == ERR) {
      continue;
    }
    running = handle_key(key);
  }

  return 0;
}

bool LogViewerScreen::handle_key(int key) {
  if (backend_->filter_prompt_open_) {
    return handle_filter_prompt_key(key);
  }
  if (backend_->detail_popup_open_) {
    return handle_detail_popup_key(key);
  }
  if (key == KEY_F(9)) {
    search_state_.active = false;
    terminal_pane_.toggle();
    return true;
  }
  if (terminal_pane_.visible()) {
    if (key == KEY_F(10)) {
      return false;
    }
    return terminal_pane_.handle_key(key);
  }
  if (search_state_.active) {
    return handle_search_key(key);
  }
  if (backend_->view_mode_ == LogViewerViewMode::CodeInspect) {
    return handle_code_view_key(key);
  }

  switch (key) {
    case KEY_F(10):
      return false;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        backend_->set_status("Search.");
        return true;
      }
      if (backend_->view_mode_ == LogViewerViewMode::SourceLive) {
        backend_->close_live_source();
        return true;
      }
      if (embedded_mode_) {
        return false;
      }
      break;
    case '\t':
      if (backend_->view_mode_ == LogViewerViewMode::Split) {
        backend_->focus_ = backend_->focus_ == PaneFocus::Sources ? PaneFocus::Logs : PaneFocus::Sources;
      }
      return true;
    case KEY_F(4):
      backend_->refresh_sources();
      return true;
    case KEY_F(5):
      backend_->hide_unselected_ = !backend_->hide_unselected_;
      backend_->log_scroll_ = 0;
      backend_->selected_log_index_ = 0;
      backend_->set_status(backend_->hide_unselected_ ? "Showing selected sources only." : "Showing all sources.");
      return true;
    case KEY_F(6):
      backend_->cycle_minimum_level();
      return true;
    case '/':
      backend_->filter_prompt_open_ = true;
      backend_->filter_buffer_ = backend_->text_filter_;
      backend_->set_status("Filter text: " + backend_->filter_buffer_);
      return true;
    default:
      break;
  }

  if (backend_->view_mode_ == LogViewerViewMode::SourceLive) {
    return handle_live_source_key(key);
  }
  if (backend_->focus_ == PaneFocus::Sources) {
    return handle_source_key(key);
  }
  return handle_log_key(key);
}

bool LogViewerScreen::handle_search_key(int key) {
  const SearchInputResult result = handle_search_input(search_state_, key);
  if (result == SearchInputResult::Cancelled) {
    backend_->set_status("Search cancelled.");
    return true;
  }
  if (result == SearchInputResult::Accepted) {
    backend_->set_status(search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query);
    return true;
  }
  if (result != SearchInputResult::Changed) {
    return true;
  }

  if (backend_->view_mode_ == LogViewerViewMode::SourceLive) {
    const auto snapshot = backend_->live_source_logs_snapshot();
    std::vector<std::string> labels;
    labels.reserve(snapshot.size());
    for (const auto & entry : snapshot) {
      labels.push_back(level_string(entry.level) + " " + entry.message);
    }
    const int match = find_best_match(labels, search_state_.query, backend_->selected_live_log_index_);
    if (match >= 0) {
      backend_->selected_live_log_index_ = match;
    }
    backend_->set_status("Search: " + search_state_.query);
    return true;
  }

  if (backend_->focus_ == PaneFocus::Sources) {
    const auto snapshot = backend_->source_snapshot();
    std::vector<std::string> labels;
    labels.reserve(snapshot.size());
    for (const auto & entry : snapshot) {
      labels.push_back(entry.name);
    }
    const int match = find_best_match(labels, search_state_.query, backend_->selected_source_index_);
    if (match >= 0) {
      backend_->selected_source_index_ = match;
    }
  } else {
    const auto snapshot = backend_->filtered_logs_snapshot();
    std::vector<std::string> labels;
    labels.reserve(snapshot.size());
    for (const auto & entry : snapshot) {
      labels.push_back(entry.source + " " + entry.message);
    }
    const int match = find_best_match(labels, search_state_.query, backend_->selected_log_index_);
    if (match >= 0) {
      backend_->selected_log_index_ = match;
    }
  }
  backend_->set_status("Search: " + search_state_.query);
  return true;
}

bool LogViewerScreen::handle_filter_prompt_key(int key) {
  switch (key) {
    case 27:
      backend_->filter_prompt_open_ = false;
      backend_->filter_buffer_.clear();
      backend_->set_status("Filter unchanged.");
      return true;
    case '\n':
    case KEY_ENTER:
      backend_->text_filter_ = backend_->filter_buffer_;
      backend_->filter_prompt_open_ = false;
      backend_->filter_buffer_.clear();
      backend_->selected_log_index_ = 0;
      backend_->log_scroll_ = 0;
      backend_->set_status(backend_->text_filter_.empty() ? "Text filter cleared." : "Text filter applied.");
      return true;
    case KEY_BACKSPACE:
    case 127:
    case '\b':
      if (!backend_->filter_buffer_.empty()) {
        backend_->filter_buffer_.pop_back();
      }
      backend_->set_status("Filter text: " + backend_->filter_buffer_);
      return true;
    default:
      if (key >= 32 && key <= 126) {
        backend_->filter_buffer_.push_back(static_cast<char>(key));
        backend_->set_status("Filter text: " + backend_->filter_buffer_);
      }
      return true;
  }
}

bool LogViewerScreen::handle_detail_popup_key(int key) {
  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(2):
      backend_->open_code_view();
      return true;
    case 27:
    case '\n':
    case KEY_ENTER:
      backend_->detail_popup_open_ = false;
      return true;
    default:
      return true;
  }
}

bool LogViewerScreen::handle_source_key(int key) {
  const auto snapshot = backend_->source_snapshot();
  backend_->clamp_source_selection(snapshot);

  switch (key) {
    case KEY_UP:
    case 'k':
      if (backend_->selected_source_index_ > 0) {
        --backend_->selected_source_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_source_index_ + 1 < static_cast<int>(snapshot.size())) {
        ++backend_->selected_source_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_source_index_ = std::max(0, backend_->selected_source_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!snapshot.empty()) {
        backend_->selected_source_index_ = std::min(
          static_cast<int>(snapshot.size()) - 1, backend_->selected_source_index_ + page_step());
      }
      return true;
    case ' ':
    case KEY_IC:
      backend_->toggle_selected_source();
      return true;
    case '\n':
    case KEY_ENTER:
      backend_->open_live_source();
      return true;
    default:
      return true;
  }
}

bool LogViewerScreen::handle_live_source_key(int key) {
  const auto snapshot = backend_->live_source_logs_snapshot();
  backend_->clamp_live_log_selection(snapshot);

  switch (key) {
    case KEY_UP:
    case 'k':
      if (backend_->selected_live_log_index_ > 0) {
        --backend_->selected_live_log_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_live_log_index_ + 1 < static_cast<int>(snapshot.size())) {
        ++backend_->selected_live_log_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_live_log_index_ = std::max(0, backend_->selected_live_log_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!snapshot.empty()) {
        backend_->selected_live_log_index_ = std::min(
          static_cast<int>(snapshot.size()) - 1,
          backend_->selected_live_log_index_ + page_step());
      }
      return true;
    case '\n':
    case KEY_ENTER:
      backend_->open_live_detail_popup(snapshot);
      return true;
    default:
      return true;
  }
}

bool LogViewerScreen::handle_log_key(int key) {
  const auto snapshot = backend_->filtered_logs_snapshot();
  backend_->clamp_log_selection(snapshot);

  switch (key) {
    case KEY_UP:
    case 'k':
      if (backend_->selected_log_index_ > 0) {
        --backend_->selected_log_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_log_index_ + 1 < static_cast<int>(snapshot.size())) {
        ++backend_->selected_log_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_log_index_ = std::max(0, backend_->selected_log_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!snapshot.empty()) {
        backend_->selected_log_index_ = std::min(
          static_cast<int>(snapshot.size()) - 1, backend_->selected_log_index_ + page_step());
      }
      return true;
    case '\n':
    case KEY_ENTER:
      backend_->open_detail_popup(snapshot);
      return true;
    default:
      return true;
  }
}

bool LogViewerScreen::handle_code_view_key(int key) {
  switch (key) {
    case KEY_F(10):
      return false;
    case 27:
    case KEY_ENTER:
    case '\n':
      backend_->close_code_view();
      return true;
    case KEY_UP:
    case 'k':
      if (backend_->code_view_top_line_ > 0) {
        --backend_->code_view_top_line_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->code_view_top_line_ + 1 < static_cast<int>(backend_->code_view_lines_.size())) {
        ++backend_->code_view_top_line_;
      }
      return true;
    case KEY_PPAGE:
      backend_->code_view_top_line_ = std::max(0, backend_->code_view_top_line_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!backend_->code_view_lines_.empty()) {
        backend_->code_view_top_line_ = std::min(
          std::max(0, static_cast<int>(backend_->code_view_lines_.size()) - 1),
          backend_->code_view_top_line_ + page_step());
      }
      return true;
    case KEY_HOME:
    case 'g':
      backend_->code_view_top_line_ = 0;
      return true;
    case KEY_END:
    case 'G':
      if (!backend_->code_view_lines_.empty()) {
        backend_->code_view_top_line_ = std::max(0, static_cast<int>(backend_->code_view_lines_.size()) - 1);
      }
      return true;
    default:
      return true;
  }
}

int LogViewerScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

void LogViewerScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const auto layout = tui::make_commander_layout(rows, terminal_pane_.visible());
  const int help_row = layout.help_row;
  const int status_row = layout.status_row;
  const int content_bottom = layout.content_bottom;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "Log Viewer ");
  attroff(theme_attr(kColorTitle));
  if (backend_->view_mode_ == LogViewerViewMode::CodeInspect) {
    draw_code_view_pane(1, 1, content_bottom - 1, columns - 2);
  } else if (backend_->view_mode_ == LogViewerViewMode::Split) {
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
  if (backend_->detail_popup_open_) {
    draw_detail_popup(rows, columns);
  }
  draw_search_box(layout.pane_rows, columns, search_state_);
  if (terminal_pane_.visible()) {
    terminal_pane_.draw(layout.terminal_top, 0, rows - 1, columns - 1);
  }

  refresh();
}

bool LogViewerScreen::is_identifier_char(char character) {
  return std::isalnum(static_cast<unsigned char>(character)) || character == '_';
}

bool LogViewerScreen::is_keyword(const std::string & token) {
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

std::vector<StyledSpan> LogViewerScreen::highlight_code_line(const std::string & line) const {
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

void LogViewerScreen::draw_code_line(
  int row, int left, int width, int line_number, const std::string & line, bool highlight) const
{
  mvhline(row, left, ' ', width);
  if (highlight) {
    apply_role_chgat(row, left, width, kColorSelection);
  }

  const int line_number_width = std::max(4, static_cast<int>(std::to_string(
    std::max(1, static_cast<int>(backend_->code_view_lines_.size()))).size()));
  const std::string line_prefix =
    (highlight ? ">" : " ") + std::string(line_number_width - static_cast<int>(std::to_string(line_number).size()), ' ')
    + std::to_string(line_number) + " ";

  if (highlight) {
    mvaddnstr(row, left, line_prefix.c_str(), width);
    mvaddnstr(
      row, left + static_cast<int>(line_prefix.size()),
      truncate_text(line, width - static_cast<int>(line_prefix.size())).c_str(),
      width - static_cast<int>(line_prefix.size()));
    apply_role_chgat(row, left, width, kColorSelection);
    return;
  }

  attron(theme_attr(kColorHeader));
  mvaddnstr(row, left, line_prefix.c_str(), width);
  attroff(theme_attr(kColorHeader));

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

void LogViewerScreen::draw_code_view_pane(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  const std::string title =
    truncate_text("Code: " + backend_->code_view_path_ + ":" + std::to_string(backend_->code_view_target_line_), width);

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, title.c_str());
  attroff(theme_attr(kColorHeader));

  const int max_top_line = std::max(0, static_cast<int>(backend_->code_view_lines_.size()) - (visible_rows - 1));
  const int top_line = std::clamp(backend_->code_view_top_line_, 0, max_top_line);
  for (int row = top + 1; row <= bottom; ++row) {
    const int line_index = top_line + (row - top - 1);
    if (line_index >= static_cast<int>(backend_->code_view_lines_.size())) {
      mvhline(row, left, ' ', width);
      continue;
    }
    draw_code_line(
      row, left, width, line_index + 1, backend_->code_view_lines_[static_cast<std::size_t>(line_index)],
      line_index + 1 == backend_->code_view_target_line_);
  }
}

void LogViewerScreen::draw_sources_pane(int top, int left, int bottom, int right) {
  const auto snapshot = backend_->source_snapshot();
  backend_->clamp_source_selection(snapshot);

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  if (backend_->selected_source_index_ < backend_->source_scroll_) {
    backend_->source_scroll_ = backend_->selected_source_index_;
  }
  if (backend_->selected_source_index_ >= backend_->source_scroll_ + visible_rows - 1) {
    backend_->source_scroll_ = std::max(0, backend_->selected_source_index_ - visible_rows + 2);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Sources");
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->source_scroll_;
  const int last_row = std::min(static_cast<int>(snapshot.size()), first_row + visible_rows - 1);
  for (int row = top + 1; row <= bottom; ++row) {
    const bool has_item = first_row + (row - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row - top - 1) == backend_->selected_source_index_);
    mvhline(row, left, ' ', width);
    if (selected && backend_->focus_ == PaneFocus::Sources) {
      apply_role_chgat(row, left, width, kColorSelection);
    }
    if (!has_item) {
      continue;
    }

    const auto & entry = snapshot[static_cast<std::size_t>(first_row + (row - top - 1))];
    const std::string label = std::string(entry.selected ? "* " : "  ") + entry.name;
    const int color = selected && backend_->focus_ == PaneFocus::Sources ? kColorSelection : kColorSource;
    attron(COLOR_PAIR(color));
    mvprintw(row, left, "%-*s", width, truncate_text(label, width).c_str());
    attroff(COLOR_PAIR(color));
    if (selected && backend_->focus_ == PaneFocus::Sources) {
      apply_role_chgat(row, left, width, kColorSelection);
      apply_role_chgat(row, left, std::min(width, static_cast<int>(label.size())), color);
    }
  }
}

void LogViewerScreen::draw_logs_pane(int top, int left, int bottom, int right) {
  const auto snapshot = backend_->filtered_logs_snapshot();
  backend_->clamp_log_selection(snapshot);

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  const int time_width = 12;
  const int level_width = 6;
  const int source_width = std::max(16, width / 4);
  const int message_width = std::max(10, width - time_width - level_width - source_width - 3);
  const int sep_one_x = left + time_width;
  const int sep_two_x = sep_one_x + 1 + level_width;
  const int sep_three_x = sep_two_x + 1 + source_width;

  if (backend_->selected_log_index_ < backend_->log_scroll_) {
    backend_->log_scroll_ = backend_->selected_log_index_;
  }
  if (backend_->selected_log_index_ >= backend_->log_scroll_ + visible_rows - 1) {
    backend_->log_scroll_ = std::max(0, backend_->selected_log_index_ - visible_rows + 2);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", time_width, "Time");
  draw_box_char(top, sep_one_x, WACS_VLINE, '|');
  mvprintw(top, sep_one_x + 1, "%-*s", level_width, "Level");
  draw_box_char(top, sep_two_x, WACS_VLINE, '|');
  mvprintw(top, sep_two_x + 1, "%-*s", source_width, "Source");
  draw_box_char(top, sep_three_x, WACS_VLINE, '|');
  mvprintw(top, sep_three_x + 1, "%-*s", message_width, "Message");
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->log_scroll_;
  const int last_row = std::min(static_cast<int>(snapshot.size()), first_row + visible_rows - 1);
  for (int row = top + 1; row <= bottom; ++row) {
    const bool has_item = first_row + (row - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row - top - 1) == backend_->selected_log_index_);
    mvhline(row, left, ' ', width);
    if (selected && backend_->focus_ == PaneFocus::Logs) {
      apply_role_chgat(row, left, width, kColorSelection);
    }

    draw_box_char(row, sep_one_x, WACS_VLINE, '|');
    draw_box_char(row, sep_two_x, WACS_VLINE, '|');
    draw_box_char(row, sep_three_x, WACS_VLINE, '|');
    if (!has_item) {
      continue;
    }

    const auto & entry = snapshot[static_cast<std::size_t>(first_row + (row - top - 1))];
    const int color = level_color(entry.level, selected && backend_->focus_ == PaneFocus::Logs);
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
    if (selected && backend_->focus_ == PaneFocus::Logs) {
      apply_role_chgat(row, left, width, kColorSelection);
      draw_box_char(row, sep_one_x, WACS_VLINE, '|');
      draw_box_char(row, sep_two_x, WACS_VLINE, '|');
      draw_box_char(row, sep_three_x, WACS_VLINE, '|');
      if (color != 0) {
        apply_role_chgat(row, left, time_width, color);
        apply_role_chgat(row, sep_one_x + 1, level_width, color);
        apply_role_chgat(row, sep_two_x + 1, source_width, color);
        apply_role_chgat(row, sep_three_x + 1, message_width, color);
      }
    }
  }
}

void LogViewerScreen::draw_live_source_pane(int top, int left, int bottom, int right) {
  const auto snapshot = backend_->live_source_logs_snapshot();
  backend_->clamp_live_log_selection(snapshot);

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  if (backend_->selected_live_log_index_ < backend_->live_log_scroll_) {
    backend_->live_log_scroll_ = backend_->selected_live_log_index_;
  }
  if (backend_->selected_live_log_index_ >= backend_->live_log_scroll_ + visible_rows - 1) {
    backend_->live_log_scroll_ = std::max(0, backend_->selected_live_log_index_ - visible_rows + 2);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, truncate_text("Live: " + backend_->live_source_name_, width).c_str());
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->live_log_scroll_;
  const int last_row = std::min(static_cast<int>(snapshot.size()), first_row + visible_rows - 1);
  for (int row = top + 1; row <= bottom; ++row) {
    const bool has_item = first_row + (row - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row - top - 1) == backend_->selected_live_log_index_);
    mvhline(row, left, ' ', width);
    if (selected) {
      apply_role_chgat(row, left, width, kColorSelection);
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
      apply_role_chgat(row, left, width, kColorSelection);
      apply_role_chgat(row, left, static_cast<int>(rendered.size()), color);
    }
  }
}

void LogViewerScreen::draw_status_line(int row, int columns) const {
  std::string line = backend_->status_line_;
  if (backend_->filter_prompt_open_) {
    line = "Filter text: " + backend_->filter_buffer_;
  } else {
    line += "  level>=" + level_string(backend_->minimum_level_);
    if (!backend_->text_filter_.empty()) {
      line += "  filter=\"" + backend_->text_filter_ + "\"";
    }
    line += "  follow=" + std::string(backend_->hide_unselected_ ? "selected" : "all");
  }
  draw_status_bar(row, columns, line);
}

void LogViewerScreen::draw_help_line(int row, int columns) const {
  const std::string help =
    backend_->view_mode_ == LogViewerViewMode::CodeInspect
    ? "Esc Back  Up Down Scroll  PgUp PgDn Page  Home Top  End Bottom  F10 Exit"
    : backend_->view_mode_ == LogViewerViewMode::SourceLive
    ? "Enter Inspect  Alt+S Search  Esc Back  F4 Refresh  F6 Level  / Filter  F10 Exit"
    : "Tab Pane  Space Mark  Enter Open  Alt+S Search  F4 Refresh  F5 Hide Unselected  F6 Level  / Filter  F10 Exit";
  draw_help_bar(row, columns, tui::with_terminal_help(help, terminal_pane_.visible()));
}

void LogViewerScreen::draw_detail_popup(int rows, int columns) const {
  const int popup_width = std::min(columns - 6, 96);
  const int popup_height = std::min(rows - 6, 12);
  const int left = std::max(2, (columns - popup_width) / 2);
  const int top = std::max(2, (rows - popup_height) / 2);
  const int right = left + popup_width - 1;
  const int bottom = top + popup_height - 1;

  for (int row = top; row <= bottom; ++row) {
    mvhline(row, left, ' ', popup_width);
    apply_role_chgat(row, left, popup_width, kColorPopup);
  }
  draw_box(top, left, bottom, right, kColorFrame);

  const auto print_line = [&](int row, const std::string & label, const std::string & value) {
    const std::string text = label + value;
    mvprintw(row, left + 2, "%s", truncate_text(text, popup_width - 4).c_str());
  };

  print_line(top + 1, "time: ", time_string(backend_->detail_entry_.stamp));
  print_line(top + 2, "level: ", level_string(backend_->detail_entry_.level));
  print_line(top + 3, "source: ", backend_->detail_entry_.source);
  print_line(top + 4, "file: ", backend_->detail_entry_.file);
  print_line(top + 5, "function: ", backend_->detail_entry_.function_name);
  print_line(top + 6, "line: ", std::to_string(backend_->detail_entry_.line));
  print_line(top + 8, "message: ", backend_->detail_entry_.message);
  draw_help_bar_region(bottom - 1, left + 2, popup_width - 4, "F2 Inspect Code  Esc Close  Enter Close  F10 Exit");
}

}  // namespace ros2_console_tools
