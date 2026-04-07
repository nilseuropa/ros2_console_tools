#include "ros2_console_tools/diagnostics_viewer.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <thread>
#include <vector>

namespace ros2_console_tools {

int run_diagnostics_viewer_tool(bool embedded_mode) {
  auto backend = std::make_shared<DiagnosticsViewerBackend>();
  backend->initialize_subscriptions();
  DiagnosticsViewerScreen screen(backend, embedded_mode);

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
};

using tui::Session;
using tui::apply_role_chgat;
using tui::draw_box;
using tui::draw_help_bar;
using tui::draw_search_box;
using tui::draw_status_bar;
using tui::draw_text_vline;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::start_search;
using tui::theme_attr;
using tui::truncate_text;

}  // namespace

DiagnosticsViewerScreen::DiagnosticsViewerScreen(
  std::shared_ptr<DiagnosticsViewerBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int DiagnosticsViewerScreen::run() {
  Session ncurses_session;

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

bool DiagnosticsViewerScreen::handle_key(int key) {
  if (is_alt_binding(key, 't')) {
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

  switch (key) {
    case KEY_F(10):
      return false;
    case '\t':
      focus_ = focus_ == DiagnosticsPaneFocus::StatusList
        ? DiagnosticsPaneFocus::Details
        : DiagnosticsPaneFocus::StatusList;
      return true;
    case KEY_F(4):
      backend_->clear_entries();
      return true;
    case KEY_F(5):
      backend_->cycle_minimum_level();
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        backend_->set_status(
          focus_ == DiagnosticsPaneFocus::StatusList ? "Search diagnostic statuses." : "Search details.");
        return true;
      }
      if (embedded_mode_) {
        return false;
      }
      return true;
    default:
      break;
  }

  if (focus_ == DiagnosticsPaneFocus::StatusList) {
    return handle_status_list_key(key);
  }
  return handle_detail_key(key);
}

bool DiagnosticsViewerScreen::handle_search_key(int key) {
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

  if (focus_ == DiagnosticsPaneFocus::StatusList) {
    const auto snapshot = backend_->status_snapshot();
    std::vector<std::string> labels;
    labels.reserve(snapshot.size());
    for (const auto & row : snapshot) {
      labels.push_back(row.name + " " + row.message + " " + row.hardware_id);
    }
    const int match = find_best_match(labels, search_state_.query, backend_->selected_status_index_);
    if (match >= 0) {
      backend_->selected_status_index_ = match;
      backend_->selected_detail_index_ = 0;
      backend_->detail_scroll_ = 0;
    }
  } else {
    const auto status_snapshot = backend_->status_snapshot();
    const auto detail_snapshot = backend_->detail_snapshot(backend_->selected_entry_key(status_snapshot));
    std::vector<std::string> labels;
    labels.reserve(detail_snapshot.size());
    for (const auto & line : detail_snapshot) {
      labels.push_back(line.text);
    }
    const int match = find_best_match(labels, search_state_.query, backend_->selected_detail_index_);
    if (match >= 0) {
      backend_->selected_detail_index_ = match;
    }
  }

  backend_->set_status("Search: " + search_state_.query);
  return true;
}

bool DiagnosticsViewerScreen::handle_status_list_key(int key) {
  const auto snapshot = backend_->status_snapshot();
  backend_->clamp_status_selection(snapshot);

  switch (key) {
    case KEY_UP:
    case 'k':
      if (backend_->selected_status_index_ > 0) {
        --backend_->selected_status_index_;
        backend_->selected_detail_index_ = 0;
        backend_->detail_scroll_ = 0;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_status_index_ + 1 < static_cast<int>(snapshot.size())) {
        ++backend_->selected_status_index_;
        backend_->selected_detail_index_ = 0;
        backend_->detail_scroll_ = 0;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_status_index_ = std::max(0, backend_->selected_status_index_ - page_step());
      backend_->selected_detail_index_ = 0;
      backend_->detail_scroll_ = 0;
      return true;
    case KEY_NPAGE:
      if (!snapshot.empty()) {
        backend_->selected_status_index_ = std::min(
          static_cast<int>(snapshot.size()) - 1, backend_->selected_status_index_ + page_step());
        backend_->selected_detail_index_ = 0;
        backend_->detail_scroll_ = 0;
      }
      return true;
    case '\n':
    case KEY_ENTER:
      focus_ = DiagnosticsPaneFocus::Details;
      return true;
    default:
      return true;
  }
}

bool DiagnosticsViewerScreen::handle_detail_key(int key) {
  const auto status_snapshot = backend_->status_snapshot();
  const auto detail_snapshot = backend_->detail_snapshot(backend_->selected_entry_key(status_snapshot));
  backend_->clamp_detail_selection(detail_snapshot);

  switch (key) {
    case KEY_UP:
    case 'k':
      if (backend_->selected_detail_index_ > 0) {
        --backend_->selected_detail_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_detail_index_ + 1 < static_cast<int>(detail_snapshot.size())) {
        ++backend_->selected_detail_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_detail_index_ = std::max(0, backend_->selected_detail_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!detail_snapshot.empty()) {
        backend_->selected_detail_index_ = std::min(
          static_cast<int>(detail_snapshot.size()) - 1,
          backend_->selected_detail_index_ + page_step());
      }
      return true;
    case '\n':
    case KEY_ENTER:
      focus_ = DiagnosticsPaneFocus::StatusList;
      return true;
    default:
      return true;
  }
}

int DiagnosticsViewerScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

void DiagnosticsViewerScreen::draw() {
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
  mvprintw(0, 1, "Diagnostics Viewer ");
  attroff(theme_attr(kColorTitle));

  const int left_width = std::max(34, (columns - 2) / 2);
  const int separator_x = 1 + left_width;
  draw_status_list(1, 1, content_bottom - 1, separator_x - 1);
  attron(COLOR_PAIR(kColorFrame));
  draw_text_vline(1, separator_x, content_bottom - 1);
  attroff(COLOR_PAIR(kColorFrame));
  draw_details(1, separator_x + 1, content_bottom - 1, columns - 2);
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(layout.pane_rows, columns, search_state_);
  if (terminal_pane_.visible()) {
    terminal_pane_.draw(layout.terminal_top, 0, rows - 1, columns - 1);
  }
  refresh();
}

void DiagnosticsViewerScreen::draw_status_list(int top, int left, int bottom, int right) {
  const auto snapshot = backend_->status_snapshot();
  backend_->clamp_status_selection(snapshot);

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top);
  if (backend_->selected_status_index_ < backend_->status_scroll_) {
    backend_->status_scroll_ = backend_->selected_status_index_;
  }
  if (backend_->selected_status_index_ >= backend_->status_scroll_ + visible_rows) {
    backend_->status_scroll_ = std::max(0, backend_->selected_status_index_ - visible_rows + 1);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(
    top, left, "%-*s", width,
    focus_ == DiagnosticsPaneFocus::StatusList ? "Statuses <" : "Statuses");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  const int first_row = backend_->status_scroll_;
  const int last_row = std::min(static_cast<int>(snapshot.size()), first_row + visible_rows);
  for (int index = first_row; index < last_row && row_y <= bottom; ++index, ++row_y) {
    const auto & row = snapshot[static_cast<std::size_t>(index)];
    const bool selected = focus_ == DiagnosticsPaneFocus::StatusList && index == backend_->selected_status_index_;
    mvhline(row_y, left, ' ', width);
    const std::string header =
      "[" + diagnostic_level_string(row.level) + "] " + row.name;
    const std::string suffix = row.age == "-" ? "" : "  " + row.age;
    const std::string rendered = truncate_text(header + suffix, width);
    mvaddnstr(row_y, left, rendered.c_str(), width);

    const int color = diagnostic_level_color(row.level, selected);
    if (color != 0) {
      apply_role_chgat(row_y, left, width, color);
    }
    if (!row.message.empty() && row_y + 1 <= bottom) {
      ++row_y;
      mvhline(row_y, left, ' ', width);
      mvaddnstr(row_y, left, truncate_text("  " + row.message, width).c_str(), width);
      if (color != 0) {
        apply_role_chgat(row_y, left, width, color);
      }
    }
    if (selected) {
      apply_role_chgat(row_y - (row.message.empty() ? 0 : 1), left, width, kColorSelection);
      if (!row.message.empty()) {
        apply_role_chgat(row_y, left, width, kColorSelection);
      }
    }
  }
  for (; row_y <= bottom; ++row_y) {
    mvhline(row_y, left, ' ', width);
  }
}

void DiagnosticsViewerScreen::draw_details(int top, int left, int bottom, int right) {
  const auto status_snapshot = backend_->status_snapshot();
  const auto detail_snapshot = backend_->detail_snapshot(backend_->selected_entry_key(status_snapshot));
  backend_->clamp_detail_selection(detail_snapshot);

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top);
  if (backend_->selected_detail_index_ < backend_->detail_scroll_) {
    backend_->detail_scroll_ = backend_->selected_detail_index_;
  }
  if (backend_->selected_detail_index_ >= backend_->detail_scroll_ + visible_rows) {
    backend_->detail_scroll_ = std::max(0, backend_->selected_detail_index_ - visible_rows + 1);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, focus_ == DiagnosticsPaneFocus::Details ? "Details <" : "Details");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  const int first_row = backend_->detail_scroll_;
  const int last_row = std::min(static_cast<int>(detail_snapshot.size()), first_row + visible_rows);
  for (int index = first_row; index < last_row && row_y <= bottom; ++index, ++row_y) {
    const auto & line = detail_snapshot[static_cast<std::size_t>(index)];
    const bool selected = focus_ == DiagnosticsPaneFocus::Details && index == backend_->selected_detail_index_;
    mvhline(row_y, left, ' ', width);
    if (line.is_header) {
      attron(theme_attr(kColorHeader));
      mvaddnstr(row_y, left, truncate_text(line.text, width).c_str(), width);
      attroff(theme_attr(kColorHeader));
    } else {
      mvaddnstr(row_y, left, truncate_text(line.text, width).c_str(), width);
    }
    if (selected) {
      apply_role_chgat(row_y, left, width, kColorSelection);
    }
  }
  for (; row_y <= bottom; ++row_y) {
    mvhline(row_y, left, ' ', width);
  }
}

void DiagnosticsViewerScreen::draw_status_line(int row, int columns) const {
  std::string line;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    line = backend_->status_line_;
  }
  draw_status_bar(row, columns, line);
}

void DiagnosticsViewerScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(
    row,
    columns,
    tui::with_terminal_help(
      "Tab Switch Pane  Enter Focus Pane  F4 Clear  F5 Level Filter  Alt+S Search  F10 Exit",
      terminal_pane_.visible()));
}

}  // namespace ros2_console_tools
