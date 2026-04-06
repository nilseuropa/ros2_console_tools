#include "ros2_console_tools/action_commander.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_console_tools/tui.hpp"

namespace ros2_console_tools {

int run_action_commander_tool(const std::string & initial_action, bool embedded_mode) {
  auto backend = std::make_shared<ActionCommanderBackend>(initial_action);
  ActionCommanderScreen screen(backend, embedded_mode);

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
using tui::draw_box_char;
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

ActionCommanderScreen::ActionCommanderScreen(
  std::shared_ptr<ActionCommanderBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int ActionCommanderScreen::run() {
  Session ncurses_session;
  backend_->refresh_actions();

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

bool ActionCommanderScreen::handle_key(int key) {
  if (search_state_.active) {
    return handle_search_key(key);
  }
  return backend_->detail_open_ ? handle_detail_key(key) : handle_list_key(key);
}

bool ActionCommanderScreen::handle_search_key(int key) {
  const SearchInputResult result = handle_search_input(search_state_, key);
  if (result == SearchInputResult::Cancelled) {
    backend_->status_line_ = "Search cancelled.";
    return true;
  }
  if (result == SearchInputResult::Accepted) {
    backend_->status_line_ =
      search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query;
    return true;
  }
  if (result != SearchInputResult::Changed || backend_->detail_open_) {
    return true;
  }

  std::vector<std::string> labels;
  labels.reserve(backend_->actions_.size());
  for (const auto & entry : backend_->actions_) {
    labels.push_back(entry.name);
  }
  const int match = find_best_match(labels, search_state_.query, backend_->selected_index_);
  if (match >= 0) {
    backend_->selected_index_ = match;
  }
  backend_->status_line_ = "Search: " + search_state_.query;
  return true;
}

bool ActionCommanderScreen::handle_list_key(int key) {
  backend_->clamp_selection();
  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(4):
      backend_->refresh_actions();
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        backend_->status_line_ = "Search.";
        return true;
      }
      if (embedded_mode_) {
        return false;
      }
      return true;
    case KEY_UP:
    case 'k':
      if (backend_->selected_index_ > 0) {
        --backend_->selected_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_index_ + 1 < static_cast<int>(backend_->actions_.size())) {
        ++backend_->selected_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_index_ = std::max(0, backend_->selected_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!backend_->actions_.empty()) {
        backend_->selected_index_ = std::min(
          static_cast<int>(backend_->actions_.size()) - 1,
          backend_->selected_index_ + page_step());
      }
      return true;
    case '\n':
    case KEY_ENTER:
      backend_->open_selected_action();
      return true;
    default:
      return true;
  }
}

bool ActionCommanderScreen::handle_detail_key(int key) {
  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(4):
      backend_->refresh_actions();
      return true;
    case 27:
      if (embedded_mode_) {
        return false;
      }
      backend_->close_action_detail();
      return true;
    default:
      return true;
  }
}

int ActionCommanderScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

void ActionCommanderScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = std::max(1, status_row - 1);

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "Action Commander ");
  attroff(theme_attr(kColorTitle));

  if (backend_->detail_open_) {
    draw_action_detail(1, 1, content_bottom - 1, columns - 2);
  } else {
    draw_action_list(1, 1, content_bottom - 1, columns - 2);
  }

  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(rows, columns, search_state_);
  refresh();
}

void ActionCommanderScreen::draw_action_list(int top, int left, int bottom, int right) {
  backend_->clamp_selection();
  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  const int name_width = std::max(32, width / 2);
  const int type_width = std::max(12, width - name_width - 1);
  const int separator_x = left + name_width;

  if (backend_->selected_index_ < backend_->list_scroll_) {
    backend_->list_scroll_ = backend_->selected_index_;
  }
  if (backend_->selected_index_ >= backend_->list_scroll_ + visible_rows - 1) {
    backend_->list_scroll_ = std::max(0, backend_->selected_index_ - visible_rows + 2);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", name_width, "Action");
  draw_box_char(top, separator_x, WACS_VLINE, '|');
  mvprintw(top, separator_x + 1, "%-*s", type_width, "Type");
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->list_scroll_;
  const int last_row = std::min(static_cast<int>(backend_->actions_.size()), first_row + visible_rows - 1);
  for (int row = top + 1; row <= bottom; ++row) {
    const bool has_item = first_row + (row - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row - top - 1) == backend_->selected_index_);

    mvhline(row, left, ' ', width);
    if (selected) {
      apply_role_chgat(row, left, width, kColorSelection);
    }
    draw_box_char(row, separator_x, WACS_VLINE, '|');
    if (!has_item) {
      continue;
    }

    const auto & entry = backend_->actions_[static_cast<std::size_t>(first_row + (row - top - 1))];
    mvprintw(row, left, "%-*s", name_width, truncate_text(entry.name, name_width).c_str());
    mvprintw(row, separator_x + 1, "%-*s", type_width, truncate_text(entry.type, type_width).c_str());
    if (selected) {
      apply_role_chgat(row, left, width, kColorSelection);
      mvaddch(row, separator_x, '|');
    }
  }
}

void ActionCommanderScreen::draw_action_detail(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  const auto lines = backend_->selected_action_details();

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Details");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  for (const auto & line : lines) {
    if (row_y > bottom) {
      break;
    }
    if (line.is_header) {
      attron(theme_attr(kColorHeader));
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
      attroff(theme_attr(kColorHeader));
    } else {
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
    }
    ++row_y;
  }
  for (; row_y <= bottom; ++row_y) {
    mvhline(row_y, left, ' ', width);
  }
}

void ActionCommanderScreen::draw_status_line(int row, int columns) const {
  std::string line = backend_->status_line_;
  if (backend_->detail_open_ && !backend_->selected_action_.name.empty()) {
    line = truncate_text(
      backend_->selected_action_.name + " [" + backend_->selected_action_.type + "]  " + backend_->status_line_,
      columns - 1);
  }
  draw_status_bar(row, columns, line);
}

void ActionCommanderScreen::draw_help_line(int row, int columns) const {
  const std::string help =
    backend_->detail_open_
    ? "Esc Back  F4 Refresh  F10 Exit"
    : "Enter Inspect  Alt+S Search  F4 Refresh  F10 Exit";
  draw_help_bar(row, columns, help);
}

}  // namespace ros2_console_tools
