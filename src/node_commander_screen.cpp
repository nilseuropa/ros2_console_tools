#include "ros2_console_tools/node_commander.hpp"

#include <ncursesw/ncurses.h>

namespace ros2_console_tools {

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
};

using tui::Session;
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
using tui::truncate_text;

}  // namespace

NodeCommanderScreen::NodeCommanderScreen(std::shared_ptr<NodeCommanderBackend> backend)
: backend_(std::move(backend)) {}

int NodeCommanderScreen::run() {
  Session ncurses_session;
  backend_->refresh_nodes();
  backend_->warm_up_node_list();

  bool running = true;
  while (running && rclcpp::ok()) {
    backend_->maybe_refresh_nodes();
    draw();
    const int key = getch();
    if (key == ERR) {
      continue;
    }
    running = handle_key(key);
  }

  return 0;
}

bool NodeCommanderScreen::handle_key(int key) {
  if (search_state_.active) {
    return handle_search_key(key);
  }

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(4):
      backend_->refresh_nodes();
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        backend_->status_line_ = "Search.";
        return true;
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
      if (backend_->selected_index_ + 1 < static_cast<int>(backend_->node_entries_.size())) {
        ++backend_->selected_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_index_ = std::max(0, backend_->selected_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!backend_->node_entries_.empty()) {
        backend_->selected_index_ = std::min(
          static_cast<int>(backend_->node_entries_.size()) - 1,
          backend_->selected_index_ + page_step());
      }
      return true;
    default:
      return true;
  }
}

bool NodeCommanderScreen::handle_search_key(int key) {
  const SearchInputResult result = handle_search_input(search_state_, key);
  if (result == SearchInputResult::Cancelled) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Search cancelled.";
    return true;
  }
  if (result == SearchInputResult::Accepted) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query;
    return true;
  }
  if (result != SearchInputResult::Changed) {
    return true;
  }

  const int match = find_best_match(backend_->node_entries_, search_state_.query, backend_->selected_index_);
  if (match >= 0) {
    backend_->selected_index_ = match;
  }
  std::lock_guard<std::mutex> lock(backend_->mutex_);
  backend_->status_line_ = "Search: " + search_state_.query;
  return true;
}

int NodeCommanderScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

void NodeCommanderScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = std::max(1, status_row - 1);

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(COLOR_PAIR(kColorTitle));
  mvprintw(0, 1, "Node Commander ");
  attroff(COLOR_PAIR(kColorTitle));

  const int left_width = std::max(28, (columns - 2) / 3);
  const int separator_x = 1 + left_width;
  draw_node_list(1, 1, content_bottom - 1, separator_x - 1);
  attron(COLOR_PAIR(kColorFrame));
  draw_text_vline(1, separator_x, content_bottom - 1);
  attroff(COLOR_PAIR(kColorFrame));
  draw_detail_pane(1, separator_x + 1, content_bottom - 1, columns - 2);
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(rows, columns, search_state_);
  refresh();
}

void NodeCommanderScreen::draw_node_list(int top, int left, int bottom, int right) {
  backend_->clamp_selection();

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  if (backend_->selected_index_ < backend_->node_scroll_) {
    backend_->node_scroll_ = backend_->selected_index_;
  }
  if (backend_->selected_index_ >= backend_->node_scroll_ + visible_rows - 1) {
    backend_->node_scroll_ = std::max(0, backend_->selected_index_ - visible_rows + 2);
  }

  attron(COLOR_PAIR(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Nodes");
  attroff(COLOR_PAIR(kColorHeader));

  const int first_row = backend_->node_scroll_;
  const int last_row = std::min(static_cast<int>(backend_->node_entries_.size()), first_row + visible_rows - 1);
  for (int row_y = top + 1; row_y <= bottom; ++row_y) {
    const bool has_item = first_row + (row_y - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row_y - top - 1) == backend_->selected_index_);
    mvhline(row_y, left, ' ', width);
    if (selected) {
      mvchgat(row_y, left, width, A_NORMAL, kColorSelection, nullptr);
    }
    if (!has_item) {
      continue;
    }

    const auto & entry = backend_->node_entries_[static_cast<std::size_t>(first_row + (row_y - top - 1))];
    mvprintw(row_y, left, "%-*s", width, truncate_text(entry, width).c_str());
    if (selected) {
      mvchgat(row_y, left, width, A_NORMAL, kColorSelection, nullptr);
    }
  }
}

void NodeCommanderScreen::draw_detail_pane(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  const auto lines = backend_->selected_node_details();

  attron(COLOR_PAIR(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Details");
  attroff(COLOR_PAIR(kColorHeader));

  int row_y = top + 1;
  for (const auto & line : lines) {
    if (row_y > bottom) {
      break;
    }
    if (line.is_header) {
      attron(COLOR_PAIR(kColorHeader));
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
      attroff(COLOR_PAIR(kColorHeader));
    } else {
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
    }
    ++row_y;
  }
  for (; row_y <= bottom; ++row_y) {
    mvhline(row_y, left, ' ', width);
  }
}

void NodeCommanderScreen::draw_status_line(int row, int columns) const {
  std::string line;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    line = backend_->status_line_;
    if (!backend_->node_entries_.empty() &&
      backend_->selected_index_ >= 0 &&
      backend_->selected_index_ < static_cast<int>(backend_->node_entries_.size()))
    {
      line = backend_->node_entries_[static_cast<std::size_t>(backend_->selected_index_)] + "  " + line;
    }
  }
  draw_status_bar(row, columns, line);
}

void NodeCommanderScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(row, columns, "Alt+S Search  F4 Refresh  F10 Exit");
}

}  // namespace ros2_console_tools
