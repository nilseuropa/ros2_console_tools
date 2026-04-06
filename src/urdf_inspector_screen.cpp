#include "ros2_console_tools/urdf_inspector.hpp"

#include <ncursesw/ncurses.h>

namespace ros2_console_tools {

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorLink = tui::kColorAccent,
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

UrdfInspectorScreen::UrdfInspectorScreen(std::shared_ptr<UrdfInspectorBackend> backend)
: backend_(std::move(backend)) {}

int UrdfInspectorScreen::run() {
  Session ncurses_session;
  backend_->refresh_model();
  backend_->warm_up_model();

  bool running = true;
  while (running && rclcpp::ok()) {
    backend_->maybe_refresh_model();
    draw();
    const int key = getch();
    if (key == ERR) {
      continue;
    }
    running = handle_key(key);
  }

  return 0;
}

bool UrdfInspectorScreen::handle_key(int key) {
  if (search_state_.active) {
    return handle_search_key(key);
  }

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(4):
      backend_->refresh_model();
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
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
      if (backend_->selected_index_ + 1 < static_cast<int>(backend_->rows_.size())) {
        ++backend_->selected_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_index_ = std::max(0, backend_->selected_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!backend_->rows_.empty()) {
        backend_->selected_index_ = std::min(
          static_cast<int>(backend_->rows_.size()) - 1,
          backend_->selected_index_ + page_step());
      }
      return true;
    case KEY_RIGHT:
    case 'l':
      backend_->expand_selected();
      return true;
    case KEY_LEFT:
    case 'h':
      backend_->collapse_selected();
      return true;
    case '\n':
    case KEY_ENTER:
      if (!backend_->rows_.empty() && backend_->rows_[static_cast<std::size_t>(backend_->selected_index_)].is_link) {
        const auto & row = backend_->rows_[static_cast<std::size_t>(backend_->selected_index_)];
        const bool collapsed = backend_->collapsed_links_[row.link_name];
        if (collapsed) {
          backend_->expand_selected();
        } else {
          backend_->collapse_selected();
        }
      }
      return true;
    default:
      return true;
  }
}

bool UrdfInspectorScreen::handle_search_key(int key) {
  const SearchInputResult result = handle_search_input(search_state_, key);
  if (result == SearchInputResult::Cancelled) {
    backend_->status_line_ = "Search cancelled.";
    return true;
  }
  if (result == SearchInputResult::Accepted) {
    backend_->status_line_ = search_state_.query.empty() ? "Search closed." : "Search: " + search_state_.query;
    return true;
  }
  if (result != SearchInputResult::Changed) {
    return true;
  }

  std::vector<std::string> labels;
  labels.reserve(backend_->rows_.size());
  for (const auto & row : backend_->rows_) {
    labels.push_back(row.label);
  }
  const int match = find_best_match(labels, search_state_.query, backend_->selected_index_);
  if (match >= 0) {
    backend_->selected_index_ = match;
  }
  backend_->status_line_ = "Search: " + search_state_.query;
  return true;
}

int UrdfInspectorScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

void UrdfInspectorScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = std::max(1, status_row - 1);

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(COLOR_PAIR(kColorTitle));
  mvprintw(0, 1, "URDF Inspector ");
  attroff(COLOR_PAIR(kColorTitle));

  const int left_width = std::max(28, (columns - 2) / 2);
  const int separator_x = 1 + left_width;
  draw_tree_pane(1, 1, content_bottom - 1, separator_x - 1);
  attron(COLOR_PAIR(kColorFrame));
  draw_text_vline(1, separator_x, content_bottom - 1);
  attroff(COLOR_PAIR(kColorFrame));
  draw_details_pane(1, separator_x + 1, content_bottom - 1, columns - 2);
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(rows, columns, search_state_);
  refresh();
}

void UrdfInspectorScreen::draw_tree_pane(int top, int left, int bottom, int right) {
  backend_->clamp_selection();

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  if (backend_->selected_index_ < backend_->tree_scroll_) {
    backend_->tree_scroll_ = backend_->selected_index_;
  }
  if (backend_->selected_index_ >= backend_->tree_scroll_ + visible_rows - 1) {
    backend_->tree_scroll_ = std::max(0, backend_->selected_index_ - visible_rows + 2);
  }

  attron(COLOR_PAIR(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Tree");
  attroff(COLOR_PAIR(kColorHeader));

  const int first_row = backend_->tree_scroll_;
  const int last_row = std::min(static_cast<int>(backend_->rows_.size()), first_row + visible_rows - 1);
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

    const auto & row = backend_->rows_[static_cast<std::size_t>(first_row + (row_y - top - 1))];
    const std::string prefix =
      std::string(static_cast<std::size_t>(row.depth * 2), ' ')
      + (row.is_link ? "[] " : "() ");
    const std::string text = prefix + row.label;
    const int color = selected ? kColorSelection : (row.is_link ? kColorLink : 0);
    if (color != 0) {
      attron(COLOR_PAIR(color));
    }
    mvprintw(row_y, left, "%-*s", width, truncate_text(text, width).c_str());
    if (color != 0) {
      attroff(COLOR_PAIR(color));
    }
    if (selected) {
      mvchgat(row_y, left, width, A_NORMAL, kColorSelection, nullptr);
      mvchgat(row_y, left, std::min(width, static_cast<int>(text.size())), A_NORMAL, color, nullptr);
    }
  }
}

void UrdfInspectorScreen::draw_details_pane(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  const auto lines = backend_->selected_details();

  attron(COLOR_PAIR(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Details");
  attroff(COLOR_PAIR(kColorHeader));

  int row_y = top + 1;
  for (const auto & line : lines) {
    if (row_y > bottom) {
      break;
    }
    mvprintw(row_y, left, "%-*s", width, truncate_text(line, width).c_str());
    ++row_y;
  }
  for (; row_y <= bottom; ++row_y) {
    mvhline(row_y, left, ' ', width);
  }
}

void UrdfInspectorScreen::draw_status_line(int row, int columns) const {
  std::string line = backend_->status_line_;
  if (!backend_->source_node_.empty()) {
    line = "src=" + backend_->source_node_ + "  " + line;
  }
  draw_status_bar(row, columns, line);
}

void UrdfInspectorScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(row, columns, "Enter Toggle  Left Fold  Right Unfold  Alt+S Search  F4 Reload  F10 Exit");
}

}  // namespace ros2_console_tools
