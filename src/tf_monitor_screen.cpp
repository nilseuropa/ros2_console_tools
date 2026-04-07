#include "ros2_console_tools/tf_monitor.hpp"

#include <ncursesw/ncurses.h>
#include <thread>

namespace ros2_console_tools {

int run_tf_monitor_tool(bool embedded_mode) {
  auto backend = std::make_shared<TfMonitorBackend>();
  backend->initialize_subscriptions();
  TfMonitorScreen screen(backend, embedded_mode);

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
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorStatus = tui::kColorStatus,
  kColorHelp = tui::kColorHelp,
  kColorDynamic = tui::kColorAccent,
  kColorStale = tui::kColorError,
};

using tui::Session;
using tui::draw_box;
using tui::draw_box_char;
using tui::draw_help_bar;
using tui::draw_help_bar_region;
using tui::draw_search_box;
using tui::draw_status_bar;
using tui::apply_role_chgat;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::start_search;
using tui::theme_attr;
using tui::truncate_text;

}  // namespace

TfMonitorScreen::TfMonitorScreen(std::shared_ptr<TfMonitorBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int TfMonitorScreen::run() {
  Session ncurses_session;
  backend_->refresh_rows();

  bool running = true;
  while (running && rclcpp::ok()) {
    backend_->refresh_rows();
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

bool TfMonitorScreen::handle_key(int key) {
  if (backend_->inspect_popup_open_) {
    return handle_popup_key(key);
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

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(4):
      backend_->refresh_rows();
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        backend_->set_status("Search.");
        return true;
      }
      if (embedded_mode_) {
        return false;
      }
      return true;
    case ' ':
    case KEY_IC:
      backend_->toggle_selected_row();
      return true;
    case '\n':
    case KEY_ENTER:
      backend_->open_inspect_popup();
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
    default:
      return true;
  }
}

bool TfMonitorScreen::handle_popup_key(int key) {
  switch (key) {
    case KEY_F(10):
      return false;
    case 27:
    case '\n':
    case KEY_ENTER:
      backend_->inspect_popup_open_ = false;
      return true;
    default:
      return true;
  }
}

bool TfMonitorScreen::handle_search_key(int key) {
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

  std::vector<std::string> labels;
  labels.reserve(backend_->rows_.size());
  for (const auto & row : backend_->rows_) {
    labels.push_back(row.parent_frame + " " + row.child_frame);
  }
  const int match = find_best_match(labels, search_state_.query, backend_->selected_index_);
  if (match >= 0) {
    backend_->selected_index_ = match;
  }
  backend_->set_status("Search: " + search_state_.query);
  return true;
}

int TfMonitorScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

void TfMonitorScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const auto layout = tui::make_commander_layout(rows, terminal_pane_.visible());
  const int help_row = layout.help_row;
  const int status_row = layout.status_row;
  const int content_bottom = layout.content_bottom;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  mvprintw(0, 1, "TF Monitor ");
  draw_tree_pane(1, 1, content_bottom - 1, columns - 2);
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(layout.pane_rows, columns, search_state_);
  if (backend_->inspect_popup_open_) {
    draw_inspect_popup(rows, columns);
  }
  if (terminal_pane_.visible()) {
    terminal_pane_.draw(layout.terminal_top, 0, rows - 1, columns - 1);
  }
  refresh();
}

void TfMonitorScreen::draw_tree_pane(int top, int left, int bottom, int right) {
  backend_->clamp_selection();

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  const int tree_width = std::max(24, width - 14);
  const int freshness_width = std::max(10, width - tree_width - 1);
  const int sep_one_x = left + tree_width;

  if (backend_->selected_index_ < backend_->scroll_) {
    backend_->scroll_ = backend_->selected_index_;
  }
  if (backend_->selected_index_ >= backend_->scroll_ + visible_rows - 1) {
    backend_->scroll_ = std::max(0, backend_->selected_index_ - visible_rows + 2);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", tree_width, "Transform");
  draw_box_char(top, sep_one_x, WACS_VLINE, '|');
  mvprintw(top, sep_one_x + 1, "%-*s", freshness_width, "Freshness");
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->scroll_;
  const int last_row = std::min(static_cast<int>(backend_->rows_.size()), first_row + visible_rows - 1);
  for (int row = top + 1; row <= bottom; ++row) {
    const bool has_item = first_row + (row - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row - top - 1) == backend_->selected_index_);

    mvhline(row, left, ' ', width);
    if (selected) {
      apply_role_chgat(row, left, width, kColorSelection);
    }
    draw_box_char(row, sep_one_x, WACS_VLINE, '|');

    if (!has_item) {
      continue;
    }

    const auto & entry = backend_->rows_[static_cast<std::size_t>(first_row + (row - top - 1))];
    const bool marked = std::find(
      backend_->selected_frames_.begin(), backend_->selected_frames_.end(), entry.child_frame) !=
      backend_->selected_frames_.end();
    const std::string tree_text =
      std::string(marked ? "* " : "  ") +
      std::string(static_cast<std::size_t>(entry.depth * 2), ' ') +
      entry.child_frame;
    const int text_color = entry.is_root ? 0 : (entry.stale ? kColorStale : (entry.is_static ? 0 : kColorDynamic));

    if (text_color != 0 && !selected) {
      attron(COLOR_PAIR(text_color));
    }
    mvprintw(row, left, "%-*s", tree_width, truncate_text(tree_text, tree_width).c_str());
    mvprintw(row, sep_one_x + 1, "%-*s", freshness_width, truncate_text(entry.freshness, freshness_width).c_str());
    if (text_color != 0 && !selected) {
      attroff(COLOR_PAIR(text_color));
    }

    if (selected) {
      apply_role_chgat(row, left, width, kColorSelection);
      mvaddch(row, sep_one_x, '|');
    }
  }
}

void TfMonitorScreen::draw_status_line(int row, int columns) const {
  std::string line = backend_->status_line_;
  if (!backend_->rows_.empty() &&
    backend_->selected_index_ >= 0 &&
    backend_->selected_index_ < static_cast<int>(backend_->rows_.size()))
  {
    const auto & selected = backend_->rows_[static_cast<std::size_t>(backend_->selected_index_)];
    line = selected.is_root
      ? (selected.child_frame + "  root frame  " + backend_->status_line_)
      : (selected.parent_frame + " -> " + selected.child_frame + "  " + backend_->status_line_);
  }
  if (!backend_->selected_frames_.empty()) {
    line += "  selected=" + std::to_string(backend_->selected_frames_.size());
  }
  draw_status_bar(row, columns, line);
}

void TfMonitorScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(
    row,
    columns,
    tui::with_terminal_help(
      "Space Select  Enter Inspect  Alt+S Search  F4 Refresh  F10 Exit",
      terminal_pane_.visible()));
}

void TfMonitorScreen::draw_inspect_popup(int rows, int columns) const {
  const int popup_width = std::min(columns - 6, 72);
  const int popup_height = 11;
  const int left = std::max(2, (columns - popup_width) / 2);
  const int top = std::max(2, (rows - popup_height) / 2);
  const int right = left + popup_width - 1;
  const int bottom = top + popup_height - 1;
  const int inner_width = popup_width - 2;

  for (int row = top + 1; row < bottom; ++row) {
    attron(COLOR_PAIR(tui::kColorPopup));
    mvhline(row, left + 1, ' ', inner_width);
    attroff(COLOR_PAIR(tui::kColorPopup));
  }
  draw_box(top, left, bottom, right, kColorFrame);

  attron(theme_attr(kColorHeader));
  mvprintw(top, left + 2, " TF Inspect ");
  attroff(theme_attr(kColorHeader));

  if (!backend_->inspect_result_available_) {
    mvaddnstr(
      top + 2, left + 2,
      "Selected frames are not connected.",
      popup_width - 4);
    draw_help_bar_region(bottom - 1, left + 2, popup_width - 4, "Enter Close  Esc Close  F10 Exit");
    return;
  }

  const Vec3 rpy = rpy_from_quat(backend_->inspect_result_.transform.rotation);
  const auto print_line = [&](int row, const std::string & text) {
    mvaddnstr(row, left + 2, truncate_text(text, popup_width - 4).c_str(), popup_width - 4);
  };

  char buffer[128];
  print_line(top + 1, backend_->inspect_result_.from_frame + " -> " + backend_->inspect_result_.to_frame);
  std::snprintf(
    buffer, sizeof(buffer), "XYZ:  %.3f  %.3f  %.3f",
    backend_->inspect_result_.transform.translation.x,
    backend_->inspect_result_.transform.translation.y,
    backend_->inspect_result_.transform.translation.z);
  print_line(top + 3, buffer);
  std::snprintf(
    buffer, sizeof(buffer), "RPY:  %.3f  %.3f  %.3f",
    rpy.x, rpy.y, rpy.z);
  print_line(top + 4, buffer);
  std::snprintf(
    buffer, sizeof(buffer), "Quat: %.4f  %.4f  %.4f  %.4f",
    backend_->inspect_result_.transform.rotation.x,
    backend_->inspect_result_.transform.rotation.y,
    backend_->inspect_result_.transform.rotation.z,
    backend_->inspect_result_.transform.rotation.w);
  print_line(top + 6, buffer);
  draw_help_bar_region(bottom - 1, left + 2, popup_width - 4, "Enter Close  Esc Close  F10 Exit");
}

}  // namespace ros2_console_tools
