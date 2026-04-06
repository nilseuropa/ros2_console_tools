#include "ros2_console_tools/topic_monitor.hpp"

#include "ros2_console_tools/map_viewer.hpp"

#include <ncursesw/ncurses.h>
#include <thread>

namespace ros2_console_tools {

int run_topic_monitor_tool(const std::string & initial_topic, bool embedded_mode) {
  auto backend = std::make_shared<TopicMonitorBackend>(initial_topic);
  TopicMonitorScreen screen(backend, embedded_mode);

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
  kColorMonitored = tui::kColorPositive,
  kColorMonitoredSelection = tui::kColorPositiveSelection,
  kColorStale = tui::kColorError,
  kColorStaleSelection = tui::kColorFatal,
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
using tui::theme_attr;
using tui::truncate_text;

}  // namespace

TopicMonitorScreen::TopicMonitorScreen(
  std::shared_ptr<TopicMonitorBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int TopicMonitorScreen::run() {
  Session ncurses_session;
  backend_->refresh_topics();
  backend_->warm_up_topic_list();

  bool running = true;
  while (running && rclcpp::ok()) {
    backend_->maybe_refresh_topics();
    draw();
    const int key = getch();
    if (key == ERR) {
      continue;
    }
    running = handle_key(key);
  }

  return 0;
}

bool TopicMonitorScreen::handle_key(int key) {
  if (search_state_.active) {
    return handle_search_key(key);
  }
  switch (backend_->view_mode_) {
    case TopicMonitorViewMode::TopicList:
      return handle_topic_list_key(key);
    case TopicMonitorViewMode::TopicDetail:
      return handle_topic_detail_key(key);
  }
  return true;
}

bool TopicMonitorScreen::handle_topic_list_key(int key) {
  const auto items = backend_->visible_topic_items();
  backend_->clamp_topic_selection(items);

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(2):
      return launch_selected_visualizer();
    case KEY_F(4):
      backend_->refresh_topics();
      return true;
    case KEY_F(5):
      backend_->show_only_monitored_ = !backend_->show_only_monitored_;
      backend_->selected_index_ = 0;
      backend_->list_scroll_ = 0;
      backend_->status_line_ = backend_->show_only_monitored_
        ? "Showing monitored topics only."
        : "Showing all topics.";
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
      if (backend_->selected_index_ + 1 < static_cast<int>(items.size())) {
        ++backend_->selected_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_index_ = std::max(0, backend_->selected_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!items.empty()) {
        backend_->selected_index_ =
          std::min(static_cast<int>(items.size()) - 1, backend_->selected_index_ + page_step());
      }
      return true;
    case KEY_RIGHT:
    case 'l':
      backend_->expand_selected_namespace();
      return true;
    case KEY_LEFT:
    case 'h':
      backend_->collapse_selected_namespace();
      return true;
    case ' ':
    case KEY_IC:
      backend_->toggle_selected_topic_monitoring();
      return true;
    case '\n':
    case KEY_ENTER:
      backend_->open_selected_topic_detail();
      return true;
    default:
      return true;
  }
}

bool TopicMonitorScreen::handle_search_key(int key) {
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
  if (result != SearchInputResult::Changed || backend_->view_mode_ != TopicMonitorViewMode::TopicList) {
    return true;
  }

  const auto items = backend_->visible_topic_items();
  std::vector<std::string> labels;
  labels.reserve(items.size());
  for (const auto & item : items) {
    labels.push_back(item.label);
  }
  const int match = find_best_match(labels, search_state_.query, backend_->selected_index_);
  if (match >= 0) {
    backend_->selected_index_ = match;
  }
  backend_->status_line_ = "Search: " + search_state_.query;
  return true;
}

bool TopicMonitorScreen::handle_topic_detail_key(int key) {
  const auto rows = backend_->detail_rows_snapshot(backend_->detail_topic_name_);
  backend_->clamp_detail_selection(rows);

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(2):
      return launch_selected_visualizer();
    case 27:
      if (embedded_mode_) {
        return false;
      }
      backend_->close_topic_detail();
      return true;
    case KEY_F(4):
      backend_->refresh_topics();
      return true;
    case KEY_UP:
    case 'k':
      if (backend_->selected_detail_index_ > 0) {
        --backend_->selected_detail_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (backend_->selected_detail_index_ + 1 < static_cast<int>(rows.size())) {
        ++backend_->selected_detail_index_;
      }
      return true;
    case KEY_PPAGE:
      backend_->selected_detail_index_ = std::max(0, backend_->selected_detail_index_ - page_step());
      return true;
    case KEY_NPAGE:
      if (!rows.empty()) {
        backend_->selected_detail_index_ =
          std::min(static_cast<int>(rows.size()) - 1, backend_->selected_detail_index_ + page_step());
      }
      return true;
    default:
      return true;
  }
}

int TopicMonitorScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

bool TopicMonitorScreen::launch_selected_visualizer() {
  std::string topic_name;
  std::string topic_type;

  if (backend_->view_mode_ == TopicMonitorViewMode::TopicDetail) {
    topic_name = backend_->detail_topic_name_;
    auto found = backend_->topics_.find(topic_name);
    if (found != backend_->topics_.end()) {
      topic_type = found->second.type;
    }
  } else {
    const auto items = backend_->visible_topic_items();
    backend_->clamp_topic_selection(items);
    if (items.empty() || backend_->selected_index_ < 0 || backend_->selected_index_ >= static_cast<int>(items.size())) {
      backend_->status_line_ = "No topic selected.";
      return true;
    }
    const auto & item = items[static_cast<std::size_t>(backend_->selected_index_)];
    if (item.is_namespace) {
      backend_->status_line_ = "Select a topic, not a folder.";
      return true;
    }
    topic_name = item.row.name;
    topic_type = item.row.type;
  }

  if (topic_name.empty()) {
    backend_->status_line_ = "No topic selected.";
    return true;
  }

  if (topic_type == "nav_msgs/msg/OccupancyGrid") {
    def_prog_mode();
    endwin();
    (void)run_map_viewer_tool(topic_name);
    reset_prog_mode();
    refresh();
    clear();
    clearok(stdscr, TRUE);
    curs_set(0);
    timeout(100);
    backend_->status_line_ = "Returned from map_viewer for " + topic_name + ".";
    return true;
  }

  backend_->status_line_ = "No visualizer available for " + topic_type + ".";
  return true;
}

void TopicMonitorScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = std::max(1, status_row - 1);

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  mvprintw(0, 1, "Topic Monitor ");
  if (backend_->view_mode_ == TopicMonitorViewMode::TopicList) {
    draw_topic_list(1, 1, content_bottom - 1, columns - 2);
  } else {
    draw_topic_detail(1, 1, content_bottom - 1, columns - 2);
  }
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(rows, columns, search_state_);
  refresh();
}

void TopicMonitorScreen::draw_topic_list(int top, int left, int bottom, int right) {
  const auto items = backend_->visible_topic_items();
  backend_->clamp_topic_selection(items);

  const int visible_rows = std::max(1, bottom - top + 1);
  const int width = right - left + 1;
  const int topic_width = std::max(24, width / 2);
  const int avg_width = std::max(8, width / 10);
  const int minmax_width = std::max(12, width / 8);
  const int bandwidth_width = std::max(12, width - topic_width - avg_width - minmax_width - 3);
  const int sep_one_x = left + topic_width;
  const int sep_two_x = sep_one_x + 1 + avg_width;
  const int sep_three_x = sep_two_x + 1 + minmax_width;

  if (backend_->selected_index_ < backend_->list_scroll_) {
    backend_->list_scroll_ = backend_->selected_index_;
  }
  if (backend_->selected_index_ >= backend_->list_scroll_ + visible_rows - 1) {
    backend_->list_scroll_ = std::max(0, backend_->selected_index_ - visible_rows + 2);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", topic_width, "Topic");
  draw_box_char(top, sep_one_x, WACS_VLINE, '|');
  mvprintw(top, sep_one_x + 1, "%-*s", avg_width, "Avg Hz");
  draw_box_char(top, sep_two_x, WACS_VLINE, '|');
  mvprintw(top, sep_two_x + 1, "%-*s", minmax_width, "Min/Max Hz");
  draw_box_char(top, sep_three_x, WACS_VLINE, '|');
  mvprintw(top, sep_three_x + 1, "%-*s", bandwidth_width, "Bandwidth");
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->list_scroll_;
  const int last_row = std::min(static_cast<int>(items.size()), first_row + visible_rows - 1);
  for (int row = top + 1; row <= bottom; ++row) {
    const bool has_item = first_row + (row - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row - top - 1) == backend_->selected_index_);

    mvhline(row, left, ' ', width);
    if (selected) {
      mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
    }
    draw_box_char(row, sep_one_x, WACS_VLINE, '|');
    draw_box_char(row, sep_two_x, WACS_VLINE, '|');
    draw_box_char(row, sep_three_x, WACS_VLINE, '|');

    if (!has_item) {
      continue;
    }

    const auto & entry = items[static_cast<std::size_t>(first_row + (row - top - 1))];
    const std::string topic_text = std::string(static_cast<std::size_t>(entry.depth * 2), ' ') + entry.label;
    if (entry.is_namespace) {
      const int text_color = selected ? kColorSelection : kColorFrame;
      if (text_color != 0) {
        attron(COLOR_PAIR(text_color));
      }
      mvprintw(row, left, "%-*s", topic_width, truncate_text(topic_text, topic_width).c_str());
      if (text_color != 0) {
        attroff(COLOR_PAIR(text_color));
      }
      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
        draw_box_char(row, sep_one_x, WACS_VLINE, '|');
        draw_box_char(row, sep_two_x, WACS_VLINE, '|');
        draw_box_char(row, sep_three_x, WACS_VLINE, '|');
        mvchgat(row, left, topic_width, A_NORMAL, kColorSelection, nullptr);
      }
      continue;
    }

    const auto & row_data = entry.row;
    const int text_color =
      row_data.stale
      ? (selected ? kColorStaleSelection : kColorStale)
      : row_data.monitored
      ? (selected ? kColorMonitoredSelection : kColorMonitored)
      : (selected ? kColorSelection : 0);
    if (text_color != 0) {
      attron(COLOR_PAIR(text_color));
    }
    mvprintw(row, left, "%-*s", topic_width, truncate_text(topic_text, topic_width).c_str());
    mvprintw(row, sep_one_x + 1, "%-*s", avg_width, truncate_text(row_data.avg_hz, avg_width).c_str());
    mvprintw(
      row, sep_two_x + 1, "%-*s", minmax_width,
      truncate_text(row_data.min_max_hz, minmax_width).c_str());
    mvprintw(
      row, sep_three_x + 1, "%-*s", bandwidth_width,
      truncate_text(row_data.bandwidth, bandwidth_width).c_str());
    if (text_color != 0) {
      attroff(COLOR_PAIR(text_color));
    }
    if (selected) {
      mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      mvaddch(row, sep_one_x, '|');
      mvaddch(row, sep_two_x, '|');
      mvaddch(row, sep_three_x, '|');
      if (text_color != 0) {
        mvchgat(row, left, topic_width, A_NORMAL, text_color, nullptr);
        mvchgat(row, sep_one_x + 1, avg_width, A_NORMAL, text_color, nullptr);
        mvchgat(row, sep_two_x + 1, minmax_width, A_NORMAL, text_color, nullptr);
        mvchgat(row, sep_three_x + 1, bandwidth_width, A_NORMAL, text_color, nullptr);
      }
    }
  }
}

void TopicMonitorScreen::draw_topic_detail(int top, int left, int bottom, int right) {
  const auto rows = backend_->detail_rows_snapshot(backend_->detail_topic_name_);
  const auto detail_error = backend_->detail_error_snapshot(backend_->detail_topic_name_);
  backend_->clamp_detail_selection(rows);

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top + 1);
  const int field_width = std::max(24, width / 2);
  const int value_width = std::max(16, width - field_width - 1);
  const int separator_x = left + field_width;

  if (backend_->selected_detail_index_ < backend_->detail_scroll_) {
    backend_->detail_scroll_ = backend_->selected_detail_index_;
  }
  if (backend_->selected_detail_index_ >= backend_->detail_scroll_ + visible_rows - 1) {
    backend_->detail_scroll_ = std::max(0, backend_->selected_detail_index_ - visible_rows + 2);
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", field_width, "Field");
  draw_box_char(top, separator_x, WACS_VLINE, '|');
  mvprintw(top, separator_x + 1, "%-*s", value_width, "Value");
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->detail_scroll_;
  const int last_row = std::min(static_cast<int>(rows.size()), first_row + visible_rows - 1);
  for (int row = top + 1; row <= bottom; ++row) {
    const bool has_item = first_row + (row - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row - top - 1) == backend_->selected_detail_index_);

    mvhline(row, left, ' ', width);
    if (selected) {
      mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
    }
    draw_box_char(row, separator_x, WACS_VLINE, '|');

    if (!has_item) {
      continue;
    }

    const auto & entry = rows[static_cast<std::size_t>(first_row + (row - top - 1))];
    const std::string indent(static_cast<std::size_t>(entry.depth * 2), ' ');
    const std::string field_text = indent + entry.field;
    mvprintw(row, left, "%-*s", field_width, truncate_text(field_text, field_width).c_str());
    mvprintw(row, separator_x + 1, "%-*s", value_width, truncate_text(entry.value, value_width).c_str());

    if (selected) {
      mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      draw_box_char(row, separator_x, WACS_VLINE, '|');
    }
  }

  if (rows.empty()) {
    const std::string placeholder = detail_error.empty()
      ? "Waiting for the first message..."
      : "Decode error: " + detail_error;
    mvprintw(top + 1, left + 1, "%s", truncate_text(placeholder, width - 2).c_str());
  }
}

void TopicMonitorScreen::draw_status_line(int row, int columns) const {
  std::string line = backend_->status_line_;

  if (backend_->view_mode_ == TopicMonitorViewMode::TopicDetail && !backend_->detail_topic_name_.empty()) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    const auto found = backend_->topics_.find(backend_->detail_topic_name_);
    if (found != backend_->topics_.end()) {
      line = truncate_text(
        backend_->detail_topic_name_ + " [" + found->second.type + "]  " + backend_->status_line_,
        columns - 1);
    }
  } else {
    const auto items = backend_->visible_topic_items();
    if (!items.empty() && backend_->selected_index_ >= 0 && backend_->selected_index_ < static_cast<int>(items.size())) {
      const auto & selected = items[static_cast<std::size_t>(backend_->selected_index_)];
      if (selected.is_namespace) {
        line = truncate_text(selected.namespace_path + "  " + backend_->status_line_, columns - 1);
      } else {
        line = truncate_text(
          selected.row.name + " [" + selected.row.type + "]  " + backend_->status_line_,
          columns - 1);
      }
    }
  }

  draw_status_bar(row, columns, line);
}

void TopicMonitorScreen::draw_help_line(int row, int columns) const {
  const std::string help =
    backend_->view_mode_ == TopicMonitorViewMode::TopicDetail
    ? "F2 Visualize  Esc Back  F4 Refresh  F10 Exit"
    : "Enter Inspect  F2 Visualize  Alt+S Search  Space Mark  F4 Refresh  F5 Filter  F10 Exit";
  draw_help_bar(row, columns, help);
}

}  // namespace ros2_console_tools
