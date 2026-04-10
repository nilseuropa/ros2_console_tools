#include "ros2_console_tools/topic_monitor.hpp"

#include "ros2_console_tools/image_viewer.hpp"
#include "ros2_console_tools/joy_viewer.hpp"
#include "ros2_console_tools/map_viewer.hpp"

#include <ncursesw/ncurses.h>
#include <thread>

namespace ros2_console_tools {

int run_topic_monitor_tool(const TopicMonitorLaunchOptions & options) {
  auto backend = std::make_shared<TopicMonitorBackend>(options);
  TopicMonitorScreen screen(backend, options.embedded_mode);

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

int run_topic_monitor_tool(const std::string & initial_topic, bool embedded_mode) {
  TopicMonitorLaunchOptions options;
  options.initial_topic = initial_topic;
  options.embedded_mode = embedded_mode;
  return run_topic_monitor_tool(options);
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
using tui::draw_help_bar_region;
using tui::draw_search_box;
using tui::draw_status_bar;
using tui::draw_text_vline;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::apply_role_chgat;
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

bool TopicMonitorScreen::handle_key(int key) {
  if (plot_popup_open_) {
    switch (key) {
      case KEY_F(10):
        return false;
      case 27:
      case '\n':
      case KEY_ENTER:
      case KEY_F(3):
        plot_popup_open_ = false;
        return true;
      default:
        return true;
    }
  }
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
    case KEY_F(3):
      backend_->status_line_ = "Open a topic detail view to plot numeric fields.";
      return true;
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
  const auto rows = backend_->visible_detail_rows_snapshot(backend_->detail_topic_name_);
  backend_->clamp_detail_selection(rows);

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(2):
      return launch_selected_visualizer();
    case KEY_F(3):
      return launch_selected_plot();
    case 27:
      if (embedded_mode_ && backend_->exit_on_detail_escape_) {
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
    case KEY_RIGHT:
    case 'l':
      backend_->expand_selected_detail_row();
      return true;
    case KEY_LEFT:
    case 'h':
      backend_->collapse_selected_detail_row();
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
    flushinp();
    (void)run_map_viewer_tool(topic_name, true);
    backend_->status_line_ = "Returned from map_viewer for " + topic_name + ".";
    return true;
  }

  if (topic_type == "sensor_msgs/msg/Image") {
    flushinp();
    (void)run_image_viewer_tool(topic_name, true);
    backend_->status_line_ = "Returned from image_viewer for " + topic_name + ".";
    return true;
  }

  if (topic_type == "sensor_msgs/msg/Joy") {
    flushinp();
    (void)run_joy_viewer_tool(topic_name, true);
    backend_->status_line_ = "Returned from joy_viewer for " + topic_name + ".";
    return true;
  }

  backend_->status_line_ = "No visualizer available for " + topic_type + ".";
  return true;
}

bool TopicMonitorScreen::launch_selected_plot() {
  if (backend_->view_mode_ != TopicMonitorViewMode::TopicDetail) {
    backend_->status_line_ = "Plotting is available from topic detail view.";
    return true;
  }

  const auto selected = backend_->selected_visible_detail_row();
  if (!selected.has_value()) {
    backend_->status_line_ = "No field selected.";
    return true;
  }
  if (!selected->numeric) {
    backend_->status_line_ = "Selected field is not plottable.";
    return true;
  }

  plot_topic_name_ = backend_->detail_topic_name_;
  plot_field_name_ = selected->field;
  plot_field_path_ = selected->path;
  plot_popup_open_ = true;
  backend_->status_line_ = "Plotting " + plot_field_path_ + ".";
  return true;
}

void TopicMonitorScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const auto layout = tui::make_commander_layout(rows, terminal_pane_.visible());
  const int help_row = layout.help_row;
  const int status_row = layout.status_row;
  const int content_bottom = layout.content_bottom;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  mvprintw(0, 1, "Topic Monitor ");
  if (backend_->view_mode_ == TopicMonitorViewMode::TopicList) {
    draw_topic_list(1, 1, content_bottom - 1, columns - 2);
  } else {
    draw_topic_detail(1, 1, content_bottom - 1, columns - 2);
  }
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  draw_search_box(layout.pane_rows, columns, search_state_);
  if (plot_popup_open_) {
    draw_plot_popup(rows, columns);
  }
  if (terminal_pane_.visible()) {
    terminal_pane_.draw(layout.terminal_top, 0, rows - 1, columns - 1);
  }
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
      apply_role_chgat(row, left, width, kColorSelection);
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
        apply_role_chgat(row, left, width, kColorSelection);
        draw_box_char(row, sep_one_x, WACS_VLINE, '|');
        draw_box_char(row, sep_two_x, WACS_VLINE, '|');
        draw_box_char(row, sep_three_x, WACS_VLINE, '|');
        apply_role_chgat(row, left, topic_width, kColorSelection);
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
      apply_role_chgat(row, left, width, kColorSelection);
      mvaddch(row, sep_one_x, '|');
      mvaddch(row, sep_two_x, '|');
      mvaddch(row, sep_three_x, '|');
      if (text_color != 0) {
        apply_role_chgat(row, left, topic_width, text_color);
        apply_role_chgat(row, sep_one_x + 1, avg_width, text_color);
        apply_role_chgat(row, sep_two_x + 1, minmax_width, text_color);
        apply_role_chgat(row, sep_three_x + 1, bandwidth_width, text_color);
      }
    }
  }
}

void TopicMonitorScreen::draw_topic_detail(int top, int left, int bottom, int right) {
  const auto all_rows = backend_->detail_rows_snapshot(backend_->detail_topic_name_);
  const auto rows = backend_->visible_detail_rows_snapshot(backend_->detail_topic_name_);
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
      apply_role_chgat(row, left, width, kColorSelection);
    }
    draw_box_char(row, separator_x, WACS_VLINE, '|');

    if (!has_item) {
      continue;
    }

    const auto & entry = rows[static_cast<std::size_t>(first_row + (row - top - 1))];
    bool expandable = false;
    for (std::size_t index = 0; index < all_rows.size(); ++index) {
      if (all_rows[index].path == entry.path) {
        expandable = backend_->detail_row_has_children(all_rows, index);
        break;
      }
    }
    const bool collapsed = expandable &&
      backend_->collapsed_detail_paths_.find(entry.path) != backend_->collapsed_detail_paths_.end() &&
      backend_->collapsed_detail_paths_.at(entry.path);
    const std::string indent(static_cast<std::size_t>(entry.depth * 2), ' ');
    const std::string marker = expandable ? (collapsed ? "+ " : "- ") : "  ";
    const std::string field_text = indent + marker + entry.field;
    mvprintw(row, left, "%-*s", field_width, truncate_text(field_text, field_width).c_str());
    mvprintw(row, separator_x + 1, "%-*s", value_width, truncate_text(entry.value, value_width).c_str());

    if (selected) {
      apply_role_chgat(row, left, width, kColorSelection);
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
    ? (embedded_mode_ && backend_->exit_on_detail_escape_
      ? "F2 Visualize  F3 Plot  Left/Right Fold  Esc Return  F4 Refresh  F10 Exit"
      : "F2 Visualize  F3 Plot  Left/Right Fold  Esc Back  F4 Refresh  F10 Exit")
    : "Enter Inspect  F2 Visualize  Alt+S Search  Space Mark  F4 Refresh  F5 Filter  F10 Exit";
  draw_help_bar(row, columns, tui::with_terminal_help(help, terminal_pane_.visible()));
}

void TopicMonitorScreen::draw_plot_popup(int rows, int columns) const {
  const auto samples = backend_->plot_samples_snapshot(plot_topic_name_, plot_field_path_);
  const int popup_width = std::min(columns - 6, 96);
  const int popup_height = std::min(rows - 4, 22);
  const int left = std::max(2, (columns - popup_width) / 2);
  const int top = std::max(1, (rows - popup_height) / 2);
  const int right = left + popup_width - 1;
  const int bottom = top + popup_height - 1;
  const int inner_width = popup_width - 2;
  const int plot_left = left + 2;
  const int plot_top = top + 3;
  const int plot_right = right - 2;
  const int plot_bottom = bottom - 3;
  const int plot_width = std::max(8, plot_right - plot_left + 1);
  const int plot_height = std::max(4, plot_bottom - plot_top + 1);

  for (int row = top + 1; row < bottom; ++row) {
    attron(COLOR_PAIR(tui::kColorPopup));
    mvhline(row, left + 1, ' ', inner_width);
    attroff(COLOR_PAIR(tui::kColorPopup));
  }
  draw_box(top, left, bottom, right, kColorFrame);
  attron(theme_attr(kColorHeader));
  mvprintw(top, left + 2, " Plot: %s ", truncate_text(plot_field_path_, popup_width - 12).c_str());
  attroff(theme_attr(kColorHeader));

  if (samples.empty()) {
    mvprintw(top + 2, left + 2, "%s", "Waiting for numeric samples...");
    draw_help_bar_region(bottom - 1, left + 2, popup_width - 4, "F3 Close  Enter Close  Esc Close  F10 Exit");
    return;
  }

  double min_value = samples.front().value;
  double max_value = samples.front().value;
  for (const auto & sample : samples) {
    min_value = std::min(min_value, sample.value);
    max_value = std::max(max_value, sample.value);
  }
  mvprintw(top + 1, left + 2, "%s", truncate_text(plot_topic_name_ + " :: " + plot_field_name_, popup_width - 4).c_str());
  mvprintw(top + 2, left + 2, "min=%g  max=%g  samples=%zu", min_value, max_value, samples.size());

  for (int row = plot_top; row <= plot_bottom; ++row) {
    mvhline(row, plot_left, ' ', plot_width);
  }

  const bool ascii_only = tui::terminal_context() == tui::TerminalContext::Ascii;
  const bool crosses_zero = min_value < 0.0 && max_value > 0.0;
  const auto first_time = samples.front().time;
  const auto last_time = samples.back().time;
  const double span = std::max(1e-9, std::chrono::duration<double>(last_time - first_time).count());
  const double display_min = crosses_zero ? min_value : min_value;
  const double display_max = crosses_zero ? max_value : max_value;
  const double display_range = std::max(1e-9, display_max - display_min);

  for (const auto & sample : samples) {
    const double t = samples.size() == 1
      ? 1.0
      : std::chrono::duration<double>(sample.time - first_time).count() / span;
    const int x = std::clamp(static_cast<int>(t * static_cast<double>(plot_width - 1)), 0, plot_width - 1);
    const double normalized = (sample.value - display_min) / display_range;
    const int y = plot_bottom - std::clamp(
      static_cast<int>(normalized * static_cast<double>(plot_height - 1) + 0.5), 0, plot_height - 1);
    mvaddch(y, plot_left + x, ascii_only ? '.' : ACS_BULLET);
  }

  draw_help_bar_region(bottom - 1, left + 2, popup_width - 4, "F3 Close  Enter Close  Esc Close  F10 Exit");
}

}  // namespace ros2_console_tools
