#include "ros2_console_tools/node_commander.hpp"

#include "ros2_console_tools/log_viewer.hpp"
#include "ros2_console_tools/tf_monitor.hpp"
#include "ros2_console_tools/urdf_inspector.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <utility>

namespace ros2_console_tools {

int run_parameter_commander_tool(const std::string & target_node, bool embedded_mode);
int run_topic_monitor_tool(const std::string & initial_topic, bool embedded_mode);
int run_service_commander_tool(const std::string & initial_service, bool embedded_mode);

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorAccent = tui::kColorAccent,
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

void resume_parent_screen() {
  reset_prog_mode();
  refresh();
  clear();
  clearok(stdscr, TRUE);
  curs_set(0);
  timeout(100);
}

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
  if (help_popup_open_) {
    switch (key) {
      case KEY_F(10):
        return false;
      case 27:
      case '\n':
      case KEY_ENTER:
      case KEY_F(1):
        help_popup_open_ = false;
        return true;
      default:
        return true;
    }
  }

  if (search_state_.active) {
    return handle_search_key(key);
  }

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(1):
      help_popup_open_ = true;
      return true;
    case KEY_F(2):
      return launch_log_viewer();
    case KEY_F(3):
      return launch_service_commander();
    case KEY_F(4):
      return launch_topic_monitor();
    case KEY_F(5):
      return launch_tf_monitor();
    case KEY_F(6):
      return launch_urdf_inspector();
    case '\t':
      focus_pane_ = focus_pane_ == NodeCommanderFocusPane::NodeList
        ? NodeCommanderFocusPane::DetailPane
        : NodeCommanderFocusPane::NodeList;
      if (focus_pane_ == NodeCommanderFocusPane::DetailPane) {
        for (std::size_t index = 0; index < detail_lines_cache_.size(); ++index) {
          if (!detail_lines_cache_[index].is_header && detail_lines_cache_[index].action != NodeDetailAction::None) {
            detail_selected_index_ = static_cast<int>(index);
            break;
          }
        }
      }
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        backend_->status_line_ = "Search.";
      }
      return true;
    case KEY_UP:
    case 'k':
      if (focus_pane_ == NodeCommanderFocusPane::NodeList) {
        if (backend_->selected_index_ > 0) {
          --backend_->selected_index_;
        }
      } else if (detail_selected_index_ > 0) {
        --detail_selected_index_;
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (focus_pane_ == NodeCommanderFocusPane::NodeList) {
        if (backend_->selected_index_ + 1 < static_cast<int>(backend_->node_entries_.size())) {
          ++backend_->selected_index_;
        }
      } else if (detail_selected_index_ + 1 < static_cast<int>(detail_lines_cache_.size())) {
        ++detail_selected_index_;
      }
      return true;
    case KEY_PPAGE:
      if (focus_pane_ == NodeCommanderFocusPane::NodeList) {
        backend_->selected_index_ = std::max(0, backend_->selected_index_ - page_step());
      } else {
        detail_selected_index_ = std::max(0, detail_selected_index_ - page_step());
      }
      return true;
    case KEY_NPAGE:
      if (focus_pane_ == NodeCommanderFocusPane::NodeList) {
        if (!backend_->node_entries_.empty()) {
          backend_->selected_index_ = std::min(
            static_cast<int>(backend_->node_entries_.size()) - 1,
            backend_->selected_index_ + page_step());
        }
      } else if (!detail_lines_cache_.empty()) {
        detail_selected_index_ = std::min(
          static_cast<int>(detail_lines_cache_.size()) - 1,
          detail_selected_index_ + page_step());
      }
      return true;
    case '\n':
    case KEY_ENTER:
      if (focus_pane_ == NodeCommanderFocusPane::NodeList) {
        return launch_selected_node_parameters();
      }
      return launch_selected_detail_action();
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

bool NodeCommanderScreen::launch_log_viewer() {
  def_prog_mode();
  endwin();
  (void)run_log_viewer_tool();
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from log_viewer.";
  }
  return true;
}

bool NodeCommanderScreen::launch_service_commander() {
  def_prog_mode();
  endwin();
  (void)run_service_commander_tool("", true);
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from service_commander.";
  }
  return true;
}

bool NodeCommanderScreen::launch_topic_monitor() {
  def_prog_mode();
  endwin();
  (void)run_topic_monitor_tool("", true);
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from topic_monitor.";
  }
  return true;
}

bool NodeCommanderScreen::launch_tf_monitor() {
  def_prog_mode();
  endwin();
  (void)run_tf_monitor_tool();
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from tf_monitor.";
  }
  return true;
}

bool NodeCommanderScreen::launch_urdf_inspector() {
  def_prog_mode();
  endwin();
  (void)run_urdf_inspector_tool("");
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from urdf_inspector.";
  }
  return true;
}

bool NodeCommanderScreen::launch_selected_node_parameters() {
  std::string selected_node;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    if (backend_->node_entries_.empty() ||
      backend_->selected_index_ < 0 ||
      backend_->selected_index_ >= static_cast<int>(backend_->node_entries_.size()))
    {
      backend_->status_line_ = "No node selected.";
      return true;
    }
    selected_node = backend_->node_entries_[static_cast<std::size_t>(backend_->selected_index_)];
  }

  def_prog_mode();
  endwin();
  (void)run_parameter_commander_tool(selected_node, true);
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from parameter_commander for " + selected_node + ".";
  }
  return true;
}

bool NodeCommanderScreen::launch_selected_detail_action() {
  const DetailLine * line = selected_detail_line();
  if (line == nullptr || line->is_header || line->action == NodeDetailAction::None) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "No actionable item selected.";
    return true;
  }

  def_prog_mode();
  endwin();
  switch (line->action) {
    case NodeDetailAction::OpenParameters:
      (void)run_parameter_commander_tool(line->target, true);
      break;
    case NodeDetailAction::OpenTopicMonitor:
      (void)run_topic_monitor_tool(line->target, true);
      break;
    case NodeDetailAction::OpenServiceCommander:
      (void)run_service_commander_tool(line->target, true);
      break;
    case NodeDetailAction::None:
      break;
  }
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned to node_commander.";
  }
  return true;
}

const DetailLine * NodeCommanderScreen::selected_detail_line() const {
  if (detail_lines_cache_.empty() ||
    detail_selected_index_ < 0 ||
    detail_selected_index_ >= static_cast<int>(detail_lines_cache_.size()))
  {
    return nullptr;
  }
  return &detail_lines_cache_[static_cast<std::size_t>(detail_selected_index_)];
}

void NodeCommanderScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = rows - 3;

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
  if (help_popup_open_) {
    const int popup_width = std::min(columns - 8, 76);
    const int popup_height = 11;
    const int popup_left = std::max(2, (columns - popup_width) / 2);
    const int popup_top = std::max(1, (rows - popup_height) / 2);
    const int popup_right = popup_left + popup_width - 1;
    const int popup_bottom = popup_top + popup_height - 1;

    draw_box(popup_top, popup_left, popup_bottom, popup_right, kColorFrame);
    attron(COLOR_PAIR(kColorHeader));
    mvprintw(popup_top, popup_left + 2, "Node Commander Help ");
    attroff(COLOR_PAIR(kColorHeader));

    const int text_left = popup_left + 2;
    const int text_width = popup_width - 4;
    mvprintw(popup_top + 2, text_left, "%-*s", text_width, "Enter: open selected node or selected detail item");
    mvprintw(popup_top + 3, text_left, "%-*s", text_width, "Tab: switch between node list and detail pane");
    mvprintw(popup_top + 4, text_left, "%-*s", text_width, "Alt+S: search nodes");
    mvprintw(popup_top + 5, text_left, "%-*s", text_width, "F2: log_viewer    F3: service_commander");
    mvprintw(popup_top + 6, text_left, "%-*s", text_width, "F4: topic_monitor F5: tf_monitor");
    mvprintw(popup_top + 7, text_left, "%-*s", text_width, "F6: urdf_inspector");
    mvprintw(popup_top + 8, text_left, "%-*s", text_width, "Esc/Enter/F1: close help");
  }
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
  mvprintw(top, left, "%-*s", width, focus_pane_ == NodeCommanderFocusPane::NodeList ? "Nodes <" : "Nodes");
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

void NodeCommanderScreen::draw_detail_pane(int top, int left, int bottom, int right) {
  const int width = right - left + 1;
  detail_lines_cache_ = backend_->selected_node_details();

  if (detail_lines_cache_.empty()) {
    detail_selected_index_ = 0;
    detail_scroll_ = 0;
  } else {
    detail_selected_index_ = std::clamp(detail_selected_index_, 0, static_cast<int>(detail_lines_cache_.size()) - 1);
  }

  const int visible_rows = std::max(1, bottom - top);
  if (detail_selected_index_ < detail_scroll_) {
    detail_scroll_ = detail_selected_index_;
  }
  if (detail_selected_index_ >= detail_scroll_ + visible_rows) {
    detail_scroll_ = std::max(0, detail_selected_index_ - visible_rows + 1);
  }

  attron(COLOR_PAIR(kColorHeader));
  mvprintw(top, left, "%-*s", width, focus_pane_ == NodeCommanderFocusPane::DetailPane ? "Details <" : "Details");
  attroff(COLOR_PAIR(kColorHeader));

  int row_y = top + 1;
  const int first_row = detail_scroll_;
  const int last_row = std::min(static_cast<int>(detail_lines_cache_.size()), first_row + visible_rows);
  for (int index = first_row; index < last_row && row_y <= bottom; ++index, ++row_y) {
    const auto & line = detail_lines_cache_[static_cast<std::size_t>(index)];
    const bool selected = focus_pane_ == NodeCommanderFocusPane::DetailPane && index == detail_selected_index_;
    mvhline(row_y, left, ' ', width);
    if (selected) {
      mvchgat(row_y, left, width, A_NORMAL, kColorSelection, nullptr);
    }
    if (line.is_header) {
      attron(COLOR_PAIR(kColorHeader));
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
      attroff(COLOR_PAIR(kColorHeader));
    } else if (line.action != NodeDetailAction::None) {
      attron(COLOR_PAIR(kColorAccent));
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
      attroff(COLOR_PAIR(kColorAccent));
    } else {
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
    }
    if (selected) {
      mvchgat(row_y, left, width, A_NORMAL, kColorSelection, nullptr);
    }
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
  draw_help_bar(
    row, columns,
    "F1 Help  F2 Logs  F3 Services  F4 Topics  F5 Transforms  F6 URDF  Enter Open  Tab Pane  Alt+S Search  F10 Exit");
}

}  // namespace ros2_console_tools
