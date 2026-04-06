#include "ros2_console_tools/node_commander.hpp"

#include "ros2_console_tools/action_commander.hpp"
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
using tui::apply_role_chgat;
using tui::SearchInputResult;
using tui::start_search;
using tui::theme_attr;
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
      return launch_topic_monitor();
    case KEY_F(4):
      return launch_selected_node_parameters();
    case KEY_F(5):
      return launch_service_commander();
    case KEY_F(6):
      return launch_action_commander();
    case KEY_F(7):
      return launch_tf_monitor();
    case KEY_F(8):
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
        backend_->status_line_ = focus_pane_ == NodeCommanderFocusPane::NodeList
          ? "Search nodes."
          : "Search details.";
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

  if (focus_pane_ == NodeCommanderFocusPane::NodeList) {
    const int match = find_best_match(backend_->node_entries_, search_state_.query, backend_->selected_index_);
    if (match >= 0) {
      backend_->selected_index_ = match;
    }
  } else {
    std::vector<std::string> labels;
    labels.reserve(detail_lines_cache_.size());
    for (const auto & line : detail_lines_cache_) {
      labels.push_back(line.text);
    }
    const int match = find_best_match(labels, search_state_.query, detail_selected_index_);
    if (match >= 0) {
      detail_selected_index_ = match;
    }
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

bool NodeCommanderScreen::launch_action_commander() {
  def_prog_mode();
  endwin();
  (void)run_action_commander_tool("", true);
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from action_commander.";
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
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "Node Commander ");
  attroff(theme_attr(kColorTitle));

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
    const int popup_height = 14;
    const int popup_left = std::max(2, (columns - popup_width) / 2);
    const int popup_top = std::max(1, (rows - popup_height) / 2);
    const int popup_right = popup_left + popup_width - 1;
    const int popup_bottom = popup_top + popup_height - 1;
    const int inner_width = popup_width - 2;

    for (int row = popup_top + 1; row < popup_bottom; ++row) {
      attron(COLOR_PAIR(tui::kColorPopup));
      mvhline(row, popup_left + 1, ' ', inner_width);
      attroff(COLOR_PAIR(tui::kColorPopup));
    }
    draw_box(popup_top, popup_left, popup_bottom, popup_right, kColorFrame);
    attron(COLOR_PAIR(kColorHeader));
    mvprintw(popup_top, popup_left + 2, "Node Commander Help ");
    attroff(COLOR_PAIR(kColorHeader));

    const int text_left = popup_left + 2;
    const int text_width = popup_width - 4;
    constexpr int key_width = 10;
    auto draw_help_item = [&](int row, const std::string & key, const std::string & description) {
      mvhline(row, text_left, ' ', text_width);
      attron(COLOR_PAIR(tui::kColorPopup) | A_BOLD);
      mvprintw(row, text_left, "%-*s", key_width, key.c_str());
      attroff(COLOR_PAIR(tui::kColorPopup) | A_BOLD);
      attron(COLOR_PAIR(tui::kColorPopup));
      mvaddnstr(row, text_left + key_width, description.c_str(), std::max(0, text_width - key_width));
      attroff(COLOR_PAIR(tui::kColorPopup));
    };
    draw_help_item(popup_top + 2, "Enter", "open selected node or detail item");
    draw_help_item(popup_top + 3, "Tab", "switch node list and detail pane");
    draw_help_item(popup_top + 4, "Alt+S", "search nodes");
    draw_help_item(popup_top + 5, "F2", "log_viewer");
    draw_help_item(popup_top + 6, "F3", "topic_monitor");
    draw_help_item(popup_top + 7, "F4", "parameter_commander");
    draw_help_item(popup_top + 8, "F5", "service_commander");
    draw_help_item(popup_top + 9, "F6", "action_commander");
    draw_help_item(popup_top + 10, "F7", "tf_monitor");
    draw_help_item(popup_top + 11, "F8", "urdf_inspector");
    draw_help_item(popup_top + 12, "Esc/F1", "close help");
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

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, focus_pane_ == NodeCommanderFocusPane::NodeList ? "Nodes <" : "Nodes");
  attroff(theme_attr(kColorHeader));

  const int first_row = backend_->node_scroll_;
  const int last_row = std::min(static_cast<int>(backend_->node_entries_.size()), first_row + visible_rows - 1);
  for (int row_y = top + 1; row_y <= bottom; ++row_y) {
    const bool has_item = first_row + (row_y - top - 1) < last_row;
    const bool selected = has_item && (first_row + (row_y - top - 1) == backend_->selected_index_);
    mvhline(row_y, left, ' ', width);
    if (selected) {
      apply_role_chgat(row_y, left, width, kColorSelection);
    }
    if (!has_item) {
      continue;
    }

    const auto & entry = backend_->node_entries_[static_cast<std::size_t>(first_row + (row_y - top - 1))];
    mvprintw(row_y, left, "%-*s", width, truncate_text(entry, width).c_str());
    if (selected) {
      apply_role_chgat(row_y, left, width, kColorSelection);
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

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, focus_pane_ == NodeCommanderFocusPane::DetailPane ? "Details <" : "Details");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  const int first_row = detail_scroll_;
  const int last_row = std::min(static_cast<int>(detail_lines_cache_.size()), first_row + visible_rows);
  for (int index = first_row; index < last_row && row_y <= bottom; ++index, ++row_y) {
    const auto & line = detail_lines_cache_[static_cast<std::size_t>(index)];
    const bool selected = focus_pane_ == NodeCommanderFocusPane::DetailPane && index == detail_selected_index_;
    mvhline(row_y, left, ' ', width);
    if (selected) {
      apply_role_chgat(row_y, left, width, kColorSelection);
    }
    if (line.is_header) {
      attron(theme_attr(kColorHeader));
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
      attroff(theme_attr(kColorHeader));
    } else if (line.action != NodeDetailAction::None) {
      attron(COLOR_PAIR(kColorAccent));
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
      attroff(COLOR_PAIR(kColorAccent));
    } else {
      mvprintw(row_y, left, "%-*s", width, truncate_text(line.text, width).c_str());
    }
    if (selected) {
      apply_role_chgat(row_y, left, width, kColorSelection);
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
    "F1 Help  F2 Logs  F3 Topics  F4 Parameters  F5 Services  F6 Actions  F7 Transforms  F8 URDF  Enter Open  Tab Pane  Alt+S Search  F10 Exit");
}

}  // namespace ros2_console_tools
