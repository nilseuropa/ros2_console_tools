#include "ros2_console_tools/node_commander.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>

#include "ros2_console_tools/action_commander.hpp"
#include "ros2_console_tools/diagnostics_viewer.hpp"
#include "ros2_console_tools/log_viewer.hpp"
#include "ros2_console_tools/topic_monitor.hpp"
#include "ros2_console_tools/tf_monitor.hpp"
#include "ros2_console_tools/urdf_inspector.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

#ifndef ROS2_CONSOLE_TOOLS_PACKAGE_VERSION
#define ROS2_CONSOLE_TOOLS_PACKAGE_VERSION "unknown"
#endif

namespace ros2_console_tools {

int run_parameter_commander_tool(const std::string & target_node, bool embedded_mode);

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

bool run_service_commander_subprocess(
  const std::string & initial_node,
  const std::string & initial_service,
  std::string * error)
{
  std::string executable_path;
  try {
    executable_path =
      ament_index_cpp::get_package_prefix("ros2_console_tools") + "/lib/ros2_console_tools/service_commander";
  } catch (const std::exception & exception) {
    if (error != nullptr) {
      *error = exception.what();
    }
    return false;
  }

  std::vector<std::string> arguments{executable_path, "--embedded"};
  if (!initial_node.empty()) {
    arguments.push_back("--node");
    arguments.push_back(initial_node);
  }
  if (!initial_service.empty()) {
    arguments.push_back("--service");
    arguments.push_back(initial_service);
  }

  std::vector<char *> argv;
  argv.reserve(arguments.size() + 1);
  for (auto & argument : arguments) {
    argv.push_back(argument.data());
  }
  argv.push_back(nullptr);

  const pid_t child_pid = fork();
  if (child_pid == -1) {
    if (error != nullptr) {
      *error = std::strerror(errno);
    }
    return false;
  }

  if (child_pid == 0) {
    execv(executable_path.c_str(), argv.data());
    std::perror("execv");
    _exit(127);
  }

  int child_status = 0;
  while (waitpid(child_pid, &child_status, 0) == -1) {
    if (errno == EINTR) {
      continue;
    }
    if (error != nullptr) {
      *error = std::strerror(errno);
    }
    return false;
  }

  if (WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0) {
    return true;
  }

  if (error != nullptr) {
    if (WIFEXITED(child_status)) {
      *error = "service_commander exited with code " + std::to_string(WEXITSTATUS(child_status));
    } else if (WIFSIGNALED(child_status)) {
      *error = "service_commander terminated by signal " + std::to_string(WTERMSIG(child_status));
    } else {
      *error = "service_commander terminated unexpectedly";
    }
  }
  return false;
}

std::string make_help_popup_title() {
  std::string title = "Node Commander";
  const std::string version = ROS2_CONSOLE_TOOLS_PACKAGE_VERSION;
  if (!version.empty() && version != "unknown") {
    title += " v";
    title += version;
  }
  return title;
}

}  // namespace

NodeCommanderScreen::NodeCommanderScreen(std::shared_ptr<NodeCommanderBackend> backend)
: backend_(std::move(backend)),
  help_popup_title_(make_help_popup_title()) {}

int NodeCommanderScreen::run() {
  Session ncurses_session;
  backend_->refresh_nodes();
  backend_->warm_up_node_list();
  refresh_detail_lines_cache(true);

  bool running = true;
  while (running && rclcpp::ok()) {
    backend_->maybe_refresh_nodes();
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
    case KEY_F(9):
      return launch_diagnostics_viewer();
    case '\t':
      focus_pane_ = focus_pane_ == NodeCommanderFocusPane::NodeList
        ? NodeCommanderFocusPane::DetailPane
        : NodeCommanderFocusPane::NodeList;
      if (focus_pane_ == NodeCommanderFocusPane::DetailPane) {
        focus_detail_pane();
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
        refresh_detail_lines_cache(true);
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
        refresh_detail_lines_cache(true);
      } else if (detail_selected_index_ + 1 < static_cast<int>(detail_lines_cache_.size())) {
        ++detail_selected_index_;
      }
      return true;
    case KEY_PPAGE:
      if (focus_pane_ == NodeCommanderFocusPane::NodeList) {
        backend_->selected_index_ = std::max(0, backend_->selected_index_ - page_step());
        refresh_detail_lines_cache(true);
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
        refresh_detail_lines_cache(true);
      } else if (!detail_lines_cache_.empty()) {
        detail_selected_index_ = std::min(
          static_cast<int>(detail_lines_cache_.size()) - 1,
          detail_selected_index_ + page_step());
      }
      return true;
    case KEY_RIGHT:
    case 'l':
      if (focus_pane_ == NodeCommanderFocusPane::DetailPane) {
        return expand_selected_detail_section();
      }
      return true;
    case KEY_LEFT:
    case 'h':
      if (focus_pane_ == NodeCommanderFocusPane::DetailPane) {
        return collapse_selected_detail_section();
      }
      return true;
    case '\n':
    case KEY_ENTER:
      if (focus_pane_ == NodeCommanderFocusPane::NodeList) {
        focus_detail_pane();
        return true;
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
      refresh_detail_lines_cache(true);
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

std::string NodeCommanderScreen::selected_node_name() const {
  std::lock_guard<std::mutex> lock(backend_->mutex_);
  if (
    backend_->node_entries_.empty() ||
    backend_->selected_index_ < 0 ||
    backend_->selected_index_ >= static_cast<int>(backend_->node_entries_.size()))
  {
    return "";
  }
  return backend_->node_entries_[static_cast<std::size_t>(backend_->selected_index_)];
}

void NodeCommanderScreen::refresh_detail_lines_cache(bool force) {
  const std::string selected_node = selected_node_name();
  const auto now = std::chrono::steady_clock::now();
  const bool cache_expired = now - detail_cache_refresh_time_ >= std::chrono::seconds(1);
  const bool node_changed = selected_node != detail_cache_node_name_;

  if (selected_node.empty()) {
    detail_cache_node_name_.clear();
    raw_detail_lines_cache_.clear();
    detail_lines_cache_.clear();
    detail_selected_index_ = 0;
    detail_scroll_ = 0;
    detail_cache_refresh_time_ = now;
    return;
  }

  if (force || node_changed || raw_detail_lines_cache_.empty() || cache_expired) {
    raw_detail_lines_cache_ = backend_->selected_node_details();
    detail_cache_node_name_ = selected_node;
    detail_cache_refresh_time_ = now;
  }

  detail_lines_cache_.clear();
  detail_lines_cache_.reserve(raw_detail_lines_cache_.size());

  for (const auto & line : raw_detail_lines_cache_) {
    if (!line.is_header && !line.section_key.empty() && is_detail_section_collapsed(line.section_key)) {
      continue;
    }
    detail_lines_cache_.push_back(line);
  }

  if (detail_lines_cache_.empty()) {
    detail_selected_index_ = 0;
    detail_scroll_ = 0;
    return;
  }

  detail_selected_index_ =
    std::clamp(detail_selected_index_, 0, static_cast<int>(detail_lines_cache_.size()) - 1);
}

void NodeCommanderScreen::focus_detail_pane() {
  refresh_detail_lines_cache(true);
  focus_pane_ = NodeCommanderFocusPane::DetailPane;
  if (detail_lines_cache_.empty()) {
    detail_selected_index_ = 0;
    return;
  }

  const int preferred_index = preferred_detail_index();
  if (preferred_index >= 0) {
    detail_selected_index_ = preferred_index;
  }
}

int NodeCommanderScreen::preferred_detail_index() const {
  for (std::size_t index = 0; index < detail_lines_cache_.size(); ++index) {
    const auto & line = detail_lines_cache_[index];
    if (line.is_header && line.action != NodeDetailAction::None) {
      return static_cast<int>(index);
    }
  }
  for (std::size_t index = 0; index < detail_lines_cache_.size(); ++index) {
    if (detail_lines_cache_[index].action != NodeDetailAction::None) {
      return static_cast<int>(index);
    }
  }
  return detail_lines_cache_.empty() ? -1 : 0;
}

bool NodeCommanderScreen::is_detail_section_collapsed(const std::string & section_key) const {
  const auto found = collapsed_detail_sections_.find(section_key);
  return found != collapsed_detail_sections_.end() && found->second;
}

bool NodeCommanderScreen::expand_selected_detail_section() {
  const DetailLine * line = selected_detail_line();
  if (line == nullptr || !line->is_header || line->section_key.empty()) {
    return true;
  }

  const std::string section_key = line->section_key;
  if (!is_detail_section_collapsed(section_key)) {
    return true;
  }

  collapsed_detail_sections_[section_key] = false;
  refresh_detail_lines_cache(false);
  for (std::size_t index = 0; index < detail_lines_cache_.size(); ++index) {
    if (detail_lines_cache_[index].is_header && detail_lines_cache_[index].section_key == section_key) {
      detail_selected_index_ = static_cast<int>(index);
      break;
    }
  }
  return true;
}

bool NodeCommanderScreen::collapse_selected_detail_section() {
  const DetailLine * line = selected_detail_line();
  if (line == nullptr || line->section_key.empty()) {
    return true;
  }

  const std::string section_key = line->section_key;
  if (line->is_header) {
    if (is_detail_section_collapsed(section_key)) {
      return true;
    }
    collapsed_detail_sections_[section_key] = true;
    refresh_detail_lines_cache(false);
    return true;
  }

  collapsed_detail_sections_[section_key] = true;
  refresh_detail_lines_cache(false);
  for (std::size_t index = 0; index < detail_lines_cache_.size(); ++index) {
    if (detail_lines_cache_[index].is_header && detail_lines_cache_[index].section_key == section_key) {
      detail_selected_index_ = static_cast<int>(index);
      break;
    }
  }
  return true;
}

int NodeCommanderScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

bool NodeCommanderScreen::launch_parameter_commander() {
  def_prog_mode();
  endwin();
  (void)run_parameter_commander_tool("", true);
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from parameter_commander.";
  }
  return true;
}

bool NodeCommanderScreen::launch_log_viewer() {
  def_prog_mode();
  endwin();
  (void)run_log_viewer_tool(true);
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from log_viewer.";
  }
  return true;
}

bool NodeCommanderScreen::launch_service_commander() {
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
  std::string error;
  const bool ok = run_service_commander_subprocess(selected_node, "", &error);
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = ok
      ? "Returned from service_commander for " + selected_node + "."
      : "service_commander failed: " + error;
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
  (void)run_tf_monitor_tool(true);
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
  (void)run_urdf_inspector_tool("", true);
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from urdf_inspector.";
  }
  return true;
}

bool NodeCommanderScreen::launch_diagnostics_viewer() {
  def_prog_mode();
  endwin();
  (void)run_diagnostics_viewer_tool(true);
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Returned from diagnostics_viewer.";
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
  if (line == nullptr || line->action == NodeDetailAction::None) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "No actionable item selected.";
    return true;
  }

  std::string selected_node;
  if (line->action == NodeDetailAction::OpenServiceCommander && !line->is_header) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    if (!backend_->node_entries_.empty() &&
      backend_->selected_index_ >= 0 &&
      backend_->selected_index_ < static_cast<int>(backend_->node_entries_.size()))
    {
      selected_node = backend_->node_entries_[static_cast<std::size_t>(backend_->selected_index_)];
    }
  }

  def_prog_mode();
  endwin();
  std::string service_error;
  bool service_launch_ok = true;
  switch (line->action) {
    case NodeDetailAction::OpenParameters:
      (void)run_parameter_commander_tool(line->target, true);
      break;
    case NodeDetailAction::OpenTopicMonitor:
      if (line->is_header) {
        const auto topic_targets = detail_targets_for_section(line->section_key, NodeDetailAction::OpenTopicMonitor);
        if (topic_targets.empty()) {
          resume_parent_screen();
          std::lock_guard<std::mutex> lock(backend_->mutex_);
          backend_->status_line_ = "No topics available in " + line->text + ".";
          return true;
        }

        TopicMonitorLaunchOptions options;
        options.allowed_topics = topic_targets;
        options.embedded_mode = true;
        options.monitor_allowed_topics_on_start = true;
        (void)run_topic_monitor_tool(options);
      } else {
        TopicMonitorLaunchOptions options;
        options.initial_topic = line->target;
        options.allowed_topics.push_back(line->target);
        options.embedded_mode = true;
        options.open_initial_topic_detail = true;
        options.exit_on_detail_escape = true;
        (void)run_topic_monitor_tool(options);
      }
      break;
    case NodeDetailAction::OpenServiceCommander:
      service_launch_ok = run_service_commander_subprocess(
        line->is_header ? line->target : selected_node,
        line->is_header ? "" : line->target,
        &service_error);
      break;
    case NodeDetailAction::None:
      break;
  }
  resume_parent_screen();
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    if (line->action == NodeDetailAction::OpenServiceCommander && !service_launch_ok) {
      backend_->status_line_ = "service_commander failed: " + service_error;
    } else {
      backend_->status_line_ = "Returned to node_commander.";
    }
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

std::vector<std::string> NodeCommanderScreen::detail_targets_for_section(
  const std::string & section_key, NodeDetailAction action) const
{
  std::vector<std::string> targets;
  for (const auto & line : raw_detail_lines_cache_) {
    if (line.is_header || line.section_key != section_key || line.action != action || line.target.empty()) {
      continue;
    }
    targets.push_back(line.target);
  }
  return targets;
}

void NodeCommanderScreen::draw() {
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
  draw_search_box(layout.pane_rows, columns, search_state_);
  if (help_popup_open_) {
    const int popup_width = std::min(columns - 8, 76);
    const int popup_height = 18;
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
    const std::string popup_title = help_popup_title_ + " ";
    mvaddnstr(popup_top, popup_left + 2, popup_title.c_str(), std::max(0, popup_width - 4));
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
    draw_help_item(popup_top + 2, "Enter", "focus detail pane or open detail item");
    draw_help_item(popup_top + 3, "Tab", "switch node list and detail pane");
    draw_help_item(popup_top + 4, "Left/H", "collapse selected detail section");
    draw_help_item(popup_top + 5, "Right/L", "expand selected detail section");
    draw_help_item(popup_top + 6, "Alt+S", "search nodes");
    draw_help_item(popup_top + 7, "F2", "log_viewer");
    draw_help_item(popup_top + 8, "F3", "topic_monitor");
    draw_help_item(popup_top + 9, "F4", "parameter_commander (selected node)");
    draw_help_item(popup_top + 10, "F5", "service_commander (selected node)");
    draw_help_item(popup_top + 11, "F6", "action_commander");
    draw_help_item(popup_top + 12, "F7", "tf_monitor");
    draw_help_item(popup_top + 13, "F8", "urdf_inspector");
    draw_help_item(popup_top + 14, "F9", "diagnostics_viewer");
    draw_help_item(popup_top + 15, "Alt+T", "toggle terminal");
    draw_help_item(popup_top + 16, "Esc/F1", "close help");
  }
  if (terminal_pane_.visible()) {
    terminal_pane_.draw(layout.terminal_top, 0, rows - 1, columns - 1);
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
  refresh_detail_lines_cache();

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
    const std::string indent(static_cast<std::size_t>(std::max(0, line.depth) * 2), ' ');
    mvhline(row_y, left, ' ', width);
    if (selected) {
      apply_role_chgat(row_y, left, width, kColorSelection);
    }
    if (line.is_header) {
      const std::string marker = is_detail_section_collapsed(line.section_key) ? "[+]" : "[-]";
      const std::string rendered = marker + " " + line.text;
      if (line.action != NodeDetailAction::None) {
        attron(COLOR_PAIR(kColorAccent) | A_BOLD);
      } else {
        attron(theme_attr(kColorHeader));
      }
      mvprintw(row_y, left, "%-*s", width, truncate_text(rendered, width).c_str());
      if (line.action != NodeDetailAction::None) {
        attroff(COLOR_PAIR(kColorAccent) | A_BOLD);
      } else {
        attroff(theme_attr(kColorHeader));
      }
    } else if (line.action != NodeDetailAction::None) {
      attron(COLOR_PAIR(kColorAccent));
      mvprintw(row_y, left, "%-*s", width, truncate_text(indent + line.text, width).c_str());
      attroff(COLOR_PAIR(kColorAccent));
    } else {
      mvprintw(row_y, left, "%-*s", width, truncate_text(indent + line.text, width).c_str());
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
    row,
    columns,
    tui::with_terminal_help(
      "F1 Help  F2 Logs  F3 Topics  F4 Node Params  F5 Node Services  F6 Actions  F7 Transforms  F8 URDF  F9 Diagnostics  F10 Exit  Alt+S Search",
      terminal_pane_.visible()));
}

}  // namespace ros2_console_tools
