#include "ros2_console_tools/systemd_commander.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include "ros2_console_tools/journal_viewer.hpp"
#include "ros2_console_tools/process_runner.hpp"

namespace ros2_console_tools {

int run_systemd_commander_tool(const std::string & initial_unit, bool embedded_mode) {
  auto backend = std::make_shared<SystemdCommanderBackend>(initial_unit);
  SystemdCommanderScreen screen(backend, embedded_mode);
  return screen.run();
}

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorPositive = tui::kColorPositive,
  kColorAccent = tui::kColorAccent,
  kColorPopup = tui::kColorPopup,
  kColorWarn = tui::kColorWarn,
  kColorError = tui::kColorError,
};

using tui::Session;
using tui::apply_role_chgat;
using tui::draw_box;
using tui::draw_help_bar;
using tui::draw_help_bar_region;
using tui::draw_search_box;
using tui::draw_status_bar;
using tui::find_best_match;
using tui::handle_search_input;
using tui::is_alt_binding;
using tui::SearchInputResult;
using tui::start_search;
using tui::theme_attr;
using tui::truncate_text;

int unit_state_color(const SystemdUnitSummary & unit, bool selected) {
  if (selected) {
    return kColorSelection;
  }
  if (unit.active_state == "active" && unit.sub_state == "running") {
    return kColorPositive;
  }
  if (unit.active_state == "active" && unit.sub_state == "exited") {
    return kColorWarn;
  }
  if (unit.active_state == "failed") {
    return kColorError;
  }
  if (unit.active_state == "activating" || unit.active_state == "reloading") {
    return kColorWarn;
  }
  return 0;
}

struct StyledSpan {
  std::string text;
  int color{0};
};

std::string trim_left(std::string text) {
  const auto first_non_space = std::find_if(
    text.begin(), text.end(),
    [](unsigned char ch) { return std::isspace(ch) == 0; });
  text.erase(text.begin(), first_non_space);
  return text;
}

std::vector<StyledSpan> highlight_unit_line(const std::string & line) {
  std::vector<StyledSpan> spans;
  auto push_span = [&](const std::string & text, int color) {
    if (text.empty()) {
      return;
    }
    if (!spans.empty() && spans.back().color == color) {
      spans.back().text += text;
    } else {
      spans.push_back({text, color});
    }
  };

  const std::string trimmed = trim_left(line);
  if (trimmed.empty()) {
    push_span(line, 0);
    return spans;
  }
  if (trimmed.front() == '#' || trimmed.front() == ';') {
    push_span(line, kColorAccent);
    return spans;
  }
  if (trimmed.front() == '[' && trimmed.back() == ']') {
    const std::size_t section_start = line.find('[');
    if (section_start != std::string::npos) {
      push_span(line.substr(0, section_start), 0);
      push_span(line.substr(section_start), kColorHeader);
      return spans;
    }
  }

  const std::size_t equals = line.find('=');
  if (equals != std::string::npos) {
    push_span(line.substr(0, equals), kColorWarn);
    push_span("=", 0);
    push_span(line.substr(equals + 1), kColorPositive);
    return spans;
  }

  push_span(line, 0);
  return spans;
}

std::string trim(std::string text) {
  const auto first_non_space = std::find_if(
    text.begin(), text.end(),
    [](unsigned char ch) { return std::isspace(ch) == 0; });
  text.erase(text.begin(), first_non_space);

  const auto trailing_base = text;
  const auto last_non_space = std::find_if(
    text.rbegin(), text.rend(),
    [](unsigned char ch) { return std::isspace(ch) == 0; });
  if (last_non_space == text.rend()) {
    return "";
  }
  text.erase(last_non_space.base(), text.end());
  (void)trailing_base;
  return text;
}

}  // namespace

SystemdCommanderScreen::SystemdCommanderScreen(
  std::shared_ptr<SystemdCommanderBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int SystemdCommanderScreen::run() {
  std::unique_ptr<Session> ncurses_session;
  if (!embedded_mode_) {
    ncurses_session = std::make_unique<Session>();
  } else {
    curs_set(0);
    keypad(stdscr, TRUE);
    cbreak();
    noecho();
  }

  timeout(100);
  if (embedded_mode_) {
    flushinp();
    timeout(0);
    while (getch() != ERR) {
    }
    timeout(100);
  }

  bool running = true;
  while (running) {
    terminal_pane_.update();
    draw();
    const int key = getch();
    if (key == ERR) {
      continue;
    }
    running = handle_key(key);
  }

  if (embedded_mode_) {
    timeout(100);
    curs_set(0);
    clear();
    clearok(stdscr, TRUE);
    refresh();
  }

  return 0;
}

bool SystemdCommanderScreen::handle_key(int key) {
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

  if (editor_open_) {
    return handle_editor_key(key);
  }

  if (detail_popup_open_) {
    switch (key) {
      case KEY_F(10):
        return false;
      case KEY_F(2):
        detail_scroll_ = 0;
        return perform_selected_unit_action("start");
      case KEY_F(3):
        detail_scroll_ = 0;
        return perform_selected_unit_action("stop");
      case KEY_F(4):
        detail_scroll_ = 0;
        backend_->refresh_units();
        return true;
      case KEY_F(5):
        detail_scroll_ = 0;
        return perform_selected_unit_action("restart");
      case KEY_F(6):
        detail_scroll_ = 0;
        return perform_selected_unit_action("reload");
      case KEY_F(7):
        return open_selected_service_editor();
      case KEY_F(9):
        return launch_selected_logs();
      default:
        return handle_detail_popup_key(key);
    }
  }

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(2):
      detail_scroll_ = 0;
      return perform_selected_unit_action("start");
    case KEY_F(3):
      detail_scroll_ = 0;
      return perform_selected_unit_action("stop");
    case KEY_F(4):
      detail_scroll_ = 0;
      backend_->refresh_units();
      return true;
    case KEY_F(5):
      detail_scroll_ = 0;
      return perform_selected_unit_action("restart");
    case KEY_F(6):
      detail_scroll_ = 0;
      return perform_selected_unit_action("reload");
    case KEY_F(9):
      return launch_selected_logs();
    case '\n':
    case KEY_ENTER:
      detail_popup_open_ = true;
      detail_scroll_ = 0;
      return true;
    case 27:
      if (is_alt_binding(key, 's')) {
        start_search(search_state_);
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        backend_->status_line_ = "Search service list.";
        return true;
      }
      if (embedded_mode_) {
        return false;
      }
      return true;
    default:
      break;
  }

  return handle_unit_list_key(key);
}

bool SystemdCommanderScreen::handle_search_key(int key) {
  const SearchInputResult result = handle_search_input(search_state_, key);
  if (result == SearchInputResult::Cancelled) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Search cancelled.";
    return true;
  }
  if (result == SearchInputResult::Accepted) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ =
      search_state_.query.empty() ? "Search closed." : ("Search: " + search_state_.query);
    return true;
  }
  if (result != SearchInputResult::Changed) {
    return true;
  }

  std::vector<SystemdUnitSummary> units;
  int current_index = 0;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    units = backend_->units_;
    current_index = backend_->selected_index_;
  }

  std::vector<std::string> labels;
  labels.reserve(units.size());
  for (const auto & unit : units) {
    labels.push_back(
      unit.name + " " + unit.description + " " +
      unit.load_state + " " + unit.active_state + " " + unit.sub_state);
  }

  const int match = find_best_match(labels, search_state_.query, current_index);
  if (match >= 0) {
    {
      std::lock_guard<std::mutex> lock(backend_->mutex_);
      backend_->selected_index_ = match;
      backend_->clamp_selection();
    }
    detail_scroll_ = 0;
    backend_->refresh_selected_unit_details();
  }

  std::lock_guard<std::mutex> lock(backend_->mutex_);
  backend_->status_line_ = "Search: " + search_state_.query;
  return true;
}

bool SystemdCommanderScreen::handle_unit_list_key(int key) {
  std::vector<SystemdUnitSummary> units;
  int selected_index = 0;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    units = backend_->units_;
    selected_index = backend_->selected_index_;
  }

  switch (key) {
    case KEY_UP:
    case 'k':
      if (selected_index > 0) {
        {
          std::lock_guard<std::mutex> lock(backend_->mutex_);
          --backend_->selected_index_;
        }
        detail_scroll_ = 0;
        backend_->refresh_selected_unit_details();
      }
      return true;
    case KEY_DOWN:
    case 'j':
      if (selected_index + 1 < static_cast<int>(units.size())) {
        {
          std::lock_guard<std::mutex> lock(backend_->mutex_);
          ++backend_->selected_index_;
        }
        detail_scroll_ = 0;
        backend_->refresh_selected_unit_details();
      }
      return true;
    case KEY_PPAGE:
      {
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        backend_->selected_index_ = std::max(0, backend_->selected_index_ - page_step());
        backend_->clamp_selection();
      }
      detail_scroll_ = 0;
      backend_->refresh_selected_unit_details();
      return true;
    case KEY_NPAGE:
      if (!units.empty()) {
        {
          std::lock_guard<std::mutex> lock(backend_->mutex_);
          backend_->selected_index_ = std::min(
            static_cast<int>(units.size()) - 1, backend_->selected_index_ + page_step());
          backend_->clamp_selection();
        }
        detail_scroll_ = 0;
        backend_->refresh_selected_unit_details();
      }
      return true;
    case '\n':
    case KEY_ENTER:
      detail_popup_open_ = true;
      detail_scroll_ = 0;
      return true;
    default:
      return true;
  }
}

bool SystemdCommanderScreen::handle_detail_popup_key(int key) {
  const auto rows = backend_->detail_rows_snapshot();
  int screen_rows = 0;
  int screen_columns = 0;
  getmaxyx(stdscr, screen_rows, screen_columns);
  const int box_height = std::min(screen_rows - 4, std::max(8, screen_rows * 3 / 4));
  const int visible_rows = std::max(1, box_height - 3);
  const int max_scroll = std::max(0, static_cast<int>(rows.size()) - visible_rows);

  switch (key) {
    case 27:
      detail_popup_open_ = false;
      return true;
    case KEY_UP:
    case 'k':
      detail_scroll_ = std::max(0, detail_scroll_ - 1);
      return true;
    case KEY_DOWN:
    case 'j':
      detail_scroll_ = std::min(max_scroll, detail_scroll_ + 1);
      return true;
    case KEY_PPAGE:
      detail_scroll_ = std::max(0, detail_scroll_ - page_step());
      return true;
    case KEY_NPAGE:
      detail_scroll_ = std::min(max_scroll, detail_scroll_ + page_step());
      return true;
    case '\n':
    case KEY_ENTER:
      detail_popup_open_ = false;
      return true;
    default:
      return true;
  }
}

bool SystemdCommanderScreen::handle_editor_key(int key) {
  if (editor_lines_.empty()) {
    editor_lines_.push_back("");
  }

  auto clamp_cursor = [&]() {
    editor_cursor_row_ = std::clamp(editor_cursor_row_, 0, static_cast<int>(editor_lines_.size()) - 1);
    editor_cursor_column_ = std::clamp(
      editor_cursor_column_, 0, static_cast<int>(editor_lines_[static_cast<std::size_t>(editor_cursor_row_)].size()));
  };

  switch (key) {
    case KEY_F(10):
      return false;
    case KEY_F(2):
      return save_editor();
    case 27:
      close_editor();
      return true;
    case KEY_UP:
      if (editor_cursor_row_ > 0) {
        --editor_cursor_row_;
      }
      clamp_cursor();
      return true;
    case KEY_DOWN:
      if (editor_cursor_row_ + 1 < static_cast<int>(editor_lines_.size())) {
        ++editor_cursor_row_;
      }
      clamp_cursor();
      return true;
    case KEY_LEFT:
      if (editor_cursor_column_ > 0) {
        --editor_cursor_column_;
      } else if (editor_cursor_row_ > 0) {
        --editor_cursor_row_;
        editor_cursor_column_ = static_cast<int>(editor_lines_[static_cast<std::size_t>(editor_cursor_row_)].size());
      }
      clamp_cursor();
      return true;
    case KEY_RIGHT:
      if (editor_cursor_column_ <
        static_cast<int>(editor_lines_[static_cast<std::size_t>(editor_cursor_row_)].size()))
      {
        ++editor_cursor_column_;
      } else if (editor_cursor_row_ + 1 < static_cast<int>(editor_lines_.size())) {
        ++editor_cursor_row_;
        editor_cursor_column_ = 0;
      }
      clamp_cursor();
      return true;
    case KEY_HOME:
      editor_cursor_column_ = 0;
      return true;
    case KEY_END:
      editor_cursor_column_ =
        static_cast<int>(editor_lines_[static_cast<std::size_t>(editor_cursor_row_)].size());
      return true;
    case KEY_PPAGE:
      editor_cursor_row_ = std::max(0, editor_cursor_row_ - page_step());
      clamp_cursor();
      return true;
    case KEY_NPAGE:
      editor_cursor_row_ = std::min(
        static_cast<int>(editor_lines_.size()) - 1, editor_cursor_row_ + page_step());
      clamp_cursor();
      return true;
    case KEY_BACKSPACE:
    case 127:
    case '\b':
      if (editor_cursor_column_ > 0) {
        auto & line = editor_lines_[static_cast<std::size_t>(editor_cursor_row_)];
        line.erase(static_cast<std::size_t>(editor_cursor_column_ - 1), 1);
        --editor_cursor_column_;
        editor_dirty_ = true;
      } else if (editor_cursor_row_ > 0) {
        std::string current_line = editor_lines_[static_cast<std::size_t>(editor_cursor_row_)];
        editor_cursor_column_ =
          static_cast<int>(editor_lines_[static_cast<std::size_t>(editor_cursor_row_ - 1)].size());
        editor_lines_[static_cast<std::size_t>(editor_cursor_row_ - 1)] += current_line;
        editor_lines_.erase(editor_lines_.begin() + editor_cursor_row_);
        --editor_cursor_row_;
        editor_dirty_ = true;
      }
      clamp_cursor();
      return true;
    case KEY_DC:
      {
        auto & line = editor_lines_[static_cast<std::size_t>(editor_cursor_row_)];
        if (editor_cursor_column_ < static_cast<int>(line.size())) {
          line.erase(static_cast<std::size_t>(editor_cursor_column_), 1);
          editor_dirty_ = true;
        } else if (editor_cursor_row_ + 1 < static_cast<int>(editor_lines_.size())) {
          line += editor_lines_[static_cast<std::size_t>(editor_cursor_row_ + 1)];
          editor_lines_.erase(editor_lines_.begin() + editor_cursor_row_ + 1);
          editor_dirty_ = true;
        }
      }
      clamp_cursor();
      return true;
    case '\n':
    case KEY_ENTER:
      {
        auto & line = editor_lines_[static_cast<std::size_t>(editor_cursor_row_)];
        std::string remainder = line.substr(static_cast<std::size_t>(editor_cursor_column_));
        line.erase(static_cast<std::size_t>(editor_cursor_column_));
        editor_lines_.insert(editor_lines_.begin() + editor_cursor_row_ + 1, remainder);
        ++editor_cursor_row_;
        editor_cursor_column_ = 0;
        editor_dirty_ = true;
      }
      clamp_cursor();
      return true;
    case '\t':
      editor_lines_[static_cast<std::size_t>(editor_cursor_row_)].insert(
        static_cast<std::size_t>(editor_cursor_column_), "  ");
      editor_cursor_column_ += 2;
      editor_dirty_ = true;
      return true;
    default:
      if (key >= 32 && key <= 126) {
        editor_lines_[static_cast<std::size_t>(editor_cursor_row_)].insert(
          static_cast<std::size_t>(editor_cursor_column_), 1, static_cast<char>(key));
        ++editor_cursor_column_;
        editor_dirty_ = true;
      }
      clamp_cursor();
      return true;
  }
}

void SystemdCommanderScreen::restore_after_shell_prompt() {
  reset_prog_mode();
  refresh();
  clear();
  clearok(stdscr, TRUE);
  redrawwin(stdscr);
  keypad(stdscr, TRUE);
  noecho();
  cbreak();
  timeout(100);
  flushinp();
  curs_set(0);
  draw();
}

bool SystemdCommanderScreen::ensure_sudo_credentials(const std::string & reason) {
  if (run_process({"sudo", "-n", "true"}).succeeded()) {
    return true;
  }

  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = reason.empty() ? "Sudo authentication required." : reason;
  }

  def_prog_mode();
  endwin();
  std::fflush(stdout);
  if (!reason.empty()) {
    std::fprintf(stderr, "\n%s\n", reason.c_str());
  }
  const ProcessResult validation = run_process_interactive({"sudo", "-v"});
  std::fprintf(stderr, "\n");

  restore_after_shell_prompt();

  if (!validation.succeeded()) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Sudo authentication failed or was cancelled.";
    return false;
  }

  std::lock_guard<std::mutex> lock(backend_->mutex_);
  backend_->status_line_ = "Sudo authentication succeeded.";
  return true;
}

ProcessResult SystemdCommanderScreen::run_sudo_command(
  const std::vector<std::string> & arguments,
  const std::string & reason)
{
  if (!ensure_sudo_credentials(reason)) {
    ProcessResult result;
    result.output = "Sudo authentication failed or was cancelled.";
    return result;
  }

  std::vector<std::string> command = {"sudo", "-n"};
  command.insert(command.end(), arguments.begin(), arguments.end());
  return run_process(command);
}

bool SystemdCommanderScreen::perform_selected_unit_action(const std::string & action) {
  const std::string unit_name = backend_->selected_unit_name();
  if (unit_name.empty()) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "No service selected.";
    return true;
  }

  std::string error;
  if (backend_->client_.execute_unit_action(unit_name, action, &error)) {
    backend_->refresh_units();
    backend_->refresh_selected_unit_details();
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Ran `" + action + "` for " + unit_name + ".";
    return true;
  }

  const ProcessResult result = run_sudo_command(
    {"systemctl", "--no-ask-password", action, unit_name},
    "Authentication required to " + action + " " + unit_name + ".");
  if (!result.succeeded()) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    if (!result.output.empty()) {
      backend_->status_line_ = trim(result.output);
    } else if (!error.empty()) {
      backend_->status_line_ = error;
    } else {
      backend_->status_line_ = "Failed to " + action + " " + unit_name + ".";
    }
    return true;
  }

  backend_->refresh_units();
  backend_->refresh_selected_unit_details();
  std::lock_guard<std::mutex> lock(backend_->mutex_);
  backend_->status_line_ = "Ran `" + action + "` for " + unit_name + " via sudo.";
  return true;
}

bool SystemdCommanderScreen::reload_systemd_manager(bool prefer_sudo) {
  if (!prefer_sudo) {
    const ProcessResult direct_result = run_process(
      {"systemctl", "--no-ask-password", "daemon-reload"});
    if (direct_result.succeeded()) {
      return true;
    }
  }

  const ProcessResult sudo_result = run_sudo_command(
    {"systemctl", "--no-ask-password", "daemon-reload"},
    "Authentication required to reload the systemd manager.");
  if (!sudo_result.succeeded()) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ =
      sudo_result.output.empty() ? "systemctl daemon-reload failed." : trim(sudo_result.output);
    return false;
  }

  return true;
}

bool SystemdCommanderScreen::launch_selected_logs() {
  const std::string unit_name = backend_->selected_unit_name();
  if (unit_name.empty()) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "No service selected for log viewing.";
    return true;
  }

  (void)run_journal_viewer_tool(unit_name, true);
  return true;
}

bool SystemdCommanderScreen::open_selected_service_editor() {
  const std::string fragment_path = backend_->selected_fragment_path();
  if (fragment_path.empty() || fragment_path == "/dev/null") {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Selected service does not expose an editable FragmentPath.";
    return true;
  }

  std::ifstream input(fragment_path);
  if (!input.is_open()) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Failed to open " + fragment_path + " for editing.";
    return true;
  }

  editor_lines_.clear();
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    editor_lines_.push_back(line);
  }
  if (editor_lines_.empty()) {
    editor_lines_.push_back("");
  }

  editor_path_ = fragment_path;
  editor_cursor_row_ = 0;
  editor_cursor_column_ = 0;
  editor_scroll_row_ = 0;
  editor_scroll_column_ = 0;
  editor_dirty_ = false;
  editor_open_ = true;
  editor_return_to_detail_popup_ = detail_popup_open_;
  detail_popup_open_ = false;
  search_state_.active = false;

  std::lock_guard<std::mutex> lock(backend_->mutex_);
  backend_->status_line_ = "Editing " + fragment_path + ". F2 saves changes.";
  return true;
}

bool SystemdCommanderScreen::save_editor() {
  if (editor_path_.empty()) {
    return true;
  }

  auto write_lines = [&](std::ostream & stream) {
    for (std::size_t index = 0; index < editor_lines_.size(); ++index) {
      stream << editor_lines_[index];
      if (index + 1 < editor_lines_.size() || !editor_lines_[index].empty()) {
        stream << '\n';
      }
    }
  };

  bool saved = false;
  bool used_sudo = false;
  {
    std::ofstream output(editor_path_, std::ios::trunc);
    if (output.is_open()) {
      write_lines(output);
      output.close();
      saved = static_cast<bool>(output);
    }
  }

  if (!saved) {
    char temp_template[] = "/tmp/ros2_console_tools_service_XXXXXX";
    const int temp_fd = mkstemp(temp_template);
    if (temp_fd == -1) {
      std::lock_guard<std::mutex> lock(backend_->mutex_);
      backend_->status_line_ = "Failed to create a temporary file for privileged save.";
      return true;
    }

    {
      std::ofstream temp_output(temp_template, std::ios::trunc);
      if (!temp_output.is_open()) {
        close(temp_fd);
        unlink(temp_template);
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        backend_->status_line_ = "Failed to open a temporary file for privileged save.";
        return true;
      }
      write_lines(temp_output);
      temp_output.close();
      if (!temp_output) {
        close(temp_fd);
        unlink(temp_template);
        std::lock_guard<std::mutex> lock(backend_->mutex_);
        backend_->status_line_ = "Failed while writing a temporary file for privileged save.";
        return true;
      }
    }
    close(temp_fd);

    const ProcessResult install_result = run_sudo_command(
      {"install", "-m", "0644", temp_template, editor_path_},
      "Authentication required to save " + editor_path_ + ".");
    unlink(temp_template);
    if (!install_result.succeeded()) {
      std::lock_guard<std::mutex> lock(backend_->mutex_);
      backend_->status_line_ =
        install_result.output.empty() ? ("Failed to save " + editor_path_ + ".") : trim(install_result.output);
      return true;
    }
    saved = true;
    used_sudo = true;
  }

  if (!saved) {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->status_line_ = "Failed to save " + editor_path_ + ".";
    return true;
  }

  editor_dirty_ = false;
  std::string status_message = "Saved " + editor_path_;
  if (used_sudo) {
    status_message += " via sudo";
  }
  status_message += ".";

  const bool reload_ok = reload_systemd_manager(used_sudo);
  status_message += reload_ok ? " Reloaded systemd manager." : " `systemctl daemon-reload` failed.";

  backend_->refresh_units();
  backend_->refresh_selected_unit_details();

  std::lock_guard<std::mutex> lock(backend_->mutex_);
  backend_->status_line_ = status_message;
  return true;
}

void SystemdCommanderScreen::close_editor() {
  editor_open_ = false;
  editor_dirty_ = false;
  editor_path_.clear();
  editor_lines_.clear();
  editor_cursor_row_ = 0;
  editor_cursor_column_ = 0;
  editor_scroll_row_ = 0;
  editor_scroll_column_ = 0;
  detail_popup_open_ = editor_return_to_detail_popup_;
  editor_return_to_detail_popup_ = false;
  curs_set(0);
}

int SystemdCommanderScreen::page_step() const {
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  (void)columns;
  return std::max(5, rows - 8);
}

void SystemdCommanderScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const auto layout = tui::make_commander_layout(rows, terminal_pane_.visible());
  const int help_row = layout.help_row;
  const int status_row = layout.status_row;
  const int content_bottom = layout.content_bottom;
  editor_cursor_visible_ = false;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "Systemd Commander ");
  attroff(theme_attr(kColorTitle));

  if (editor_open_) {
    draw_editor(1, 1, content_bottom - 1, columns - 2);
  } else {
    draw_unit_list(1, 1, content_bottom - 1, columns - 2);
  }
  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  if (!editor_open_) {
    draw_search_box(layout.pane_rows, columns, search_state_);
  }
  if (!editor_open_ && detail_popup_open_) {
    draw_detail_popup(layout.pane_rows, columns);
  }
  if (terminal_pane_.visible()) {
    terminal_pane_.draw(layout.terminal_top, 0, rows - 1, columns - 1);
  }
  if (editor_open_ && !terminal_pane_.visible() && editor_cursor_visible_) {
    move(editor_cursor_screen_row_, editor_cursor_screen_column_);
    curs_set(1);
  } else if (!terminal_pane_.visible()) {
    curs_set(0);
  }
  refresh();
}

void SystemdCommanderScreen::draw_unit_list(int top, int left, int bottom, int right) {
  std::vector<SystemdUnitSummary> units;
  int selected_index = 0;
  int scroll = 0;
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    units = backend_->units_;
    selected_index = backend_->selected_index_;
    scroll = backend_->unit_scroll_;
  }

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top);
  if (selected_index < scroll) {
    scroll = selected_index;
  }
  if (selected_index >= scroll + visible_rows) {
    scroll = std::max(0, selected_index - visible_rows + 1);
  }
  {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    backend_->unit_scroll_ = scroll;
  }

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Services");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  const int first_row = scroll;
  const int last_row = std::min(static_cast<int>(units.size()), first_row + visible_rows);
  for (int index = first_row; index < last_row && row_y <= bottom; ++index, ++row_y) {
    const auto & unit = units[static_cast<std::size_t>(index)];
    const bool selected = index == selected_index;
    const std::string text =
      unit.name + "  [" + unit.active_state + "/" + unit.sub_state + "] " + unit.description;
    mvhline(row_y, left, ' ', width);
    mvaddnstr(row_y, left, truncate_text(text, width).c_str(), width);
    const int color = unit_state_color(unit, selected);
    if (color != 0) {
      apply_role_chgat(row_y, left, width, color);
    }
  }

  for (; row_y <= bottom; ++row_y) {
    mvhline(row_y, left, ' ', width);
  }
}

void SystemdCommanderScreen::draw_detail_popup(int rows, int columns) {
  const auto detail_rows = backend_->detail_rows_snapshot();
  if (rows < 8 || columns < 32) {
    return;
  }

  const int box_width = std::min(columns - 4, std::max(32, columns * 4 / 5));
  const int box_height = std::min(rows - 4, std::max(8, rows * 3 / 4));
  const int left = std::max(2, (columns - box_width) / 2);
  const int top = std::max(1, (rows - box_height) / 2);
  const int right = left + box_width - 1;
  const int bottom = top + box_height - 1;
  const int width = right - left + 1;
  const int help_row = bottom - 1;
  const int visible_rows = std::max(1, box_height - 4);
  const int max_scroll = std::max(0, static_cast<int>(detail_rows.size()) - visible_rows);
  detail_scroll_ = std::clamp(detail_scroll_, 0, max_scroll);

  attron(theme_attr(kColorPopup));
  for (int row = top + 1; row < bottom; ++row) {
    mvhline(row, left + 1, ' ', box_width - 2);
  }
  attroff(theme_attr(kColorPopup));
  draw_box(top, left, bottom, right, kColorFrame);

  attron(theme_attr(kColorHeader));
  mvprintw(top, left + 2, " Service Details ");
  attroff(theme_attr(kColorHeader));

  int row_y = top + 1;
  const int first_row = detail_scroll_;
  const int last_row = std::min(static_cast<int>(detail_rows.size()), first_row + visible_rows);
  for (int index = first_row; index < last_row && row_y < help_row; ++index, ++row_y) {
    const auto & row = detail_rows[static_cast<std::size_t>(index)];
    mvhline(row_y, left + 1, ' ', box_width - 2);
    if (row.is_header) {
      attron(theme_attr(kColorHeader));
      mvaddnstr(row_y, left + 1, truncate_text(row.text, width - 2).c_str(), width - 2);
      attroff(theme_attr(kColorHeader));
    } else {
      mvaddnstr(row_y, left + 1, truncate_text(row.text, width - 2).c_str(), width - 2);
    }
  }

  for (; row_y < help_row; ++row_y) {
    mvhline(row_y, left + 1, ' ', box_width - 2);
  }

  draw_help_bar_region(
    help_row,
    left + 1,
    box_width - 2,
    "F2 Start  F3 Stop  F5 Restart  F6 Reload  F7 Edit  F9 Logs  Esc Close");
}

void SystemdCommanderScreen::draw_editor(int top, int left, int bottom, int right) {
  if (editor_lines_.empty()) {
    editor_lines_.push_back("");
  }

  const int width = right - left + 1;
  const int visible_rows = std::max(1, bottom - top);
  const int line_number_width = std::max(
    4, static_cast<int>(std::to_string(std::max(1, static_cast<int>(editor_lines_.size()))).size()));
  const int prefix_width = line_number_width + 3;
  const int text_width = std::max(1, width - prefix_width);

  editor_cursor_row_ = std::clamp(editor_cursor_row_, 0, static_cast<int>(editor_lines_.size()) - 1);
  editor_cursor_column_ = std::clamp(
    editor_cursor_column_, 0,
    static_cast<int>(editor_lines_[static_cast<std::size_t>(editor_cursor_row_)].size()));

  if (editor_cursor_row_ < editor_scroll_row_) {
    editor_scroll_row_ = editor_cursor_row_;
  }
  if (editor_cursor_row_ >= editor_scroll_row_ + visible_rows) {
    editor_scroll_row_ = editor_cursor_row_ - visible_rows + 1;
  }
  if (editor_cursor_column_ < editor_scroll_column_) {
    editor_scroll_column_ = editor_cursor_column_;
  }
  if (editor_cursor_column_ >= editor_scroll_column_ + text_width) {
    editor_scroll_column_ = editor_cursor_column_ - text_width + 1;
  }

  const std::string title = truncate_text(
    std::string(editor_dirty_ ? "Edit* " : "Edit ") + editor_path_, width);
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, title.c_str());
  attroff(theme_attr(kColorHeader));

  for (int row = top + 1; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
    const int line_index = editor_scroll_row_ + (row - top - 1);
    if (line_index >= static_cast<int>(editor_lines_.size())) {
      continue;
    }

    const bool current_line = line_index == editor_cursor_row_;
    const std::string line_number = std::to_string(line_index + 1);
    const std::string prefix =
      std::string(current_line ? ">" : " ") +
      std::string(line_number_width - static_cast<int>(line_number.size()), ' ') +
      line_number + " ";
    attron(theme_attr(kColorHeader));
    mvaddnstr(row, left, prefix.c_str(), prefix_width);
    attroff(theme_attr(kColorHeader));

    const std::string & full_line = editor_lines_[static_cast<std::size_t>(line_index)];
    const std::string visible_text =
      editor_scroll_column_ < static_cast<int>(full_line.size())
      ? full_line.substr(static_cast<std::size_t>(editor_scroll_column_))
      : "";
    int column = left + prefix_width;
    int drawn = 0;
    for (const auto & span : highlight_unit_line(visible_text)) {
      if (drawn >= text_width) {
        break;
      }
      const std::string text = truncate_text(span.text, text_width - drawn);
      if (span.color != 0) {
        attron(COLOR_PAIR(span.color));
      }
      mvaddnstr(row, column, text.c_str(), text_width - drawn);
      if (span.color != 0) {
        attroff(COLOR_PAIR(span.color));
      }
      column += static_cast<int>(text.size());
      drawn += static_cast<int>(text.size());
    }
  }

  const int cursor_screen_row = top + 1 + (editor_cursor_row_ - editor_scroll_row_);
  const int cursor_screen_column = left + prefix_width + (editor_cursor_column_ - editor_scroll_column_);
  if (cursor_screen_row >= top + 1 && cursor_screen_row <= bottom &&
    cursor_screen_column >= left + prefix_width && cursor_screen_column <= right)
  {
    editor_cursor_visible_ = true;
    editor_cursor_screen_row_ = cursor_screen_row;
    editor_cursor_screen_column_ = cursor_screen_column;
  } else {
    editor_cursor_visible_ = false;
  }
}

void SystemdCommanderScreen::draw_status_line(int row, int columns) const {
  std::string status_line;
  if (editor_open_) {
    status_line =
      std::string(editor_dirty_ ? "Modified  " : "Saved  ") +
      "line=" + std::to_string(editor_cursor_row_ + 1) +
      " col=" + std::to_string(editor_cursor_column_ + 1) +
      "  " + editor_path_;
  } else {
    std::lock_guard<std::mutex> lock(backend_->mutex_);
    status_line = backend_->status_line_;
  }
  draw_status_bar(row, columns, status_line);
}

void SystemdCommanderScreen::draw_help_line(int row, int columns) const {
  if (editor_open_) {
    draw_help_bar(
      row,
      columns,
      tui::with_terminal_help(
        "Arrows Move  Enter Split Line  Backspace/Delete Delete  F2 Save  Esc Back  F10 Exit",
        terminal_pane_.visible()));
    return;
  }
  draw_help_bar(
    row,
    columns,
    tui::with_terminal_help(
      "Up/Down Move  Enter Details  F2 Start  F3 Stop  F4 Refresh  F5 Restart  F6 Reload  F9 Logs  Alt+S Search  F10 Exit",
      terminal_pane_.visible()));
}

}  // namespace ros2_console_tools
