#include "ros2_console_tools/tui.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <clocale>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <fcntl.h>
#include <langinfo.h>
#include <map>
#include <poll.h>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace ros2_console_tools::tui {

namespace {

Theme g_theme = make_default_theme();
constexpr std::size_t kTerminalScrollbackLimit = 2000;

struct HelpBlock {
  std::string key;
  std::string label;
};

std::string to_lower(std::string text);

void set_color_role(
  Theme & theme, int role, short foreground, short background, int attributes = A_NORMAL)
{
  theme[static_cast<std::size_t>(role)] = ThemeColor{foreground, background, attributes};
}

std::string trim(const std::string & text) {
  std::size_t start = 0;
  while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start]))) {
    ++start;
  }
  std::size_t end = text.size();
  while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1]))) {
    --end;
  }
  return text.substr(start, end - start);
}

std::vector<HelpBlock> parse_help_blocks(const std::string & text) {
  std::vector<HelpBlock> blocks;
  std::size_t start = 0;
  while (start < text.size()) {
    std::size_t separator = text.find("  ", start);
    std::string block_text =
      separator == std::string::npos ? text.substr(start) : text.substr(start, separator - start);
    block_text = trim(block_text);
    if (!block_text.empty()) {
      const std::size_t split = block_text.find(' ');
      if (split == std::string::npos) {
        blocks.push_back({block_text, ""});
      } else {
        blocks.push_back({block_text.substr(0, split), trim(block_text.substr(split + 1))});
      }
    }
    if (separator == std::string::npos) {
      break;
    }
    start = separator;
    while (start < text.size() && text[start] == ' ') {
      ++start;
    }
  }
  return blocks;
}

std::vector<std::string> split_list(const std::string & text) {
  std::vector<std::string> values;
  std::string current;
  for (char ch : text) {
    if (ch == ',') {
      const std::string trimmed = trim(current);
      if (!trimmed.empty()) {
        values.push_back(trimmed);
      }
      current.clear();
      continue;
    }
    current.push_back(ch);
  }
  const std::string trimmed = trim(current);
  if (!trimmed.empty()) {
    values.push_back(trimmed);
  }
  return values;
}

short parse_color_name(const std::string & value) {
  static const std::map<std::string, short> colors = {
    {"default", -1},
    {"black", COLOR_BLACK},
    {"red", COLOR_RED},
    {"green", COLOR_GREEN},
    {"yellow", COLOR_YELLOW},
    {"blue", COLOR_BLUE},
    {"magenta", COLOR_MAGENTA},
    {"cyan", COLOR_CYAN},
    {"white", COLOR_WHITE},
  };
  const auto found = colors.find(to_lower(trim(value)));
  return found == colors.end() ? static_cast<short>(-2) : found->second;
}

int parse_attributes(const std::string & value) {
  const std::string trimmed = trim(value);
  if (trimmed == "[]" || trimmed.empty()) {
    return A_NORMAL;
  }

  std::string inner = trimmed;
  if (!inner.empty() && inner.front() == '[') {
    inner.erase(inner.begin());
  }
  if (!inner.empty() && inner.back() == ']') {
    inner.pop_back();
  }

  int attributes = A_NORMAL;
  for (const auto & item : split_list(inner)) {
    const std::string name = to_lower(item);
    if (name == "bold") {
      attributes |= A_BOLD;
    } else if (name == "reverse") {
      attributes |= A_REVERSE;
    } else if (name == "underline") {
      attributes |= A_UNDERLINE;
    } else if (name == "dim") {
      attributes |= A_DIM;
    }
  }
  return attributes;
}

bool parse_theme_role(const std::string & value, int * role) {
  static const std::map<std::string, int> roles = {
    {"frame", kColorFrame},
    {"title", kColorTitle},
    {"header", kColorHeader},
    {"selection", kColorSelection},
    {"status", kColorStatus},
    {"help", kColorHelp},
    {"popup", kColorPopup},
    {"input", kColorInput},
    {"dirty", kColorDirty},
    {"cursor", kColorCursor},
    {"positive", kColorPositive},
    {"positive_selection", kColorPositiveSelection},
    {"warn", kColorWarn},
    {"error", kColorError},
    {"fatal", kColorFatal},
    {"accent", kColorAccent},
    {"help_key", kColorHelpKey},
  };
  const auto found = roles.find(to_lower(trim(value)));
  if (found == roles.end()) {
    return false;
  }
  *role = found->second;
  return true;
}

std::string to_lower(std::string text) {
  for (char & character : text) {
    character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  }
  return text;
}

void initialize_theme() {
  if (!has_colors()) {
    return;
  }

  start_color();
  use_default_colors();
  for (int role = 1; role < kThemeColorCount; ++role) {
    const auto & color = g_theme[static_cast<std::size_t>(role)];
    init_pair(role, color.foreground, color.background);
  }
}

void install_function_key_sequences() {
  struct Binding {
    const char * sequence;
    int keycode;
  };

  static const Binding bindings[] = {
    {"\x1bOP", KEY_F(1)},
    {"\x1bOQ", KEY_F(2)},
    {"\x1bOR", KEY_F(3)},
    {"\x1bOS", KEY_F(4)},
    {"\x1b[11~", KEY_F(1)},
    {"\x1b[12~", KEY_F(2)},
    {"\x1b[13~", KEY_F(3)},
    {"\x1b[14~", KEY_F(4)},
    {"\x1b[15~", KEY_F(5)},
    {"\x1b[17~", KEY_F(6)},
    {"\x1b[18~", KEY_F(7)},
    {"\x1b[19~", KEY_F(8)},
    {"\x1b[20~", KEY_F(9)},
    {"\x1b[21~", KEY_F(10)},
    {"\x1b[23~", KEY_F(11)},
    {"\x1b[24~", KEY_F(12)},
  };

  for (const auto & binding : bindings) {
    define_key(binding.sequence, binding.keycode);
  }
}

}  // namespace

Theme make_default_theme() {
  Theme theme{};
  theme[0] = ThemeColor{};
  set_color_role(theme, kColorFrame, COLOR_CYAN, -1);
  set_color_role(theme, kColorTitle, COLOR_WHITE, -1);
  set_color_role(theme, kColorHeader, COLOR_YELLOW, -1);
  set_color_role(theme, kColorSelection, COLOR_BLACK, COLOR_YELLOW);
  set_color_role(theme, kColorStatus, COLOR_GREEN, -1);
  set_color_role(theme, kColorHelp, COLOR_BLACK, COLOR_CYAN);
  set_color_role(theme, kColorPopup, COLOR_WHITE, -1);
  set_color_role(theme, kColorInput, COLOR_YELLOW, -1);
  set_color_role(theme, kColorDirty, COLOR_RED, -1);
  set_color_role(theme, kColorCursor, COLOR_BLACK, COLOR_YELLOW);
  set_color_role(theme, kColorPositive, COLOR_GREEN, -1);
  set_color_role(theme, kColorPositiveSelection, COLOR_GREEN, COLOR_YELLOW);
  set_color_role(theme, kColorWarn, COLOR_YELLOW, -1);
  set_color_role(theme, kColorError, COLOR_RED, -1);
  set_color_role(theme, kColorFatal, COLOR_RED, COLOR_YELLOW);
  set_color_role(theme, kColorAccent, COLOR_CYAN, -1);
  set_color_role(theme, kColorHelpKey, COLOR_BLACK, COLOR_YELLOW, A_BOLD);
  return theme;
}

const Theme & current_theme() {
  return g_theme;
}

int monochrome_fallback_attr(int role) {
  switch (role) {
    case kColorSelection:
    case kColorPositiveSelection:
    case kColorFatal:
    case kColorCursor:
      return A_REVERSE;
    default:
      return A_NORMAL;
  }
}

int theme_attr(int role) {
  if (role <= 0 || role >= kThemeColorCount) {
    return A_NORMAL;
  }
  const int attributes =
    g_theme[static_cast<std::size_t>(role)].attributes |
    (!has_colors() ? monochrome_fallback_attr(role) : A_NORMAL);
  return (has_colors() ? COLOR_PAIR(role) : 0) | attributes;
}

void apply_role_chgat(int row, int col, int count, int role, int extra_attributes) {
  if (count <= 0) {
    return;
  }
  if (role <= 0 || role >= kThemeColorCount) {
    mvchgat(row, col, count, extra_attributes, 0, nullptr);
    return;
  }
  const int attributes =
    g_theme[static_cast<std::size_t>(role)].attributes |
    (!has_colors() ? monochrome_fallback_attr(role) : A_NORMAL) |
    extra_attributes;
  mvchgat(row, col, count, attributes, has_colors() ? role : 0, nullptr);
}

void set_theme(const Theme & theme) {
  g_theme = theme;
  if (!has_colors()) {
    return;
  }
  for (int role = 1; role < kThemeColorCount; ++role) {
    const auto & color = g_theme[static_cast<std::size_t>(role)];
    init_pair(role, color.foreground, color.background);
  }
}

bool is_alt_binding(int key, int expected) {
  if (key != 27) {
    return false;
  }

  const int next = getch();
  if (next == ERR) {
    return false;
  }
  if (next == expected || next == std::toupper(expected)) {
    return true;
  }
  ungetch(next);
  return false;
}

void start_search(SearchState & state) {
  state.active = true;
  state.query.clear();
}

SearchInputResult handle_search_input(SearchState & state, int key) {
  switch (key) {
    case 27:
      state.active = false;
      state.query.clear();
      return SearchInputResult::Cancelled;
    case '\n':
    case KEY_ENTER:
      state.active = false;
      return SearchInputResult::Accepted;
    case KEY_BACKSPACE:
    case 127:
    case '\b':
      if (!state.query.empty()) {
        state.query.pop_back();
      }
      return SearchInputResult::Changed;
    default:
      if (key >= 32 && key <= 126) {
        state.query.push_back(static_cast<char>(key));
        return SearchInputResult::Changed;
      }
      return SearchInputResult::None;
  }
}

int find_best_match(const std::vector<std::string> & labels, const std::string & query, int current_index) {
  if (labels.empty() || query.empty()) {
    return -1;
  }

  const std::string needle = to_lower(query);
  const int count = static_cast<int>(labels.size());
  const int start = std::clamp(current_index, 0, count - 1);

  for (int pass = 0; pass < 2; ++pass) {
    for (int offset = 0; offset < count; ++offset) {
      const int index = (start + offset) % count;
      const std::string haystack = to_lower(labels[static_cast<std::size_t>(index)]);
      const std::size_t position = haystack.find(needle);
      if (position == std::string::npos) {
        continue;
      }
      if ((pass == 0 && position == 0) || pass == 1) {
        return index;
      }
    }
  }

  return -1;
}

Session::Session() {
  std::setlocale(LC_ALL, "");
  initscr();
  set_escdelay(25);
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  install_function_key_sequences();
  curs_set(0);
  timeout(100);
  initialize_theme();
}

Session::~Session() {
  curs_set(1);
  endwin();
}

TerminalPane::TerminalPane() = default;

TerminalPane::~TerminalPane() {
  shutdown_process();
}

bool TerminalPane::visible() const {
  return visible_;
}

void TerminalPane::toggle() {
  if (visible_) {
    hide();
  } else {
    show();
  }
}

void TerminalPane::show() {
  visible_ = true;
  ensure_process();
}

void TerminalPane::hide() {
  visible_ = false;
}

void TerminalPane::update() {
  reap_process();
  if (master_fd_ < 0) {
    return;
  }

  char buffer[1024];
  while (true) {
    const ssize_t bytes_read = ::read(master_fd_, buffer, sizeof(buffer));
    if (bytes_read > 0) {
      for (ssize_t index = 0; index < bytes_read; ++index) {
        process_output_char(buffer[index]);
      }
      continue;
    }
    if (bytes_read == 0) {
      reap_process();
      break;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    append_line("Terminal read failed.");
    shutdown_process();
    break;
  }
}

bool TerminalPane::handle_key(int key) {
  if (!visible_) {
    return false;
  }

  ensure_process();
  if (master_fd_ < 0) {
    return true;
  }

  const std::string encoded = encode_key(key);
  if (encoded.empty()) {
    return true;
  }

  const char * data = encoded.data();
  std::size_t remaining = encoded.size();
  while (remaining > 0) {
    const ssize_t written = ::write(master_fd_, data, remaining);
    if (written > 0) {
      data += written;
      remaining -= static_cast<std::size_t>(written);
      continue;
    }
    if (written == -1 && errno == EINTR) {
      continue;
    }
    if (written == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      break;
    }
    append_line("Terminal write failed.");
    shutdown_process();
    break;
  }
  return true;
}

void TerminalPane::draw(int top, int left, int bottom, int right) {
  if (!visible_ || top >= bottom || left >= right) {
    return;
  }

  const int inner_rows = std::max(1, bottom - top - 1);
  const int inner_columns = std::max(1, right - left - 1);
  resize_pty(inner_rows, inner_columns);

  draw_box(top, left, bottom, right, kColorFrame);
  attron(theme_attr(kColorHeader));
  mvprintw(top, left + 2, " Terminal ");
  attroff(theme_attr(kColorHeader));

  for (int row = top + 1; row < bottom; ++row) {
    attron(COLOR_PAIR(kColorPopup));
    mvhline(row, left + 1, ' ', inner_columns);
    attroff(COLOR_PAIR(kColorPopup));
  }

  std::vector<std::string> lines;
  lines.reserve(scrollback_.size() + (current_line_.empty() ? 0 : 1));
  lines.insert(lines.end(), scrollback_.begin(), scrollback_.end());
  if (!current_line_.empty()) {
    lines.push_back(current_line_);
  }

  const int first_line = std::max(0, static_cast<int>(lines.size()) - inner_rows);
  for (int row = 0; row < inner_rows; ++row) {
    const int line_index = first_line + row;
    if (line_index >= static_cast<int>(lines.size())) {
      continue;
    }
    mvaddnstr(
      top + 1 + row,
      left + 1,
      truncate_text(lines[static_cast<std::size_t>(line_index)], inner_columns).c_str(),
      inner_columns);
  }
}

void TerminalPane::ensure_process() {
  reap_process();
  if (child_running_) {
    return;
  }
  spawn_process();
}

void TerminalPane::spawn_process() {
  shutdown_process();

  const int master_fd = posix_openpt(O_RDWR | O_NOCTTY);
  if (master_fd == -1) {
    append_line("Failed to open pseudo terminal.");
    return;
  }
  if (grantpt(master_fd) != 0 || unlockpt(master_fd) != 0) {
    ::close(master_fd);
    append_line("Failed to initialize pseudo terminal.");
    return;
  }

  char * slave_name = ptsname(master_fd);
  if (slave_name == nullptr) {
    ::close(master_fd);
    append_line("Failed to resolve pseudo terminal path.");
    return;
  }

  const pid_t child_pid = fork();
  if (child_pid == -1) {
    ::close(master_fd);
    append_line("Failed to fork shell.");
    return;
  }

  if (child_pid == 0) {
    setsid();
    const int slave_fd = ::open(slave_name, O_RDWR);
    if (slave_fd == -1) {
      _exit(127);
    }

    if (ioctl(slave_fd, TIOCSCTTY, 0) == -1) {
      ::close(slave_fd);
      _exit(127);
    }

    dup2(slave_fd, STDIN_FILENO);
    dup2(slave_fd, STDOUT_FILENO);
    dup2(slave_fd, STDERR_FILENO);
    if (slave_fd > STDERR_FILENO) {
      ::close(slave_fd);
    }
    ::close(master_fd);

    const char * shell = std::getenv("SHELL");
    if (shell == nullptr || *shell == '\0') {
      shell = "/bin/bash";
    }
    execlp(shell, shell, "-i", static_cast<char *>(nullptr));
    execl("/bin/sh", "sh", "-i", static_cast<char *>(nullptr));
    _exit(127);
  }

  const int flags = fcntl(master_fd, F_GETFL, 0);
  if (flags >= 0) {
    fcntl(master_fd, F_SETFL, flags | O_NONBLOCK);
  }

  master_fd_ = master_fd;
  child_pid_ = child_pid;
  child_running_ = true;
  escape_mode_ = EscapeMode::None;
  osc_saw_escape_ = false;
  current_line_.clear();
  scrollback_.clear();
  append_line("Shell started.");
}

void TerminalPane::shutdown_process() {
  if (master_fd_ >= 0) {
    ::close(master_fd_);
    master_fd_ = -1;
  }

  if (child_pid_ > 0) {
    int status = 0;
    if (waitpid(child_pid_, &status, WNOHANG) == 0) {
      kill(child_pid_, SIGHUP);
      for (int attempt = 0; attempt < 10; ++attempt) {
        if (waitpid(child_pid_, &status, WNOHANG) != 0) {
          break;
        }
        napms(20);
      }
      if (waitpid(child_pid_, &status, WNOHANG) == 0) {
        kill(child_pid_, SIGKILL);
        waitpid(child_pid_, &status, 0);
      }
    }
  }

  child_pid_ = -1;
  child_running_ = false;
  last_rows_ = 0;
  last_columns_ = 0;
  escape_mode_ = EscapeMode::None;
  osc_saw_escape_ = false;
}

void TerminalPane::reap_process() {
  if (child_pid_ <= 0) {
    return;
  }

  int status = 0;
  const pid_t wait_result = waitpid(child_pid_, &status, WNOHANG);
  if (wait_result == 0 || wait_result == -1) {
    return;
  }

  if (WIFEXITED(status)) {
    append_line("Shell exited with code " + std::to_string(WEXITSTATUS(status)) + ".");
  } else if (WIFSIGNALED(status)) {
    append_line("Shell terminated by signal " + std::to_string(WTERMSIG(status)) + ".");
  } else {
    append_line("Shell terminated.");
  }

  if (master_fd_ >= 0) {
    ::close(master_fd_);
    master_fd_ = -1;
  }
  child_pid_ = -1;
  child_running_ = false;
  last_rows_ = 0;
  last_columns_ = 0;
  escape_mode_ = EscapeMode::None;
  osc_saw_escape_ = false;
}

void TerminalPane::resize_pty(int rows, int columns) {
  if (master_fd_ < 0 || rows <= 0 || columns <= 0) {
    return;
  }
  if (rows == last_rows_ && columns == last_columns_) {
    return;
  }

  winsize size{};
  size.ws_row = static_cast<unsigned short>(rows);
  size.ws_col = static_cast<unsigned short>(columns);
  ioctl(master_fd_, TIOCSWINSZ, &size);
  if (child_pid_ > 0) {
    kill(child_pid_, SIGWINCH);
  }
  last_rows_ = rows;
  last_columns_ = columns;
}

void TerminalPane::append_line(const std::string & line) {
  scrollback_.push_back(line);
  while (scrollback_.size() > kTerminalScrollbackLimit) {
    scrollback_.pop_front();
  }
}

void TerminalPane::process_output_char(char byte) {
  switch (escape_mode_) {
    case EscapeMode::Escape:
      if (byte == '[') {
        escape_mode_ = EscapeMode::Csi;
      } else if (byte == ']') {
        escape_mode_ = EscapeMode::Osc;
        osc_saw_escape_ = false;
      } else {
        escape_mode_ = EscapeMode::None;
      }
      return;
    case EscapeMode::Csi:
      if (byte >= '@' && byte <= '~') {
        escape_mode_ = EscapeMode::None;
      }
      return;
    case EscapeMode::Osc:
      if (byte == '\a') {
        escape_mode_ = EscapeMode::None;
        osc_saw_escape_ = false;
        return;
      }
      if (osc_saw_escape_ && byte == '\\') {
        escape_mode_ = EscapeMode::None;
        osc_saw_escape_ = false;
        return;
      }
      osc_saw_escape_ = byte == '\x1b';
      return;
    case EscapeMode::None:
      break;
  }

  if (byte == '\x1b') {
    escape_mode_ = EscapeMode::Escape;
    return;
  }
  if (byte == '\r') {
    current_line_.clear();
    return;
  }
  if (byte == '\n') {
    append_line(current_line_);
    current_line_.clear();
    return;
  }
  if (byte == '\b' || byte == 127) {
    if (!current_line_.empty()) {
      current_line_.pop_back();
    }
    return;
  }
  if (byte == '\t') {
    current_line_ += "  ";
    return;
  }
  if (static_cast<unsigned char>(byte) < 32U) {
    return;
  }
  current_line_.push_back(byte);
}

std::string TerminalPane::encode_key(int key) const {
  switch (key) {
    case '\n':
    case KEY_ENTER:
      return "\r";
    case '\t':
      return "\t";
    case 27:
      return "\x1b";
    case KEY_UP:
      return "\x1b[A";
    case KEY_DOWN:
      return "\x1b[B";
    case KEY_RIGHT:
      return "\x1b[C";
    case KEY_LEFT:
      return "\x1b[D";
    case KEY_HOME:
      return "\x1b[H";
    case KEY_END:
      return "\x1b[F";
    case KEY_PPAGE:
      return "\x1b[5~";
    case KEY_NPAGE:
      return "\x1b[6~";
    case KEY_DC:
      return "\x1b[3~";
    case KEY_IC:
      return "\x1b[2~";
    case KEY_BACKSPACE:
    case 127:
    case '\b':
      return "\x7f";
    case KEY_F(1):
      return "\x1bOP";
    case KEY_F(2):
      return "\x1bOQ";
    case KEY_F(3):
      return "\x1bOR";
    case KEY_F(4):
      return "\x1bOS";
    case KEY_F(5):
      return "\x1b[15~";
    case KEY_F(6):
      return "\x1b[17~";
    case KEY_F(7):
      return "\x1b[18~";
    case KEY_F(8):
      return "\x1b[19~";
    case KEY_F(11):
      return "\x1b[23~";
    case KEY_F(12):
      return "\x1b[24~";
    case KEY_RESIZE:
      return "";
    default:
      break;
  }

  if (key >= 1 && key <= 26) {
    return std::string(1, static_cast<char>(key));
  }
  if (key >= 32 && key <= 255) {
    return std::string(1, static_cast<char>(key));
  }
  return "";
}

CommanderLayout make_commander_layout(int rows, bool terminal_visible) {
  CommanderLayout layout;
  layout.terminal_visible = terminal_visible;
  layout.terminal_top = terminal_visible ? std::max(4, rows / 2) : rows;
  layout.pane_rows = terminal_visible ? layout.terminal_top : rows;
  layout.help_row = std::max(0, layout.pane_rows - 1);
  layout.status_row = std::max(0, layout.pane_rows - 2);
  layout.content_bottom = std::max(1, layout.status_row - 1);
  return layout;
}

std::string with_terminal_help(const std::string & text, bool terminal_visible) {
  const std::string suffix = terminal_visible ? "F9 Hide" : "F9 Terminal";
  return text.empty() ? suffix : (text + "  " + suffix);
}

std::string truncate_text(const std::string & text, int width) {
  if (width <= 0) {
    return "";
  }
  if (static_cast<int>(text.size()) <= width) {
    return text;
  }
  if (width <= 3) {
    return text.substr(0, static_cast<std::size_t>(width));
  }
  return text.substr(0, static_cast<std::size_t>(width - 3)) + "...";
}

bool use_unicode_line_drawing() {
  const char * codeset = nl_langinfo(CODESET);
  return codeset != nullptr && std::string(codeset).find("UTF-8") != std::string::npos;
}

TerminalContext terminal_context() {
  if (!use_unicode_line_drawing()) {
    return TerminalContext::Ascii;
  }
  return has_colors() ? TerminalContext::Color : TerminalContext::Mono;
}

std::string default_theme_config_path() {
  const std::string installed =
    ament_index_cpp::get_package_share_directory("ros2_console_tools") + "/config/tui_theme.yaml";
  if (std::filesystem::exists(installed)) {
    return installed;
  }

  const std::filesystem::path source_fallback =
    std::filesystem::path(__FILE__).parent_path().parent_path() / "config" / "tui_theme.yaml";
  return source_fallback.string();
}

bool load_theme_from_file(const std::string & path, std::string * error) {
  std::ifstream input(path);
  if (!input.is_open()) {
    if (error != nullptr) {
      *error = "Failed to open theme config: " + path;
    }
    return false;
  }

  Theme theme = make_default_theme();
  std::string line;
  int current_role = 0;
  bool in_theme = false;

  while (std::getline(input, line)) {
    const std::size_t comment = line.find('#');
    if (comment != std::string::npos) {
      line.erase(comment);
    }
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (trim(line).empty()) {
      continue;
    }

    const std::size_t indent = line.find_first_not_of(' ');
    const std::string stripped = trim(line);
    if (indent == 0 && stripped == "tui_theme:") {
      in_theme = true;
      current_role = 0;
      continue;
    }
    if (!in_theme) {
      continue;
    }

    if (indent == 2 && stripped.back() == ':') {
      std::string role_name = stripped.substr(0, stripped.size() - 1);
      if (!parse_theme_role(role_name, &current_role)) {
        current_role = 0;
      }
      continue;
    }

    if (indent == 4 && current_role != 0) {
      const std::size_t separator = stripped.find(':');
      if (separator == std::string::npos) {
        continue;
      }
      const std::string key = to_lower(trim(stripped.substr(0, separator)));
      const std::string value = trim(stripped.substr(separator + 1));

      auto & color = theme[static_cast<std::size_t>(current_role)];
      if (key == "foreground") {
        const short parsed = parse_color_name(value);
        if (parsed == -2) {
          if (error != nullptr) {
            *error = "Unknown foreground color in theme: " + value;
          }
          return false;
        }
        color.foreground = parsed;
      } else if (key == "background") {
        const short parsed = parse_color_name(value);
        if (parsed == -2) {
          if (error != nullptr) {
            *error = "Unknown background color in theme: " + value;
          }
          return false;
        }
        color.background = parsed;
      } else if (key == "attributes") {
        color.attributes = parse_attributes(value);
      }
    }
  }

  set_theme(theme);
  return true;
}

void draw_box_char(int row, int col, const cchar_t * wide_char, char ascii_char) {
  if (use_unicode_line_drawing()) {
    mvadd_wch(row, col, wide_char);
  } else {
    mvaddch(row, col, ascii_char);
  }
}

void draw_text_hline(int row, int col, int count) {
  for (int index = 0; index < count; ++index) {
    draw_box_char(row, col + index, WACS_HLINE, '-');
  }
}

void draw_text_vline(int row, int col, int count) {
  for (int index = 0; index < count; ++index) {
    draw_box_char(row + index, col, WACS_VLINE, '|');
  }
}

void draw_box(int top, int left, int bottom, int right, int color_pair) {
  const auto & color = current_theme()[static_cast<std::size_t>(color_pair)];
  attron(COLOR_PAIR(color_pair) | color.attributes);
  draw_box_char(top, left, WACS_ULCORNER, '+');
  draw_box_char(top, right, WACS_URCORNER, '+');
  draw_box_char(bottom, left, WACS_LLCORNER, '+');
  draw_box_char(bottom, right, WACS_LRCORNER, '+');
  draw_text_hline(top, left + 1, right - left - 1);
  draw_text_hline(bottom, left + 1, right - left - 1);
  draw_text_vline(top + 1, left, bottom - top - 1);
  draw_text_vline(top + 1, right, bottom - top - 1);
  attroff(COLOR_PAIR(color_pair) | color.attributes);
}

void draw_bar(int row, int columns, const std::string & text, int color_pair, int left_padding) {
  const auto & color = current_theme()[static_cast<std::size_t>(color_pair)];
  attron(COLOR_PAIR(color_pair) | color.attributes);
  mvhline(row, 0, ' ', columns);
  const int content_width = std::max(0, columns - left_padding);
  mvaddnstr(row, left_padding, truncate_text(text, content_width).c_str(), content_width);
  attroff(COLOR_PAIR(color_pair) | color.attributes);
}

void draw_status_bar(int row, int columns, const std::string & text) {
  const auto & color = current_theme()[kColorStatus];
  attron(COLOR_PAIR(kColorStatus) | color.attributes);
  mvhline(row, 0, ' ', columns);
  mvaddnstr(row, 1, truncate_text(text, std::max(0, columns - 1)).c_str(), std::max(0, columns - 1));
  attroff(COLOR_PAIR(kColorStatus) | color.attributes);
}

void draw_help_bar_region(int row, int left, int width, const std::string & text) {
  attrset(A_NORMAL);
  mvhline(row, left, ' ', width);

  const auto blocks = parse_help_blocks(text);
  const auto & help_key_color = current_theme()[kColorHelpKey];
  int column = left;
  const int right = left + width;
  auto block_width = [](const HelpBlock & block) {
    if (block.label.empty()) {
      return static_cast<int>(block.key.size());
    }
    return static_cast<int>(block.key.size() + 1 + block.label.size() + 1);
  };
  auto draw_block = [&](const HelpBlock & block) {
    const std::string key = block.key;
    attron(COLOR_PAIR(kColorHelpKey) | help_key_color.attributes);
    mvaddnstr(row, column, key.c_str(), right - column);
    attroff(COLOR_PAIR(kColorHelpKey) | help_key_color.attributes);
    column += static_cast<int>(key.size());

    if (!block.label.empty()) {
      attron(COLOR_PAIR(kColorHelp));
      mvaddch(row, column, ' ');
      attroff(COLOR_PAIR(kColorHelp));
      ++column;

      const std::string label = block.label;
      attron(COLOR_PAIR(kColorHelp));
      mvaddnstr(row, column, label.c_str(), right - column);
      attroff(COLOR_PAIR(kColorHelp));
      column += static_cast<int>(label.size());

      attron(COLOR_PAIR(kColorHelp));
      mvaddch(row, column, ' ');
      attroff(COLOR_PAIR(kColorHelp));
      ++column;
    }
  };

  bool first_block = true;
  std::size_t index = 0;
  for (; index < blocks.size(); ++index) {
    const int separator_width = first_block ? 0 : 1;
    const int needed = separator_width + block_width(blocks[index]);
    if (column + needed > right) {
      break;
    }

    if (!first_block) {
      attrset(A_NORMAL);
      mvaddch(row, column, ' ');
      ++column;
    }

    draw_block(blocks[index]);

    first_block = false;
  }

  if (index < blocks.size() && column < right) {
    const int ellipsis_width = first_block ? 3 : 4;
    if (column + ellipsis_width <= right) {
      if (!first_block) {
        attrset(A_NORMAL);
        mvaddch(row, column, ' ');
        ++column;
      }
      attron(COLOR_PAIR(kColorHelp));
      mvaddnstr(row, column, "...", right - column);
      attroff(COLOR_PAIR(kColorHelp));
    } else if (first_block && width >= 3) {
      attron(COLOR_PAIR(kColorHelp));
      mvaddnstr(row, left, "...", width);
      attroff(COLOR_PAIR(kColorHelp));
    }
  }
}

void draw_help_bar(int row, int columns, const std::string & text) {
  draw_help_bar_region(row, 0, columns, text);
}

void draw_search_box(int rows, int columns, const SearchState & state, const std::string & prompt) {
  if (!state.active || rows < 6 || columns < 20) {
    return;
  }

  const int box_width = std::min(columns - 4, 48);
  const int left = std::max(2, (columns - box_width) / 2);
  const int top = std::max(1, rows - 5);
  const int bottom = top + 2;
  const int right = left + box_width - 1;
  const int inner_width = box_width - 2;

  attron(COLOR_PAIR(kColorPopup));
  mvhline(top + 1, left + 1, ' ', inner_width);
  attroff(COLOR_PAIR(kColorPopup));
  draw_box(top, left, bottom, right, kColorFrame);

  const std::string label = prompt + ": ";
  attron(COLOR_PAIR(kColorHeader) | A_BOLD);
  mvaddnstr(top + 1, left + 1, label.c_str(), inner_width);
  attroff(COLOR_PAIR(kColorHeader) | A_BOLD);

  const int input_left = left + 1 + static_cast<int>(label.size());
  const int input_width = std::max(0, right - input_left);
  attron(COLOR_PAIR(kColorInput) | A_BOLD);
  mvhline(top + 1, input_left, ' ', input_width);
  mvaddnstr(top + 1, input_left, truncate_text(state.query, input_width).c_str(), input_width);
  attroff(COLOR_PAIR(kColorInput) | A_BOLD);
}

}  // namespace ros2_console_tools::tui
