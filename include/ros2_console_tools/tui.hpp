#ifndef ROS2_CONSOLE_TOOLS__TUI_HPP_
#define ROS2_CONSOLE_TOOLS__TUI_HPP_

#include <ncursesw/ncurses.h>

#include <array>
#include <deque>
#include <string>
#include <sys/types.h>
#include <vector>

namespace ros2_console_tools::tui {

enum ColorPairId {
  kColorFrame = 1,
  kColorTitle = 2,
  kColorHeader = 3,
  kColorSelection = 4,
  kColorStatus = 5,
  kColorHelp = 6,
  kColorPopup = 7,
  kColorInput = 8,
  kColorDirty = 9,
  kColorCursor = 10,
  kColorPositive = 11,
  kColorPositiveSelection = 12,
  kColorWarn = 13,
  kColorError = 14,
  kColorFatal = 15,
  kColorAccent = 16,
  kColorHelpKey = 17,
};

constexpr int kThemeColorCount = kColorHelpKey + 1;

struct ThemeColor {
  short foreground{-1};
  short background{-1};
  int attributes{A_NORMAL};
};

using Theme = std::array<ThemeColor, kThemeColorCount>;

struct SearchState {
  bool active{false};
  std::string query;
};

struct CommanderLayout {
  bool terminal_visible{false};
  int pane_rows{0};
  int content_bottom{0};
  int status_row{0};
  int help_row{0};
  int terminal_top{0};
};

enum class TerminalContext {
  Color,
  Mono,
  Ascii,
};

enum class SearchInputResult {
  None,
  Changed,
  Accepted,
  Cancelled,
};

class Session {
public:
  Session();
  ~Session();

  Session(const Session &) = delete;
  Session & operator=(const Session &) = delete;
};

class TerminalPane {
public:
  TerminalPane();
  ~TerminalPane();

  TerminalPane(const TerminalPane &) = delete;
  TerminalPane & operator=(const TerminalPane &) = delete;

  bool visible() const;
  void toggle();
  void show();
  void hide();
  void update();
  bool handle_key(int key);
  void draw(int top, int left, int bottom, int right);

private:
  enum class EscapeMode {
    None,
    Escape,
    Csi,
    Osc,
  };

  void ensure_process();
  void spawn_process();
  void shutdown_process();
  void reap_process();
  void resize_pty(int rows, int columns);
  void append_line(const std::string & line);
  void process_output_char(char byte);
  std::string encode_key(int key) const;

  bool visible_{false};
  int master_fd_{-1};
  pid_t child_pid_{-1};
  bool child_running_{false};
  int last_rows_{0};
  int last_columns_{0};
  EscapeMode escape_mode_{EscapeMode::None};
  bool osc_saw_escape_{false};
  std::deque<std::string> scrollback_;
  std::string current_line_;
};

Theme make_default_theme();
const Theme & current_theme();
int theme_attr(int role);
void apply_role_chgat(int row, int col, int count, int role, int extra_attributes = A_NORMAL);
void set_theme(const Theme & theme);
std::string default_theme_config_path();
bool load_theme_from_file(const std::string & path, std::string * error = nullptr);
bool is_alt_binding(int key, int expected);
void start_search(SearchState & state);
SearchInputResult handle_search_input(SearchState & state, int key);
int find_best_match(const std::vector<std::string> & labels, const std::string & query, int current_index);
CommanderLayout make_commander_layout(int rows, bool terminal_visible);
std::string with_terminal_help(const std::string & text, bool terminal_visible);

std::string truncate_text(const std::string & text, int width);

bool use_unicode_line_drawing();
TerminalContext terminal_context();
void draw_box_char(int row, int col, const cchar_t * wide_char, char ascii_char);
void draw_text_hline(int row, int col, int count);
void draw_text_vline(int row, int col, int count);
void draw_box(int top, int left, int bottom, int right, int color_pair);
void draw_bar(int row, int columns, const std::string & text, int color_pair, int left_padding = 1);
void draw_status_bar(int row, int columns, const std::string & text);
void draw_help_bar_region(int row, int left, int width, const std::string & text);
void draw_help_bar(int row, int columns, const std::string & text);
void draw_search_box(int rows, int columns, const SearchState & state, const std::string & prompt = "Search");

}  // namespace ros2_console_tools::tui

#endif  // ROS2_CONSOLE_TOOLS__TUI_HPP_
