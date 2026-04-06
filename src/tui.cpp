#include "ros2_console_tools/tui.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <algorithm>
#include <cctype>
#include <clocale>
#include <filesystem>
#include <fstream>
#include <langinfo.h>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace ros2_console_tools::tui {

namespace {

Theme g_theme = make_default_theme();

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
  curs_set(0);
  timeout(100);
  initialize_theme();
}

Session::~Session() {
  curs_set(1);
  endwin();
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

  int column = left;
  const int right = left + width;
  bool first_block = true;
  for (const auto & block : parse_help_blocks(text)) {
    if (column >= right) {
      break;
    }

    if (!first_block) {
      attrset(A_NORMAL);
      mvaddch(row, column, ' ');
      ++column;
      if (column >= right) {
        break;
      }
    }

    const std::string key = truncate_text(block.key, std::max(0, right - column));
    if (key.empty()) {
      continue;
    }

    const auto & help_key_color = current_theme()[kColorHelpKey];
    attron(COLOR_PAIR(kColorHelpKey) | help_key_color.attributes);
    mvaddnstr(row, column, key.c_str(), right - column);
    attroff(COLOR_PAIR(kColorHelpKey) | help_key_color.attributes);
    column += static_cast<int>(key.size());
    if (column >= right) {
      break;
    }

    if (!block.label.empty()) {
      const std::string separator = " ";
      attron(COLOR_PAIR(kColorHelp));
      mvaddnstr(row, column, separator.c_str(), right - column);
      attroff(COLOR_PAIR(kColorHelp));
      column += static_cast<int>(separator.size());
      if (column >= right) {
        break;
      }

      const std::string label = truncate_text(block.label, std::max(0, right - column));
      attron(COLOR_PAIR(kColorHelp));
      mvaddnstr(row, column, label.c_str(), right - column);
      attroff(COLOR_PAIR(kColorHelp));
      column += static_cast<int>(label.size());
      if (column >= right) {
        break;
      }

      attron(COLOR_PAIR(kColorHelp));
      mvaddch(row, column, ' ');
      attroff(COLOR_PAIR(kColorHelp));
      ++column;
      if (column >= right) {
        break;
      }
    }
    first_block = false;
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
