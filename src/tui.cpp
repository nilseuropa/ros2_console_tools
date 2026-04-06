#include "ros2_console_tools/tui.hpp"

#include <cctype>
#include <clocale>
#include <langinfo.h>
#include <string>
#include <vector>

namespace ros2_console_tools::tui {

namespace {

Theme g_theme = make_default_theme();

struct HelpBlock {
  std::string key;
  std::string label;
};

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

Session::Session() {
  std::setlocale(LC_ALL, "");
  initscr();
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

}  // namespace ros2_console_tools::tui
