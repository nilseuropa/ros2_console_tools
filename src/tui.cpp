#include "ros2_console_tools/tui.hpp"

#include <cctype>
#include <clocale>
#include <langinfo.h>
#include <string>
#include <vector>

namespace ros2_console_tools::tui {

namespace {

struct HelpBlock {
  std::string key;
  std::string label;
};

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
  init_pair(kColorFrame, COLOR_CYAN, -1);
  init_pair(kColorTitle, COLOR_WHITE, -1);
  init_pair(kColorHeader, COLOR_YELLOW, -1);
  init_pair(kColorSelection, COLOR_BLACK, COLOR_YELLOW);
  init_pair(kColorStatus, COLOR_GREEN, -1);
  init_pair(kColorHelp, COLOR_BLACK, COLOR_CYAN);
  init_pair(kColorPopup, COLOR_WHITE, -1);
  init_pair(kColorInput, COLOR_YELLOW, -1);
  init_pair(kColorDirty, COLOR_RED, -1);
  init_pair(kColorCursor, COLOR_BLACK, COLOR_YELLOW);
  init_pair(kColorPositive, COLOR_GREEN, -1);
  init_pair(kColorPositiveSelection, COLOR_GREEN, COLOR_YELLOW);
  init_pair(kColorWarn, COLOR_YELLOW, -1);
  init_pair(kColorError, COLOR_RED, -1);
  init_pair(kColorFatal, COLOR_RED, COLOR_YELLOW);
  init_pair(kColorAccent, COLOR_CYAN, -1);
  init_pair(kColorHelpKey, COLOR_BLACK, COLOR_YELLOW);
}

}  // namespace

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
  attron(COLOR_PAIR(color_pair));
  draw_box_char(top, left, WACS_ULCORNER, '+');
  draw_box_char(top, right, WACS_URCORNER, '+');
  draw_box_char(bottom, left, WACS_LLCORNER, '+');
  draw_box_char(bottom, right, WACS_LRCORNER, '+');
  draw_text_hline(top, left + 1, right - left - 1);
  draw_text_hline(bottom, left + 1, right - left - 1);
  draw_text_vline(top + 1, left, bottom - top - 1);
  draw_text_vline(top + 1, right, bottom - top - 1);
  attroff(COLOR_PAIR(color_pair));
}

void draw_bar(int row, int columns, const std::string & text, int color_pair, int left_padding) {
  attron(COLOR_PAIR(color_pair));
  mvhline(row, 0, ' ', columns);
  const int content_width = std::max(0, columns - left_padding);
  mvaddnstr(row, left_padding, truncate_text(text, content_width).c_str(), content_width);
  attroff(COLOR_PAIR(color_pair));
}

void draw_status_bar(int row, int columns, const std::string & text) {
  draw_bar(row, columns, text, kColorStatus, 1);
}

void draw_help_bar(int row, int columns, const std::string & text) {
  attrset(A_NORMAL);
  mvhline(row, 0, ' ', columns);

  int column = 0;
  bool first_block = true;
  for (const auto & block : parse_help_blocks(text)) {
    if (column >= columns) {
      break;
    }

    if (!first_block) {
      attrset(A_NORMAL);
      mvaddch(row, column, ' ');
      ++column;
      if (column >= columns) {
        break;
      }
    }

    const std::string key = truncate_text(block.key, std::max(0, columns - column));
    if (key.empty()) {
      continue;
    }

    attron(COLOR_PAIR(kColorHelpKey) | A_BOLD);
    mvaddnstr(row, column, key.c_str(), columns - column);
    attroff(COLOR_PAIR(kColorHelpKey) | A_BOLD);
    column += static_cast<int>(key.size());
    if (column >= columns) {
      break;
    }

    if (!block.label.empty()) {
      const std::string separator = " ";
      attron(COLOR_PAIR(kColorHelp));
      mvaddnstr(row, column, separator.c_str(), columns - column);
      attroff(COLOR_PAIR(kColorHelp));
      column += static_cast<int>(separator.size());
      if (column >= columns) {
        break;
      }

      const std::string label = truncate_text(block.label, std::max(0, columns - column));
      attron(COLOR_PAIR(kColorHelp));
      mvaddnstr(row, column, label.c_str(), columns - column);
      attroff(COLOR_PAIR(kColorHelp));
      column += static_cast<int>(label.size());
      if (column >= columns) {
        break;
      }

      attron(COLOR_PAIR(kColorHelp));
      mvaddch(row, column, ' ');
      attroff(COLOR_PAIR(kColorHelp));
      ++column;
      if (column >= columns) {
        break;
      }
    }
    first_block = false;
  }
}

}  // namespace ros2_console_tools::tui
