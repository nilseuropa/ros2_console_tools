#include "ros2_console_tools/tui.hpp"

#include <clocale>
#include <langinfo.h>
#include <string>

namespace ros2_console_tools::tui {

namespace {

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
  draw_bar(row, columns, text, kColorHelp, 0);
}

}  // namespace ros2_console_tools::tui
