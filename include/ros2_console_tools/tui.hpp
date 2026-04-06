#ifndef ROS2_CONSOLE_TOOLS__TUI_HPP_
#define ROS2_CONSOLE_TOOLS__TUI_HPP_

#include <ncursesw/ncurses.h>

#include <string>

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
};

class Session {
public:
  Session();
  ~Session();

  Session(const Session &) = delete;
  Session & operator=(const Session &) = delete;
};

std::string truncate_text(const std::string & text, int width);

bool use_unicode_line_drawing();
void draw_box_char(int row, int col, const cchar_t * wide_char, char ascii_char);
void draw_text_hline(int row, int col, int count);
void draw_text_vline(int row, int col, int count);
void draw_box(int top, int left, int bottom, int right, int color_pair);

}  // namespace ros2_console_tools::tui

#endif  // ROS2_CONSOLE_TOOLS__TUI_HPP_
