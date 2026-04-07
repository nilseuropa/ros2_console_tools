#include <gtest/gtest.h>

#define private public
#include "ros2_console_tools/tui.hpp"
#undef private

namespace ros2_console_tools::tui {
namespace {

TEST(TuiTest, CommanderLayoutUsesTopHalfWhenTerminalVisible) {
  const auto layout = make_commander_layout(40, true);

  EXPECT_TRUE(layout.terminal_visible);
  EXPECT_EQ(layout.terminal_top, 20);
  EXPECT_EQ(layout.pane_rows, 20);
  EXPECT_EQ(layout.status_row, 18);
  EXPECT_EQ(layout.help_row, 19);
  EXPECT_EQ(layout.content_bottom, 17);
}

TEST(TuiTest, CommanderLayoutUsesFullHeightWhenTerminalHidden) {
  const auto layout = make_commander_layout(25, false);

  EXPECT_FALSE(layout.terminal_visible);
  EXPECT_EQ(layout.terminal_top, 25);
  EXPECT_EQ(layout.pane_rows, 25);
  EXPECT_EQ(layout.status_row, 23);
  EXPECT_EQ(layout.help_row, 24);
  EXPECT_EQ(layout.content_bottom, 22);
}

TEST(TuiTest, TerminalHelpReflectsVisibilityState) {
  EXPECT_EQ(with_terminal_help("", false), "Alt+T Terminal");
  EXPECT_EQ(with_terminal_help("", true), "Alt+T Hide");
  EXPECT_EQ(with_terminal_help("F10 Exit", false), "F10 Exit  Alt+T Terminal");
  EXPECT_EQ(with_terminal_help("F10 Exit", true), "F10 Exit  Alt+T Hide");
}

TEST(TuiTest, TerminalPaneScrollsScreenBuffer) {
  TerminalPane pane;
  pane.resize_screen_buffer(3, 6);

  const std::string text = "aaa\r\nbbb\r\nccc\r\nddd";
  for (char ch : text) {
    pane.process_output_char(ch);
  }

  EXPECT_EQ(pane.screen_rows_[0][0].glyph, 'b');
  EXPECT_EQ(pane.screen_rows_[1][0].glyph, 'c');
  EXPECT_EQ(pane.screen_rows_[2][0].glyph, 'd');
}

TEST(TuiTest, TerminalPaneHandlesCarriageReturnAndCursorMotion) {
  TerminalPane pane;
  pane.resize_screen_buffer(2, 5);

  const std::string text = "abc\rZ\x1b[2DXY";
  for (char ch : text) {
    pane.process_output_char(ch);
  }

  EXPECT_EQ(pane.screen_rows_[0][0].glyph, 'X');
  EXPECT_EQ(pane.screen_rows_[0][1].glyph, 'Y');
  EXPECT_EQ(pane.screen_rows_[0][2].glyph, 'c');
}

TEST(TuiTest, TerminalPaneAppliesAnsiSgrColors) {
  TerminalPane pane;
  pane.resize_screen_buffer(1, 4);

  const std::string text = "\x1b[31mR\x1b[0mN";
  for (char ch : text) {
    pane.process_output_char(ch);
  }

  EXPECT_EQ(pane.screen_rows_[0][0].glyph, 'R');
  EXPECT_EQ(pane.screen_rows_[0][0].style.foreground, COLOR_RED);
  EXPECT_EQ(pane.screen_rows_[0][1].glyph, 'N');
  EXPECT_EQ(pane.screen_rows_[0][1].style.foreground, COLOR_WHITE);
}

}  // namespace
}  // namespace ros2_console_tools::tui
