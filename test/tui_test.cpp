#include <gtest/gtest.h>

#include "ros2_console_tools/tui.hpp"

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
  EXPECT_EQ(with_terminal_help("", false), "F9 Terminal");
  EXPECT_EQ(with_terminal_help("", true), "F9 Hide");
  EXPECT_EQ(with_terminal_help("F10 Exit", false), "F10 Exit  F9 Terminal");
  EXPECT_EQ(with_terminal_help("F10 Exit", true), "F10 Exit  F9 Hide");
}

}  // namespace
}  // namespace ros2_console_tools::tui
