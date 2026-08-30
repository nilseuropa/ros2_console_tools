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

TEST(TuiTest, TerminalSizeSupportIncludesCompactTerminals) {
  EXPECT_TRUE(terminal_size_supported(18, 40));
  EXPECT_TRUE(terminal_size_supported(24, 60));
  EXPECT_FALSE(terminal_size_supported(17, 80));
  EXPECT_FALSE(terminal_size_supported(24, 39));
}

TEST(TuiTest, FitColumnWidthsNeverExceedsAvailableWidth) {
  const auto widths = fit_column_widths(35, {14, 7, 10}, {4, 1, 2});

  ASSERT_EQ(widths.size(), 3u);
  EXPECT_EQ(widths[0] + widths[1] + widths[2], 35);
  EXPECT_GE(widths[0], 14);
  EXPECT_GE(widths[1], 7);
  EXPECT_GE(widths[2], 10);
}

TEST(TuiTest, FitColumnWidthsCompressesOversizedMinimums) {
  const auto widths = fit_column_widths(12, {14, 7, 10}, {4, 1, 2});

  ASSERT_EQ(widths.size(), 3u);
  EXPECT_EQ(widths[0] + widths[1] + widths[2], 12);
  EXPECT_GT(widths[0], 0);
  EXPECT_GT(widths[1], 0);
  EXPECT_GT(widths[2], 0);
}

TEST(TuiTest, FitSplitWidthPreservesBothPanes) {
  EXPECT_EQ(fit_split_width(38, 34, 14, 12), 26);
  EXPECT_EQ(fit_split_width(58, 28, 14, 12), 28);
}

TEST(TuiTest, ScrollOffsetKeepsSelectionInsideVisibleItemRows) {
  EXPECT_EQ(scroll_offset_for_selection(11, 0, 13), 0);
  EXPECT_EQ(scroll_offset_for_selection(12, 0, 13), 0);
  EXPECT_EQ(scroll_offset_for_selection(13, 0, 13), 1);
  EXPECT_EQ(scroll_offset_for_selection(20, 8, 13), 8);
  EXPECT_EQ(scroll_offset_for_selection(21, 8, 13), 9);
  EXPECT_EQ(scroll_offset_for_selection(4, 8, 13), 4);
}

TEST(TuiTest, BrailleDotMaskUsesUnicodeDotLayout) {
  EXPECT_EQ(braille_dot_mask(0, 0), 0x01);
  EXPECT_EQ(braille_dot_mask(0, 1), 0x02);
  EXPECT_EQ(braille_dot_mask(0, 2), 0x04);
  EXPECT_EQ(braille_dot_mask(0, 3), 0x40);
  EXPECT_EQ(braille_dot_mask(1, 0), 0x08);
  EXPECT_EQ(braille_dot_mask(1, 1), 0x10);
  EXPECT_EQ(braille_dot_mask(1, 2), 0x20);
  EXPECT_EQ(braille_dot_mask(1, 3), 0x80);
  EXPECT_EQ(braille_dot_mask(2, 0), 0x00);
}

TEST(TuiTest, BrailleGlyphEncodesUtf8) {
  EXPECT_EQ(braille_glyph(0x00), " ");
  EXPECT_EQ(braille_glyph(0x01), std::string("\xE2\xA0\x81"));
  EXPECT_EQ(braille_glyph(0xFF), std::string("\xE2\xA3\xBF"));
}

TEST(TuiTest, AddBrailleDotMapsVirtualCoordinatesToCells) {
  std::vector<uint8_t> cells(2);

  add_braille_dot(cells, 2, 1, 0, 0);
  add_braille_dot(cells, 2, 1, 1, 3);
  add_braille_dot(cells, 2, 1, 2, 2);

  EXPECT_EQ(cells[0], 0x81);
  EXPECT_EQ(cells[1], 0x04);
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
