#include "ros2_console_tools/joy_viewer.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <thread>

namespace ros2_console_tools {

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorSelection = tui::kColorSelection,
  kColorPositive = tui::kColorPositive,
  kColorWarn = tui::kColorWarn,
  kColorAccent = tui::kColorAccent,
};

using tui::Session;
using tui::draw_box;
using tui::draw_help_bar;
using tui::draw_status_bar;
using tui::draw_text_hline;
using tui::draw_text_vline;
using tui::terminal_context;
using tui::theme_attr;
using tui::truncate_text;

std::string format_age(const JoyFrame & frame) {
  const auto age_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(JoyViewerClock::now() - frame.received_time);
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << static_cast<double>(age_ms.count()) / 1000.0 << 's';
  return stream.str();
}

std::string format_axis(float value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << value;
  return stream.str();
}

std::string glyph_or_ascii(const char * unicode_glyph, const char * ascii_glyph) {
  return terminal_context() == tui::TerminalContext::Ascii ? ascii_glyph : unicode_glyph;
}

void draw_glyph_cells(int row, int left, const std::vector<std::string> & cells, int max_columns) {
  for (int column = 0; column < max_columns && column < static_cast<int>(cells.size()); ++column) {
    mvaddstr(row, left + column, cells[static_cast<std::size_t>(column)].c_str());
  }
}

void draw_glyph_hline(int row, int left, int count, const std::string & glyph) {
  for (int column = 0; column < count; ++column) {
    mvaddstr(row, left + column, glyph.c_str());
  }
}

void draw_glyph_vline(int top, int column, int count, const std::string & glyph) {
  for (int row = 0; row < count; ++row) {
    mvaddstr(top + row, column, glyph.c_str());
  }
}

}  // namespace

int run_joy_viewer_tool(const std::string & topic, bool embedded_mode) {
  auto backend = std::make_shared<JoyViewerBackend>(topic);
  JoyViewerScreen screen(backend, embedded_mode);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(backend);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const int result = screen.run();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  return result;
}

JoyViewerScreen::JoyViewerScreen(
  std::shared_ptr<JoyViewerBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int JoyViewerScreen::run() {
  std::unique_ptr<Session> ncurses_session;
  if (!embedded_mode_) {
    ncurses_session = std::make_unique<Session>();
  } else {
    curs_set(0);
    keypad(stdscr, TRUE);
    cbreak();
    noecho();
  }

  startup_time_ = std::chrono::steady_clock::now();
  timeout(std::max(1, static_cast<int>(1000.0 / backend_->render_hz_)));
  if (embedded_mode_) {
    flushinp();
    timeout(0);
    while (getch() != ERR) {
    }
    timeout(std::max(1, static_cast<int>(1000.0 / backend_->render_hz_)));
  }

  bool running = true;
  while (running && rclcpp::ok()) {
    draw();
    const int key = getch();
    if (key == ERR) {
      continue;
    }
    running = handle_key(key);
  }

  if (embedded_mode_) {
    timeout(100);
    curs_set(0);
    clear();
    clearok(stdscr, TRUE);
    refresh();
  }

  return 0;
}

bool JoyViewerScreen::handle_key(int key) {
  if (
    embedded_mode_ &&
    std::chrono::steady_clock::now() - startup_time_ < std::chrono::milliseconds(750))
  {
    return true;
  }

  switch (key) {
    case KEY_F(10):
    case 27:
      return false;
    case 'f':
    case 'F': {
      frozen_ = !frozen_;
      if (frozen_) {
        frozen_frame_ = backend_->latest_frame_snapshot();
        status_line_ = frozen_frame_ ? "Frame frozen." : "No frame available to freeze.";
      } else {
        frozen_frame_.reset();
        status_line_ = "Live view restored.";
      }
      return true;
    }
    default:
      return true;
  }
}

void JoyViewerScreen::draw_waiting_message(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Joystick");
  attroff(theme_attr(kColorHeader));
  mvprintw(
    top + 1, left, "%-*s", width,
    truncate_text("waiting for sensor_msgs/msg/Joy on " + backend_->topic_ + "...", width).c_str());
  for (int row = top + 2; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void JoyViewerScreen::draw_axes(int top, int left, int bottom, int right, const JoyFrame & frame) const {
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Axes");
  attroff(theme_attr(kColorHeader));

  const int label_width = std::min(10, std::max(6, width / 4));
  const int value_width = 7;
  const int bar_left_offset = label_width + value_width + 1;
  const int bar_width = std::max(1, width - bar_left_offset - 1);
  int row = top + 1;
  for (std::size_t index = 0; index < frame.axes.size() && row <= bottom; ++index, ++row) {
    mvhline(row, left, ' ', width);
    const float value = std::clamp(frame.axes[index], -1.0f, 1.0f);
    const int center = bar_width / 2;
    const int filled = static_cast<int>(std::lround(std::abs(value) * static_cast<float>(center)));
    std::vector<std::string> bar(static_cast<std::size_t>(bar_width), glyph_or_ascii("░", "."));
    if (center >= 0 && center < bar_width) {
      bar[static_cast<std::size_t>(center)] = glyph_or_ascii("│", "|");
    }
    if (value < 0.0f) {
      const int start = std::max(0, center - filled);
      for (int column = start; column < center; ++column) {
        bar[static_cast<std::size_t>(column)] = glyph_or_ascii("█", "#");
      }
      if (filled > 0) {
        bar[static_cast<std::size_t>(start)] = glyph_or_ascii("◄", "<");
      }
    } else {
      for (int column = center + 1; column <= std::min(bar_width - 1, center + filled); ++column) {
        bar[static_cast<std::size_t>(column)] = glyph_or_ascii("█", "#");
      }
      if (filled > 0) {
        bar[static_cast<std::size_t>(std::min(bar_width - 1, center + filled))] = glyph_or_ascii("►", ">");
      }
    }

    std::ostringstream line;
    line << std::left << std::setw(label_width) << ("axis " + std::to_string(index))
         << std::right << std::setw(value_width - 1) << format_axis(frame.axes[index]) << " ";
    mvaddnstr(row, left, truncate_text(line.str(), bar_left_offset).c_str(), bar_left_offset);
    draw_glyph_cells(row, left + bar_left_offset, bar, bar_width);
  }

  for (; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void JoyViewerScreen::draw_buttons(int top, int left, int bottom, int right, const JoyFrame & frame) const {
  const int width = right - left + 1;
  const int label_width = std::min(10, std::max(6, width / 4));
  const int state_left = left + label_width + 7 + 1;
  const int state_width = std::max(1, right - state_left + 1);
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Buttons");
  attroff(theme_attr(kColorHeader));

  int row = top + 1;
  for (std::size_t index = 0; index < frame.buttons.size() && row <= bottom; ++index, ++row) {
    const bool pressed = frame.buttons[index] != 0;
    mvhline(row, left, ' ', width);
    std::ostringstream name;
    name << "button " << index;
    mvaddnstr(row, left, truncate_text(name.str(), label_width).c_str(), label_width);
    if (pressed) {
      attron(COLOR_PAIR(kColorPositive) | A_BOLD);
    }
    mvaddstr(
      row,
      std::min(state_left, right),
      pressed ? glyph_or_ascii("●", "*").c_str() : glyph_or_ascii("○", "o").c_str());
    if (state_left + 2 <= right) {
      mvaddnstr(row, state_left + 2, pressed ? "pressed" : "released", std::max(0, state_width - 2));
    }
    if (pressed) {
      attroff(COLOR_PAIR(kColorPositive) | A_BOLD);
    }
  }

  for (; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void JoyViewerScreen::draw_stick_pad(
  int top, int left, int bottom, int right, const JoyFrame & frame,
  int x_axis, int y_axis, const std::string & title) const
{
  const int available_width = right - left + 1;
  const int available_height = bottom - top + 1;
  const int available_box_height = available_height - 1;
  const int box_height = std::max(5, std::min(available_box_height, std::max(5, available_width / 2)));
  const int box_width = std::max(10, std::min(available_width, box_height * 2));
  const int box_left = left + std::max(0, (available_width - box_width) / 2);
  const int box_right = box_left + box_width - 1;
  const int box_top = top + 1;
  const int box_bottom = box_top + box_height - 1;
  const int width = box_width;
  attron(theme_attr(kColorHeader));
  mvprintw(top, box_left, "%-*s", width, truncate_text(title, width).c_str());
  attroff(theme_attr(kColorHeader));

  if (available_box_height < 5 || available_width < 10 || x_axis >= static_cast<int>(frame.axes.size()) ||
    y_axis >= static_cast<int>(frame.axes.size()))
  {
    for (int row = top + 1; row <= bottom; ++row) {
      mvhline(row, left, ' ', available_width);
    }
    return;
  }

  draw_box(box_top, box_left, box_bottom, box_right, kColorFrame);
  const int inner_left = box_left + 1;
  const int inner_right = box_right - 1;
  const int inner_top = box_top + 1;
  const int inner_bottom = box_bottom - 1;
  const int inner_width = std::max(1, inner_right - inner_left + 1);
  const int inner_height = std::max(1, inner_bottom - inner_top + 1);
  for (int row = inner_top; row <= inner_bottom; ++row) {
    mvhline(row, inner_left, ' ', inner_width);
  }

  const int center_x = inner_left + inner_width / 2;
  const int center_y = inner_top + inner_height / 2;
  attron(COLOR_PAIR(kColorFrame));
  draw_glyph_hline(center_y, inner_left, inner_width, glyph_or_ascii("┄", "-"));
  draw_glyph_vline(inner_top, center_x, inner_height, glyph_or_ascii("┊", ":"));
  mvaddstr(center_y, center_x, glyph_or_ascii("┼", "+").c_str());
  mvaddstr(center_y, box_left, glyph_or_ascii("├", "+").c_str());
  mvaddstr(center_y, box_right, glyph_or_ascii("┤", "+").c_str());
  mvaddstr(box_top, center_x, glyph_or_ascii("┬", "+").c_str());
  mvaddstr(box_bottom, center_x, glyph_or_ascii("┴", "+").c_str());
  attroff(COLOR_PAIR(kColorFrame));

  const float x = std::clamp(frame.axes[static_cast<std::size_t>(x_axis)], -1.0f, 1.0f);
  const float y = std::clamp(frame.axes[static_cast<std::size_t>(y_axis)], -1.0f, 1.0f);
  const int dot_x = std::clamp(
    center_x - static_cast<int>(std::lround(x * static_cast<float>(inner_width / 2))),
    inner_left,
    inner_right);
  const int dot_y = std::clamp(
    center_y - static_cast<int>(std::lround(y * static_cast<float>(inner_height / 2))),
    inner_top,
    inner_bottom);
  attron(COLOR_PAIR(kColorAccent) | A_BOLD);
  mvaddstr(dot_y, dot_x, glyph_or_ascii("●", "O").c_str());
  attroff(COLOR_PAIR(kColorAccent) | A_BOLD);
}

void JoyViewerScreen::draw_joy_view(int top, int left, int bottom, int right, const JoyFrame & frame) const {
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  std::ostringstream header;
  header << "topic=" << backend_->topic_
         << " age=" << format_age(frame)
         << " axes=" << frame.axes.size()
         << " buttons=" << frame.buttons.size()
         << (frozen_ ? " frozen" : "");
  mvprintw(top, left, "%-*s", width, truncate_text(header.str(), width).c_str());
  attroff(theme_attr(kColorHeader));

  const int pane_top = top + 2;
  if (width < 30) {
    draw_axes(pane_top, left, bottom, right, frame);
    return;
  }

  const int left_pane_width = width >= 80 ? std::max(34, width / 2) : std::max(24, width / 2);
  const int split_x = std::clamp(left + left_pane_width, left + 12, right - 12);

  const int left_bottom = bottom;
  const int left_height = std::max(1, left_bottom - pane_top + 1);
  const int axes_rows = std::min(
    std::max(4, static_cast<int>(frame.axes.size()) + 1),
    std::max(3, left_height / 2));
  const int axes_bottom = std::min(left_bottom, pane_top + axes_rows - 1);
  const int buttons_top = std::min(left_bottom, axes_bottom + 2);

  draw_axes(pane_top, left, axes_bottom, split_x - 1, frame);
  if (buttons_top <= left_bottom) {
    draw_buttons(buttons_top, left, left_bottom, split_x - 1, frame);
  }

  attron(COLOR_PAIR(kColorFrame));
  draw_text_vline(pane_top, split_x, bottom - pane_top + 1);
  attroff(COLOR_PAIR(kColorFrame));

  draw_stick_pad(pane_top, split_x + 1, bottom, right, frame, 0, 1, "Main stick axes 0/1");
}

void JoyViewerScreen::draw_status_line(int row, int columns) const {
  draw_status_bar(row, columns, truncate_text(status_line_, columns - 1));
}

void JoyViewerScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(
    row,
    columns,
    embedded_mode_
    ? "F Freeze  Esc Return  F10 Exit"
    : "F Freeze  Esc Exit  F10 Exit");
}

void JoyViewerScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = rows - 3;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "Joy Viewer ");
  attroff(theme_attr(kColorTitle));

  const auto latest_frame = backend_->latest_frame_snapshot();
  const auto frame_to_draw = frozen_ && frozen_frame_ ? frozen_frame_ : latest_frame;
  if (frame_to_draw) {
    draw_joy_view(1, 1, content_bottom - 1, columns - 2, *frame_to_draw);
    status_line_ = "Last message " + format_age(*frame_to_draw) + " ago.";
  } else {
    draw_waiting_message(1, 1, content_bottom - 1, columns - 2);
  }

  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  refresh();
}

}  // namespace ros2_console_tools
