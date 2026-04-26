#include "ros2_console_tools/laser_scan_viewer.hpp"

#include <ncursesw/ncurses.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include <vector>

namespace ros2_console_tools {

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorHeader = tui::kColorHeader,
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

constexpr double kPi = 3.14159265358979323846;

std::string glyph_or_ascii(const char * unicode_glyph, const char * ascii_glyph) {
  return terminal_context() == tui::TerminalContext::Ascii ? ascii_glyph : unicode_glyph;
}

LaserScanViewerRenderMode effective_render_mode(
  LaserScanViewerRenderMode mode, tui::TerminalContext context)
{
  if (mode == LaserScanViewerRenderMode::Auto) {
    return context == tui::TerminalContext::Ascii
      ? LaserScanViewerRenderMode::Points
      : LaserScanViewerRenderMode::Braille;
  }
  if (context == tui::TerminalContext::Ascii && mode == LaserScanViewerRenderMode::Braille) {
    return LaserScanViewerRenderMode::Points;
  }
  return mode;
}

std::string render_mode_label(LaserScanViewerRenderMode mode, tui::TerminalContext context) {
  switch (mode) {
    case LaserScanViewerRenderMode::Braille:
      return context == tui::TerminalContext::Ascii ? "braille-points" : "braille";
    case LaserScanViewerRenderMode::Points:
      return "points";
    case LaserScanViewerRenderMode::Auto:
    default:
      return context == tui::TerminalContext::Ascii ? "auto-points" : "auto-braille";
  }
}

std::string format_age(const LaserScanFrame & frame) {
  const auto age_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(LaserScanViewerClock::now() - frame.received_time);
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << static_cast<double>(age_ms.count()) / 1000.0 << 's';
  return stream.str();
}

std::string format_float(double value, int precision = 2) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

bool range_is_valid(float range, const LaserScanFrame & frame) {
  return std::isfinite(range) && range >= frame.range_min && range <= frame.range_max;
}

double angle_span_degrees(const LaserScanFrame & frame) {
  return static_cast<double>(frame.angle_max - frame.angle_min) * 180.0 / kPi;
}

double base_view_radius(const LaserScanFrame & frame, const LaserScanStats & stats) {
  if (std::isfinite(frame.range_max) && frame.range_max > frame.range_min) {
    return static_cast<double>(frame.range_max);
  }
  if (stats.valid_count > 0 && stats.max_range > 0.0f) {
    return static_cast<double>(stats.max_range);
  }
  return 1.0;
}

void draw_axis_labels(int center_row, int center_column, int top, int left, int bottom, int right) {
  attron(theme_attr(kColorFrame));
  if (center_row >= top && center_row <= bottom) {
    draw_text_hline(center_row, left, right - left + 1);
  }
  if (center_column >= left && center_column <= right) {
    draw_text_vline(top, center_column, bottom - top + 1);
  }
  mvaddstr(center_row, center_column, glyph_or_ascii("┼", "+").c_str());
  attroff(theme_attr(kColorFrame));
}

}  // namespace

LaserScanStats compute_laser_scan_stats(const LaserScanFrame & frame) {
  LaserScanStats stats;
  stats.total_count = frame.ranges.size();
  float sum = 0.0f;
  stats.min_range = std::numeric_limits<float>::infinity();
  stats.max_range = 0.0f;

  for (const float range : frame.ranges) {
    if (!range_is_valid(range, frame)) {
      ++stats.invalid_count;
      continue;
    }
    ++stats.valid_count;
    stats.min_range = std::min(stats.min_range, range);
    stats.max_range = std::max(stats.max_range, range);
    sum += range;
  }

  if (stats.valid_count == 0) {
    stats.min_range = 0.0f;
    stats.max_range = 0.0f;
    stats.average_range = 0.0f;
  } else {
    stats.average_range = sum / static_cast<float>(stats.valid_count);
  }
  return stats;
}

int run_laser_scan_viewer_tool(const std::string & topic, bool embedded_mode) {
  auto backend = std::make_shared<LaserScanViewerBackend>(topic);
  LaserScanViewerScreen screen(backend, embedded_mode);

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

LaserScanViewerScreen::LaserScanViewerScreen(
  std::shared_ptr<LaserScanViewerBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int LaserScanViewerScreen::run() {
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

bool LaserScanViewerScreen::handle_key(int key) {
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
    case '+':
    case '=':
      zoom_factor_ = std::min(32.0, zoom_factor_ * 1.5);
      status_line_ = "Range zoom " + std::to_string(static_cast<int>(std::lround(zoom_factor_ * 100.0))) + "%.";
      return true;
    case '-':
    case '_':
      zoom_factor_ = std::max(1.0, zoom_factor_ / 1.5);
      status_line_ = "Range zoom " + std::to_string(static_cast<int>(std::lround(zoom_factor_ * 100.0))) + "%.";
      return true;
    case 'r':
    case 'R':
      reset_view();
      status_line_ = "View reset.";
      return true;
    case 'i':
    case 'I':
      show_invalid_ = !show_invalid_;
      status_line_ = show_invalid_ ? "Invalid range markers enabled." : "Invalid range markers hidden.";
      return true;
    case 'f':
    case 'F':
      frozen_ = !frozen_;
      if (frozen_) {
        frozen_frame_ = backend_->latest_frame_snapshot();
        status_line_ = frozen_frame_ ? "Frame frozen." : "No frame available to freeze.";
      } else {
        frozen_frame_.reset();
        status_line_ = "Live view restored.";
      }
      return true;
    case 'm':
    case 'M':
      render_mode_ = static_cast<LaserScanViewerRenderMode>((static_cast<int>(render_mode_) + 1) % 3);
      status_line_ = "Render mode: " + render_mode_label(render_mode_, terminal_context()) + ".";
      return true;
    default:
      return true;
  }
}

void LaserScanViewerScreen::reset_view() {
  zoom_factor_ = 1.0;
}

void LaserScanViewerScreen::draw_waiting_message(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "LaserScan");
  attroff(theme_attr(kColorHeader));
  mvprintw(
    top + 1, left, "%-*s", width,
    truncate_text("waiting for sensor_msgs/msg/LaserScan on " + backend_->topic_ + "...", width).c_str());
  for (int row = top + 2; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void LaserScanViewerScreen::draw_scan_view(
  int top, int left, int bottom, int right, const LaserScanFrame & frame) const
{
  const int width = right - left + 1;
  const LaserScanStats stats = compute_laser_scan_stats(frame);
  const auto context = terminal_context();
  const auto effective_mode = effective_render_mode(render_mode_, context);

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "LaserScan");
  attroff(theme_attr(kColorHeader));

  std::ostringstream header_line;
  header_line << "topic=" << backend_->topic_
              << " frame=" << frame.frame_id
              << " age=" << format_age(frame)
              << " beams=" << frame.ranges.size()
              << " valid=" << stats.valid_count
              << " span=" << format_float(angle_span_degrees(frame), 1) << "deg"
              << " zoom=" << std::fixed << std::setprecision(1) << zoom_factor_ << 'x'
              << " mode=" << render_mode_label(render_mode_, context)
              << (frozen_ ? " frozen" : "");
  mvprintw(top + 1, left, "%-*s", width, truncate_text(header_line.str(), width).c_str());

  const int stats_height = 4;
  const int stats_top = std::max(top + 2, bottom - stats_height + 1);
  const int plot_bottom = stats_top - 1;
  if (top + 4 > plot_bottom || left + 4 > right) {
    draw_stats_panel(top + 2, left, bottom, right, frame, stats);
    return;
  }

  draw_box(top + 2, left, plot_bottom, right, kColorFrame);
  const int inner_left = left + 1;
  const int inner_right = right - 1;
  const int inner_top = top + 3;
  const int inner_bottom = plot_bottom - 1;
  const int inner_width = std::max(1, inner_right - inner_left + 1);
  const int inner_height = std::max(1, inner_bottom - inner_top + 1);
  for (int row = inner_top; row <= inner_bottom; ++row) {
    mvhline(row, inner_left, ' ', inner_width);
  }

  const int center_column = inner_left + inner_width / 2;
  const int center_row = inner_top + inner_height / 2;
  draw_axis_labels(center_row, center_column, inner_top, inner_left, inner_bottom, inner_right);

  const double radius = std::max(0.1, base_view_radius(frame, stats) / std::max(1.0, zoom_factor_));
  const double cell_height_over_width = 2.0;
  const double horizontal_limit_cells = std::max(1.0, static_cast<double>(inner_width - 1) / 2.0);
  const double vertical_limit_cells =
    std::max(1.0, static_cast<double>(inner_height - 1) * cell_height_over_width / 2.0);
  const double radius_cells = std::min(horizontal_limit_cells, vertical_limit_cells);

  if (effective_mode == LaserScanViewerRenderMode::Braille) {
    std::vector<uint8_t> valid_cells(
      static_cast<std::size_t>(inner_width) * static_cast<std::size_t>(inner_height));
    std::vector<uint8_t> invalid_cells(valid_cells.size());
    const int virtual_width = std::max(1, inner_width * 2);
    const int virtual_height = std::max(1, inner_height * 4);
    const double virtual_center_column = static_cast<double>(virtual_width - 1) / 2.0;
    const double virtual_center_row = static_cast<double>(virtual_height - 1) / 2.0;
    const double virtual_radius =
      std::min(virtual_center_column, virtual_center_row);

    for (std::size_t index = 0; index < frame.ranges.size(); ++index) {
      const float raw_range = frame.ranges[index];
      const bool valid = range_is_valid(raw_range, frame);
      if (!valid && !show_invalid_) {
        continue;
      }

      const double range = valid ? static_cast<double>(raw_range) : radius;
      if (valid && range > radius) {
        continue;
      }
      const double angle =
        static_cast<double>(frame.angle_min) + static_cast<double>(index) * static_cast<double>(frame.angle_increment);
      const double normalized = std::clamp(range / radius, 0.0, 1.0);
      const double forward = std::cos(angle) * normalized;
      const double leftward = std::sin(angle) * normalized;
      const int virtual_column = std::clamp(
        static_cast<int>(std::lround(virtual_center_column + leftward * virtual_radius)),
        0,
        virtual_width - 1);
      const int virtual_row = std::clamp(
        static_cast<int>(std::lround(virtual_center_row + forward * virtual_radius)),
        0,
        virtual_height - 1);
      tui::add_braille_dot(
        valid ? valid_cells : invalid_cells,
        inner_width,
        inner_height,
        virtual_column,
        virtual_row);
    }

    attron(theme_attr(kColorPositive));
    for (int row_offset = 0; row_offset < inner_height; ++row_offset) {
      for (int column_offset = 0; column_offset < inner_width; ++column_offset) {
        const auto cell_index = static_cast<std::size_t>(row_offset * inner_width + column_offset);
        if (valid_cells[cell_index] != 0) {
          mvaddstr(
            inner_top + row_offset,
            inner_left + column_offset,
            tui::braille_glyph(valid_cells[cell_index]).c_str());
        }
      }
    }
    attroff(theme_attr(kColorPositive));

    attron(theme_attr(kColorWarn));
    for (int row_offset = 0; row_offset < inner_height; ++row_offset) {
      for (int column_offset = 0; column_offset < inner_width; ++column_offset) {
        const auto cell_index = static_cast<std::size_t>(row_offset * inner_width + column_offset);
        if (invalid_cells[cell_index] != 0) {
          const uint8_t merged_mask = static_cast<uint8_t>(invalid_cells[cell_index] | valid_cells[cell_index]);
          mvaddstr(
            inner_top + row_offset,
            inner_left + column_offset,
            tui::braille_glyph(merged_mask).c_str());
        }
      }
    }
    attroff(theme_attr(kColorWarn));
  } else {
    for (std::size_t index = 0; index < frame.ranges.size(); ++index) {
      const float raw_range = frame.ranges[index];
      const bool valid = range_is_valid(raw_range, frame);
      if (!valid && !show_invalid_) {
        continue;
      }

      const double range = valid ? static_cast<double>(raw_range) : radius;
      if (valid && range > radius) {
        continue;
      }
      const double angle =
        static_cast<double>(frame.angle_min) + static_cast<double>(index) * static_cast<double>(frame.angle_increment);
      const double normalized = std::clamp(range / radius, 0.0, 1.0);
      const double forward = std::cos(angle) * normalized;
      const double leftward = std::sin(angle) * normalized;
      const int column =
        std::clamp(
          center_column + static_cast<int>(std::lround(leftward * radius_cells)),
          inner_left,
          inner_right);
      const int row =
        std::clamp(
          center_row + static_cast<int>(std::lround(forward * radius_cells / cell_height_over_width)),
          inner_top,
          inner_bottom);

      if (valid) {
        attron(theme_attr(kColorPositive));
        mvaddstr(row, column, glyph_or_ascii("•", "*").c_str());
        attroff(theme_attr(kColorPositive));
      } else {
        attron(theme_attr(kColorWarn));
        mvaddstr(row, column, glyph_or_ascii("×", "x").c_str());
        attroff(theme_attr(kColorWarn));
      }
    }
  }

  attron(theme_attr(kColorAccent));
  mvaddstr(center_row, center_column, glyph_or_ascii("◆", "o").c_str());
  attroff(theme_attr(kColorAccent));

  const std::string range_label = "view radius " + format_float(radius) + " m";
  mvprintw(
    plot_bottom, left + 2, "%-*s", std::max(0, width - 4),
    truncate_text(range_label, std::max(0, width - 4)).c_str());

  draw_stats_panel(stats_top, left, bottom, right, frame, stats);
}

void LaserScanViewerScreen::draw_stats_panel(
  int top, int left, int bottom, int right, const LaserScanFrame & frame,
  const LaserScanStats & stats) const
{
  const int width = right - left + 1;
  if (top > bottom) {
    return;
  }
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Scan Stats");
  attroff(theme_attr(kColorHeader));

  const std::string invalid_count = std::to_string(stats.invalid_count);
  std::ostringstream range_line;
  range_line << "range min/avg/max "
             << format_float(stats.min_range) << " / "
             << format_float(stats.average_range) << " / "
             << format_float(stats.max_range) << " m"
             << "  limits " << format_float(frame.range_min) << ".." << format_float(frame.range_max) << " m";
  if (top + 1 <= bottom) {
    mvprintw(top + 1, left, "%-*s", width, truncate_text(range_line.str(), width).c_str());
  }

  std::ostringstream timing_line;
  timing_line << "angle inc " << format_float(static_cast<double>(frame.angle_increment) * 180.0 / kPi, 3)
              << " deg  scan " << format_float(frame.scan_time, 3)
              << " s  time inc " << format_float(frame.time_increment, 6) << " s";
  if (top + 2 <= bottom) {
    mvprintw(top + 2, left, "%-*s", width, truncate_text(timing_line.str(), width).c_str());
  }

  std::ostringstream health_line;
  health_line << "invalid " << invalid_count
              << "  intensities " << frame.intensities.size()
              << "  invalid markers " << (show_invalid_ ? "on" : "off");
  if (top + 3 <= bottom) {
    mvprintw(top + 3, left, "%-*s", width, truncate_text(health_line.str(), width).c_str());
  }

  for (int row = top + 4; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void LaserScanViewerScreen::draw_status_line(int row, int columns) const {
  draw_status_bar(row, columns, truncate_text(status_line_, columns - 1));
}

void LaserScanViewerScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(
    row,
    columns,
    embedded_mode_
    ? "+/- Range Zoom  R Reset  I Invalids  M Mode  F Freeze  Esc Return  F10 Exit"
    : "+/- Range Zoom  R Reset  I Invalids  M Mode  F Freeze  Esc Exit  F10 Exit");
}

void LaserScanViewerScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = rows - 3;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  mvprintw(0, 1, "LaserScan Viewer ");

  const auto latest_frame = backend_->latest_frame_snapshot();
  const auto frame_to_draw = frozen_ && frozen_frame_ ? frozen_frame_ : latest_frame;
  if (frame_to_draw) {
    draw_scan_view(1, 1, content_bottom - 1, columns - 2, *frame_to_draw);
  } else {
    draw_waiting_message(1, 1, content_bottom - 1, columns - 2);
  }

  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  refresh();
}

}  // namespace ros2_console_tools
