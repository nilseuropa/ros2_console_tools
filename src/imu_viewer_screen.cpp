#include "ros2_console_tools/imu_viewer.hpp"

#include <geometry_msgs/msg/quaternion.hpp>
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

std::string format_age(const ImuFrame & frame) {
  const auto age_ms =
    std::chrono::duration_cast<std::chrono::milliseconds>(ImuViewerClock::now() - frame.received_time);
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << static_cast<double>(age_ms.count()) / 1000.0 << 's';
  return stream.str();
}

std::string format_value(double value, int precision = 2) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(precision) << value;
  return stream.str();
}

std::string covariance_label(ImuCovarianceState state) {
  switch (state) {
    case ImuCovarianceState::Unknown:
      return "unknown";
    case ImuCovarianceState::Zero:
      return "zero";
    case ImuCovarianceState::Valid:
      return "valid";
  }
  return "unknown";
}

std::string yaw_compass(double yaw_deg) {
  static const char * directions[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  double normalized = std::fmod(yaw_deg, 360.0);
  if (normalized < 0.0) {
    normalized += 360.0;
  }
  const int index = static_cast<int>(std::lround(normalized / 45.0)) % 8;
  return directions[index];
}

}  // namespace

ImuOrientation orientation_from_quaternion(const geometry_msgs::msg::Quaternion & quaternion) {
  const double x = quaternion.x;
  const double y = quaternion.y;
  const double z = quaternion.z;
  const double w = quaternion.w;

  const double sinr_cosp = 2.0 * (w * x + y * z);
  const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  const double roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (w * y - z * x);
  const double pitch = std::abs(sinp) >= 1.0
    ? std::copysign(kPi / 2.0, sinp)
    : std::asin(sinp);

  const double siny_cosp = 2.0 * (w * z + x * y);
  const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  const double yaw = std::atan2(siny_cosp, cosy_cosp);

  return ImuOrientation{
    roll * 180.0 / kPi,
    pitch * 180.0 / kPi,
    yaw * 180.0 / kPi};
}

ImuCovarianceState covariance_state(const std::array<double, 9> & covariance) {
  if (covariance[0] < 0.0) {
    return ImuCovarianceState::Unknown;
  }
  const bool all_zero = std::all_of(
    covariance.begin(), covariance.end(),
    [](double value) { return std::abs(value) <= 1e-12; });
  return all_zero ? ImuCovarianceState::Zero : ImuCovarianceState::Valid;
}

int run_imu_viewer_tool(const std::string & topic, bool embedded_mode) {
  auto backend = std::make_shared<ImuViewerBackend>(topic);
  ImuViewerScreen screen(backend, embedded_mode);

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

ImuViewerScreen::ImuViewerScreen(
  std::shared_ptr<ImuViewerBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int ImuViewerScreen::run() {
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

bool ImuViewerScreen::handle_key(int key) {
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
    default:
      return true;
  }
}

void ImuViewerScreen::draw_waiting_message(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "IMU");
  attroff(theme_attr(kColorHeader));
  mvprintw(
    top + 1, left, "%-*s", width,
    truncate_text("waiting for sensor_msgs/msg/Imu on " + backend_->topic_ + "...", width).c_str());
  for (int row = top + 2; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void ImuViewerScreen::draw_vector_panel(
  int top, int left, int bottom, int right, const std::string & title,
  const std::array<double, 3> & values, double scale) const
{
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, truncate_text(title, width).c_str());
  attroff(theme_attr(kColorHeader));

  static const char * labels[] = {"x", "y", "z"};
  const int label_width = 2;
  const int value_width = 8;
  const int bar_left_offset = label_width + value_width + 1;
  const int bar_width = std::max(1, width - bar_left_offset - 1);

  int row = top + 1;
  for (int index = 0; index < 3 && row <= bottom; ++index, ++row) {
    mvhline(row, left, ' ', width);
    const double normalized = std::clamp(values[static_cast<std::size_t>(index)] / scale, -1.0, 1.0);
    const int center = bar_width / 2;
    const int filled = static_cast<int>(std::lround(std::abs(normalized) * static_cast<double>(center)));
    std::vector<std::string> bar(static_cast<std::size_t>(bar_width), glyph_or_ascii("░", "."));
    bar[static_cast<std::size_t>(center)] = glyph_or_ascii("│", "|");
    if (normalized < 0.0) {
      const int start = std::max(0, center - filled);
      for (int column = start; column < center; ++column) {
        bar[static_cast<std::size_t>(column)] = glyph_or_ascii("█", "#");
      }
      if (filled > 0) {
        bar[static_cast<std::size_t>(start)] = glyph_or_ascii("◄", "<");
      }
    } else {
      const int end = std::min(bar_width - 1, center + filled);
      for (int column = center + 1; column <= end; ++column) {
        bar[static_cast<std::size_t>(column)] = glyph_or_ascii("█", "#");
      }
      if (filled > 0) {
        bar[static_cast<std::size_t>(end)] = glyph_or_ascii("►", ">");
      }
    }

    std::ostringstream line;
    line << labels[index] << " " << std::setw(value_width - 2)
         << format_value(values[static_cast<std::size_t>(index)]) << " ";
    mvaddnstr(row, left, truncate_text(line.str(), bar_left_offset).c_str(), bar_left_offset);
    draw_glyph_cells(row, left + bar_left_offset, bar, bar_width);
  }

  for (; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void ImuViewerScreen::draw_orientation_panel(
  int top, int left, int bottom, int right, const ImuFrame & frame,
  ImuCovarianceState covariance_state) const
{
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Orientation");
  attroff(theme_attr(kColorHeader));

  if (covariance_state == ImuCovarianceState::Unknown) {
    mvprintw(top + 1, left, "%-*s", width, "orientation unavailable");
    for (int row = top + 2; row <= bottom; ++row) {
      mvhline(row, left, ' ', width);
    }
    return;
  }

  const auto orientation = orientation_from_quaternion(frame.message.orientation);
  const std::array<double, 3> values{orientation.roll_deg, orientation.pitch_deg, orientation.yaw_deg};
  static const char * labels[] = {"roll", "pitch", "yaw"};
  int row = top + 1;
  for (int index = 0; index < 3 && row <= bottom; ++index, ++row) {
    mvhline(row, left, ' ', width);
    std::ostringstream line;
    line << std::left << std::setw(7) << labels[index]
         << std::right << std::setw(8) << format_value(values[static_cast<std::size_t>(index)], 1)
         << " deg";
    if (index == 2) {
      line << "  compass: " << yaw_compass(orientation.yaw_deg);
    }
    mvaddnstr(row, left, truncate_text(line.str(), width).c_str(), width);
  }
  for (; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void ImuViewerScreen::draw_tilt_panel(
  int top, int left, int bottom, int right, const ImuFrame & frame,
  ImuCovarianceState covariance_state) const
{
  const int available_width = right - left + 1;
  const int available_height = bottom - top + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", available_width, "Tilt");
  attroff(theme_attr(kColorHeader));

  if (covariance_state == ImuCovarianceState::Unknown || available_height < 5 || available_width < 12) {
    mvprintw(top + 1, left, "%-*s", available_width, "orientation unavailable");
    for (int row = top + 2; row <= bottom; ++row) {
      mvhline(row, left, ' ', available_width);
    }
    return;
  }

  const int available_box_height = available_height - 1;
  const int box_height = std::max(5, std::min(available_box_height, std::max(5, available_width / 2)));
  const int box_width = std::max(12, std::min(available_width, box_height * 2));
  const int box_left = left + std::max(0, (available_width - box_width) / 2);
  const int box_right = box_left + box_width - 1;
  const int box_top = top + 1;
  const int box_bottom = box_top + box_height - 1;
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

  const auto orientation = orientation_from_quaternion(frame.message.orientation);
  const double roll = std::clamp(orientation.roll_deg / 45.0, -1.0, 1.0);
  const double pitch = std::clamp(orientation.pitch_deg / 45.0, -1.0, 1.0);
  const int dot_x = std::clamp(
    center_x + static_cast<int>(std::lround(roll * static_cast<double>(inner_width / 2))),
    inner_left,
    inner_right);
  const int dot_y = std::clamp(
    center_y - static_cast<int>(std::lround(pitch * static_cast<double>(inner_height / 2))),
    inner_top,
    inner_bottom);
  attron(COLOR_PAIR(kColorAccent) | A_BOLD);
  mvaddstr(dot_y, dot_x, glyph_or_ascii("●", "O").c_str());
  attroff(COLOR_PAIR(kColorAccent) | A_BOLD);
}

void ImuViewerScreen::draw_covariance_line(int row, int left, int width, const ImuFrame & frame) const {
  const std::string line =
    "Covariance  orientation: " + covariance_label(covariance_state(frame.message.orientation_covariance)) +
    "       angular velocity: " + covariance_label(covariance_state(frame.message.angular_velocity_covariance)) +
    "       linear accel: " + covariance_label(covariance_state(frame.message.linear_acceleration_covariance));
  mvaddnstr(row, left, truncate_text(line, width).c_str(), width);
}

void ImuViewerScreen::draw_imu_view(int top, int left, int bottom, int right, const ImuFrame & frame) const {
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  std::ostringstream header;
  header << "topic=" << backend_->topic_
         << " frame=" << frame.frame_id
         << " age=" << format_age(frame)
         << (frozen_ ? " frozen" : "");
  mvprintw(top, left, "%-*s", width, truncate_text(header.str(), width).c_str());
  attroff(theme_attr(kColorHeader));

  const int content_top = top + 2;
  const int covariance_row = bottom;
  const int layout_bottom = bottom - 2;
  if (layout_bottom < content_top + 8) {
    draw_vector_panel(
      content_top, left, layout_bottom, right, "Angular Velocity rad/s",
      {frame.message.angular_velocity.x, frame.message.angular_velocity.y, frame.message.angular_velocity.z},
      2.0);
    return;
  }

  const int top_panel_bottom = content_top + 4;
  const int split_x = left + width / 2;
  draw_vector_panel(
    content_top, left, top_panel_bottom, split_x - 1, "Angular Velocity rad/s",
    {frame.message.angular_velocity.x, frame.message.angular_velocity.y, frame.message.angular_velocity.z},
    2.0);
  draw_vector_panel(
    content_top, split_x + 1, top_panel_bottom, right, "Linear Acceleration m/s^2",
    {frame.message.linear_acceleration.x, frame.message.linear_acceleration.y, frame.message.linear_acceleration.z},
    12.0);

  const auto orientation_covariance = covariance_state(frame.message.orientation_covariance);
  const int orientation_top = top_panel_bottom + 2;
  const int orientation_bottom = std::min(layout_bottom, orientation_top + 4);
  draw_orientation_panel(orientation_top, left, orientation_bottom, right, frame, orientation_covariance);

  const int tilt_top = orientation_bottom + 2;
  if (tilt_top <= layout_bottom) {
    draw_tilt_panel(tilt_top, left, layout_bottom, right, frame, orientation_covariance);
  }

  draw_covariance_line(covariance_row, left, width, frame);
}

void ImuViewerScreen::draw_status_line(int row, int columns) const {
  draw_status_bar(row, columns, truncate_text(status_line_, columns - 1));
}

void ImuViewerScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(
    row,
    columns,
    embedded_mode_
    ? "F Freeze  Esc Return  F10 Exit"
    : "F Freeze  Esc Exit  F10 Exit");
}

void ImuViewerScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = rows - 3;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "IMU Viewer ");
  attroff(theme_attr(kColorTitle));

  const auto latest_frame = backend_->latest_frame_snapshot();
  const auto frame_to_draw = frozen_ && frozen_frame_ ? frozen_frame_ : latest_frame;
  if (frame_to_draw) {
    draw_imu_view(1, 1, content_bottom - 1, columns - 2, *frame_to_draw);
    status_line_ = "Last message " + format_age(*frame_to_draw) + " ago.";
  } else {
    draw_waiting_message(1, 1, content_bottom - 1, columns - 2);
  }

  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  refresh();
}

}  // namespace ros2_console_tools
