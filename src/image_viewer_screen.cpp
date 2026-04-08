#include "ros2_console_tools/image_viewer.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <ncursesw/ncurses.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>

namespace ros2_console_tools {

namespace {

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorHeader = tui::kColorHeader,
};

struct RenderGeometry {
  int width{1};
  int height{1};
  double window_width{1.0};
  double window_height{1.0};
  double offset_x{0.0};
  double offset_y{0.0};
};

using tui::Session;
using tui::draw_box;
using tui::draw_help_bar;
using tui::draw_status_bar;
using tui::terminal_context;
using tui::theme_attr;
using tui::truncate_text;

std::string format_age(const builtin_interfaces::msg::Time & stamp, const rclcpp::Clock & clock) {
  const rclcpp::Time message_time(stamp);
  if (message_time.nanoseconds() <= 0) {
    return "n/a";
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << (clock.now() - message_time).seconds() << 's';
  return stream.str();
}

const char * intensity_glyph(
  uint8_t value, ImageViewerRenderMode mode, tui::TerminalContext context)
{
  if (mode == ImageViewerRenderMode::Auto) {
    mode = context == tui::TerminalContext::Ascii
      ? ImageViewerRenderMode::Ascii
      : ImageViewerRenderMode::Shade;
  }

  if (mode == ImageViewerRenderMode::Ascii) {
    static const char * ramp = " .:-=+*#%@";
    const std::size_t index =
      std::min<std::size_t>(9, static_cast<std::size_t>((static_cast<unsigned int>(value) * 10u) / 256u));
    static thread_local char glyph[2];
    glyph[0] = ramp[index];
    glyph[1] = '\0';
    return glyph;
  }

  static const char * shade_ramp[] = {" ", "░", "▒", "▓", "█"};
  const std::size_t index =
    std::min<std::size_t>(4, static_cast<std::size_t>((static_cast<unsigned int>(value) * 5u) / 256u));
  return shade_ramp[index];
}

std::string render_mode_label(
  ImageViewerRenderMode mode, tui::TerminalContext context)
{
  switch (mode) {
    case ImageViewerRenderMode::Ascii:
      return "ascii";
    case ImageViewerRenderMode::Shade:
      return "shade";
    case ImageViewerRenderMode::Auto:
    default:
      return context == tui::TerminalContext::Ascii ? "auto-ascii" : "auto-shade";
  }
}

RenderGeometry compute_render_geometry(
  int inner_width,
  int inner_height,
  const ImageFrame & frame,
  double zoom_factor,
  int pan_x,
  int pan_y)
{
  const double cell_height_over_width = 2.0;
  const double source_width = static_cast<double>(frame.width);
  const double source_height = static_cast<double>(frame.height);
  const double window_width = std::max(1.0, source_width / std::max(1.0, zoom_factor));
  const double window_height = std::max(1.0, source_height / std::max(1.0, zoom_factor));

  const double max_center_x = source_width - window_width / 2.0;
  const double min_center_x = window_width / 2.0;
  const double max_center_y = source_height - window_height / 2.0;
  const double min_center_y = window_height / 2.0;
  const double center_x = std::clamp(source_width / 2.0 + static_cast<double>(pan_x), min_center_x, max_center_x);
  const double center_y = std::clamp(source_height / 2.0 + static_cast<double>(pan_y), min_center_y, max_center_y);
  const double offset_x = std::clamp(center_x - window_width / 2.0, 0.0, std::max(0.0, source_width - window_width));
  const double offset_y = std::clamp(center_y - window_height / 2.0, 0.0, std::max(0.0, source_height - window_height));

  const double available_width = std::max(1.0, static_cast<double>(inner_width));
  const double available_virtual_height =
    std::max(1.0, static_cast<double>(inner_height) * cell_height_over_width);
  const double source_aspect = window_width / window_height;
  const double available_aspect = available_width / available_virtual_height;

  int render_width = inner_width;
  int render_height = inner_height;
  if (source_aspect > available_aspect) {
    render_width = inner_width;
    render_height = std::max(
      1,
      std::min(
        inner_height,
        static_cast<int>(std::lround(available_width / (source_aspect * cell_height_over_width)))));
  } else {
    render_height = inner_height;
    render_width = std::max(
      1,
      std::min(
        inner_width,
        static_cast<int>(std::lround(source_aspect * available_virtual_height))));
  }

  return {render_width, render_height, window_width, window_height, offset_x, offset_y};
}

uint8_t sample_grayscale(
  const ImageFrame & frame, const RenderGeometry & geometry, int render_x, int render_y)
{
  const double normalized_x =
    geometry.width <= 1 ? 0.0 : static_cast<double>(render_x) / static_cast<double>(geometry.width - 1);
  const double normalized_y =
    geometry.height <= 1 ? 0.0 : static_cast<double>(render_y) / static_cast<double>(geometry.height - 1);
  const int source_x = std::clamp(
    static_cast<int>(std::lround(geometry.offset_x + normalized_x * std::max(0.0, geometry.window_width - 1.0))),
    0,
    std::max(0, static_cast<int>(frame.width) - 1));
  const int source_y = std::clamp(
    static_cast<int>(std::lround(geometry.offset_y + normalized_y * std::max(0.0, geometry.window_height - 1.0))),
    0,
    std::max(0, static_cast<int>(frame.height) - 1));
  return frame.gray8[static_cast<std::size_t>(source_y) * frame.width + static_cast<std::size_t>(source_x)];
}

}  // namespace

int run_image_viewer_tool(const std::string & topic, bool embedded_mode) {
  auto backend = std::make_shared<ImageViewerBackend>(topic);
  ImageViewerScreen screen(backend, embedded_mode);

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

ImageViewerScreen::ImageViewerScreen(
  std::shared_ptr<ImageViewerBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int ImageViewerScreen::run() {
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

bool ImageViewerScreen::handle_key(int key) {
  if (
    embedded_mode_ &&
    std::chrono::steady_clock::now() - startup_time_ < std::chrono::milliseconds(750))
  {
    return true;
  }

  const auto current_frame = backend_->latest_frame_snapshot();
  const int pan_step_x = current_frame
    ? std::max(1, static_cast<int>(std::lround(static_cast<double>(current_frame->width) / zoom_factor_ / 10.0)))
    : 8;
  const int pan_step_y = current_frame
    ? std::max(1, static_cast<int>(std::lround(static_cast<double>(current_frame->height) / zoom_factor_ / 10.0)))
    : 4;

  switch (key) {
    case KEY_F(10):
      return false;
    case 27:
      return false;
    case '+':
    case '=':
      zoom_factor_ = std::min(32.0, zoom_factor_ * 1.5);
      status_line_ = "Zoom " + std::to_string(static_cast<int>(std::lround(zoom_factor_ * 100.0))) + "%.";
      return true;
    case '-':
    case '_':
      zoom_factor_ = std::max(1.0, zoom_factor_ / 1.5);
      if (zoom_factor_ <= 1.01) {
        zoom_factor_ = 1.0;
        pan_x_ = 0;
        pan_y_ = 0;
      }
      status_line_ = "Zoom " + std::to_string(static_cast<int>(std::lround(zoom_factor_ * 100.0))) + "%.";
      return true;
    case 'r':
    case 'R':
      reset_view();
      status_line_ = "View reset.";
      return true;
    case 'i':
    case 'I':
      invert_grayscale_ = !invert_grayscale_;
      status_line_ = invert_grayscale_ ? "Grayscale inverted." : "Grayscale restored.";
      return true;
    case 'm':
    case 'M':
      render_mode_ = static_cast<ImageViewerRenderMode>((static_cast<int>(render_mode_) + 1) % 3);
      status_line_ = "Render mode: " + render_mode_label(render_mode_, terminal_context()) + ".";
      return true;
    case 'f':
    case 'F':
      frozen_ = !frozen_;
      if (frozen_) {
        frozen_frame_ = current_frame;
        status_line_ = frozen_frame_ ? "Frame frozen." : "No frame available to freeze.";
      } else {
        frozen_frame_.reset();
        status_line_ = "Live view restored.";
      }
      return true;
    case KEY_LEFT:
    case 'h':
      pan_x_ -= pan_step_x;
      status_line_ = "Pan left.";
      return true;
    case KEY_RIGHT:
    case 'l':
      pan_x_ += pan_step_x;
      status_line_ = "Pan right.";
      return true;
    case KEY_UP:
    case 'k':
      pan_y_ -= pan_step_y;
      status_line_ = "Pan up.";
      return true;
    case KEY_DOWN:
    case 'j':
      pan_y_ += pan_step_y;
      status_line_ = "Pan down.";
      return true;
    default:
      return true;
  }
}

void ImageViewerScreen::reset_view() {
  zoom_factor_ = 1.0;
  pan_x_ = 0;
  pan_y_ = 0;
}

void ImageViewerScreen::draw_waiting_message(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Image");
  attroff(theme_attr(kColorHeader));
  mvprintw(top + 1, left, "%-*s", width, truncate_text("waiting for image frames...", width).c_str());
  for (int row = top + 2; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void ImageViewerScreen::draw_error_message(
  int top, int left, int bottom, int right, const std::string & error) const
{
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Image");
  attroff(theme_attr(kColorHeader));
  mvprintw(top + 1, left, "%-*s", width, truncate_text("decode error", width).c_str());
  mvprintw(top + 2, left, "%-*s", width, truncate_text(error, width).c_str());
  for (int row = top + 3; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void ImageViewerScreen::draw_image_view(
  int top, int left, int bottom, int right, const ImageFrame & frame) const
{
  const int width = right - left + 1;
  const auto context = terminal_context();
  const auto effective_mode =
    render_mode_ == ImageViewerRenderMode::Auto
    ? (context == tui::TerminalContext::Ascii ? ImageViewerRenderMode::Ascii : ImageViewerRenderMode::Shade)
    : render_mode_;

  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Image");
  attroff(theme_attr(kColorHeader));

  std::ostringstream header_line;
  header_line << "topic=" << backend_->topic_
              << " frame=" << frame.frame_id
              << " age=" << format_age(frame.stamp, *backend_->get_clock())
              << " src=" << frame.width << 'x' << frame.height
              << " enc=" << frame.source_encoding
              << " mode=" << render_mode_label(effective_mode, context)
              << " zoom=" << std::fixed << std::setprecision(1) << zoom_factor_ << 'x'
              << (frozen_ ? " frozen" : "");
  mvprintw(top + 1, left, "%-*s", width, truncate_text(header_line.str(), width).c_str());

  const int image_top = top + 2;
  if (image_top + 2 > bottom) {
    return;
  }

  draw_box(image_top, left, bottom, right, kColorFrame);
  const int inner_left = left + 1;
  const int inner_top = image_top + 1;
  const int inner_width = std::max(1, right - left - 1);
  const int inner_height = std::max(1, bottom - image_top - 1);
  for (int row = inner_top; row < bottom; ++row) {
    mvhline(row, inner_left, ' ', inner_width);
  }

  const RenderGeometry geometry =
    compute_render_geometry(inner_width, inner_height, frame, zoom_factor_, pan_x_, pan_y_);
  const int draw_left = inner_left + std::max(0, (inner_width - geometry.width) / 2);
  const int draw_top = inner_top + std::max(0, (inner_height - geometry.height) / 2);

  for (int render_y = 0; render_y < geometry.height; ++render_y) {
    const int row = draw_top + render_y;
    if (row >= bottom) {
      break;
    }
    for (int render_x = 0; render_x < geometry.width; ++render_x) {
      const int column = draw_left + render_x;
      if (column >= right) {
        break;
      }
      uint8_t value = sample_grayscale(frame, geometry, render_x, render_y);
      if (invert_grayscale_) {
        value = static_cast<uint8_t>(255u - value);
      }
      mvaddstr(row, column, intensity_glyph(value, effective_mode, context));
    }
  }
}

void ImageViewerScreen::draw_status_line(int row, int columns) const {
  draw_status_bar(row, columns, truncate_text(status_line_, columns - 1));
}

void ImageViewerScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(
    row,
    columns,
    embedded_mode_
    ? "Arrows Pan  +/- Zoom  R Reset  I Invert  M Mode  F Freeze  Esc Return  F10 Exit"
    : "Arrows Pan  +/- Zoom  R Reset  I Invert  M Mode  F Freeze  Esc Exit  F10 Exit");
}

void ImageViewerScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = rows - 3;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  mvprintw(0, 1, "Image Viewer ");

  const auto latest_frame = backend_->latest_frame_snapshot();
  const auto latest_error = backend_->latest_error_snapshot();
  const auto frame_to_draw = frozen_ && frozen_frame_ ? frozen_frame_ : latest_frame;
  if (frame_to_draw) {
    draw_image_view(1, 1, content_bottom - 1, columns - 2, *frame_to_draw);
  } else if (!latest_error.empty()) {
    draw_error_message(1, 1, content_bottom - 1, columns - 2, latest_error);
  } else {
    draw_waiting_message(1, 1, content_bottom - 1, columns - 2);
  }

  draw_status_line(status_row, columns);
  draw_help_line(help_row, columns);
  refresh();
}

}  // namespace ros2_console_tools
