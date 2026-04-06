#include "ros2_console_tools/map_viewer.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <ncursesw/ncurses.h>

#include <algorithm>
#include <array>
#include <iomanip>
#include <memory>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

namespace ros2_console_tools {

namespace {

using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

constexpr int kColorCellWidth = 2;
constexpr int kAsciiCellWidth = 1;
constexpr int kHeaderRows = 2;
constexpr int kBorderRows = 2;
constexpr int8_t kOccupancyUnknown = -1;
constexpr int8_t kOccupancyFreeThreshold = 0;
constexpr int8_t kCostLowThreshold = 25;
constexpr int8_t kCostMediumThreshold = 50;
constexpr int8_t kCostHighThreshold = 75;

enum class CostBucket : std::size_t {
  Free = 0,
  Low = 1,
  Medium = 2,
  High = 3,
  Lethal = 4,
  Unknown = 5,
};

struct AggregatedCell {
  int8_t occupancy_value{kOccupancyUnknown};
  bool initialized{false};
};

struct RenderGeometry {
  int width{1};
  int height{1};
  int cell_width{1};
};

enum ColorPairId {
  kColorFrame = tui::kColorFrame,
  kColorTitle = tui::kColorTitle,
  kColorHeader = tui::kColorHeader,
  kColorAccent = tui::kColorAccent,
  kColorWarn = tui::kColorWarn,
  kColorError = tui::kColorError,
};

using tui::Session;
using tui::draw_box;
using tui::draw_help_bar;
using tui::draw_status_bar;
using tui::terminal_context;
using tui::theme_attr;
using tui::truncate_text;

bool rotation_swaps_axes(int rotation_degrees) {
  return rotation_degrees == 90 || rotation_degrees == 270;
}

std::pair<std::size_t, std::size_t> rotated_dimensions(
  std::size_t source_width, std::size_t source_height, int rotation_degrees)
{
  if (rotation_swaps_axes(rotation_degrees)) {
    return {source_height, source_width};
  }
  return {source_width, source_height};
}

int8_t normalize_occupancy_value(int8_t occupancy_value) {
  if (occupancy_value < kOccupancyUnknown) {
    return kOccupancyUnknown;
  }
  if (occupancy_value > 100) {
    return 100;
  }
  return occupancy_value;
}

int occupancy_severity(int8_t occupancy_value) {
  const int8_t normalized_value = normalize_occupancy_value(occupancy_value);
  if (normalized_value == kOccupancyUnknown) {
    return 1;
  }
  return 2 + normalized_value;
}

CostBucket cost_bucket(int8_t occupancy_value) {
  const int8_t normalized_value = normalize_occupancy_value(occupancy_value);
  if (normalized_value == kOccupancyUnknown) {
    return CostBucket::Unknown;
  }
  if (normalized_value <= kOccupancyFreeThreshold) {
    return CostBucket::Free;
  }
  if (normalized_value <= kCostLowThreshold) {
    return CostBucket::Low;
  }
  if (normalized_value <= kCostMediumThreshold) {
    return CostBucket::Medium;
  }
  if (normalized_value <= kCostHighThreshold) {
    return CostBucket::High;
  }
  return CostBucket::Lethal;
}

const char * bucket_glyph(CostBucket bucket, tui::TerminalContext context) {
  if (context == tui::TerminalContext::Ascii) {
    switch (bucket) {
      case CostBucket::Free:
        return ".";
      case CostBucket::Low:
        return ":";
      case CostBucket::Medium:
        return "=";
      case CostBucket::High:
        return "#";
      case CostBucket::Lethal:
        return "@";
      case CostBucket::Unknown:
      default:
        return "?";
    }
  }
  switch (bucket) {
    case CostBucket::Free:
      return "·";
    case CostBucket::Low:
      return "░";
    case CostBucket::Medium:
      return "▒";
    case CostBucket::High:
      return "▓";
    case CostBucket::Lethal:
      return "█";
    case CostBucket::Unknown:
    default:
      return "·";
  }
}

int bucket_color(CostBucket bucket, bool monochrome) {
  if (monochrome) {
    return 0;
  }
  switch (bucket) {
    case CostBucket::Low:
      return kColorAccent;
    case CostBucket::Medium:
      return kColorWarn;
    case CostBucket::High:
      return kColorHeader;
    case CostBucket::Lethal:
      return kColorError;
    case CostBucket::Unknown:
      return kColorFrame;
    case CostBucket::Free:
    default:
      return 0;
  }
}

std::string format_age(const builtin_interfaces::msg::Time & stamp, const rclcpp::Clock & clock) {
  const rclcpp::Time message_time(stamp);
  if (message_time.nanoseconds() <= 0) {
    return "n/a";
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << (clock.now() - message_time).seconds() << 's';
  return stream.str();
}

tui::TerminalContext render_context() {
  const auto context = terminal_context();
  return context;
}

RenderGeometry compute_render_geometry(
  int rows,
  int columns,
  int top,
  int left,
  int bottom,
  int right,
  std::size_t source_width,
  std::size_t source_height,
  int rotation_degrees,
  int max_width,
  int max_height,
  tui::TerminalContext context)
{
  (void)rows;
  (void)columns;
  const auto [rotated_width, rotated_height] =
    rotated_dimensions(source_width, source_height, rotation_degrees);
  const int cell_width = context == tui::TerminalContext::Ascii ? kAsciiCellWidth : kColorCellWidth;
  const int available_columns = std::max(cell_width, right - left - 1);
  const int available_rows = std::max(1, bottom - top - 1);

  int render_width = rotated_width == 0 ? 1 : static_cast<int>(rotated_width);
  int render_height = rotated_height == 0 ? 1 : static_cast<int>(rotated_height);
  render_width = std::min(render_width, std::max(1, available_columns / cell_width));
  render_height = std::min(render_height, available_rows);
  if (max_width > 0) {
    render_width = std::min(render_width, max_width);
  }
  if (max_height > 0) {
    render_height = std::min(render_height, max_height);
  }

  return {std::max(1, render_width), std::max(1, render_height), cell_width};
}

void maybe_replace_cell(AggregatedCell & target_cell, int8_t occupancy_value) {
  if (!target_cell.initialized) {
    target_cell.occupancy_value = occupancy_value;
    target_cell.initialized = true;
    return;
  }

  const int incoming_severity = occupancy_severity(occupancy_value);
  const int current_severity = occupancy_severity(target_cell.occupancy_value);
  if (incoming_severity > current_severity ||
    (incoming_severity == current_severity && occupancy_value > target_cell.occupancy_value))
  {
    target_cell.occupancy_value = occupancy_value;
  }
}

void aggregate_cells(
  const OccupancyGrid & message,
  const RenderGeometry & geometry,
  int rotation_degrees,
  std::vector<AggregatedCell> & aggregated_cells)
{
  const std::size_t source_width = static_cast<std::size_t>(message.info.width);
  const std::size_t source_height = static_cast<std::size_t>(message.info.height);
  const auto [rotated_width, rotated_height] =
    rotated_dimensions(source_width, source_height, rotation_degrees);
  const std::size_t normalized_rotated_width = std::max<std::size_t>(1, rotated_width);
  const std::size_t normalized_rotated_height = std::max<std::size_t>(1, rotated_height);

  for (std::size_t source_y = 0; source_y < source_height; ++source_y) {
    for (std::size_t source_x = 0; source_x < source_width; ++source_x) {
      std::size_t rotated_x = source_x;
      std::size_t rotated_y = source_y;
      switch (rotation_degrees) {
        case 90:
          rotated_x = source_height - 1 - source_y;
          rotated_y = source_x;
          break;
        case 180:
          rotated_x = source_width - 1 - source_x;
          rotated_y = source_height - 1 - source_y;
          break;
        case 270:
          rotated_x = source_y;
          rotated_y = source_width - 1 - source_x;
          break;
        default:
          break;
      }

      const int render_x = std::min(
        geometry.width - 1,
        static_cast<int>((rotated_x * static_cast<std::size_t>(geometry.width)) / normalized_rotated_width));
      const int render_y = std::min(
        geometry.height - 1,
        static_cast<int>((rotated_y * static_cast<std::size_t>(geometry.height)) / normalized_rotated_height));
      const auto source_index = source_y * source_width + source_x;
      const auto render_index = static_cast<std::size_t>(render_y * geometry.width + render_x);
      const int8_t occupancy_value =
        source_index < message.data.size() ? normalize_occupancy_value(message.data[source_index]) : kOccupancyUnknown;
      maybe_replace_cell(aggregated_cells[render_index], occupancy_value);
    }
  }
}

}  // namespace

int run_map_viewer_tool(const std::string & topic, bool embedded_mode) {
  auto backend = std::make_shared<MapViewerBackend>(topic);
  MapViewerScreen screen(backend, embedded_mode);

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

MapViewerScreen::MapViewerScreen(std::shared_ptr<MapViewerBackend> backend, bool embedded_mode)
: backend_(std::move(backend)),
  embedded_mode_(embedded_mode) {}

int MapViewerScreen::run() {
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

bool MapViewerScreen::handle_key(int key) {
  if (
    embedded_mode_ &&
    std::chrono::steady_clock::now() - startup_time_ < std::chrono::milliseconds(750))
  {
    return true;
  }

  switch (key) {
    case KEY_F(10):
      return false;
    case 27:
      if (embedded_mode_) {
        return false;
      }
      return false;
    default:
      return true;
  }
}

void MapViewerScreen::draw_waiting_message(int top, int left, int bottom, int right) const {
  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Map");
  attroff(theme_attr(kColorHeader));
  mvprintw(top + 1, left, "%-*s", width, truncate_text("waiting for occupancy grid...", width).c_str());
  for (int row = top + 2; row <= bottom; ++row) {
    mvhline(row, left, ' ', width);
  }
}

void MapViewerScreen::draw_legend_line(int row, int left, int width) const {
  const auto context = render_context();
  const bool monochrome = context != tui::TerminalContext::Color;
  mvhline(row, left, ' ', width);
  int col = left;
  auto draw_part = [&](CostBucket bucket, const std::string & label) {
    const int color = bucket_color(bucket, monochrome);
    if (color != 0) {
      attron(COLOR_PAIR(color));
    }
    mvaddstr(row, col, bucket_glyph(bucket, context));
    mvaddstr(row, col + 1, " ");
    if (color != 0) {
      attroff(COLOR_PAIR(color));
    }
    mvaddnstr(row, col + 2, label.c_str(), std::max(0, width - (col - left) - 2));
    col += static_cast<int>(label.size()) + 4;
  };

  draw_part(CostBucket::Free, "free");
  draw_part(CostBucket::Low, "low");
  draw_part(CostBucket::Medium, "med");
  draw_part(CostBucket::High, "high");
  draw_part(CostBucket::Lethal, "lethal");
  draw_part(CostBucket::Unknown, "unk");
}

void MapViewerScreen::draw_grid_view(
  int top, int left, int bottom, int right, const OccupancyGrid & message) const
{
  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);

  const int width = right - left + 1;
  attron(theme_attr(kColorHeader));
  mvprintw(top, left, "%-*s", width, "Map");
  attroff(theme_attr(kColorHeader));

  const std::size_t source_width = static_cast<std::size_t>(message.info.width);
  const std::size_t source_height = static_cast<std::size_t>(message.info.height);
  const auto context = render_context();
  const bool monochrome = context != tui::TerminalContext::Color;
  std::ostringstream header_line;
  header_line << "topic=" << backend_->topic_
              << " frame=" << message.header.frame_id
              << " age=" << format_age(message.header.stamp, *backend_->get_clock())
              << " src=" << source_width << 'x' << source_height
              << '@' << std::fixed << std::setprecision(2) << message.info.resolution << "m"
              << " rot=" << backend_->rotation_degrees_
              << " mode=" << (
                context == tui::TerminalContext::Ascii ? "ascii" :
                context == tui::TerminalContext::Mono ? "mono" : "costmap");
  mvprintw(top + 1, left, "%-*s", width, truncate_text(header_line.str(), width).c_str());

  int map_top = top + 2;
  if (backend_->show_legend_) {
    draw_legend_line(top + 2, left, width);
    map_top = top + 3;
  } else {
    mvhline(top + 2, left, ' ', width);
  }

  if (map_top + 2 > bottom) {
    return;
  }

  draw_box(map_top, left, bottom, right, kColorFrame);
  const RenderGeometry geometry = compute_render_geometry(
    rows,
    columns,
    map_top,
    left,
    bottom,
    right,
    source_width,
    source_height,
    backend_->rotation_degrees_,
    backend_->max_width_,
    backend_->max_height_,
    context);
  std::vector<AggregatedCell> aggregated_cells(
    static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.height));
  aggregate_cells(message, geometry, backend_->rotation_degrees_, aggregated_cells);

  for (int render_y = 0; render_y < geometry.height; ++render_y) {
    const int row = bottom - 1 - render_y;
    if (row <= map_top || row >= bottom) {
      continue;
    }
    int col = left + 1;
    mvhline(row, col, ' ', std::max(0, right - left - 1));
    for (int render_x = 0; render_x < geometry.width && col < right; ++render_x) {
      const auto index = static_cast<std::size_t>(render_y * geometry.width + render_x);
      const int8_t value = aggregated_cells[index].occupancy_value;
      if (value <= kOccupancyFreeThreshold && value != kOccupancyUnknown && !backend_->show_free_) {
        mvaddstr(row, col, context == tui::TerminalContext::Ascii ? " " : "  ");
        col += geometry.cell_width;
        continue;
      }

      const auto bucket = cost_bucket(value);
      const int color = bucket_color(bucket, monochrome);
      if (color != 0) {
        attron(COLOR_PAIR(color));
      }
      for (int repeat = 0; repeat < geometry.cell_width && col < right; ++repeat) {
        mvaddstr(row, col, bucket_glyph(bucket, context));
        ++col;
      }
      if (color != 0) {
        attroff(COLOR_PAIR(color));
      }
    }
  }
}

void MapViewerScreen::draw_status_line(int row, int columns, const std::string & text) const {
  draw_status_bar(row, columns, text);
}

void MapViewerScreen::draw_help_line(int row, int columns) const {
  draw_help_bar(row, columns, "Esc Exit  F10 Exit");
}

void MapViewerScreen::draw() {
  erase();

  int rows = 0;
  int columns = 0;
  getmaxyx(stdscr, rows, columns);
  const int help_row = rows - 1;
  const int status_row = rows - 2;
  const int content_bottom = rows - 3;

  draw_box(0, 0, content_bottom, columns - 1, kColorFrame);
  attron(theme_attr(kColorTitle));
  mvprintw(0, 1, "Map Viewer ");
  attroff(theme_attr(kColorTitle));

  const auto latest_grid = backend_->latest_grid_snapshot();
  if (latest_grid) {
    draw_grid_view(1, 1, content_bottom - 1, columns - 2, *latest_grid);
    draw_status_line(status_row, columns, "Rendering occupancy grid from " + backend_->topic_ + ".");
  } else {
    draw_waiting_message(1, 1, content_bottom - 1, columns - 2);
    draw_status_line(status_row, columns, "Waiting for occupancy grid on " + backend_->topic_ + ".");
  }
  draw_help_line(help_row, columns);
  refresh();
}

}  // namespace ros2_console_tools
