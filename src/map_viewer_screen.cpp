#include "ros2_console_tools/map_viewer.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <ncursesw/ncurses.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
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

struct BrailleAggregatedCell {
  uint8_t dot_mask{0};
  int8_t occupancy_value{kOccupancyUnknown};
  bool initialized{false};
};

struct RenderGeometry {
  int width{1};
  int height{1};
  int cell_width{1};
  int column_offset{0};
  int row_offset{0};
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

const char * bucket_braille_legend_glyph(CostBucket bucket) {
  switch (bucket) {
    case CostBucket::Free:
      return "\xE2\xA0\x81";
    case CostBucket::Low:
      return "\xE2\xA0\x87";
    case CostBucket::Medium:
      return "\xE2\xA0\xBF";
    case CostBucket::High:
      return "\xE2\xA1\xBF";
    case CostBucket::Lethal:
      return "\xE2\xA3\xBF";
    case CostBucket::Unknown:
    default:
      return "\xE2\xA2\x80";
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

MapViewerRenderMode effective_render_mode(
  MapViewerRenderMode mode, tui::TerminalContext context)
{
  if (mode == MapViewerRenderMode::Auto) {
    return context == tui::TerminalContext::Ascii
      ? MapViewerRenderMode::Ascii
      : MapViewerRenderMode::Costmap;
  }
  if (context == tui::TerminalContext::Ascii && mode != MapViewerRenderMode::Ascii) {
    return MapViewerRenderMode::Ascii;
  }
  return mode;
}

std::string render_mode_label(MapViewerRenderMode mode, tui::TerminalContext context) {
  switch (mode) {
    case MapViewerRenderMode::Braille:
      return context == tui::TerminalContext::Ascii ? "braille-ascii" : "braille";
    case MapViewerRenderMode::Costmap:
      return context == tui::TerminalContext::Ascii ? "costmap-ascii" : "costmap";
    case MapViewerRenderMode::Ascii:
      return "ascii";
    case MapViewerRenderMode::Auto:
    default:
      return context == tui::TerminalContext::Ascii ? "auto-ascii" : "auto-costmap";
  }
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
  const int available_render_width = std::max(1, available_columns / cell_width);
  const int available_render_height = available_rows;

  int max_render_width = std::min(
    rotated_width == 0 ? 1 : static_cast<int>(rotated_width),
    available_render_width);
  int max_render_height = std::min(
    rotated_height == 0 ? 1 : static_cast<int>(rotated_height),
    available_render_height);
  if (max_width > 0) {
    max_render_width = std::min(max_render_width, max_width);
  }
  if (max_height > 0) {
    max_render_height = std::min(max_render_height, max_height);
  }
  max_render_width = std::max(1, max_render_width);
  max_render_height = std::max(1, max_render_height);

  constexpr double kCellHeightOverWidth = 2.0;
  const double source_aspect =
    rotated_height == 0 ? 1.0 : static_cast<double>(std::max<std::size_t>(1, rotated_width)) /
    static_cast<double>(std::max<std::size_t>(1, rotated_height));
  const double render_cell_aspect =
    source_aspect * kCellHeightOverWidth / static_cast<double>(cell_width);

  int render_width = max_render_width;
  int render_height = std::max(1, static_cast<int>(std::lround(render_width / render_cell_aspect)));
  if (render_height > max_render_height) {
    render_height = max_render_height;
    render_width = std::max(
      1,
      std::min(max_render_width, static_cast<int>(std::lround(render_height * render_cell_aspect))));
  }

  const int used_columns = render_width * cell_width;
  const int column_offset = std::max(0, (available_columns - used_columns) / 2);
  const int row_offset = std::max(0, (available_rows - render_height) / 2);

  return {
    std::max(1, render_width),
    std::max(1, render_height),
    cell_width,
    column_offset,
    row_offset};
}

RenderGeometry compute_braille_render_geometry(
  int top,
  int left,
  int bottom,
  int right,
  std::size_t source_width,
  std::size_t source_height,
  int rotation_degrees,
  int max_width,
  int max_height)
{
  const auto [rotated_width, rotated_height] =
    rotated_dimensions(source_width, source_height, rotation_degrees);
  const int available_columns = std::max(1, right - left - 1);
  const int available_rows = std::max(1, bottom - top - 1);
  const int available_virtual_width = available_columns * 2;
  const int available_virtual_height = available_rows * 4;

  int max_virtual_width = std::min(
    rotated_width == 0 ? 1 : static_cast<int>(rotated_width),
    available_virtual_width);
  int max_virtual_height = std::min(
    rotated_height == 0 ? 1 : static_cast<int>(rotated_height),
    available_virtual_height);
  if (max_width > 0) {
    max_virtual_width = std::min(max_virtual_width, max_width);
  }
  if (max_height > 0) {
    max_virtual_height = std::min(max_virtual_height, max_height);
  }
  max_virtual_width = std::max(1, max_virtual_width);
  max_virtual_height = std::max(1, max_virtual_height);

  const double source_aspect =
    rotated_height == 0 ? 1.0 : static_cast<double>(std::max<std::size_t>(1, rotated_width)) /
    static_cast<double>(std::max<std::size_t>(1, rotated_height));

  int virtual_width = max_virtual_width;
  int virtual_height = std::max(1, static_cast<int>(std::lround(virtual_width / source_aspect)));
  if (virtual_height > max_virtual_height) {
    virtual_height = max_virtual_height;
    virtual_width = std::max(
      1,
      std::min(max_virtual_width, static_cast<int>(std::lround(virtual_height * source_aspect))));
  }

  const int render_width = std::max(1, std::min(available_columns, (virtual_width + 1) / 2));
  const int render_height = std::max(1, std::min(available_rows, (virtual_height + 3) / 4));
  const int column_offset = std::max(0, (available_columns - render_width) / 2);
  const int row_offset = std::max(0, (available_rows - render_height) / 2);
  return {render_width, render_height, 1, column_offset, row_offset};
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

bool braille_dot_visible(int8_t occupancy_value, bool show_free, int subcolumn, int subrow) {
  switch (cost_bucket(occupancy_value)) {
    case CostBucket::Free:
      return show_free && subcolumn == 0 && subrow == 0;
    case CostBucket::Unknown:
      return subcolumn == 1 && subrow == 3;
    case CostBucket::Low:
    case CostBucket::Medium:
    case CostBucket::High:
    case CostBucket::Lethal:
      return true;
  }
  return false;
}

void maybe_add_braille_dot(
  BrailleAggregatedCell & target_cell, int8_t occupancy_value, uint8_t dot_mask)
{
  if (dot_mask == 0) {
    return;
  }
  target_cell.dot_mask |= dot_mask;
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

void aggregate_braille_cells(
  const OccupancyGrid & message,
  const RenderGeometry & geometry,
  int rotation_degrees,
  bool show_free,
  std::vector<BrailleAggregatedCell> & aggregated_cells)
{
  const std::size_t source_width = static_cast<std::size_t>(message.info.width);
  const std::size_t source_height = static_cast<std::size_t>(message.info.height);
  const auto [rotated_width, rotated_height] =
    rotated_dimensions(source_width, source_height, rotation_degrees);
  const std::size_t normalized_rotated_width = std::max<std::size_t>(1, rotated_width);
  const std::size_t normalized_rotated_height = std::max<std::size_t>(1, rotated_height);
  const int virtual_width = std::max(1, geometry.width * 2);
  const int virtual_height = std::max(1, geometry.height * 4);

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

      const int virtual_x = std::min(
        virtual_width - 1,
        static_cast<int>((rotated_x * static_cast<std::size_t>(virtual_width)) / normalized_rotated_width));
      const int virtual_y = std::min(
        virtual_height - 1,
        static_cast<int>((rotated_y * static_cast<std::size_t>(virtual_height)) / normalized_rotated_height));
      const int render_x = virtual_x / 2;
      const int render_y = virtual_y / 4;
      const int subcolumn = virtual_x % 2;
      const int subrow = 3 - (virtual_y % 4);
      const auto source_index = source_y * source_width + source_x;
      const int8_t occupancy_value =
        source_index < message.data.size() ? normalize_occupancy_value(message.data[source_index]) : kOccupancyUnknown;
      if (!braille_dot_visible(occupancy_value, show_free, subcolumn, subrow)) {
        continue;
      }
      const auto render_index = static_cast<std::size_t>(render_y * geometry.width + render_x);
      if (render_index >= aggregated_cells.size()) {
        continue;
      }
      maybe_add_braille_dot(
        aggregated_cells[render_index],
        occupancy_value,
        tui::braille_dot_mask(subcolumn, subrow));
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
    case 'm':
    case 'M':
      render_mode_ = static_cast<MapViewerRenderMode>((static_cast<int>(render_mode_) + 1) % 4);
      return true;
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
  const auto mode = effective_render_mode(render_mode_, context);
  const auto glyph_context = mode == MapViewerRenderMode::Ascii ? tui::TerminalContext::Ascii : context;
  const bool monochrome = context != tui::TerminalContext::Color;
  mvhline(row, left, ' ', width);
  int col = left;
  auto draw_part = [&](CostBucket bucket, const std::string & label) {
    const int color = bucket_color(bucket, monochrome);
    if (color != 0) {
      attron(COLOR_PAIR(color));
    }
    mvaddstr(
      row,
      col,
      mode == MapViewerRenderMode::Braille ? bucket_braille_legend_glyph(bucket) :
      bucket_glyph(bucket, glyph_context));
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
  const auto effective_mode = effective_render_mode(render_mode_, context);
  const bool monochrome = context != tui::TerminalContext::Color;
  std::ostringstream header_line;
  header_line << "topic=" << backend_->topic_
              << " frame=" << message.header.frame_id
              << " age=" << format_age(message.header.stamp, *backend_->get_clock())
              << " src=" << source_width << 'x' << source_height
              << '@' << std::fixed << std::setprecision(2) << message.info.resolution << "m"
              << " rot=" << backend_->rotation_degrees_
              << " mode=" << render_mode_label(render_mode_, context);
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
  if (effective_mode == MapViewerRenderMode::Braille) {
    const RenderGeometry geometry = compute_braille_render_geometry(
      map_top,
      left,
      bottom,
      right,
      source_width,
      source_height,
      backend_->rotation_degrees_,
      backend_->max_width_,
      backend_->max_height_);
    std::vector<BrailleAggregatedCell> aggregated_cells(
      static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.height));
    aggregate_braille_cells(
      message, geometry, backend_->rotation_degrees_, backend_->show_free_, aggregated_cells);

    const int draw_left = left + 1 + geometry.column_offset;
    const int draw_bottom = bottom - 1 - geometry.row_offset;
    for (int render_y = 0; render_y < geometry.height; ++render_y) {
      const int row = draw_bottom - render_y;
      if (row <= map_top || row >= bottom) {
        continue;
      }
      mvhline(row, left + 1, ' ', std::max(0, right - left - 1));
      for (int render_x = 0; render_x < geometry.width; ++render_x) {
        const int col = draw_left + render_x;
        if (col >= right) {
          break;
        }
        const auto index = static_cast<std::size_t>(render_y * geometry.width + render_x);
        const auto & cell = aggregated_cells[index];
        const auto bucket = cell.initialized ? cost_bucket(cell.occupancy_value) : CostBucket::Free;
        const int color = cell.initialized ? bucket_color(bucket, monochrome) : 0;
        if (color != 0) {
          attron(COLOR_PAIR(color));
        }
        mvaddstr(row, col, tui::braille_glyph(cell.dot_mask).c_str());
        if (color != 0) {
          attroff(COLOR_PAIR(color));
        }
      }
    }
    return;
  }

  const auto glyph_context =
    effective_mode == MapViewerRenderMode::Ascii ? tui::TerminalContext::Ascii : context;
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
    glyph_context);
  std::vector<AggregatedCell> aggregated_cells(
    static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.height));
  aggregate_cells(message, geometry, backend_->rotation_degrees_, aggregated_cells);

  const int draw_left = left + 1 + geometry.column_offset;
  const int draw_bottom = bottom - 1 - geometry.row_offset;
  for (int render_y = 0; render_y < geometry.height; ++render_y) {
    const int row = draw_bottom - render_y;
    if (row <= map_top || row >= bottom) {
      continue;
    }
    mvhline(row, left + 1, ' ', std::max(0, right - left - 1));
    int col = draw_left;
    for (int render_x = 0; render_x < geometry.width && col < right; ++render_x) {
      const auto index = static_cast<std::size_t>(render_y * geometry.width + render_x);
      const int8_t value = aggregated_cells[index].occupancy_value;
      if (value <= kOccupancyFreeThreshold && value != kOccupancyUnknown && !backend_->show_free_) {
        mvaddstr(row, col, glyph_context == tui::TerminalContext::Ascii ? " " : "  ");
        col += geometry.cell_width;
        continue;
      }

      const auto bucket = cost_bucket(value);
      const int color = bucket_color(bucket, monochrome);
      if (color != 0) {
        attron(COLOR_PAIR(color));
      }
      for (int repeat = 0; repeat < geometry.cell_width && col < right; ++repeat) {
        mvaddstr(row, col, bucket_glyph(bucket, glyph_context));
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
  draw_help_bar(row, columns, "M Mode  Esc Exit  F10 Exit");
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
