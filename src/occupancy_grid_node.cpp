#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

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

struct Rgb {
  int red;
  int green;
  int blue;
};

struct TerminalSize {
  int columns;
  int rows;
};

struct AggregatedCell {
  int8_t occupancy_value{kOccupancyUnknown};
  bool initialized{false};
};

enum class CostBucket : std::size_t {
  Free = 0,
  Low = 1,
  Medium = 2,
  High = 3,
  Lethal = 4,
  Unknown = 5,
};

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

Rgb bucket_color(CostBucket bucket) {
  switch (bucket) {
    case CostBucket::Free:
      return {28, 28, 28};
    case CostBucket::Low:
      return {100, 170, 255};
    case CostBucket::Medium:
      return {255, 214, 102};
    case CostBucket::High:
      return {255, 140, 70};
    case CostBucket::Lethal:
      return {220, 45, 45};
    case CostBucket::Unknown:
    default:
      return {150, 150, 170};
  }
}

std::string bucket_glyph(CostBucket bucket) {
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

bool terminal_supports_color() {
  const char * no_color = std::getenv("NO_COLOR");
  if (no_color != nullptr && no_color[0] != '\0') {
    return false;
  }

  const char * term = std::getenv("TERM");
  return term != nullptr && std::string(term) != "dumb";
}

TerminalSize read_terminal_size() {
  winsize window_size{};
  if (::ioctl(STDOUT_FILENO, TIOCGWINSZ, &window_size) == 0
    && window_size.ws_col > 0
    && window_size.ws_row > 0)
  {
    return {static_cast<int>(window_size.ws_col), static_cast<int>(window_size.ws_row)};
  }

  return {80, 24};
}

std::string ansi_foreground_block(const Rgb & color, const std::string & glyph, int repeat) {
  std::ostringstream stream;
  stream << "\x1b[38;2;" << color.red << ';' << color.green << ';' << color.blue << "m";
  for (int index = 0; index < repeat; ++index) {
    stream << glyph;
  }
  stream << "\x1b[0m";
  return stream.str();
}

std::string repeat_glyph(const std::string & glyph, std::size_t count) {
  std::string output;
  output.reserve(glyph.size() * count);
  for (std::size_t index = 0; index < count; ++index) {
    output += glyph;
  }
  return output;
}

int visible_width(const std::string & line) {
  int width = 0;
  for (std::size_t index = 0; index < line.size(); ++index) {
    if (line[index] == '\x1b' && index + 1 < line.size() && line[index + 1] == '[') {
      index += 2;
      while (index < line.size() && line[index] != 'm' && line[index] != 'K') {
        ++index;
      }
      continue;
    }
    ++width;
  }
  return width;
}

std::string truncate_line(const std::string & line, int max_columns) {
  if (max_columns <= 0) {
    return "";
  }
  if (visible_width(line) <= max_columns) {
    return line;
  }

  const int reserved_columns = max_columns > 3 ? 3 : 0;
  const int target_width = std::max(0, max_columns - reserved_columns);
  std::string output;
  int width = 0;

  for (std::size_t index = 0; index < line.size() && width < target_width; ++index) {
    if (line[index] == '\x1b' && index + 1 < line.size() && line[index + 1] == '[') {
      const std::size_t escape_start = index;
      index += 2;
      while (index < line.size() && line[index] != 'm' && line[index] != 'K') {
        ++index;
      }
      if (index < line.size()) {
        output.append(line, escape_start, index - escape_start + 1);
      }
      continue;
    }

    output.push_back(line[index]);
    ++width;
  }

  if (reserved_columns > 0) {
    output += "...";
  }
  if (output.find("\x1b[") != std::string::npos) {
    output += "\x1b[0m";
  }
  return output;
}

void append_terminal_line(
  std::ostringstream & stream,
  int row,
  const std::string & line,
  bool stdout_is_tty)
{
  if (stdout_is_tty) {
    stream << "\x1b[" << row << ";1H";
  }
  stream << line;
  if (stdout_is_tty) {
    stream << "\x1b[K";
  } else {
    stream << '\n';
  }
}

}  // namespace

namespace ros2_cli_viz {

class OccupancyGridCliNode : public rclcpp::Node {
public:
  OccupancyGridCliNode()
  : Node("occupancy_grid_visualizer"),
    stdout_is_tty_(::isatty(STDOUT_FILENO) != 0),
    use_color_(stdout_is_tty_ && terminal_supports_color()) {
    topic_ = this->declare_parameter<std::string>("topic", "/map");
    render_hz_ = std::max(1.0, this->declare_parameter<double>("render_hz", 5.0));
    max_width_ = std::max(0, static_cast<int>(this->declare_parameter("max_width", 0)));
    max_height_ = std::max(0, static_cast<int>(this->declare_parameter("max_height", 0)));
    show_free_ = this->declare_parameter<bool>("show_free", true);
    show_legend_ = this->declare_parameter<bool>("show_legend", true);

    build_cell_cache();

    subscription_ = this->create_subscription<OccupancyGrid>(
      topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&OccupancyGridCliNode::grid_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / render_hz_));
    render_timer_ = this->create_wall_timer(period, std::bind(&OccupancyGridCliNode::render_frame, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Rendering occupancy grid from '%s' at %.1f Hz (%s mode).",
      topic_.c_str(),
      render_hz_,
      use_color_ ? "color" : "ascii");
    if (!stdout_is_tty_) {
      RCLCPP_WARN(this->get_logger(), "stdout is not a TTY; falling back to ASCII output.");
    } else if (!use_color_) {
      RCLCPP_WARN(this->get_logger(), "Terminal color support disabled; falling back to ASCII output.");
    }
  }

  ~OccupancyGridCliNode() override {
    restore_terminal();
  }

private:
  struct RenderGeometry {
    int width;
    int height;
    int cell_width;
  };

  void build_cell_cache() {
    for (std::size_t index = 0; index < color_blocks_.size(); ++index) {
      const auto bucket = static_cast<CostBucket>(index);
      color_blocks_[index] = ansi_foreground_block(
        bucket_color(bucket), bucket_glyph(bucket), kColorCellWidth);
    }
  }

  void grid_callback(OccupancyGrid::ConstSharedPtr message) {
    std::lock_guard<std::mutex> lock(latest_grid_mutex_);
    latest_grid_ = std::move(message);
  }

  void render_frame() {
    OccupancyGrid::ConstSharedPtr latest_grid;
    {
      std::lock_guard<std::mutex> lock(latest_grid_mutex_);
      latest_grid = latest_grid_;
    }

    prepare_terminal();

    std::string frame;
    if (latest_grid) {
      frame = build_grid_frame(*latest_grid);
    } else {
      frame = build_waiting_frame();
    }

    std::fwrite(frame.data(), sizeof(char), frame.size(), stdout);
    std::fflush(stdout);
  }

  void prepare_terminal() {
    if (!stdout_is_tty_ || terminal_prepared_) {
      return;
    }

    std::fputs("\x1b[?1049h\x1b[2J\x1b[?25l", stdout);
    terminal_prepared_ = true;
  }

  void restore_terminal() {
    if (!terminal_prepared_) {
      return;
    }

    std::fputs("\x1b[0m\x1b[?25h\x1b[?1049l", stdout);
    std::fflush(stdout);
    terminal_prepared_ = false;
  }

  RenderGeometry compute_render_geometry(std::size_t source_width, std::size_t source_height) const {
    const TerminalSize terminal_size = read_terminal_size();
    const int cell_width = use_color_ ? kColorCellWidth : kAsciiCellWidth;
    const int reserved_rows = kHeaderRows + kBorderRows;

    int render_width = source_width == 0 ? 1 : static_cast<int>(source_width);
    int render_height = source_height == 0 ? 1 : static_cast<int>(source_height);

    const int available_columns = std::max(cell_width, terminal_size.columns - 2);
    const int available_rows = std::max(1, terminal_size.rows - reserved_rows);

    render_width = std::min(render_width, std::max(1, available_columns / cell_width));
    render_height = std::min(render_height, available_rows);

    if (max_width_ > 0) {
      render_width = std::min(render_width, max_width_);
    }
    if (max_height_ > 0) {
      render_height = std::min(render_height, max_height_);
    }

    return {std::max(1, render_width), std::max(1, render_height), cell_width};
  }

  std::string build_waiting_frame() const {
    std::ostringstream stream;
    const TerminalSize terminal_size = read_terminal_size();
    append_terminal_line(
      stream,
      1,
      truncate_line("topic=" + topic_ + " status=waiting-for-occupancy-grid", terminal_size.columns),
      stdout_is_tty_);
    append_terminal_line(
      stream,
      2,
      truncate_line("start the producer node and this view will refresh automatically", terminal_size.columns),
      stdout_is_tty_);

    if (stdout_is_tty_) {
      stream << "\x1b[3;1H";
      stream << "\x1b[J";
    }

    return stream.str();
  }

  std::string build_grid_frame(const OccupancyGrid & message) const {
    const TerminalSize terminal_size = read_terminal_size();
    const std::size_t source_width = static_cast<std::size_t>(message.info.width);
    const std::size_t source_height = static_cast<std::size_t>(message.info.height);
    const std::size_t cell_count = source_width * source_height;
    const std::size_t rotated_width = source_height;
    const std::size_t rotated_height = source_width;
    const RenderGeometry geometry = compute_render_geometry(rotated_width, rotated_height);

    std::vector<AggregatedCell> aggregated_cells(
      static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.height));
    int current_row = 1;

    if (cell_count > 0) {
      aggregate_cells(message, geometry, aggregated_cells);
    }

    std::ostringstream stream;
    {
      std::ostringstream header_line;
      header_line << "topic=" << topic_
                  << " frame=" << message.header.frame_id
                  << " age=" << format_age(message.header.stamp)
                  << " src=" << source_width << 'x' << source_height
                  << '@' << std::fixed << std::setprecision(2) << message.info.resolution << "m"
                  << " view=" << geometry.width << 'x' << geometry.height
                  << " rot=90cw mode=costmap";
      append_terminal_line(
        stream,
        current_row++,
        truncate_line(header_line.str(), terminal_size.columns),
        stdout_is_tty_);
    }

    if (show_legend_) {
      append_terminal_line(
        stream,
        current_row++,
        truncate_line(build_legend_line(), terminal_size.columns),
        stdout_is_tty_);
    } else {
      append_terminal_line(stream, current_row++, "", stdout_is_tty_);
    }

    const std::size_t border_width = static_cast<std::size_t>(geometry.width * geometry.cell_width);
    append_terminal_line(
      stream, current_row++, "┌" + repeat_glyph("─", border_width) + "┐", stdout_is_tty_);
    for (int render_y = geometry.height - 1; render_y >= 0; --render_y) {
      std::ostringstream row_stream;
      row_stream << "│";
      for (int render_x = 0; render_x < geometry.width; ++render_x) {
        const auto index = static_cast<std::size_t>(render_y * geometry.width + render_x);
        append_cell(row_stream, aggregated_cells[index].occupancy_value);
      }
      row_stream << "│";
      append_terminal_line(stream, current_row++, row_stream.str(), stdout_is_tty_);
    }
    append_terminal_line(
      stream, current_row++, "└" + repeat_glyph("─", border_width) + "┘", stdout_is_tty_);

    if (stdout_is_tty_ && current_row <= terminal_size.rows) {
      stream << "\x1b[" << current_row << ";1H";
      stream << "\x1b[J";
    }

    return stream.str();
  }

  void aggregate_cells(
    const OccupancyGrid & message,
    const RenderGeometry & geometry,
    std::vector<AggregatedCell> & aggregated_cells) const
  {
    const std::size_t source_width = static_cast<std::size_t>(message.info.width);
    const std::size_t source_height = static_cast<std::size_t>(message.info.height);
    const std::size_t rotated_width = source_height == 0 ? 1 : source_height;
    const std::size_t rotated_height = source_width == 0 ? 1 : source_width;

    for (std::size_t source_y = 0; source_y < source_height; ++source_y) {
      for (std::size_t source_x = 0; source_x < source_width; ++source_x) {
        const std::size_t rotated_x = source_height - 1 - source_y;
        const std::size_t rotated_y = source_x;
        const int render_x = std::min(
          geometry.width - 1,
          static_cast<int>((rotated_x * static_cast<std::size_t>(geometry.width)) / rotated_width));
        const int render_y = std::min(
          geometry.height - 1,
          static_cast<int>((rotated_y * static_cast<std::size_t>(geometry.height)) / rotated_height));
        const auto source_index = source_y * source_width + source_x;
        const auto render_index = static_cast<std::size_t>(render_y * geometry.width + render_x);

        const int8_t occupancy_value = source_cell_value(message, source_index);
        maybe_replace_cell(aggregated_cells[render_index], occupancy_value);
      }
    }
  }

  int8_t source_cell_value(const OccupancyGrid & message, std::size_t source_index) const {
    if (source_index < message.data.size()) {
      return normalize_occupancy_value(message.data[source_index]);
    }
    return kOccupancyUnknown;
  }

  void maybe_replace_cell(AggregatedCell & target_cell, int8_t occupancy_value) const {
    if (!target_cell.initialized) {
      target_cell.occupancy_value = occupancy_value;
      target_cell.initialized = true;
      return;
    }

    const int incoming_severity = occupancy_severity(occupancy_value);
    const int current_severity = occupancy_severity(target_cell.occupancy_value);
    if (incoming_severity > current_severity) {
      target_cell.occupancy_value = occupancy_value;
      return;
    }

    if (incoming_severity == current_severity && occupancy_value > target_cell.occupancy_value) {
      target_cell.occupancy_value = occupancy_value;
    }
  }

  std::string format_age(const builtin_interfaces::msg::Time & stamp) const {
    const rclcpp::Time message_time(stamp);
    if (message_time.nanoseconds() <= 0) {
      return "n/a";
    }

    const double age_seconds = (this->get_clock()->now() - message_time).seconds();
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2) << age_seconds << 's';
    return stream.str();
  }

  std::string build_legend_line() const {
    if (!use_color_) {
      return "legend: · free  ░ low  ▒ med  ▓ high  █ lethal  · unk";
    }

    return std::string("legend: ")
      + color_blocks_[bucket_index(CostBucket::Free)] + " free  "
      + color_blocks_[bucket_index(CostBucket::Low)] + " low  "
      + color_blocks_[bucket_index(CostBucket::Medium)] + " med  "
      + color_blocks_[bucket_index(CostBucket::High)] + " high  "
      + color_blocks_[bucket_index(CostBucket::Lethal)] + " lethal  "
      + color_blocks_[bucket_index(CostBucket::Unknown)] + " unk";
  }

  void append_cell(std::ostringstream & stream, int8_t occupancy_value) const {
    const int8_t normalized_value = normalize_occupancy_value(occupancy_value);
    if (normalized_value <= kOccupancyFreeThreshold && normalized_value != kOccupancyUnknown && !show_free_) {
      stream << std::string(static_cast<std::size_t>(use_color_ ? kColorCellWidth : kAsciiCellWidth), ' ');
      return;
    }

    if (use_color_) {
      stream << color_blocks_[bucket_index(cost_bucket(normalized_value))];
      return;
    }

    stream << bucket_glyph(cost_bucket(normalized_value));
  }

  std::size_t bucket_index(CostBucket bucket) const {
    return static_cast<std::size_t>(bucket);
  }

  std::string topic_;
  double render_hz_{5.0};
  int max_width_{0};
  int max_height_{0};
  bool show_free_{true};
  bool show_legend_{true};
  bool stdout_is_tty_{false};
  bool use_color_{false};
  mutable bool terminal_prepared_{false};
  std::array<std::string, 6> color_blocks_{};

  std::mutex latest_grid_mutex_;
  OccupancyGrid::ConstSharedPtr latest_grid_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr render_timer_;
};

}  // namespace ros2_cli_viz

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_cli_viz::OccupancyGridCliNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
