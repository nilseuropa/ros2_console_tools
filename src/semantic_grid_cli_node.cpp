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
#include <rclcpp/rclcpp.hpp>

#include <nowtech_interfaces/msg/semantic_grid.hpp>

namespace {

using SemanticGrid = nowtech_interfaces::msg::SemanticGrid;

constexpr int kColorCellWidth = 2;
constexpr int kAsciiCellWidth = 1;
constexpr int kHeaderRows = 2;
constexpr int kBorderRows = 2;

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
  uint8_t semantic_class{SemanticGrid::CLASS_FREE};
  float confidence{0.0f};
  bool initialized{false};
};

uint8_t normalize_semantic_class(uint8_t semantic_class) {
  switch (semantic_class) {
    case SemanticGrid::CLASS_FREE:
    case SemanticGrid::CLASS_UNDER_GROUND:
    case SemanticGrid::CLASS_ABOVE_GROUND:
    case SemanticGrid::CLASS_CLIMBABLE:
    case SemanticGrid::CLASS_RAMP:
    case SemanticGrid::CLASS_OVERHANG:
    case SemanticGrid::CLASS_UNKNOWN:
      return semantic_class;
    default:
      return SemanticGrid::CLASS_UNKNOWN;
  }
}

int semantic_severity(uint8_t semantic_class) {
  switch (semantic_class) {
    case SemanticGrid::CLASS_FREE:
      return 0;
    case SemanticGrid::CLASS_CLIMBABLE:
      return 1;
    case SemanticGrid::CLASS_UNKNOWN:
      return 2;
    case SemanticGrid::CLASS_OVERHANG:
      return 3;
    case SemanticGrid::CLASS_ABOVE_GROUND:
      return 4;
    case SemanticGrid::CLASS_RAMP:
      return 5;
    case SemanticGrid::CLASS_UNDER_GROUND:
      return 6;
    default:
      return 2;
  }
}

Rgb semantic_color(uint8_t semantic_class) {
  switch (semantic_class) {
    case SemanticGrid::CLASS_FREE:
      return {50, 50, 50};
    case SemanticGrid::CLASS_UNDER_GROUND:
      return {0, 110, 255};
    case SemanticGrid::CLASS_ABOVE_GROUND:
      return {255, 70, 70};
    case SemanticGrid::CLASS_CLIMBABLE:
      return {245, 210, 70};
    case SemanticGrid::CLASS_RAMP:
      return {90, 220, 90};
    case SemanticGrid::CLASS_OVERHANG:
      return {220, 90, 220};
    case SemanticGrid::CLASS_UNKNOWN:
    default:
      return {230, 230, 230};
  }
}

char semantic_ascii(uint8_t semantic_class) {
  switch (semantic_class) {
    case SemanticGrid::CLASS_FREE:
      return '.';
    case SemanticGrid::CLASS_UNDER_GROUND:
      return 'u';
    case SemanticGrid::CLASS_ABOVE_GROUND:
      return 'A';
    case SemanticGrid::CLASS_CLIMBABLE:
      return 'c';
    case SemanticGrid::CLASS_RAMP:
      return 'r';
    case SemanticGrid::CLASS_OVERHANG:
      return 'o';
    case SemanticGrid::CLASS_UNKNOWN:
    default:
      return '?';
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

std::string ansi_background_block(const Rgb & color) {
  std::ostringstream stream;
  stream << "\x1b[48;2;" << color.red << ';' << color.green << ';' << color.blue << "m  \x1b[0m";
  return stream.str();
}

std::string legend_entry(const std::string & block, const std::string & label) {
  return block + ' ' + label;
}

}  // namespace

namespace ros2_cli_viz {

class SemanticGridCliNode : public rclcpp::Node {
public:
  SemanticGridCliNode()
  : Node("semantic_grid_cli_visualizer"),
    stdout_is_tty_(::isatty(STDOUT_FILENO) != 0),
    use_color_(stdout_is_tty_ && terminal_supports_color()) {
    topic_ = this->declare_parameter<std::string>("topic", "/cas/semantic_grid");
    render_hz_ = std::max(1.0, this->declare_parameter<double>("render_hz", 5.0));
    max_width_ = std::max(0, static_cast<int>(this->declare_parameter("max_width", 0)));
    max_height_ = std::max(0, static_cast<int>(this->declare_parameter("max_height", 0)));
    show_free_ = this->declare_parameter<bool>("show_free", true);
    show_legend_ = this->declare_parameter<bool>("show_legend", true);

    build_cell_cache();

    subscription_ = this->create_subscription<SemanticGrid>(
      topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&SemanticGridCliNode::grid_callback, this, std::placeholders::_1));

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / render_hz_));
    render_timer_ = this->create_wall_timer(period, std::bind(&SemanticGridCliNode::render_frame, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Rendering semantic grid from '%s' at %.1f Hz (%s mode).",
      topic_.c_str(),
      render_hz_,
      use_color_ ? "color" : "ascii");
    if (!stdout_is_tty_) {
      RCLCPP_WARN(this->get_logger(), "stdout is not a TTY; falling back to ASCII output.");
    } else if (!use_color_) {
      RCLCPP_WARN(this->get_logger(), "Terminal color support disabled; falling back to ASCII output.");
    }
  }

  ~SemanticGridCliNode() override {
    restore_terminal();
  }

private:
  struct RenderGeometry {
    int width;
    int height;
    int cell_width;
  };

  void build_cell_cache() {
    color_blocks_.fill(ansi_background_block(semantic_color(SemanticGrid::CLASS_UNKNOWN)));
    color_blocks_[SemanticGrid::CLASS_FREE] = ansi_background_block(semantic_color(SemanticGrid::CLASS_FREE));
    color_blocks_[SemanticGrid::CLASS_UNDER_GROUND] = ansi_background_block(semantic_color(SemanticGrid::CLASS_UNDER_GROUND));
    color_blocks_[SemanticGrid::CLASS_ABOVE_GROUND] = ansi_background_block(semantic_color(SemanticGrid::CLASS_ABOVE_GROUND));
    color_blocks_[SemanticGrid::CLASS_CLIMBABLE] = ansi_background_block(semantic_color(SemanticGrid::CLASS_CLIMBABLE));
    color_blocks_[SemanticGrid::CLASS_RAMP] = ansi_background_block(semantic_color(SemanticGrid::CLASS_RAMP));
    color_blocks_[SemanticGrid::CLASS_OVERHANG] = ansi_background_block(semantic_color(SemanticGrid::CLASS_OVERHANG));
    color_blocks_[SemanticGrid::CLASS_UNKNOWN] = ansi_background_block(semantic_color(SemanticGrid::CLASS_UNKNOWN));
  }

  void grid_callback(SemanticGrid::ConstSharedPtr message) {
    std::lock_guard<std::mutex> lock(latest_grid_mutex_);
    latest_grid_ = std::move(message);
  }

  void render_frame() {
    SemanticGrid::ConstSharedPtr latest_grid;
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

    std::fputs("\x1b[2J\x1b[?25l", stdout);
    terminal_prepared_ = true;
  }

  void restore_terminal() {
    if (!terminal_prepared_) {
      return;
    }

    std::fputs("\x1b[0m\x1b[?25h\n", stdout);
    std::fflush(stdout);
    terminal_prepared_ = false;
  }

  RenderGeometry compute_render_geometry(std::size_t source_width, std::size_t source_height) const {
    const TerminalSize terminal_size = read_terminal_size();
    const int cell_width = use_color_ ? kColorCellWidth : kAsciiCellWidth;
    const int legend_rows = show_legend_ ? 1 : 0;
    const int reserved_rows = kHeaderRows + kBorderRows + legend_rows;

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

    if (stdout_is_tty_) {
      stream << "\x1b[H";
    }

    stream << "topic=" << topic_ << " status=waiting-for-semantic-grid\n";
    stream << "start the producer node and this view will refresh automatically\n";

    if (stdout_is_tty_) {
      stream << "\x1b[J";
    }

    return stream.str();
  }

  std::string build_grid_frame(const SemanticGrid & message) const {
    const auto & occupancy = message.occupancy;
    const std::size_t source_width = static_cast<std::size_t>(occupancy.info.width);
    const std::size_t source_height = static_cast<std::size_t>(occupancy.info.height);
    const std::size_t cell_count = source_width * source_height;

    const bool use_semantic = message.class_ids.size() == cell_count;
    const bool semantic_mismatch = !message.class_ids.empty() && !use_semantic;
    const bool use_confidence = message.class_confidence.size() == cell_count;
    const RenderGeometry geometry = compute_render_geometry(source_width, source_height);

    std::vector<AggregatedCell> aggregated_cells(
      static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.height));

    if (cell_count > 0) {
      aggregate_cells(message, use_semantic, use_confidence, geometry, aggregated_cells);
    }

    std::ostringstream stream;
    if (stdout_is_tty_) {
      stream << "\x1b[H";
    }

    stream << "topic=" << topic_
           << " frame=" << occupancy.header.frame_id
           << " age=" << format_age(occupancy.header.stamp) << '\n';
    stream << "src=" << source_width << 'x' << source_height
           << '@' << std::fixed << std::setprecision(2) << occupancy.info.resolution << "m"
           << " view=" << geometry.width << 'x' << geometry.height
           << " semantic=" << semantic_mode(use_semantic, semantic_mismatch)
           << " conf=" << (use_confidence ? "yes" : "no") << '\n';

    if (show_legend_) {
      stream << build_legend_line() << '\n';
    }

    stream << '+' << std::string(static_cast<std::size_t>(geometry.width * geometry.cell_width), '-') << "+\n";
    for (int render_y = geometry.height - 1; render_y >= 0; --render_y) {
      stream << '|';
      for (int render_x = 0; render_x < geometry.width; ++render_x) {
        const auto index = static_cast<std::size_t>(render_y * geometry.width + render_x);
        append_cell(stream, aggregated_cells[index].semantic_class);
      }
      stream << "|\n";
    }
    stream << '+' << std::string(static_cast<std::size_t>(geometry.width * geometry.cell_width), '-') << "+\n";

    if (stdout_is_tty_) {
      stream << "\x1b[J";
    }

    return stream.str();
  }

  void aggregate_cells(
    const SemanticGrid & message,
    bool use_semantic,
    bool use_confidence,
    const RenderGeometry & geometry,
    std::vector<AggregatedCell> & aggregated_cells) const
  {
    const auto & occupancy = message.occupancy;
    const std::size_t source_width = static_cast<std::size_t>(occupancy.info.width);
    const std::size_t source_height = static_cast<std::size_t>(occupancy.info.height);

    for (std::size_t source_y = 0; source_y < source_height; ++source_y) {
      const int render_y = std::min(
        geometry.height - 1,
        static_cast<int>((source_y * static_cast<std::size_t>(geometry.height)) / source_height));
      for (std::size_t source_x = 0; source_x < source_width; ++source_x) {
        const int render_x = std::min(
          geometry.width - 1,
          static_cast<int>((source_x * static_cast<std::size_t>(geometry.width)) / source_width));
        const auto source_index = source_y * source_width + source_x;
        const auto render_index = static_cast<std::size_t>(render_y * geometry.width + render_x);

        const uint8_t semantic_class = source_cell_class(message, source_index, use_semantic);
        const float confidence = source_cell_confidence(message, source_index, semantic_class, use_confidence);
        maybe_replace_cell(aggregated_cells[render_index], semantic_class, confidence);
      }
    }
  }

  uint8_t source_cell_class(
    const SemanticGrid & message,
    std::size_t source_index,
    bool use_semantic) const
  {
    if (use_semantic) {
      return normalize_semantic_class(message.class_ids[source_index]);
    }

    if (source_index < message.occupancy.data.size() && message.occupancy.data[source_index] > 0) {
      return SemanticGrid::CLASS_ABOVE_GROUND;
    }

    return SemanticGrid::CLASS_FREE;
  }

  float source_cell_confidence(
    const SemanticGrid & message,
    std::size_t source_index,
    uint8_t semantic_class,
    bool use_confidence) const
  {
    if (!use_confidence) {
      return semantic_class == SemanticGrid::CLASS_FREE ? 0.0f : 1.0f;
    }

    return std::clamp(message.class_confidence[source_index], 0.0f, 1.0f);
  }

  void maybe_replace_cell(AggregatedCell & target_cell, uint8_t semantic_class, float confidence) const {
    if (!target_cell.initialized) {
      target_cell.semantic_class = semantic_class;
      target_cell.confidence = confidence;
      target_cell.initialized = true;
      return;
    }

    const int incoming_severity = semantic_severity(semantic_class);
    const int current_severity = semantic_severity(target_cell.semantic_class);
    if (incoming_severity > current_severity
      || (incoming_severity == current_severity && confidence > target_cell.confidence))
    {
      target_cell.semantic_class = semantic_class;
      target_cell.confidence = confidence;
    }
  }

  std::string semantic_mode(bool use_semantic, bool semantic_mismatch) const {
    if (use_semantic) {
      return "class_ids";
    }
    if (semantic_mismatch) {
      return "fallback:mismatch";
    }
    return "fallback:occupancy";
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
      return "legend: .=free u=under A=above c=climbable r=ramp o=overhang ?=unknown";
    }

    return std::string("legend: ")
      + legend_entry(color_blocks_[SemanticGrid::CLASS_FREE], "free")
      + "  " + legend_entry(color_blocks_[SemanticGrid::CLASS_UNDER_GROUND], "under")
      + "  " + legend_entry(color_blocks_[SemanticGrid::CLASS_ABOVE_GROUND], "above")
      + "  " + legend_entry(color_blocks_[SemanticGrid::CLASS_CLIMBABLE], "climb")
      + "  " + legend_entry(color_blocks_[SemanticGrid::CLASS_RAMP], "ramp")
      + "  " + legend_entry(color_blocks_[SemanticGrid::CLASS_OVERHANG], "overhang")
      + "  " + legend_entry(color_blocks_[SemanticGrid::CLASS_UNKNOWN], "unknown");
  }

  void append_cell(std::ostringstream & stream, uint8_t semantic_class) const {
    const uint8_t normalized_class = normalize_semantic_class(semantic_class);
    if (normalized_class == SemanticGrid::CLASS_FREE && !show_free_) {
      stream << std::string(static_cast<std::size_t>(use_color_ ? kColorCellWidth : kAsciiCellWidth), ' ');
      return;
    }

    if (use_color_) {
      stream << color_blocks_[normalized_class];
      return;
    }

    stream << semantic_ascii(normalized_class);
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
  std::array<std::string, 16> color_blocks_{};

  std::mutex latest_grid_mutex_;
  SemanticGrid::ConstSharedPtr latest_grid_;
  rclcpp::Subscription<SemanticGrid>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr render_timer_;
};

}  // namespace ros2_cli_viz

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_cli_viz::SemanticGridCliNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
