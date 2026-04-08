#include "ros2_console_tools/image_viewer.hpp"

#include <algorithm>
#include <cctype>

namespace ros2_console_tools {

namespace {

std::string to_lower(std::string text) {
  for (char & character : text) {
    character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  }
  return text;
}

uint8_t luma_from_rgb(uint8_t red, uint8_t green, uint8_t blue) {
  return static_cast<uint8_t>((299u * red + 587u * green + 114u * blue + 500u) / 1000u);
}

}  // namespace

ImageViewerBackend::ImageViewerBackend(const std::string & topic)
: Node("image_viewer"),
  topic_(this->declare_parameter<std::string>("topic", topic.empty() ? "/image" : topic))
{
  const std::string theme_config_path =
    this->declare_parameter<std::string>("theme_config_path", tui::default_theme_config_path());
  std::string theme_error;
  if (!tui::load_theme_from_file(theme_config_path, &theme_error)) {
    if (theme_config_path != tui::default_theme_config_path()) {
      RCLCPP_WARN(this->get_logger(), "%s", theme_error.c_str());
    }
  }

  render_hz_ = std::max(1.0, this->declare_parameter<double>("render_hz", 10.0));

  subscription_ = this->create_subscription<Image>(
    topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ImageViewerBackend::image_callback, this, std::placeholders::_1));
}

void ImageViewerBackend::image_callback(Image::ConstSharedPtr message) {
  auto frame = std::make_shared<ImageFrame>();
  std::string error;
  if (!decode_to_frame(*message, *frame, error)) {
    std::lock_guard<std::mutex> lock(latest_frame_mutex_);
    latest_frame_.reset();
    latest_error_ = std::move(error);
    return;
  }

  std::lock_guard<std::mutex> lock(latest_frame_mutex_);
  latest_frame_ = std::move(frame);
  latest_error_.clear();
}

std::shared_ptr<const ImageFrame> ImageViewerBackend::latest_frame_snapshot() const {
  std::lock_guard<std::mutex> lock(latest_frame_mutex_);
  return latest_frame_;
}

std::string ImageViewerBackend::latest_error_snapshot() const {
  std::lock_guard<std::mutex> lock(latest_frame_mutex_);
  return latest_error_;
}

bool ImageViewerBackend::decode_to_frame(
  const Image & message, ImageFrame & frame, std::string & error) const
{
  frame = ImageFrame();
  frame.topic = topic_;
  frame.source_encoding = message.encoding;
  frame.width = message.width;
  frame.height = message.height;
  frame.stamp = message.header.stamp;
  frame.frame_id = message.header.frame_id;

  if (message.width == 0 || message.height == 0) {
    error = "image has zero width or height";
    return false;
  }

  const std::string encoding = to_lower(message.encoding);
  std::size_t channels = 0;
  enum class PixelLayout {
    Mono,
    Rgb,
    Bgr,
    Rgba,
    Bgra,
    Yuy2,
  };
  PixelLayout layout = PixelLayout::Mono;

  if (encoding == "mono8" || encoding == "8uc1") {
    channels = 1;
    layout = PixelLayout::Mono;
  } else if (encoding == "rgb8") {
    channels = 3;
    layout = PixelLayout::Rgb;
  } else if (encoding == "bgr8") {
    channels = 3;
    layout = PixelLayout::Bgr;
  } else if (encoding == "rgba8") {
    channels = 4;
    layout = PixelLayout::Rgba;
  } else if (encoding == "bgra8") {
    channels = 4;
    layout = PixelLayout::Bgra;
  } else if (
    encoding == "yuv422_yuy2" || encoding == "yuy2" ||
    encoding == "yuyv" || encoding == "yuv422")
  {
    channels = 2;
    layout = PixelLayout::Yuy2;
  } else {
    error = "unsupported encoding '" + message.encoding + "'";
    return false;
  }

  const std::size_t min_step = static_cast<std::size_t>(message.width) * channels;
  if (message.step < min_step) {
    error = "image step is smaller than width * channels";
    return false;
  }

  const std::size_t required_bytes = static_cast<std::size_t>(message.step) * message.height;
  if (message.data.size() < required_bytes) {
    error = "image data buffer is truncated";
    return false;
  }

  frame.gray8.resize(static_cast<std::size_t>(message.width) * message.height);
  for (uint32_t row = 0; row < message.height; ++row) {
    const std::size_t row_offset = static_cast<std::size_t>(row) * message.step;
    for (uint32_t column = 0; column < message.width; ++column) {
      const std::size_t src = row_offset + static_cast<std::size_t>(column) * channels;
      const std::size_t dst = static_cast<std::size_t>(row) * message.width + column;
      switch (layout) {
        case PixelLayout::Mono:
          frame.gray8[dst] = message.data[src];
          break;
        case PixelLayout::Rgb:
          frame.gray8[dst] = luma_from_rgb(
            message.data[src], message.data[src + 1], message.data[src + 2]);
          break;
        case PixelLayout::Bgr:
          frame.gray8[dst] = luma_from_rgb(
            message.data[src + 2], message.data[src + 1], message.data[src]);
          break;
        case PixelLayout::Rgba:
          frame.gray8[dst] = luma_from_rgb(
            message.data[src], message.data[src + 1], message.data[src + 2]);
          break;
        case PixelLayout::Bgra:
          frame.gray8[dst] = luma_from_rgb(
            message.data[src + 2], message.data[src + 1], message.data[src]);
          break;
        case PixelLayout::Yuy2: {
          const std::size_t pair_src =
            row_offset + (static_cast<std::size_t>(column) / 2u) * 4u + ((column % 2u) == 0u ? 0u : 2u);
          frame.gray8[dst] = message.data[pair_src];
          break;
        }
      }
    }
  }

  return true;
}

}  // namespace ros2_console_tools
