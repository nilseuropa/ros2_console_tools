#include <gtest/gtest.h>

#include <sensor_msgs/msg/image.hpp>

#include <rclcpp/rclcpp.hpp>

#define private public
#include "ros2_console_tools/image_viewer.hpp"
#undef private

namespace ros2_console_tools {
namespace {

class ImageViewerBackendTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(ImageViewerBackendTest, DecodesMono8Image) {
  ImageViewerBackend backend("/camera/image");
  sensor_msgs::msg::Image image;
  image.width = 3;
  image.height = 2;
  image.step = 3;
  image.encoding = "mono8";
  image.data = {10, 20, 30, 40, 50, 60};

  ImageFrame frame;
  std::string error;
  ASSERT_TRUE(backend.decode_to_frame(image, frame, error)) << error;
  EXPECT_EQ(frame.topic, "/camera/image");
  EXPECT_EQ(frame.width, 3u);
  EXPECT_EQ(frame.height, 2u);
  EXPECT_EQ(frame.gray8, image.data);
}

TEST_F(ImageViewerBackendTest, ConvertsBgr8ImageToGrayscale) {
  ImageViewerBackend backend("/camera/image");
  sensor_msgs::msg::Image image;
  image.width = 2;
  image.height = 1;
  image.step = 6;
  image.encoding = "bgr8";
  image.data = {
    255, 0, 0,
    0, 255, 0,
  };

  ImageFrame frame;
  std::string error;
  ASSERT_TRUE(backend.decode_to_frame(image, frame, error)) << error;
  ASSERT_EQ(frame.gray8.size(), 2u);
  EXPECT_EQ(frame.gray8[0], 29u);
  EXPECT_EQ(frame.gray8[1], 150u);
}

TEST_F(ImageViewerBackendTest, RejectsUnsupportedEncodings) {
  ImageViewerBackend backend("/camera/image");
  sensor_msgs::msg::Image image;
  image.width = 2;
  image.height = 2;
  image.step = 4;
  image.encoding = "mono16";
  image.data = {0, 0, 0, 0, 0, 0, 0, 0};

  ImageFrame frame;
  std::string error;
  EXPECT_FALSE(backend.decode_to_frame(image, frame, error));
  EXPECT_NE(error.find("unsupported encoding"), std::string::npos);
}

}  // namespace
}  // namespace ros2_console_tools
