#include "ros2_console_tools/laser_scan_viewer.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace ros2_console_tools {
namespace {

TEST(LaserScanViewerBackendTest, ComputesStatsForValidRanges) {
  LaserScanFrame frame;
  frame.range_min = 0.1f;
  frame.range_max = 10.0f;
  frame.ranges = {1.0f, 2.0f, 4.0f};

  const LaserScanStats stats = compute_laser_scan_stats(frame);

  EXPECT_EQ(stats.total_count, 3u);
  EXPECT_EQ(stats.valid_count, 3u);
  EXPECT_EQ(stats.invalid_count, 0u);
  EXPECT_FLOAT_EQ(stats.min_range, 1.0f);
  EXPECT_FLOAT_EQ(stats.max_range, 4.0f);
  EXPECT_FLOAT_EQ(stats.average_range, 7.0f / 3.0f);
}

TEST(LaserScanViewerBackendTest, IgnoresInvalidRanges) {
  LaserScanFrame frame;
  frame.range_min = 0.5f;
  frame.range_max = 5.0f;
  frame.ranges = {
    0.25f,
    1.5f,
    std::numeric_limits<float>::infinity(),
    std::numeric_limits<float>::quiet_NaN(),
    6.0f,
    3.5f};

  const LaserScanStats stats = compute_laser_scan_stats(frame);

  EXPECT_EQ(stats.total_count, 6u);
  EXPECT_EQ(stats.valid_count, 2u);
  EXPECT_EQ(stats.invalid_count, 4u);
  EXPECT_FLOAT_EQ(stats.min_range, 1.5f);
  EXPECT_FLOAT_EQ(stats.max_range, 3.5f);
  EXPECT_FLOAT_EQ(stats.average_range, 2.5f);
}

}  // namespace
}  // namespace ros2_console_tools
