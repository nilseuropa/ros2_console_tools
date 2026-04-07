#include <gtest/gtest.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>

#define private public
#include "ros2_console_tools/tf_monitor.hpp"
#undef private

namespace ros2_console_tools {
namespace {

class TfMonitorBackendTest : public ::testing::Test {
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

TEST_F(TfMonitorBackendTest, DisplaysRootFrameBeforeChildLinks) {
  TfMonitorBackend backend;

  TFMessage message;

  TransformStamped odom_to_base_footprint;
  odom_to_base_footprint.header.frame_id = "odom";
  odom_to_base_footprint.child_frame_id = "base_footprint";
  odom_to_base_footprint.transform.rotation.w = 1.0;
  message.transforms.push_back(odom_to_base_footprint);

  TransformStamped base_footprint_to_base_link;
  base_footprint_to_base_link.header.frame_id = "base_footprint";
  base_footprint_to_base_link.child_frame_id = "base_link";
  base_footprint_to_base_link.transform.rotation.w = 1.0;
  message.transforms.push_back(base_footprint_to_base_link);

  backend.handle_tf_message(message, false);
  backend.refresh_rows();

  ASSERT_EQ(backend.rows_.size(), 3u);

  EXPECT_TRUE(backend.rows_[0].is_root);
  EXPECT_EQ(backend.rows_[0].child_frame, "odom");
  EXPECT_EQ(backend.rows_[0].depth, 0);
  EXPECT_EQ(backend.rows_[0].freshness, "root");

  EXPECT_FALSE(backend.rows_[1].is_root);
  EXPECT_EQ(backend.rows_[1].parent_frame, "odom");
  EXPECT_EQ(backend.rows_[1].child_frame, "base_footprint");
  EXPECT_EQ(backend.rows_[1].depth, 1);

  EXPECT_FALSE(backend.rows_[2].is_root);
  EXPECT_EQ(backend.rows_[2].parent_frame, "base_footprint");
  EXPECT_EQ(backend.rows_[2].child_frame, "base_link");
  EXPECT_EQ(backend.rows_[2].depth, 2);

  const auto odom_pose = backend.frame_poses_.find("odom");
  ASSERT_NE(odom_pose, backend.frame_poses_.end());
  EXPECT_EQ(odom_pose->second.root_frame, "odom");
}

}  // namespace
}  // namespace ros2_console_tools
