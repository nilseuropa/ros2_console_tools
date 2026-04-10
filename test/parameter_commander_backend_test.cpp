#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#define private public
#include "ros2_console_tools/parameter_commander.hpp"
#undef private

namespace ros2_console_tools {
namespace {

class ParameterCommanderBackendTest : public ::testing::Test {
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

ParameterEntry make_entry(
  const std::string & name, uint8_t type, const std::string & edit_buffer)
{
  ParameterEntry entry;
  entry.name = name;
  entry.descriptor.type = type;
  entry.edit_buffer = edit_buffer;
  return entry;
}

TEST_F(ParameterCommanderBackendTest, FormatsArrayValuesForEditing) {
  EXPECT_EQ(
    parameter_value_to_string(rclcpp::ParameterValue(std::vector<int64_t>{1, 2, 3})),
    "[1, 2, 3]");
  EXPECT_EQ(
    parameter_value_to_string(rclcpp::ParameterValue(std::vector<bool>{true, false, true})),
    "[true, false, true]");
  EXPECT_EQ(
    parameter_value_to_string(rclcpp::ParameterValue(std::vector<std::string>{"alpha", "b,c", "d\"e"})),
    "[\"alpha\", \"b,c\", \"d\\\"e\"]");
}

TEST_F(ParameterCommanderBackendTest, ParsesIntegerArrays) {
  ParameterCommanderBackend backend("");
  const auto parameter = backend.parse_edit_buffer(
    make_entry("numbers", ParameterType::PARAMETER_INTEGER_ARRAY, "[1, -2, 3]"));

  ASSERT_TRUE(parameter.has_value());
  const auto values = parameter->get_parameter_value().get<std::vector<int64_t>>();
  EXPECT_EQ(values, (std::vector<int64_t>{1, -2, 3}));
}

TEST_F(ParameterCommanderBackendTest, ParsesStringArraysWithQuotedCommas) {
  ParameterCommanderBackend backend("");
  const auto parameter = backend.parse_edit_buffer(
    make_entry("names", ParameterType::PARAMETER_STRING_ARRAY, R"(["alpha", "b,c", "d\"e"])"));

  ASSERT_TRUE(parameter.has_value());
  const auto values = parameter->get_parameter_value().get<std::vector<std::string>>();
  EXPECT_EQ(values, (std::vector<std::string>{"alpha", "b,c", "d\"e"}));
}

TEST_F(ParameterCommanderBackendTest, RejectsOutOfRangeByteArrays) {
  ParameterCommanderBackend backend("");
  const auto parameter = backend.parse_edit_buffer(
    make_entry("bytes", ParameterType::PARAMETER_BYTE_ARRAY, "[0, 255, 256]"));

  EXPECT_FALSE(parameter.has_value());
}

TEST_F(ParameterCommanderBackendTest, RejectsTrailingNumericText) {
  ParameterCommanderBackend backend("");
  const auto parameter = backend.parse_edit_buffer(
    make_entry("numbers", ParameterType::PARAMETER_INTEGER_ARRAY, "[1, 2ms]"));

  EXPECT_FALSE(parameter.has_value());
}

TEST_F(ParameterCommanderBackendTest, RejectsTextAfterQuotedStringArrayItem) {
  ParameterCommanderBackend backend("");
  const auto parameter = backend.parse_edit_buffer(
    make_entry("names", ParameterType::PARAMETER_STRING_ARRAY, R"(["alpha"beta])"));

  EXPECT_FALSE(parameter.has_value());
}

}  // namespace
}  // namespace ros2_console_tools
