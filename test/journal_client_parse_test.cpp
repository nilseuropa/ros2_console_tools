#include <gtest/gtest.h>

#include <string>

#include "ros2_console_tools/journal_client.hpp"

namespace ros2_console_tools {
namespace {

TEST(JournalClientParseTest, ParsesJsonLogLines) {
  const std::string output =
    "{\"__REALTIME_TIMESTAMP\":\"1712572096000000\",\"PRIORITY\":\"4\",\"_SYSTEMD_UNIT\":\"ssh.service\","
    "\"SYSLOG_IDENTIFIER\":\"sshd\",\"MESSAGE\":\"Accepted publickey\",\"_PID\":\"1234\","
    "\"CODE_FILE\":\"/tmp/daemon.cpp\",\"CODE_LINE\":\"42\",\"CODE_FUNC\":\"handle\"}\n";

  const auto entries = parse_journal_json_lines(output);
  ASSERT_EQ(entries.size(), 1u);

  EXPECT_EQ(entries[0].priority, 4);
  EXPECT_EQ(entries[0].unit, "ssh.service");
  EXPECT_EQ(entries[0].identifier, "sshd");
  EXPECT_EQ(entries[0].message, "Accepted publickey");
  EXPECT_EQ(entries[0].pid, "1234");
  EXPECT_EQ(entries[0].code_file, "/tmp/daemon.cpp");
  EXPECT_EQ(entries[0].code_line, "42");
  EXPECT_EQ(entries[0].code_function, "handle");
  EXPECT_EQ(journal_priority_label(entries[0].priority), "WARN");
  EXPECT_FALSE(entries[0].timestamp.empty());
}

TEST(JournalClientParseTest, DecodesEscapesAndSkipsInvalidLines) {
  const std::string output =
    "not-json\n"
    "{\"__REALTIME_TIMESTAMP\":\"1712572096000000\",\"PRIORITY\":\"6\",\"SYSLOG_IDENTIFIER\":\"demo\","
    "\"MESSAGE\":\"Line one\\nLine two with \\\"quote\\\" and unicode \\u00e4\",\"EXTRA\":[1,2,3]}\n";

  const auto entries = parse_journal_json_lines(output);
  ASSERT_EQ(entries.size(), 1u);
  EXPECT_EQ(entries[0].identifier, "demo");
  EXPECT_EQ(entries[0].message, "Line one\nLine two with \"quote\" and unicode ä");
}

}  // namespace
}  // namespace ros2_console_tools
