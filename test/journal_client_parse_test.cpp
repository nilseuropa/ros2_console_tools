#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "ros2_console_tools/journal_client.hpp"

namespace ros2_console_tools {
namespace {

class ScopedEnvironmentVariable {
public:
  ScopedEnvironmentVariable(const std::string & name, const std::string & value)
  : name_(name) {
    const char * current_value = std::getenv(name.c_str());
    if (current_value != nullptr) {
      had_original_value_ = true;
      original_value_ = current_value;
    }
    setenv(name_.c_str(), value.c_str(), 1);
  }

  ~ScopedEnvironmentVariable() {
    if (had_original_value_) {
      setenv(name_.c_str(), original_value_.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::string original_value_;
  bool had_original_value_{false};
};

std::string read_text_file(const std::filesystem::path & path) {
  std::ifstream input(path);
  std::ostringstream text;
  text << input.rdbuf();
  return text.str();
}

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

TEST(JournalClientParseTest, OmitsLineLimitForFullSnapshotRequests) {
  char temp_directory_template[] = "/tmp/ros2_console_tools_journal_client_test_XXXXXX";
  const char * temp_directory = mkdtemp(temp_directory_template);
  ASSERT_NE(temp_directory, nullptr);
  const std::filesystem::path root(temp_directory);
  const std::filesystem::path journalctl_path = root / "journalctl";
  const std::filesystem::path args_path = root / "journalctl_args.txt";

  {
    std::ofstream script(journalctl_path);
    ASSERT_TRUE(script.is_open());
    script
      << "#!/bin/sh\n"
      << "printf '%s\\n' \"$@\" > \"$CODEX_JOURNAL_ARGS_FILE\"\n"
      << "printf '%s\\n' '{\"__REALTIME_TIMESTAMP\":\"1712572096000000\",\"MESSAGE\":\"ok\"}'\n";
  }
  std::filesystem::permissions(
    journalctl_path,
    std::filesystem::perms::owner_exec |
    std::filesystem::perms::owner_read |
    std::filesystem::perms::owner_write,
    std::filesystem::perm_options::replace);

  const std::string original_path = std::getenv("PATH") == nullptr ? "" : std::getenv("PATH");
  ScopedEnvironmentVariable path_env("PATH", root.string() + ":" + original_path);
  ScopedEnvironmentVariable args_env("CODEX_JOURNAL_ARGS_FILE", args_path.string());

  JournalClient client;
  std::string error;

  const auto limited_entries = client.read_entries("", 6, 200, "", &error);
  ASSERT_TRUE(error.empty());
  ASSERT_EQ(limited_entries.size(), 1u);
  const std::string limited_args = read_text_file(args_path);
  EXPECT_NE(limited_args.find("--lines=200"), std::string::npos);

  const auto full_entries = client.read_entries("", 6, 0, "", &error);
  ASSERT_TRUE(error.empty());
  ASSERT_EQ(full_entries.size(), 1u);
  const std::string full_args = read_text_file(args_path);
  EXPECT_EQ(full_args.find("--lines="), std::string::npos);

  std::filesystem::remove_all(root);
}

}  // namespace
}  // namespace ros2_console_tools
