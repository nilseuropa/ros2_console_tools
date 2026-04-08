#include <gtest/gtest.h>

#include <string>

#include "ros2_console_tools/systemd_client.hpp"

namespace ros2_console_tools {
namespace {

TEST(SystemdClientParseTest, ParsesServiceListOutput) {
  const std::string output =
    "dbus.service loaded active running D-Bus System Message Bus\n"
    "ssh.service loaded inactive dead OpenBSD Secure Shell server\n"
    "broken.service loaded failed failed Broken Service\n";

  const auto units = parse_systemd_list_units_output(output);
  ASSERT_EQ(units.size(), 3u);

  EXPECT_EQ(units[0].name, "dbus.service");
  EXPECT_EQ(units[0].load_state, "loaded");
  EXPECT_EQ(units[0].active_state, "active");
  EXPECT_EQ(units[0].sub_state, "running");
  EXPECT_EQ(units[0].description, "D-Bus System Message Bus");

  EXPECT_EQ(units[2].name, "broken.service");
  EXPECT_EQ(units[2].active_state, "failed");
  EXPECT_EQ(units[2].sub_state, "failed");
}

TEST(SystemdClientParseTest, ParsesShowPropertiesAndActions) {
  const std::string output =
    "Id=ssh.service\n"
    "Description=OpenBSD Secure Shell server\n"
    "LoadState=loaded\n"
    "ActiveState=active\n"
    "SubState=running\n"
    "UnitFileState=enabled\n"
    "MainPID=1024\n"
    "ExecStart={ path=/usr/sbin/sshd ; argv[]=/usr/sbin/sshd -D }\n"
    "CanStart=yes\n"
    "CanStop=yes\n"
    "CanReload=no\n";

  const auto details = parse_systemd_show_output(output);
  EXPECT_EQ(details.properties.at("Id"), "ssh.service");
  EXPECT_EQ(details.properties.at("Description"), "OpenBSD Secure Shell server");
  EXPECT_EQ(details.properties.at("ExecStart"), "{ path=/usr/sbin/sshd ; argv[]=/usr/sbin/sshd -D }");
  EXPECT_TRUE(details.can_start);
  EXPECT_TRUE(details.can_stop);
  EXPECT_FALSE(details.can_reload);
}

}  // namespace
}  // namespace ros2_console_tools
