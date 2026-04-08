#include "ros2_console_tools/systemd_client.hpp"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

#include "ros2_console_tools/process_runner.hpp"

namespace ros2_console_tools {

namespace {

std::string trim(const std::string & text) {
  std::size_t start = 0;
  while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start])) != 0) {
    ++start;
  }

  std::size_t end = text.size();
  while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
    --end;
  }

  return text.substr(start, end - start);
}

bool parse_yes_no(const std::map<std::string, std::string> & properties, const std::string & key) {
  const auto found = properties.find(key);
  return found != properties.end() && found->second == "yes";
}

}  // namespace

std::vector<SystemdUnitSummary> parse_systemd_list_units_output(const std::string & text) {
  std::vector<SystemdUnitSummary> units;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    line = trim(line);
    if (line.empty()) {
      continue;
    }

    std::istringstream line_stream(line);
    SystemdUnitSummary unit;
    if (!(line_stream >> unit.name >> unit.load_state >> unit.active_state >> unit.sub_state)) {
      continue;
    }
    std::getline(line_stream, unit.description);
    unit.description = trim(unit.description);
    units.push_back(unit);
  }
  return units;
}

SystemdUnitDetails parse_systemd_show_output(const std::string & text) {
  SystemdUnitDetails details;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    const std::size_t separator = line.find('=');
    if (separator == std::string::npos) {
      continue;
    }

    const std::string key = trim(line.substr(0, separator));
    const std::string value = trim(line.substr(separator + 1));
    details.properties[key] = value;
  }

  details.can_start = parse_yes_no(details.properties, "CanStart");
  details.can_stop = parse_yes_no(details.properties, "CanStop");
  details.can_reload = parse_yes_no(details.properties, "CanReload");
  return details;
}

std::vector<SystemdUnitSummary> SystemdClient::list_service_units(std::string * error) const {
  const ProcessResult result = run_process({
      "systemctl",
      "list-units",
      "--type=service",
      "--all",
      "--plain",
      "--no-legend",
      "--no-pager",
      "--full"});
  if (!result.succeeded()) {
    if (error != nullptr) {
      *error = result.output.empty() ? "systemctl list-units failed." : trim(result.output);
    }
    return {};
  }
  return parse_systemd_list_units_output(result.output);
}

bool SystemdClient::show_unit_details(
  const std::string & unit_name,
  SystemdUnitDetails & details,
  std::string * error) const
{
  const ProcessResult result = run_process({
      "systemctl",
      "show",
      "--no-pager",
      "--property=Id,Description,LoadState,ActiveState,SubState,UnitFileState,MainPID,ExecMainPID,FragmentPath,ExecStart,CanStart,CanStop,CanReload",
      unit_name});
  if (!result.succeeded()) {
    if (error != nullptr) {
      *error = result.output.empty() ? "systemctl show failed." : trim(result.output);
    }
    return false;
  }
  details = parse_systemd_show_output(result.output);
  return true;
}

bool SystemdClient::execute_unit_action(
  const std::string & unit_name,
  const std::string & action,
  std::string * error) const
{
  const ProcessResult result = run_process({
      "systemctl",
      "--no-ask-password",
      action,
      unit_name});
  if (!result.succeeded()) {
    if (error != nullptr) {
      *error = result.output.empty() ? "systemctl action failed." : trim(result.output);
    }
    return false;
  }
  return true;
}

}  // namespace ros2_console_tools
