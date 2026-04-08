#include "ros2_console_tools/systemd_commander.hpp"

#include <algorithm>

namespace ros2_console_tools {

SystemdCommanderBackend::SystemdCommanderBackend(const std::string & initial_unit)
: initial_unit_(initial_unit) {
  std::string theme_error;
  (void)tui::load_theme_from_file(tui::default_theme_config_path(), &theme_error);
  refresh_units();
}

void SystemdCommanderBackend::refresh_units() {
  std::string previous_selection;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!units_.empty() && selected_index_ >= 0 && selected_index_ < static_cast<int>(units_.size())) {
      previous_selection = units_[static_cast<std::size_t>(selected_index_)].name;
    }
  }

  std::string error;
  std::vector<SystemdUnitSummary> units = client_.list_service_units(&error);

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (units.empty() && !error.empty()) {
      units_.clear();
      selected_unit_details_ = SystemdUnitDetails{};
      selected_details_unit_.clear();
      selected_index_ = 0;
      unit_scroll_ = 0;
      status_line_ = error;
      return;
    }

    units_ = std::move(units);
    const std::string preferred_selection = !initial_unit_.empty() ? initial_unit_ : previous_selection;
    if (!preferred_selection.empty()) {
      const auto found = std::find_if(
        units_.begin(), units_.end(),
        [&preferred_selection](const SystemdUnitSummary & unit) {
          return unit.name == preferred_selection;
        });
      if (found != units_.end()) {
        selected_index_ = static_cast<int>(std::distance(units_.begin(), found));
      }
    }

    clamp_selection();
    if (units_.empty()) {
      selected_unit_details_ = SystemdUnitDetails{};
      selected_details_unit_.clear();
      status_line_ = "No systemd service units were reported.";
      return;
    }

    initial_unit_.clear();
    status_line_ = "Loaded " + std::to_string(units_.size()) + " service units.";
  }

  refresh_selected_unit_details();
}

void SystemdCommanderBackend::refresh_selected_unit_details() {
  std::string unit_name;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (units_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(units_.size())) {
      selected_unit_details_ = SystemdUnitDetails{};
      selected_details_unit_.clear();
      return;
    }
    unit_name = units_[static_cast<std::size_t>(selected_index_)].name;
  }

  SystemdUnitDetails details;
  std::string error;
  const bool ok = client_.show_unit_details(unit_name, details, &error);

  std::lock_guard<std::mutex> lock(mutex_);
  if (units_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(units_.size())) {
    return;
  }
  if (units_[static_cast<std::size_t>(selected_index_)].name != unit_name) {
    return;
  }

  if (!ok) {
    selected_unit_details_ = SystemdUnitDetails{};
    selected_details_unit_.clear();
    status_line_ = error.empty() ? ("Failed to inspect " + unit_name + ".") : error;
    return;
  }

  selected_unit_details_ = std::move(details);
  selected_details_unit_ = unit_name;
  status_line_ = "Loaded details for " + unit_name + ".";
}

void SystemdCommanderBackend::clamp_selection() {
  if (units_.empty()) {
    selected_index_ = 0;
    unit_scroll_ = 0;
    return;
  }

  selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(units_.size()) - 1);
  unit_scroll_ = std::max(0, std::min(unit_scroll_, selected_index_));
}

std::string SystemdCommanderBackend::selected_unit_name() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (units_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(units_.size())) {
    return "";
  }
  return units_[static_cast<std::size_t>(selected_index_)].name;
}

std::string SystemdCommanderBackend::selected_fragment_path() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (units_.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(units_.size())) {
    return "";
  }

  const auto & unit = units_[static_cast<std::size_t>(selected_index_)];
  if (selected_details_unit_ != unit.name) {
    return "";
  }

  const auto found = selected_unit_details_.properties.find("FragmentPath");
  if (found == selected_unit_details_.properties.end()) {
    return "";
  }
  return found->second;
}

std::vector<SystemdDetailRow> SystemdCommanderBackend::detail_rows_snapshot() const {
  std::vector<SystemdDetailRow> rows;

  std::lock_guard<std::mutex> lock(mutex_);
  if (units_.empty()) {
    rows.push_back({"No services available.", false});
    return rows;
  }

  const auto & unit = units_[static_cast<std::size_t>(selected_index_)];
  rows.push_back({"Service", true});
  rows.push_back({"Name: " + unit.name, false});
  rows.push_back({"Description: " + unit.description, false});
  rows.push_back({"Load: " + unit.load_state, false});
  rows.push_back({"Active: " + unit.active_state, false});
  rows.push_back({"Sub: " + unit.sub_state, false});

  if (selected_details_unit_ != unit.name || selected_unit_details_.properties.empty()) {
    rows.push_back({"Details", true});
    rows.push_back({"No detailed properties loaded.", false});
    return rows;
  }

  rows.push_back({"Properties", true});
  const char * property_keys[] = {
    "Id",
    "Description",
    "LoadState",
    "ActiveState",
    "SubState",
    "UnitFileState",
    "MainPID",
    "ExecMainPID",
    "FragmentPath",
    "ExecStart",
  };
  for (const char * key : property_keys) {
    const auto found = selected_unit_details_.properties.find(key);
    if (found != selected_unit_details_.properties.end() && !found->second.empty()) {
      rows.push_back({std::string(key) + ": " + found->second, false});
    }
  }

  rows.push_back({"Actions", true});
  rows.push_back(
    {
      "start=" + std::string(selected_unit_details_.can_start ? "yes" : "no") +
      "  stop=" + std::string(selected_unit_details_.can_stop ? "yes" : "no") +
      "  reload=" + std::string(selected_unit_details_.can_reload ? "yes" : "no"),
      false});
  return rows;
}

bool SystemdCommanderBackend::perform_action(const std::string & action) {
  const std::string unit_name = selected_unit_name();
  if (unit_name.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_line_ = "No service selected.";
    return false;
  }

  std::string error;
  if (!client_.execute_unit_action(unit_name, action, &error)) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_line_ = error.empty() ? ("Failed to " + action + " " + unit_name + ".") : error;
    return false;
  }

  refresh_units();
  std::lock_guard<std::mutex> lock(mutex_);
  status_line_ = "Ran `" + action + "` for " + unit_name + ".";
  return true;
}

}  // namespace ros2_console_tools
