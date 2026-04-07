#include "ros2_console_tools/topic_monitor.hpp"

namespace ros2_console_tools {

TopicMonitorBackend::TopicMonitorBackend(const TopicMonitorLaunchOptions & options)
: Node("topic_monitor"),
  allowed_topic_names_(options.allowed_topics.begin(), options.allowed_topics.end()),
  initial_topic_name_(options.initial_topic),
  open_initial_topic_detail_(options.open_initial_topic_detail),
  monitor_allowed_topics_on_start_(options.monitor_allowed_topics_on_start),
  exit_on_detail_escape_(options.exit_on_detail_escape),
  pending_namespace_expansion_(!options.allowed_topics.empty() || !options.initial_topic.empty())
{
  const std::string theme_config_path =
    this->declare_parameter<std::string>("theme_config_path", tui::default_theme_config_path());
  std::string theme_error;
  if (!tui::load_theme_from_file(theme_config_path, &theme_error)) {
    if (theme_config_path != tui::default_theme_config_path()) {
      RCLCPP_WARN(this->get_logger(), "%s", theme_error.c_str());
    }
  }
}

void TopicMonitorBackend::refresh_topics() {
  const auto discovered_topics = this->get_topic_names_and_types();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::map<std::string, TopicEntry> refreshed;
    for (const auto & [name, types] : discovered_topics) {
      if (name.empty() || !topic_is_allowed(name)) {
        continue;
      }

      TopicEntry entry;
      auto existing = topics_.find(name);
      if (existing != topics_.end()) {
        entry = existing->second;
      }

      entry.name = name;
      entry.type = types.empty() ? "<unknown>" : types.front();
      refreshed[name] = std::move(entry);
    }

    topics_.swap(refreshed);
    monitored_subscriptions_.erase(
      std::remove_if(
        monitored_subscriptions_.begin(),
        monitored_subscriptions_.end(),
        [this](const auto & item) { return topics_.find(item.first) == topics_.end(); }),
      monitored_subscriptions_.end());

    selected_index_ = std::clamp(selected_index_, 0, std::max(0, static_cast<int>(topics_.size()) - 1));
    last_refresh_time_ = TopicClock::now();
    status_line_ =
      std::string(allowed_topic_names_.empty() ? "Loaded " : "Loaded filtered ")
      + std::to_string(topics_.size())
      + (topics_.size() == 1 ? " topic. Space monitors the selected topic." : " topics. Space monitors the selected topic.");
  }

  apply_startup_behavior();
}

void TopicMonitorBackend::apply_startup_behavior() {
  if (pending_namespace_expansion_) {
    if (!initial_topic_name_.empty()) {
      expand_topic_namespace_path(topic_namespace(initial_topic_name_));
    }
    for (const auto & topic_name : allowed_topic_names_) {
      expand_topic_namespace_path(topic_namespace(topic_name));
    }
    pending_namespace_expansion_ = false;
  }

  if (open_initial_topic_detail_) {
    if (select_initial_topic_if_present()) {
      open_selected_topic_detail();
      open_initial_topic_detail_ = false;
    }
  } else {
    (void)select_initial_topic_if_present();
  }

  if (!monitor_allowed_topics_on_start_) {
    return;
  }

  bool all_allowed_topics_found = allowed_topic_names_.empty();
  std::vector<TopicEntry> topics_to_monitor;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!allowed_topic_names_.empty()) {
      all_allowed_topics_found = true;
      for (const auto & topic_name : allowed_topic_names_) {
        const auto found = topics_.find(topic_name);
        if (found == topics_.end()) {
          all_allowed_topics_found = false;
          continue;
        }
        if (!found->second.monitored) {
          topics_to_monitor.push_back(found->second);
        }
      }
    }
  }

  for (const auto & topic : topics_to_monitor) {
    start_monitoring(topic);
  }

  if (!topics_to_monitor.empty()) {
    std::lock_guard<std::mutex> lock(mutex_);
    status_line_ =
      topics_to_monitor.size() == 1
      ? "Monitoring filtered topic " + topics_to_monitor.front().name + "."
      : "Monitoring " + std::to_string(topics_to_monitor.size()) + " filtered topics.";
  }

  if (all_allowed_topics_found) {
    monitor_allowed_topics_on_start_ = false;
  }
}

bool TopicMonitorBackend::topic_is_allowed(const std::string & topic_name) const {
  return allowed_topic_names_.empty() || allowed_topic_names_.find(topic_name) != allowed_topic_names_.end();
}

bool TopicMonitorBackend::select_initial_topic_if_present() {
  if (initial_topic_name_.empty()) {
    return false;
  }

  expand_topic_namespace_path(topic_namespace(initial_topic_name_));
  const auto items = visible_topic_items();
  for (std::size_t index = 0; index < items.size(); ++index) {
    if (!items[index].is_namespace && items[index].row.name == initial_topic_name_) {
      selected_index_ = static_cast<int>(index);
      initial_topic_name_.clear();
      return true;
    }
  }
  return false;
}

void TopicMonitorBackend::expand_topic_namespace_path(const std::string & namespace_path) {
  if (namespace_path.empty()) {
    return;
  }

  std::size_t start = 0;
  while (start < namespace_path.size()) {
    const auto slash = namespace_path.find('/', start);
    const std::string path =
      slash == std::string::npos ? namespace_path : namespace_path.substr(0, slash);
    collapsed_topic_namespaces_[path] = false;
    if (slash == std::string::npos) {
      break;
    }
    start = slash + 1;
  }
}

void TopicMonitorBackend::warm_up_topic_list() {
  if (!topics_.empty()) {
    return;
  }

  const auto deadline = TopicClock::now() + std::chrono::milliseconds(1200);
  while (topics_.empty() && TopicClock::now() < deadline && rclcpp::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    refresh_topics();
  }
}

void TopicMonitorBackend::maybe_refresh_topics() {
  if (TopicClock::now() - last_refresh_time_ >= std::chrono::seconds(1)) {
    refresh_topics();
  }
}

std::vector<TopicRow> TopicMonitorBackend::topic_rows_snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<TopicRow> rows;
  rows.reserve(topics_.size());

  for (const auto & [name, entry] : topics_) {
    TopicRow row;
    row.name = name;
    row.type = entry.type;
    row.monitored = entry.monitored;
    row.stale = entry.monitored && entry.has_last_message &&
      ((TopicClock::now() - entry.last_message_time) > kStaleAfter);
    row.avg_hz = "-";
    row.min_max_hz = "-";
    row.bandwidth = "-";

    if (entry.monitored) {
      compute_stats(entry, row.avg_hz, row.min_max_hz, row.bandwidth);
    }

    if (show_only_monitored_ && !row.monitored) {
      continue;
    }

    rows.push_back(std::move(row));
  }

  return rows;
}

std::vector<DetailRow> TopicMonitorBackend::detail_rows_snapshot(const std::string & topic_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = topics_.find(topic_name);
  if (found == topics_.end()) {
    return {};
  }
  return found->second.detail_rows;
}

std::vector<DetailRow> TopicMonitorBackend::visible_detail_rows_snapshot(const std::string & topic_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = topics_.find(topic_name);
  if (found == topics_.end()) {
    return {};
  }

  std::vector<DetailRow> visible;
  visible.reserve(found->second.detail_rows.size());
  int hidden_depth = -1;
  for (std::size_t index = 0; index < found->second.detail_rows.size(); ++index) {
    const auto & row = found->second.detail_rows[index];
    if (hidden_depth >= 0) {
      if (row.depth > hidden_depth) {
        continue;
      }
      hidden_depth = -1;
    }
    visible.push_back(row);
    if (detail_row_has_children(found->second.detail_rows, index)) {
      const auto collapsed = collapsed_detail_paths_.find(row.path);
      if (collapsed != collapsed_detail_paths_.end() && collapsed->second) {
        hidden_depth = row.depth;
      }
    }
  }
  return visible;
}

std::vector<PlotSample> TopicMonitorBackend::plot_samples_snapshot(
  const std::string & topic_name, const std::string & field_path) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = topics_.find(topic_name);
  if (found == topics_.end()) {
    return {};
  }
  const auto sample_found = found->second.plot_samples.find(field_path);
  if (sample_found == found->second.plot_samples.end()) {
    return {};
  }
  return std::vector<PlotSample>(sample_found->second.begin(), sample_found->second.end());
}

std::string TopicMonitorBackend::detail_error_snapshot(const std::string & topic_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto found = topics_.find(topic_name);
  if (found == topics_.end()) {
    return "Topic is no longer available.";
  }
  return found->second.detail_error;
}

void TopicMonitorBackend::compute_stats(
  const TopicEntry & entry, std::string & avg_hz, std::string & min_max_hz, std::string & bandwidth)
{
  if (!entry.intervals.empty()) {
    const auto interval_sum = std::accumulate(
      entry.intervals.begin(), entry.intervals.end(), 0.0,
      [](double sum, const IntervalSample & interval) { return sum + interval.seconds; });

    if (interval_sum > 0.0) {
      const double average = static_cast<double>(entry.intervals.size()) / interval_sum;
      double min_period = entry.intervals.front().seconds;
      double max_period = entry.intervals.front().seconds;
      for (const auto & interval : entry.intervals) {
        min_period = std::min(min_period, interval.seconds);
        max_period = std::max(max_period, interval.seconds);
      }

      avg_hz = format_hz(average);
      min_max_hz = format_hz(1.0 / max_period) + "/" + format_hz(1.0 / min_period);
    }
  }

  if (entry.samples.size() >= 2) {
    const auto span = std::chrono::duration<double>(
      entry.samples.back().time - entry.samples.front().time).count();
    if (span > 0.0) {
      bandwidth = format_bandwidth(static_cast<double>(entry.sample_bytes_sum) / span);
    }
  }
}

void TopicMonitorBackend::toggle_selected_topic_monitoring() {
  const auto items = visible_topic_items();
  clamp_topic_selection(items);
  if (items.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(items.size())) {
    status_line_ = "No topic selected.";
    return;
  }

  const auto & item = items[static_cast<std::size_t>(selected_index_)];
  if (item.is_namespace) {
    status_line_ = "Select a topic, not a folder.";
    return;
  }

  const std::string topic_name = item.row.name;
  TopicEntry snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = topics_.find(topic_name);
    if (found == topics_.end()) {
      status_line_ = "Topic disappeared before monitoring could be changed.";
      return;
    }
    snapshot = found->second;
  }

  if (snapshot.monitored) {
    stop_monitoring(topic_name);
    return;
  }

  start_monitoring(snapshot);
}

void TopicMonitorBackend::open_selected_topic_detail() {
  const auto items = visible_topic_items();
  clamp_topic_selection(items);
  if (items.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(items.size())) {
    status_line_ = "No topic selected.";
    return;
  }

  const auto & item = items[static_cast<std::size_t>(selected_index_)];
  if (item.is_namespace) {
    status_line_ = "Select a topic, not a folder.";
    return;
  }

  detail_topic_name_ = item.row.name;
  detail_scroll_ = 0;
  selected_detail_index_ = 0;
  collapsed_detail_paths_.clear();
  view_mode_ = TopicMonitorViewMode::TopicDetail;

  TopicEntry snapshot;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = topics_.find(detail_topic_name_);
    if (found == topics_.end()) {
      status_line_ = "Topic disappeared.";
      view_mode_ = TopicMonitorViewMode::TopicList;
      detail_topic_name_.clear();
      return;
    }
    snapshot = found->second;
  }

  if (!snapshot.monitored) {
    start_monitoring(snapshot);
  }

  if (snapshot.has_last_message) {
    status_line_ = "Inspecting " + detail_topic_name_ + ".";
  } else {
    status_line_ = "Waiting for data on " + detail_topic_name_ + ".";
  }
}

void TopicMonitorBackend::close_topic_detail() {
  view_mode_ = TopicMonitorViewMode::TopicList;
  detail_topic_name_.clear();
  detail_scroll_ = 0;
  selected_detail_index_ = 0;
  collapsed_detail_paths_.clear();
  status_line_ = "Returned to topic list.";
}

rclcpp::QoS TopicMonitorBackend::subscription_qos_for_topic(const std::string & topic_name) const {
  try {
    const auto publishers = this->get_publishers_info_by_topic(topic_name);
    std::vector<rclcpp::QoS> publisher_qos_profiles;
    publisher_qos_profiles.reserve(publishers.size());
    for (const auto & publisher : publishers) {
      publisher_qos_profiles.push_back(publisher.qos_profile());
    }
    return compatible_subscription_qos(publisher_qos_profiles);
  } catch (const std::exception &) {
    return compatible_subscription_qos({});
  }
}

rclcpp::QoS TopicMonitorBackend::compatible_subscription_qos(
  const std::vector<rclcpp::QoS> & publisher_qos_profiles) const
{
  std::size_t depth = 10;
  bool require_best_effort = false;
  bool require_volatile = false;
  bool saw_transient_local = false;

  for (const auto & publisher_qos : publisher_qos_profiles) {
    if (publisher_qos.history() == rclcpp::HistoryPolicy::KeepLast) {
      depth = std::max(depth, publisher_qos.depth());
    }

    if (publisher_qos.reliability() != rclcpp::ReliabilityPolicy::Reliable) {
      require_best_effort = true;
    }

    if (publisher_qos.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      saw_transient_local = true;
    } else {
      require_volatile = true;
    }
  }

  rclcpp::QoS qos{rclcpp::KeepLast(depth)};
  if (require_best_effort) {
    qos.best_effort();
  } else {
    qos.reliable();
  }

  if (saw_transient_local && !require_volatile) {
    qos.transient_local();
  } else {
    qos.durability_volatile();
  }

  qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
  return qos;
}

void TopicMonitorBackend::start_monitoring(const TopicEntry & entry) {
  try {
    const auto subscription_qos = subscription_qos_for_topic(entry.name);
    auto subscription = this->create_generic_subscription(
      entry.name,
      entry.type,
      subscription_qos,
      [this, topic_name = entry.name](std::shared_ptr<rclcpp::SerializedMessage> message) {
        on_message(topic_name, *message);
      });

    std::lock_guard<std::mutex> lock(mutex_);
    auto found = topics_.find(entry.name);
    if (found == topics_.end()) {
      return;
    }
    found->second.monitored = true;
    found->second.monitor_error.clear();
    monitored_subscriptions_.push_back({entry.name, subscription});
    status_line_ = "Monitoring " + entry.name + ".";
  } catch (const std::exception & exception) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto found = topics_.find(entry.name);
    if (found != topics_.end()) {
      found->second.monitor_error = exception.what();
    }
    status_line_ = "Monitor failed for " + entry.name + ": " + exception.what();
  }
}

void TopicMonitorBackend::stop_monitoring(const std::string & topic_name) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto found = topics_.find(topic_name);
  if (found != topics_.end()) {
    found->second.monitored = false;
    found->second.monitor_error.clear();
    found->second.samples.clear();
    found->second.intervals.clear();
    found->second.sample_bytes_sum = 0;
    found->second.has_last_message = false;
    found->second.detail_rows.clear();
    found->second.plot_samples.clear();
    found->second.detail_error.clear();
  }
  monitored_subscriptions_.erase(
    std::remove_if(
      monitored_subscriptions_.begin(),
      monitored_subscriptions_.end(),
      [&topic_name](const auto & item) { return item.first == topic_name; }),
    monitored_subscriptions_.end());
  status_line_ = "Stopped monitoring " + topic_name + ".";
}

void TopicMonitorBackend::on_message(
  const std::string & topic_name, const rclcpp::SerializedMessage & message)
{
  std::vector<DetailRow> detail_rows;
  std::string detail_error;
  {
    TopicEntry snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = topics_.find(topic_name);
      if (found == topics_.end() || !found->second.monitored) {
        return;
      }
      snapshot = found->second;
    }

    try {
      detail_rows = decode_detail_rows(snapshot.type, message);
    } catch (const std::exception & exception) {
      detail_error = exception.what();
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  auto found = topics_.find(topic_name);
  if (found == topics_.end() || !found->second.monitored) {
    return;
  }

  auto & entry = found->second;
  const auto now = TopicClock::now();
  const auto cutoff = now - kWindowDuration;
  const std::size_t bytes = message.size();

  if (entry.has_last_message) {
    const double seconds = std::chrono::duration<double>(now - entry.last_message_time).count();
    if (seconds > 0.0) {
      entry.intervals.push_back({now, seconds});
    }
  }

  entry.has_last_message = true;
  entry.last_message_time = now;
  entry.samples.push_back({now, bytes});
  entry.sample_bytes_sum += bytes;
  entry.detail_rows = std::move(detail_rows);
  entry.detail_error = std::move(detail_error);
  for (const auto & row : entry.detail_rows) {
    if (!row.numeric) {
      continue;
    }
    auto & samples = entry.plot_samples[row.path];
    samples.push_back({now, row.numeric_value});
    while (!samples.empty() && samples.front().time < cutoff) {
      samples.pop_front();
    }
  }

  while (!entry.samples.empty() && entry.samples.front().time < cutoff) {
    entry.sample_bytes_sum -= entry.samples.front().bytes;
    entry.samples.pop_front();
  }
  while (!entry.intervals.empty() && entry.intervals.front().time < cutoff) {
    entry.intervals.pop_front();
  }
}

TopicIntrospection & TopicMonitorBackend::get_or_create_introspection(const std::string & type) {
  auto found = introspection_cache_.find(type);
  if (found != introspection_cache_.end()) {
    return found->second;
  }

  TopicIntrospection introspection;
  introspection.cpp_library = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
  introspection.introspection_library = rclcpp::get_typesupport_library(
    type, "rosidl_typesupport_introspection_cpp");
  introspection.cpp_typesupport = rclcpp::get_message_typesupport_handle(
    type, "rosidl_typesupport_cpp", *introspection.cpp_library);
  const auto * introspection_handle = rclcpp::get_message_typesupport_handle(
    type, "rosidl_typesupport_introspection_cpp", *introspection.introspection_library);
  introspection.members = static_cast<const MessageMembers *>(introspection_handle->data);
  introspection.serialization = std::make_unique<rclcpp::SerializationBase>(introspection.cpp_typesupport);

  auto inserted = introspection_cache_.emplace(type, std::move(introspection));
  return inserted.first->second;
}

std::vector<DetailRow> TopicMonitorBackend::decode_detail_rows(
  const std::string & type, const rclcpp::SerializedMessage & message)
{
  auto & introspection = get_or_create_introspection(type);
  std::vector<uint8_t> storage(introspection.members->size_of_);
  introspection.members->init_function(storage.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

  try {
    introspection.serialization->deserialize_message(&message, storage.data());
    std::vector<DetailRow> rows;
    rows.reserve(introspection.members->member_count_ * 2U);
    append_message_members(rows, 0, "", introspection.members, storage.data());
    introspection.members->fini_function(storage.data());
    return rows;
  } catch (...) {
    introspection.members->fini_function(storage.data());
    throw;
  }
}

void TopicMonitorBackend::append_message_members(
  std::vector<DetailRow> & rows, int depth, const std::string & parent_path,
  const MessageMembers * members, const void * message_memory) const
{
  for (uint32_t index = 0; index < members->member_count_; ++index) {
    const auto & member = members->members_[index];
    const auto * field_memory = static_cast<const uint8_t *>(message_memory) + member.offset_;
    append_member(
      rows, depth, parent_path, normalize_member_label(*members, member), member, field_memory);
  }
}

std::string TopicMonitorBackend::normalize_member_label(
  const MessageMembers & parent_members, const MessageMember & member) const
{
  if (std::strcmp(member.name_, "structure_needs_at_least_one_member") != 0) {
    return member.name_;
  }
  if (std::strcmp(parent_members.message_name_, "Empty") == 0) {
    return "empty";
  }
  return "empty";
}

void TopicMonitorBackend::append_member(
  std::vector<DetailRow> & rows, int depth, const std::string & parent_path, const std::string & label,
  const MessageMember & member, const void * field_memory) const
{
  const std::string row_path = parent_path.empty() ? label : (parent_path + "." + label);
  if (member.is_array_) {
    const std::size_t element_count =
      member.size_function != nullptr ? member.size_function(field_memory) : member.array_size_;
    rows.push_back({depth, label, "[" + std::to_string(element_count) + "]", row_path});

    for (std::size_t index = 0; index < element_count; ++index) {
      const std::string child_label = "[" + std::to_string(index) + "]";
      const std::string child_path = row_path + child_label;
      if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
        const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
        const void * element_memory =
          member.get_const_function != nullptr ? member.get_const_function(field_memory, index) : nullptr;
        if (element_memory != nullptr) {
          rows.push_back({depth + 1, child_label, "<message>", child_path});
          append_message_members(rows, depth + 2, child_path, sub_members, element_memory);
        }
        continue;
      }
      DetailRow row;
      row.depth = depth + 1;
      row.field = child_label;
      row.value = array_element_to_string(member, field_memory, index);
      row.path = child_path;
      rows.push_back(std::move(row));
    }
    return;
  }

  if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
    rows.push_back({depth, label, "<message>", row_path});
    const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
    append_message_members(rows, depth + 1, row_path, sub_members, field_memory);
    return;
  }

  DetailRow row;
  row.depth = depth;
  row.field = label;
  row.value = scalar_field_to_string(member.type_id_, field_memory);
  row.path = row_path;
  const auto numeric = scalar_field_to_double(member.type_id_, field_memory);
  if (numeric.has_value()) {
    row.numeric = true;
    row.numeric_value = *numeric;
  }
  rows.push_back(std::move(row));
}

std::string TopicMonitorBackend::array_element_to_string(
  const MessageMember & member, const void * field_memory, std::size_t index) const
{
  switch (member.type_id_) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN: {
      bool value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8: {
      uint8_t value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8: {
      int8_t value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16: {
      uint16_t value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16: {
      int16_t value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32: {
      uint32_t value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32: {
      int32_t value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64: {
      uint64_t value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64: {
      int64_t value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT: {
      float value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE: {
      double value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE: {
      long double value{};
      member.fetch_function(field_memory, index, &value);
      return scalar_to_string(value);
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR: {
      signed char value{};
      member.fetch_function(field_memory, index, &value);
      return std::string(1, static_cast<char>(value));
    }
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING: {
      std::string value;
      member.fetch_function(field_memory, index, &value);
      return value;
    }
    default:
      return "<value>";
  }
}

std::string TopicMonitorBackend::scalar_field_to_string(uint8_t type_id, const void * field_memory) const {
  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      return scalar_to_string(*static_cast<const bool *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return scalar_to_string(*static_cast<const uint8_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return scalar_to_string(*static_cast<const int8_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return scalar_to_string(*static_cast<const uint16_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return scalar_to_string(*static_cast<const int16_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return scalar_to_string(*static_cast<const uint32_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return scalar_to_string(*static_cast<const int32_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return scalar_to_string(*static_cast<const uint64_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return scalar_to_string(*static_cast<const int64_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      return scalar_to_string(*static_cast<const float *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      return scalar_to_string(*static_cast<const double *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      return scalar_to_string(*static_cast<const long double *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
      return std::string(1, *static_cast<const char *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
      return *static_cast<const std::string *>(field_memory);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_WSTRING:
      return "<wstring>";
    default:
      return "<value>";
  }
}

std::optional<double> TopicMonitorBackend::scalar_field_to_double(
  uint8_t type_id, const void * field_memory) const
{
  switch (type_id) {
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN:
      return *static_cast<const bool *>(field_memory) ? 1.0 : 0.0;
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_OCTET:
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
      return static_cast<double>(*static_cast<const uint8_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
      return static_cast<double>(*static_cast<const int8_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
      return static_cast<double>(*static_cast<const uint16_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
      return static_cast<double>(*static_cast<const int16_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
      return static_cast<double>(*static_cast<const uint32_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
      return static_cast<double>(*static_cast<const int32_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
      return static_cast<double>(*static_cast<const uint64_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
      return static_cast<double>(*static_cast<const int64_t *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
      return static_cast<double>(*static_cast<const float *>(field_memory));
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
      return *static_cast<const double *>(field_memory);
    case rosidl_typesupport_introspection_cpp::ROS_TYPE_LONG_DOUBLE:
      return static_cast<double>(*static_cast<const long double *>(field_memory));
    default:
      return std::nullopt;
  }
}

void TopicMonitorBackend::clamp_topic_selection(const std::vector<TopicListItem> & items) {
  if (items.empty()) {
    selected_index_ = 0;
    list_scroll_ = 0;
    return;
  }
  selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(items.size()) - 1);
}

std::string TopicMonitorBackend::topic_namespace(const std::string & topic_name) const {
  const auto slash = topic_name.find_last_of('/');
  if (slash == std::string::npos || slash == 0) {
    return "";
  }
  return topic_name.substr(1, slash - 1);
}

std::string TopicMonitorBackend::topic_leaf_name(const std::string & topic_name) const {
  const auto slash = topic_name.find_last_of('/');
  if (slash == std::string::npos) {
    return topic_name;
  }
  return topic_name.substr(slash + 1);
}

bool TopicMonitorBackend::is_topic_namespace_expanded(const std::string & namespace_path) const {
  const auto found = collapsed_topic_namespaces_.find(namespace_path);
  if (found == collapsed_topic_namespaces_.end()) {
    return false;
  }
  return !found->second;
}

std::vector<TopicListItem> TopicMonitorBackend::visible_topic_items() const {
  const auto rows = topic_rows_snapshot();
  std::vector<TopicListItem> items;
  std::map<std::string, bool> emitted_namespaces;

  for (const auto & row : rows) {
    const std::string ns = topic_namespace(row.name);
    bool hidden = false;
    if (!ns.empty()) {
      std::size_t start = 0;
      int depth = 0;
      while (start < ns.size()) {
        const auto slash = ns.find('/', start);
        const std::string part =
          slash == std::string::npos ? ns.substr(start) : ns.substr(start, slash - start);
        const std::string path = ns.substr(0, slash == std::string::npos ? ns.size() : slash);
        if (!emitted_namespaces[path]) {
          items.push_back({true, part, path, depth, {}});
          emitted_namespaces[path] = true;
        }
        if (!is_topic_namespace_expanded(path)) {
          hidden = true;
          break;
        }
        if (slash == std::string::npos) {
          break;
        }
        start = slash + 1;
        ++depth;
      }
    }

    if (!hidden) {
      TopicListItem item;
      item.is_namespace = false;
      item.label = topic_leaf_name(row.name);
      item.namespace_path = ns;
      item.depth = ns.empty() ? 0 : static_cast<int>(std::count(ns.begin(), ns.end(), '/')) + 1;
      item.row = row;
      items.push_back(std::move(item));
    }
  }

  return items;
}

void TopicMonitorBackend::expand_selected_namespace() {
  const auto items = visible_topic_items();
  if (items.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(items.size())) {
    return;
  }
  const auto & item = items[static_cast<std::size_t>(selected_index_)];
  if (!item.is_namespace) {
    return;
  }
  collapsed_topic_namespaces_[item.namespace_path] = false;
}

void TopicMonitorBackend::collapse_selected_namespace() {
  const auto items = visible_topic_items();
  if (items.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(items.size())) {
    return;
  }
  const auto & item = items[static_cast<std::size_t>(selected_index_)];
  if (item.is_namespace) {
    collapsed_topic_namespaces_[item.namespace_path] = true;
    return;
  }

  if (item.namespace_path.empty()) {
    return;
  }

  for (std::size_t index = 0; index < items.size(); ++index) {
    if (items[index].is_namespace && items[index].namespace_path == item.namespace_path) {
      selected_index_ = static_cast<int>(index);
      collapsed_topic_namespaces_[item.namespace_path] = true;
      return;
    }
  }
}

void TopicMonitorBackend::clamp_detail_selection(const std::vector<DetailRow> & rows) {
  if (rows.empty()) {
    selected_detail_index_ = 0;
    detail_scroll_ = 0;
    return;
  }
  selected_detail_index_ = std::clamp(selected_detail_index_, 0, static_cast<int>(rows.size()) - 1);
}

void TopicMonitorBackend::clamp_visible_detail_selection() {
  const auto rows = visible_detail_rows_snapshot(detail_topic_name_);
  clamp_detail_selection(rows);
}

void TopicMonitorBackend::expand_selected_detail_row() {
  const auto visible = visible_detail_rows_snapshot(detail_topic_name_);
  clamp_detail_selection(visible);
  if (visible.empty() || selected_detail_index_ < 0 || selected_detail_index_ >= static_cast<int>(visible.size())) {
    return;
  }
  const auto & row = visible[static_cast<std::size_t>(selected_detail_index_)];
  collapsed_detail_paths_[row.path] = false;
}

void TopicMonitorBackend::collapse_selected_detail_row() {
  const auto visible = visible_detail_rows_snapshot(detail_topic_name_);
  clamp_detail_selection(visible);
  if (visible.empty() || selected_detail_index_ < 0 || selected_detail_index_ >= static_cast<int>(visible.size())) {
    return;
  }
  const auto & row = visible[static_cast<std::size_t>(selected_detail_index_)];
  const auto all = detail_rows_snapshot(detail_topic_name_);
  for (std::size_t index = 0; index < all.size(); ++index) {
    if (all[index].path != row.path) {
      continue;
    }
    if (detail_row_has_children(all, index)) {
      collapsed_detail_paths_[row.path] = true;
      return;
    }
    break;
  }
}

bool TopicMonitorBackend::selected_detail_row_can_plot() const {
  const auto row = selected_visible_detail_row();
  return row.has_value() && row->numeric;
}

std::optional<DetailRow> TopicMonitorBackend::selected_visible_detail_row() const {
  const auto visible = visible_detail_rows_snapshot(detail_topic_name_);
  if (visible.empty() || selected_detail_index_ < 0 || selected_detail_index_ >= static_cast<int>(visible.size())) {
    return std::nullopt;
  }
  return visible[static_cast<std::size_t>(selected_detail_index_)];
}

bool TopicMonitorBackend::detail_row_has_children(
  const std::vector<DetailRow> & rows, std::size_t index) const
{
  return index + 1 < rows.size() && rows[index + 1].depth > rows[index].depth;
}

}  // namespace ros2_console_tools
