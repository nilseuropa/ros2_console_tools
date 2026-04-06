#include <ncurses.h>

#include <algorithm>
#include <chrono>
#include <clocale>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <deque>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/typesupport_helpers.hpp>
#include <rcpputils/shared_library.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace ros2_console_tools {

namespace {

using Clock = std::chrono::steady_clock;
using MessageMember = rosidl_typesupport_introspection_cpp::MessageMember;
using MessageMembers = rosidl_typesupport_introspection_cpp::MessageMembers;

enum ColorPairId {
  kColorFrame = 1,
  kColorTitle = 2,
  kColorHeader = 3,
  kColorSelection = 4,
  kColorStatus = 5,
  kColorHelp = 6,
  kColorMonitored = 7,
  kColorMonitoredSelection = 8,
  kColorStale = 9,
  kColorStaleSelection = 10,
};

enum class ViewMode {
  TopicList,
  TopicDetail,
};

class NcursesSession {
public:
  NcursesSession() {
    std::setlocale(LC_ALL, "");
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(100);
    if (has_colors()) {
      start_color();
      use_default_colors();
      init_pair(kColorFrame, COLOR_CYAN, -1);
      init_pair(kColorTitle, COLOR_WHITE, -1);
      init_pair(kColorHeader, COLOR_YELLOW, -1);
      init_pair(kColorSelection, COLOR_BLACK, COLOR_YELLOW);
      init_pair(kColorStatus, COLOR_GREEN, -1);
      init_pair(kColorHelp, COLOR_BLACK, COLOR_CYAN);
      init_pair(kColorMonitored, COLOR_GREEN, -1);
      init_pair(kColorMonitoredSelection, COLOR_GREEN, COLOR_YELLOW);
      init_pair(kColorStale, COLOR_RED, -1);
      init_pair(kColorStaleSelection, COLOR_RED, COLOR_YELLOW);
    }
  }

  ~NcursesSession() {
    curs_set(1);
    endwin();
  }
};

std::string truncate_text(const std::string & text, int width) {
  if (width <= 0) {
    return "";
  }
  if (static_cast<int>(text.size()) <= width) {
    return text;
  }
  if (width <= 3) {
    return text.substr(0, static_cast<std::size_t>(width));
  }
  return text.substr(0, static_cast<std::size_t>(width - 3)) + "...";
}

std::string format_hz(double hz) {
  if (hz <= 0.0) {
    return "-";
  }

  std::ostringstream stream;
  if (hz >= 100.0) {
    stream << std::fixed << std::setprecision(0) << hz;
  } else if (hz >= 10.0) {
    stream << std::fixed << std::setprecision(1) << hz;
  } else {
    stream << std::fixed << std::setprecision(2) << hz;
  }
  return stream.str();
}

std::string format_bandwidth(double bytes_per_second) {
  if (bytes_per_second <= 0.0) {
    return "-";
  }

  static const char * kUnits[] = {"B/s", "KB/s", "MB/s", "GB/s"};
  int unit_index = 0;
  while (bytes_per_second >= 1024.0 && unit_index < 3) {
    bytes_per_second /= 1024.0;
    ++unit_index;
  }

  std::ostringstream stream;
  if (bytes_per_second >= 100.0) {
    stream << std::fixed << std::setprecision(0);
  } else if (bytes_per_second >= 10.0) {
    stream << std::fixed << std::setprecision(1);
  } else {
    stream << std::fixed << std::setprecision(2);
  }
  stream << bytes_per_second << kUnits[unit_index];
  return stream.str();
}

template<typename T>
std::string scalar_to_string(const T & value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

template<>
std::string scalar_to_string<bool>(const bool & value) {
  return value ? "true" : "false";
}

template<>
std::string scalar_to_string<uint8_t>(const uint8_t & value) {
  return std::to_string(static_cast<unsigned int>(value));
}

template<>
std::string scalar_to_string<int8_t>(const int8_t & value) {
  return std::to_string(static_cast<int>(value));
}

struct Sample {
  Clock::time_point time;
  std::size_t bytes;
};

struct IntervalSample {
  Clock::time_point time;
  double seconds;
};

struct DetailRow {
  int depth{0};
  std::string field;
  std::string value;
};

struct TopicIntrospection {
  std::shared_ptr<rcpputils::SharedLibrary> cpp_library;
  std::shared_ptr<rcpputils::SharedLibrary> introspection_library;
  const rosidl_message_type_support_t * cpp_typesupport{nullptr};
  const MessageMembers * members{nullptr};
  std::unique_ptr<rclcpp::SerializationBase> serialization;
};

struct TopicEntry {
  std::string name;
  std::string type;
  bool monitored{false};
  std::string monitor_error;
  bool has_last_message{false};
  Clock::time_point last_message_time{};
  std::deque<Sample> samples;
  std::deque<IntervalSample> intervals;
  std::size_t sample_bytes_sum{0};
  std::vector<DetailRow> detail_rows;
  std::string detail_error;
};

struct TopicRow {
  std::string name;
  std::string type;
  bool monitored{false};
  bool stale{false};
  std::string avg_hz;
  std::string min_max_hz;
  std::string bandwidth;
};

struct TopicListItem {
  bool is_namespace{false};
  std::string label;
  std::string namespace_path;
  int depth{0};
  TopicRow row;
};

class TopicMonitorNode : public rclcpp::Node {
public:
  TopicMonitorNode()
  : Node("topic_monitor") {}

  int run() {
    NcursesSession ncurses_session;
    refresh_topics();
    warm_up_topic_list();

    bool running = true;
    while (running && rclcpp::ok()) {
      maybe_refresh_topics();
      draw();
      const int key = getch();
      if (key == ERR) {
        continue;
      }
      running = handle_key(key);
    }

    return 0;
  }

private:
  static constexpr auto kWindowDuration = std::chrono::seconds(5);
  static constexpr auto kStaleAfter = std::chrono::milliseconds(1500);

  bool handle_key(int key) {
    switch (view_mode_) {
      case ViewMode::TopicList:
        return handle_topic_list_key(key);
      case ViewMode::TopicDetail:
        return handle_topic_detail_key(key);
    }
    return true;
  }

  bool handle_topic_list_key(int key) {
    const auto items = visible_topic_items();
    clamp_topic_selection(items);

    switch (key) {
      case KEY_F(10):
        return false;
      case KEY_F(4):
        refresh_topics();
        return true;
      case KEY_F(5):
        show_only_monitored_ = !show_only_monitored_;
        selected_index_ = 0;
        list_scroll_ = 0;
        status_line_ = show_only_monitored_
          ? "Showing monitored topics only."
          : "Showing all topics.";
        return true;
      case KEY_UP:
      case 'k':
        if (selected_index_ > 0) {
          --selected_index_;
        }
        return true;
      case KEY_DOWN:
      case 'j':
        if (selected_index_ + 1 < static_cast<int>(items.size())) {
          ++selected_index_;
        }
        return true;
      case KEY_PPAGE:
        selected_index_ = std::max(0, selected_index_ - page_step());
        return true;
      case KEY_NPAGE:
        if (!items.empty()) {
          selected_index_ = std::min(static_cast<int>(items.size()) - 1, selected_index_ + page_step());
        }
        return true;
      case KEY_RIGHT:
      case 'l':
        expand_selected_namespace();
        return true;
      case KEY_LEFT:
      case 'h':
        collapse_selected_namespace();
        return true;
      case ' ':
      case KEY_IC:
        toggle_selected_topic_monitoring();
        return true;
      case '\n':
      case KEY_ENTER:
        open_selected_topic_detail();
        return true;
      default:
        return true;
    }
  }

  bool handle_topic_detail_key(int key) {
    const auto rows = detail_rows_snapshot(detail_topic_name_);
    clamp_detail_selection(rows);

    switch (key) {
      case KEY_F(10):
        return false;
      case 27:
        close_topic_detail();
        return true;
      case KEY_F(4):
        refresh_topics();
        return true;
      case KEY_UP:
      case 'k':
        if (selected_detail_index_ > 0) {
          --selected_detail_index_;
        }
        return true;
      case KEY_DOWN:
      case 'j':
        if (selected_detail_index_ + 1 < static_cast<int>(rows.size())) {
          ++selected_detail_index_;
        }
        return true;
      case KEY_PPAGE:
        selected_detail_index_ = std::max(0, selected_detail_index_ - page_step());
        return true;
      case KEY_NPAGE:
        if (!rows.empty()) {
          selected_detail_index_ = std::min(
            static_cast<int>(rows.size()) - 1,
            selected_detail_index_ + page_step());
        }
        return true;
      default:
        return true;
    }
  }

  int page_step() const {
    int rows = 0;
    int columns = 0;
    getmaxyx(stdscr, rows, columns);
    (void)columns;
    return std::max(5, rows - 8);
  }

  void refresh_topics() {
    const auto discovered_topics = this->get_topic_names_and_types();

    std::lock_guard<std::mutex> lock(mutex_);
    std::map<std::string, TopicEntry> refreshed;
    for (const auto & [name, types] : discovered_topics) {
      if (name.empty()) {
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
    last_refresh_time_ = Clock::now();
    status_line_ = "Loaded " + std::to_string(topics_.size()) + " topics. Space monitors the selected topic.";
  }

  void warm_up_topic_list() {
    if (!topics_.empty()) {
      return;
    }

    const auto deadline = Clock::now() + std::chrono::milliseconds(1200);
    while (topics_.empty() && Clock::now() < deadline && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      refresh_topics();
    }
  }

  void maybe_refresh_topics() {
    if (Clock::now() - last_refresh_time_ >= std::chrono::seconds(1)) {
      refresh_topics();
    }
  }

  std::vector<TopicRow> topic_rows_snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<TopicRow> rows;
    rows.reserve(topics_.size());

    for (const auto & [name, entry] : topics_) {
      TopicRow row;
      row.name = name;
      row.type = entry.type;
      row.monitored = entry.monitored;
      row.stale = entry.monitored && entry.has_last_message &&
        ((Clock::now() - entry.last_message_time) > kStaleAfter);
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

  std::vector<DetailRow> detail_rows_snapshot(const std::string & topic_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = topics_.find(topic_name);
    if (found == topics_.end()) {
      return {};
    }
    return found->second.detail_rows;
  }

  std::string detail_error_snapshot(const std::string & topic_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = topics_.find(topic_name);
    if (found == topics_.end()) {
      return "Topic is no longer available.";
    }
    return found->second.detail_error;
  }

  static void compute_stats(
    const TopicEntry & entry,
    std::string & avg_hz,
    std::string & min_max_hz,
    std::string & bandwidth)
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

  void toggle_selected_topic_monitoring() {
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

  void open_selected_topic_detail() {
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
    view_mode_ = ViewMode::TopicDetail;

    TopicEntry snapshot;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = topics_.find(detail_topic_name_);
      if (found == topics_.end()) {
        status_line_ = "Topic disappeared.";
        view_mode_ = ViewMode::TopicList;
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

  void close_topic_detail() {
    view_mode_ = ViewMode::TopicList;
    detail_topic_name_.clear();
    detail_scroll_ = 0;
    selected_detail_index_ = 0;
    status_line_ = "Returned to topic list.";
  }

  void start_monitoring(const TopicEntry & entry) {
    try {
      auto subscription = this->create_generic_subscription(
        entry.name,
        entry.type,
        rclcpp::QoS(rclcpp::KeepLast(10)),
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

  void stop_monitoring(const std::string & topic_name) {
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

  void on_message(const std::string & topic_name, const rclcpp::SerializedMessage & message) {
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
    const auto now = Clock::now();
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

    const auto cutoff = now - kWindowDuration;
    while (!entry.samples.empty() && entry.samples.front().time < cutoff) {
      entry.sample_bytes_sum -= entry.samples.front().bytes;
      entry.samples.pop_front();
    }
    while (!entry.intervals.empty() && entry.intervals.front().time < cutoff) {
      entry.intervals.pop_front();
    }
  }

  TopicIntrospection & get_or_create_introspection(const std::string & type) {
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

  std::vector<DetailRow> decode_detail_rows(
    const std::string & type,
    const rclcpp::SerializedMessage & message)
  {
    auto & introspection = get_or_create_introspection(type);
    std::vector<uint8_t> storage(introspection.members->size_of_);
    introspection.members->init_function(
      storage.data(), rosidl_runtime_cpp::MessageInitialization::ALL);

    try {
      introspection.serialization->deserialize_message(&message, storage.data());
      std::vector<DetailRow> rows;
      rows.reserve(introspection.members->member_count_ * 2U);
      append_message_members(rows, 0, introspection.members, storage.data());
      introspection.members->fini_function(storage.data());
      return rows;
    } catch (...) {
      introspection.members->fini_function(storage.data());
      throw;
    }
  }

  void append_message_members(
    std::vector<DetailRow> & rows,
    int depth,
    const MessageMembers * members,
    const void * message_memory) const
  {
    for (uint32_t index = 0; index < members->member_count_; ++index) {
      const auto & member = members->members_[index];
      const auto * field_memory = static_cast<const uint8_t *>(message_memory) + member.offset_;
      append_member(rows, depth, normalize_member_label(*members, member), member, field_memory);
    }
  }

  std::string normalize_member_label(
    const MessageMembers & parent_members,
    const MessageMember & member) const
  {
    if (std::strcmp(member.name_, "structure_needs_at_least_one_member") != 0) {
      return member.name_;
    }

    if (std::strcmp(parent_members.message_name_, "Empty") == 0) {
      return "empty";
    }

    return "empty";
  }

  void append_member(
    std::vector<DetailRow> & rows,
    int depth,
    const std::string & label,
    const MessageMember & member,
    const void * field_memory) const
  {
    if (member.is_array_) {
      const std::size_t element_count =
        member.size_function != nullptr ? member.size_function(field_memory) : member.array_size_;
      rows.push_back({depth, label, "[" + std::to_string(element_count) + "]"});

      for (std::size_t index = 0; index < element_count; ++index) {
        const std::string child_label = "[" + std::to_string(index) + "]";
        if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
          const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
          const void * element_memory =
            member.get_const_function != nullptr ? member.get_const_function(field_memory, index) : nullptr;
          if (element_memory != nullptr) {
            rows.push_back({depth + 1, child_label, "<message>"});
            append_message_members(rows, depth + 2, sub_members, element_memory);
          }
          continue;
        }

        rows.push_back({depth + 1, child_label, array_element_to_string(member, field_memory, index)});
      }
      return;
    }

    if (member.type_id_ == rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE) {
      rows.push_back({depth, label, "<message>"});
      const auto * sub_members = static_cast<const MessageMembers *>(member.members_->data);
      append_message_members(rows, depth + 1, sub_members, field_memory);
      return;
    }

    rows.push_back({depth, label, scalar_field_to_string(member.type_id_, field_memory)});
  }

  std::string array_element_to_string(
    const MessageMember & member,
    const void * field_memory,
    std::size_t index) const
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

  std::string scalar_field_to_string(uint8_t type_id, const void * field_memory) const {
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

  void draw() {
    erase();

    int rows = 0;
    int columns = 0;
    getmaxyx(stdscr, rows, columns);
    const int help_row = rows - 1;
    const int status_row = rows - 2;
    const int content_bottom = std::max(1, status_row - 1);

    draw_box(0, 0, content_bottom, columns - 1);
    mvprintw(0, 1, "Topic Monitor ");
    if (view_mode_ == ViewMode::TopicList) {
      draw_topic_list(1, 1, content_bottom - 1, columns - 2);
    } else {
      draw_topic_detail(1, 1, content_bottom - 1, columns - 2);
    }
    draw_status_line(status_row, columns);
    draw_help_line(help_row, columns);

    refresh();
  }

  void draw_box(int top, int left, int bottom, int right) const {
    attron(COLOR_PAIR(kColorFrame));
    mvaddch(top, left, ACS_ULCORNER);
    mvaddch(top, right, ACS_URCORNER);
    mvaddch(bottom, left, ACS_LLCORNER);
    mvaddch(bottom, right, ACS_LRCORNER);
    mvhline(top, left + 1, ACS_HLINE, right - left - 1);
    mvhline(bottom, left + 1, ACS_HLINE, right - left - 1);
    mvvline(top + 1, left, ACS_VLINE, bottom - top - 1);
    mvvline(top + 1, right, ACS_VLINE, bottom - top - 1);
    attroff(COLOR_PAIR(kColorFrame));
  }

  void draw_topic_list(int top, int left, int bottom, int right) {
    const auto items = visible_topic_items();
    clamp_topic_selection(items);

    const int visible_rows = std::max(1, bottom - top + 1);
    const int width = right - left + 1;
    const int topic_width = std::max(24, width / 2);
    const int avg_width = std::max(8, width / 10);
    const int minmax_width = std::max(12, width / 8);
    const int bandwidth_width = std::max(12, width - topic_width - avg_width - minmax_width - 3);
    const int sep_one_x = left + topic_width;
    const int sep_two_x = sep_one_x + 1 + avg_width;
    const int sep_three_x = sep_two_x + 1 + minmax_width;

    if (selected_index_ < list_scroll_) {
      list_scroll_ = selected_index_;
    }
    if (selected_index_ >= list_scroll_ + visible_rows - 1) {
      list_scroll_ = std::max(0, selected_index_ - visible_rows + 2);
    }

    attron(COLOR_PAIR(kColorHeader));
    mvprintw(top, left, "%-*s", topic_width, "Topic");
    mvaddch(top, sep_one_x, ACS_VLINE);
    mvprintw(top, sep_one_x + 1, "%-*s", avg_width, "Avg Hz");
    mvaddch(top, sep_two_x, ACS_VLINE);
    mvprintw(top, sep_two_x + 1, "%-*s", minmax_width, "Min/Max Hz");
    mvaddch(top, sep_three_x, ACS_VLINE);
    mvprintw(top, sep_three_x + 1, "%-*s", bandwidth_width, "Bandwidth");
    attroff(COLOR_PAIR(kColorHeader));

    const int first_row = list_scroll_;
    const int last_row = std::min(static_cast<int>(items.size()), first_row + visible_rows - 1);
    for (int row = top + 1; row <= bottom; ++row) {
      const bool has_item = first_row + (row - top - 1) < last_row;
      const bool selected = has_item && (first_row + (row - top - 1) == selected_index_);

      mvhline(row, left, ' ', width);
      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      }

      chtype separator = selected ? '|' : ACS_VLINE;
      mvaddch(row, sep_one_x, separator);
      mvaddch(row, sep_two_x, separator);
      mvaddch(row, sep_three_x, separator);

      if (!has_item) {
        continue;
      }

      const auto & entry = items[static_cast<std::size_t>(first_row + (row - top - 1))];
      const std::string topic_text = std::string(static_cast<std::size_t>(entry.depth * 2), ' ') + entry.label;
      if (entry.is_namespace) {
        const int text_color = selected ? kColorSelection : kColorFrame;
        if (text_color != 0) {
          attron(COLOR_PAIR(text_color));
        }
        mvprintw(row, left, "%-*s", topic_width, truncate_text(topic_text, topic_width).c_str());
        if (text_color != 0) {
          attroff(COLOR_PAIR(text_color));
        }
        if (selected) {
          mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
          mvaddch(row, sep_one_x, '|');
          mvaddch(row, sep_two_x, '|');
          mvaddch(row, sep_three_x, '|');
          mvchgat(row, left, topic_width, A_NORMAL, kColorSelection, nullptr);
        }
        continue;
      }

      const auto & row_data = entry.row;
      const int text_color =
        row_data.stale
        ? (selected ? kColorStaleSelection : kColorStale)
        : row_data.monitored
        ? (selected ? kColorMonitoredSelection : kColorMonitored)
        : (selected ? kColorSelection : 0);
      if (text_color != 0) {
        attron(COLOR_PAIR(text_color));
      }
      mvprintw(row, left, "%-*s", topic_width, truncate_text(topic_text, topic_width).c_str());
      mvprintw(row, sep_one_x + 1, "%-*s", avg_width, truncate_text(row_data.avg_hz, avg_width).c_str());
      mvprintw(
        row, sep_two_x + 1, "%-*s", minmax_width,
        truncate_text(row_data.min_max_hz, minmax_width).c_str());
      mvprintw(
        row, sep_three_x + 1, "%-*s", bandwidth_width,
        truncate_text(row_data.bandwidth, bandwidth_width).c_str());
      if (text_color != 0) {
        attroff(COLOR_PAIR(text_color));
      }
      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
        mvaddch(row, sep_one_x, '|');
        mvaddch(row, sep_two_x, '|');
        mvaddch(row, sep_three_x, '|');
        if (text_color != 0) {
          mvchgat(row, left, topic_width, A_NORMAL, text_color, nullptr);
          mvchgat(row, sep_one_x + 1, avg_width, A_NORMAL, text_color, nullptr);
          mvchgat(row, sep_two_x + 1, minmax_width, A_NORMAL, text_color, nullptr);
          mvchgat(row, sep_three_x + 1, bandwidth_width, A_NORMAL, text_color, nullptr);
        }
      }
    }
  }

  void draw_topic_detail(int top, int left, int bottom, int right) {
    const auto rows = detail_rows_snapshot(detail_topic_name_);
    const auto detail_error = detail_error_snapshot(detail_topic_name_);
    clamp_detail_selection(rows);

    const int width = right - left + 1;
    const int visible_rows = std::max(1, bottom - top + 1);
    const int field_width = std::max(24, width / 2);
    const int value_width = std::max(16, width - field_width - 1);
    const int separator_x = left + field_width;

    if (selected_detail_index_ < detail_scroll_) {
      detail_scroll_ = selected_detail_index_;
    }
    if (selected_detail_index_ >= detail_scroll_ + visible_rows - 1) {
      detail_scroll_ = std::max(0, selected_detail_index_ - visible_rows + 2);
    }

    attron(COLOR_PAIR(kColorHeader));
    mvprintw(top, left, "%-*s", field_width, "Field");
    mvaddch(top, separator_x, ACS_VLINE);
    mvprintw(top, separator_x + 1, "%-*s", value_width, "Value");
    attroff(COLOR_PAIR(kColorHeader));

    const int first_row = detail_scroll_;
    const int last_row = std::min(static_cast<int>(rows.size()), first_row + visible_rows - 1);
    for (int row = top + 1; row <= bottom; ++row) {
      const bool has_item = first_row + (row - top - 1) < last_row;
      const bool selected = has_item && (first_row + (row - top - 1) == selected_detail_index_);

      mvhline(row, left, ' ', width);
      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
      }
      mvaddch(row, separator_x, selected ? '|' : ACS_VLINE);

      if (!has_item) {
        continue;
      }

      const auto & entry = rows[static_cast<std::size_t>(first_row + (row - top - 1))];
      const std::string indent(static_cast<std::size_t>(entry.depth * 2), ' ');
      const std::string field_text = indent + entry.field;
      mvprintw(row, left, "%-*s", field_width, truncate_text(field_text, field_width).c_str());
      mvprintw(
        row, separator_x + 1, "%-*s", value_width,
        truncate_text(entry.value, value_width).c_str());

      if (selected) {
        mvchgat(row, left, width, A_NORMAL, kColorSelection, nullptr);
        mvaddch(row, separator_x, '|');
      }
    }

    if (rows.empty()) {
      const std::string placeholder = detail_error.empty()
        ? "Waiting for the first message..."
        : "Decode error: " + detail_error;
      mvprintw(top + 1, left + 1, "%s", truncate_text(placeholder, width - 2).c_str());
    }
  }

  void draw_status_line(int row, int columns) const {
    attron(COLOR_PAIR(kColorStatus));
    mvhline(row, 0, ' ', columns);
    std::string line = status_line_;

    if (view_mode_ == ViewMode::TopicDetail && !detail_topic_name_.empty()) {
      std::lock_guard<std::mutex> lock(mutex_);
      const auto found = topics_.find(detail_topic_name_);
      if (found != topics_.end()) {
        line = truncate_text(
          detail_topic_name_ + " [" + found->second.type + "]  " + status_line_,
          columns - 1);
      }
    } else {
      const auto items = visible_topic_items();
      if (!items.empty() && selected_index_ >= 0 && selected_index_ < static_cast<int>(items.size())) {
        const auto & selected = items[static_cast<std::size_t>(selected_index_)];
        if (selected.is_namespace) {
          line = truncate_text(selected.namespace_path + "  " + status_line_, columns - 1);
        } else {
          line = truncate_text(
            selected.row.name + " [" + selected.row.type + "]  " + status_line_,
            columns - 1);
        }
      }
    }

    mvprintw(row, 1, "%s", line.c_str());
    attroff(COLOR_PAIR(kColorStatus));
  }

  void draw_help_line(int row, int columns) const {
    attron(COLOR_PAIR(kColorHelp));
    mvhline(row, 0, ' ', columns);
    const std::string help =
      view_mode_ == ViewMode::TopicDetail
      ? "Enter Details  Esc Topics  F4 Refresh  F10 Exit"
      : "Enter Details  Space/Ins Monitor  F4 Refresh  F5 Filter  F10 Exit";
    mvprintw(row, 1, "%s", truncate_text(help, columns - 1).c_str());
    attroff(COLOR_PAIR(kColorHelp));
  }

  void clamp_topic_selection(const std::vector<TopicListItem> & items) {
    if (items.empty()) {
      selected_index_ = 0;
      list_scroll_ = 0;
      return;
    }
    selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(items.size()) - 1);
  }

  std::string topic_namespace(const std::string & topic_name) const {
    const auto slash = topic_name.find_last_of('/');
    if (slash == std::string::npos || slash == 0) {
      return "";
    }
    return topic_name.substr(1, slash - 1);
  }

  std::string topic_leaf_name(const std::string & topic_name) const {
    const auto slash = topic_name.find_last_of('/');
    if (slash == std::string::npos) {
      return topic_name;
    }
    return topic_name.substr(slash + 1);
  }

  bool is_topic_namespace_expanded(const std::string & namespace_path) const {
    const auto found = collapsed_topic_namespaces_.find(namespace_path);
    if (found == collapsed_topic_namespaces_.end()) {
      return false;
    }
    return !found->second;
  }

  std::vector<TopicListItem> visible_topic_items() const {
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

  void expand_selected_namespace() {
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

  void collapse_selected_namespace() {
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

  void clamp_detail_selection(const std::vector<DetailRow> & rows) {
    if (rows.empty()) {
      selected_detail_index_ = 0;
      detail_scroll_ = 0;
      return;
    }
    selected_detail_index_ = std::clamp(selected_detail_index_, 0, static_cast<int>(rows.size()) - 1);
  }

  mutable std::mutex mutex_;
  std::map<std::string, TopicEntry> topics_;
  std::map<std::string, TopicIntrospection> introspection_cache_;
  std::map<std::string, bool> collapsed_topic_namespaces_;
  std::vector<std::pair<std::string, rclcpp::GenericSubscription::SharedPtr>> monitored_subscriptions_;
  ViewMode view_mode_{ViewMode::TopicList};
  int selected_index_{0};
  int list_scroll_{0};
  int selected_detail_index_{0};
  int detail_scroll_{0};
  bool show_only_monitored_{false};
  std::string detail_topic_name_;
  std::string status_line_{"Loading topics..."};
  Clock::time_point last_refresh_time_{};
};

}  // namespace

}  // namespace ros2_console_tools

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ros2_console_tools::TopicMonitorNode>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  const int result = node->run();

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
