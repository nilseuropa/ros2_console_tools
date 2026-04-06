#include <ncurses.h>

#include <algorithm>
#include <chrono>
#include <clocale>
#include <cstdio>
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
#include <rclcpp/serialized_message.hpp>

namespace ros2_console_tools {

namespace {

using Clock = std::chrono::steady_clock;

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

struct Sample {
  Clock::time_point time;
  std::size_t bytes;
};

struct IntervalSample {
  Clock::time_point time;
  double seconds;
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
    const auto rows = topic_rows_snapshot();
    if (rows.empty()) {
      selected_index_ = 0;
      list_scroll_ = 0;
    } else {
      selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(rows.size()) - 1);
    }

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
        if (selected_index_ + 1 < static_cast<int>(rows.size())) {
          ++selected_index_;
        }
        return true;
      case KEY_PPAGE:
        selected_index_ = std::max(0, selected_index_ - page_step());
        return true;
      case KEY_NPAGE:
        if (!rows.empty()) {
          selected_index_ = std::min(
            static_cast<int>(rows.size()) - 1,
            selected_index_ + page_step());
        }
        return true;
      case ' ':
      case KEY_IC:
        toggle_selected_topic_monitoring();
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
    const auto rows = topic_rows_snapshot();
    if (rows.empty() || selected_index_ < 0 || selected_index_ >= static_cast<int>(rows.size())) {
      status_line_ = "No topic selected.";
      return;
    }

    const std::string topic_name = rows[static_cast<std::size_t>(selected_index_)].name;
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

    const auto cutoff = now - kWindowDuration;
    while (!entry.samples.empty() && entry.samples.front().time < cutoff) {
      entry.sample_bytes_sum -= entry.samples.front().bytes;
      entry.samples.pop_front();
    }
    while (!entry.intervals.empty() && entry.intervals.front().time < cutoff) {
      entry.intervals.pop_front();
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
    draw_topic_list(1, 1, content_bottom - 1, columns - 2);
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
    const auto rows = topic_rows_snapshot();
    if (rows.empty()) {
      selected_index_ = 0;
      list_scroll_ = 0;
    } else {
      selected_index_ = std::clamp(selected_index_, 0, static_cast<int>(rows.size()) - 1);
    }
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
    const int last_row = std::min(static_cast<int>(rows.size()), first_row + visible_rows - 1);
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

      const auto & entry = rows[static_cast<std::size_t>(first_row + (row - top - 1))];
      const int text_color =
        entry.stale
        ? (selected ? kColorStaleSelection : kColorStale)
        : entry.monitored
        ? (selected ? kColorMonitoredSelection : kColorMonitored)
        : (selected ? kColorSelection : 0);
      if (text_color != 0) {
        attron(COLOR_PAIR(text_color));
      }
      mvprintw(row, left, "%-*s", topic_width, truncate_text(entry.name, topic_width).c_str());
      mvprintw(row, sep_one_x + 1, "%-*s", avg_width, truncate_text(entry.avg_hz, avg_width).c_str());
      mvprintw(
        row, sep_two_x + 1, "%-*s", minmax_width,
        truncate_text(entry.min_max_hz, minmax_width).c_str());
      mvprintw(
        row, sep_three_x + 1, "%-*s", bandwidth_width,
        truncate_text(entry.bandwidth, bandwidth_width).c_str());
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

  void draw_status_line(int row, int columns) const {
    attron(COLOR_PAIR(kColorStatus));
    mvhline(row, 0, ' ', columns);
    const auto rows = topic_rows_snapshot();
    std::string line = status_line_;
    if (!rows.empty() && selected_index_ >= 0 && selected_index_ < static_cast<int>(rows.size())) {
      const auto & selected = rows[static_cast<std::size_t>(selected_index_)];
      line = truncate_text(
        selected.name + " [" + selected.type + "]  " + status_line_,
        columns - 1);
    }
    mvprintw(row, 1, "%s", line.c_str());
    attroff(COLOR_PAIR(kColorStatus));
  }

  void draw_help_line(int row, int columns) const {
    attron(COLOR_PAIR(kColorHelp));
    mvhline(row, 0, ' ', columns);
    const std::string help = "Space/Ins Monitor  F4 Refresh  F5 Filter  F10 Exit";
    mvprintw(row, 1, "%s", truncate_text(help, columns - 1).c_str());
    attroff(COLOR_PAIR(kColorHelp));
  }

  mutable std::mutex mutex_;
  std::map<std::string, TopicEntry> topics_;
  std::vector<std::pair<std::string, rclcpp::GenericSubscription::SharedPtr>> monitored_subscriptions_;
  int selected_index_{0};
  int list_scroll_{0};
  bool show_only_monitored_{false};
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
