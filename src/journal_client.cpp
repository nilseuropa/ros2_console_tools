#include "ros2_console_tools/journal_client.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

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

std::string to_lower(std::string text) {
  for (char & character : text) {
    character = static_cast<char>(std::tolower(static_cast<unsigned char>(character)));
  }
  return text;
}

void skip_whitespace(const std::string & text, std::size_t & index) {
  while (index < text.size() && std::isspace(static_cast<unsigned char>(text[index])) != 0) {
    ++index;
  }
}

bool append_unicode_escape(const std::string & text, std::size_t & index, std::string & output) {
  if (index + 4 > text.size()) {
    return false;
  }

  unsigned int codepoint = 0;
  for (int offset = 0; offset < 4; ++offset) {
    codepoint <<= 4U;
    const char digit = text[index + static_cast<std::size_t>(offset)];
    if (digit >= '0' && digit <= '9') {
      codepoint |= static_cast<unsigned int>(digit - '0');
    } else if (digit >= 'a' && digit <= 'f') {
      codepoint |= static_cast<unsigned int>(digit - 'a' + 10);
    } else if (digit >= 'A' && digit <= 'F') {
      codepoint |= static_cast<unsigned int>(digit - 'A' + 10);
    } else {
      return false;
    }
  }
  index += 4;

  if (codepoint <= 0x7fU) {
    output.push_back(static_cast<char>(codepoint));
  } else if (codepoint <= 0x7ffU) {
    output.push_back(static_cast<char>(0xc0U | (codepoint >> 6U)));
    output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
  } else {
    output.push_back(static_cast<char>(0xe0U | (codepoint >> 12U)));
    output.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3fU)));
    output.push_back(static_cast<char>(0x80U | (codepoint & 0x3fU)));
  }
  return true;
}

bool parse_json_string(const std::string & text, std::size_t & index, std::string & value) {
  if (index >= text.size() || text[index] != '"') {
    return false;
  }

  ++index;
  value.clear();
  while (index < text.size()) {
    const char character = text[index++];
    if (character == '"') {
      return true;
    }
    if (character != '\\') {
      value.push_back(character);
      continue;
    }
    if (index >= text.size()) {
      return false;
    }

    const char escaped = text[index++];
    switch (escaped) {
      case '"':
      case '\\':
      case '/':
        value.push_back(escaped);
        break;
      case 'b':
        value.push_back('\b');
        break;
      case 'f':
        value.push_back('\f');
        break;
      case 'n':
        value.push_back('\n');
        break;
      case 'r':
        value.push_back('\r');
        break;
      case 't':
        value.push_back('\t');
        break;
      case 'u':
        if (!append_unicode_escape(text, index, value)) {
          return false;
        }
        break;
      default:
        value.push_back(escaped);
        break;
    }
  }
  return false;
}

bool skip_json_composite(const std::string & text, std::size_t & index, char open_char, char close_char) {
  if (index >= text.size() || text[index] != open_char) {
    return false;
  }

  int depth = 0;
  while (index < text.size()) {
    const char character = text[index++];
    if (character == '"') {
      std::string ignored;
      --index;
      if (!parse_json_string(text, index, ignored)) {
        return false;
      }
      continue;
    }
    if (character == open_char) {
      ++depth;
    } else if (character == close_char) {
      --depth;
      if (depth == 0) {
        return true;
      }
    }
  }
  return false;
}

bool parse_json_scalar(const std::string & text, std::size_t & index, std::string & value) {
  if (index >= text.size()) {
    return false;
  }

  if (text[index] == '"') {
    return parse_json_string(text, index, value);
  }

  if (text[index] == '{') {
    value.clear();
    return skip_json_composite(text, index, '{', '}');
  }
  if (text[index] == '[') {
    value.clear();
    return skip_json_composite(text, index, '[', ']');
  }

  const std::size_t start = index;
  while (
    index < text.size() &&
    text[index] != ',' &&
    text[index] != '}' &&
    std::isspace(static_cast<unsigned char>(text[index])) == 0)
  {
    ++index;
  }
  value = text.substr(start, index - start);
  return !value.empty();
}

std::map<std::string, std::string> parse_json_object_line(const std::string & line) {
  std::map<std::string, std::string> fields;
  std::size_t index = 0;
  skip_whitespace(line, index);
  if (index >= line.size() || line[index] != '{') {
    return fields;
  }
  ++index;

  while (index < line.size()) {
    skip_whitespace(line, index);
    if (index < line.size() && line[index] == '}') {
      break;
    }

    std::string key;
    if (!parse_json_string(line, index, key)) {
      return {};
    }
    skip_whitespace(line, index);
    if (index >= line.size() || line[index] != ':') {
      return {};
    }
    ++index;
    skip_whitespace(line, index);

    std::string value;
    if (!parse_json_scalar(line, index, value)) {
      return {};
    }
    fields[key] = value;

    skip_whitespace(line, index);
    if (index < line.size() && line[index] == ',') {
      ++index;
      continue;
    }
    if (index < line.size() && line[index] == '}') {
      break;
    }
  }

  return fields;
}

std::string first_non_empty(
  const std::map<std::string, std::string> & fields,
  const std::initializer_list<const char *> & keys)
{
  for (const char * key : keys) {
    const auto found = fields.find(key);
    if (found != fields.end() && !found->second.empty()) {
      return found->second;
    }
  }
  return "";
}

int parse_priority_value(const std::map<std::string, std::string> & fields) {
  const auto found = fields.find("PRIORITY");
  if (found == fields.end() || found->second.empty()) {
    return 6;
  }

  try {
    return std::stoi(found->second);
  } catch (...) {
    return 6;
  }
}

std::string format_timestamp(const std::string & usec_text) {
  if (usec_text.empty()) {
    return "-";
  }

  try {
    const long long usec = std::stoll(usec_text);
    const std::time_t seconds = static_cast<std::time_t>(usec / 1000000LL);
    std::tm local_time{};
    localtime_r(&seconds, &local_time);
    char buffer[32];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &local_time);
    return buffer;
  } catch (...) {
    return "-";
  }
}

bool entry_matches_filter(const JournalEntry & entry, const std::string & filter_text) {
  if (filter_text.empty()) {
    return true;
  }

  const std::string needle = to_lower(filter_text);
  const std::string haystack = to_lower(
    entry.timestamp + " " + entry.unit + " " + entry.identifier + " " + entry.message);
  return haystack.find(needle) != std::string::npos;
}

}  // namespace

std::string journal_priority_label(int priority) {
  switch (priority) {
    case 0:
      return "EMERG";
    case 1:
      return "ALERT";
    case 2:
      return "CRIT";
    case 3:
      return "ERR";
    case 4:
      return "WARN";
    case 5:
      return "NOTICE";
    case 6:
      return "INFO";
    case 7:
      return "DEBUG";
    default:
      return "LOG";
  }
}

std::vector<JournalEntry> parse_journal_json_lines(const std::string & text) {
  std::vector<JournalEntry> entries;
  std::istringstream stream(text);
  std::string line;
  while (std::getline(stream, line)) {
    line = trim(line);
    if (line.empty()) {
      continue;
    }

    auto fields = parse_json_object_line(line);
    if (fields.empty()) {
      continue;
    }

    JournalEntry entry;
    entry.timestamp = format_timestamp(first_non_empty(fields, {"__REALTIME_TIMESTAMP"}));
    entry.priority = parse_priority_value(fields);
    entry.unit = first_non_empty(fields, {"_SYSTEMD_UNIT"});
    entry.identifier = first_non_empty(fields, {"SYSLOG_IDENTIFIER", "_COMM"});
    entry.message = first_non_empty(fields, {"MESSAGE"});
    entry.pid = first_non_empty(fields, {"_PID"});
    entry.code_file = first_non_empty(fields, {"CODE_FILE"});
    entry.code_line = first_non_empty(fields, {"CODE_LINE"});
    entry.code_function = first_non_empty(fields, {"CODE_FUNC"});
    entry.fields.assign(fields.begin(), fields.end());
    entries.push_back(std::move(entry));
  }
  return entries;
}

std::vector<JournalEntry> JournalClient::read_entries(
  const std::string & unit_filter,
  int max_priority,
  int line_count,
  const std::string & text_filter,
  std::string * error) const
{
  std::vector<std::string> command = {
    "journalctl",
    "--no-pager",
    "--output=json",
    "--lines=" + std::to_string(std::max(1, line_count)),
    "--priority=0.." + std::to_string(std::clamp(max_priority, 0, 7)),
  };
  if (!unit_filter.empty()) {
    command.push_back("--unit=" + unit_filter);
  }

  const ProcessResult result = run_process(command);
  if (!result.succeeded()) {
    if (error != nullptr) {
      *error = result.output.empty() ? "journalctl failed." : trim(result.output);
    }
    return {};
  }

  std::vector<JournalEntry> entries = parse_journal_json_lines(result.output);
  if (text_filter.empty()) {
    return entries;
  }

  std::vector<JournalEntry> filtered_entries;
  filtered_entries.reserve(entries.size());
  for (const auto & entry : entries) {
    if (entry_matches_filter(entry, text_filter)) {
      filtered_entries.push_back(entry);
    }
  }
  return filtered_entries;
}

}  // namespace ros2_console_tools
