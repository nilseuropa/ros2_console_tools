#include "ros2_console_tools/journal_viewer.hpp"

#include <cstdio>
#include <string>

int main(int argc, char ** argv) {
  std::string initial_unit;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--unit") {
      if (index + 1 >= argc) {
        std::fprintf(stderr, "journal_viewer: --unit requires a value\n");
        return 2;
      }
      initial_unit = argv[++index];
      continue;
    }
    std::fprintf(stderr, "journal_viewer: unknown argument '%s'\n", argument.c_str());
    return 2;
  }

  return ros2_console_tools::run_journal_viewer_tool(initial_unit, false);
}
