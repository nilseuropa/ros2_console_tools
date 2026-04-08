#include "ros2_console_tools/process_runner.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <vector>

namespace ros2_console_tools {

ProcessResult run_process(const std::vector<std::string> & arguments) {
  ProcessResult result;
  if (arguments.empty()) {
    result.output = "No command specified.";
    return result;
  }

  int output_pipe[2] = {-1, -1};
  if (pipe(output_pipe) != 0) {
    result.output = std::string("pipe() failed: ") + std::strerror(errno);
    return result;
  }

  const pid_t child_pid = fork();
  if (child_pid == -1) {
    result.output = std::string("fork() failed: ") + std::strerror(errno);
    close(output_pipe[0]);
    close(output_pipe[1]);
    return result;
  }

  if (child_pid == 0) {
    dup2(output_pipe[1], STDOUT_FILENO);
    dup2(output_pipe[1], STDERR_FILENO);
    close(output_pipe[0]);
    close(output_pipe[1]);

    std::vector<char *> argv;
    argv.reserve(arguments.size() + 1);
    for (const auto & argument : arguments) {
      argv.push_back(const_cast<char *>(argument.c_str()));
    }
    argv.push_back(nullptr);

    execvp(argv.front(), argv.data());
    _exit(127);
  }

  result.started = true;
  close(output_pipe[1]);

  char buffer[4096];
  while (true) {
    const ssize_t bytes_read = read(output_pipe[0], buffer, sizeof(buffer));
    if (bytes_read > 0) {
      result.output.append(buffer, static_cast<std::size_t>(bytes_read));
      continue;
    }
    if (bytes_read == 0) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    result.output.append("read() failed: ");
    result.output.append(std::strerror(errno));
    break;
  }
  close(output_pipe[0]);

  int status = 0;
  while (waitpid(child_pid, &status, 0) == -1) {
    if (errno != EINTR) {
      result.output.append("\nwaitpid() failed: ");
      result.output.append(std::strerror(errno));
      return result;
    }
  }

  if (WIFEXITED(status)) {
    result.exited_normally = true;
    result.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    result.output.append("\nProcess terminated by signal ");
    result.output.append(std::to_string(WTERMSIG(status)));
  }

  return result;
}

ProcessResult run_process_interactive(const std::vector<std::string> & arguments) {
  ProcessResult result;
  if (arguments.empty()) {
    result.output = "No command specified.";
    return result;
  }

  const pid_t child_pid = fork();
  if (child_pid == -1) {
    result.output = std::string("fork() failed: ") + std::strerror(errno);
    return result;
  }

  if (child_pid == 0) {
    std::vector<char *> argv;
    argv.reserve(arguments.size() + 1);
    for (const auto & argument : arguments) {
      argv.push_back(const_cast<char *>(argument.c_str()));
    }
    argv.push_back(nullptr);

    execvp(argv.front(), argv.data());
    std::perror(argv.front());
    _exit(127);
  }

  result.started = true;
  int status = 0;
  while (waitpid(child_pid, &status, 0) == -1) {
    if (errno != EINTR) {
      result.output.append("waitpid() failed: ");
      result.output.append(std::strerror(errno));
      return result;
    }
  }

  if (WIFEXITED(status)) {
    result.exited_normally = true;
    result.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    result.output.append("Process terminated by signal ");
    result.output.append(std::to_string(WTERMSIG(status)));
  }

  return result;
}

}  // namespace ros2_console_tools
