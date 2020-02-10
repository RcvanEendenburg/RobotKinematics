#ifndef UTILITIES_LOGGER_H
#define UTILITIES_LOGGER_H

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <utilities/LogLevel.h>
#include <utilities/LogOutput.h>

namespace Utilities {

struct LogMessage {
  LogMessage(std::string aMessage, LogLevel aLevel)
      : message(std::move(aMessage)), level(aLevel) {}
  std::string message;
  LogLevel level;
};

class Logger {
public:
  static Logger &instance();
  void log(LogLevel logLevel, const char *fmt...);
  void setLogOutput(std::unique_ptr<LogOutput> logOutput);

  Logger(const Logger &) = delete;
  Logger &operator=(const Logger &) = delete;
  Logger(Logger &&) = delete;
  Logger &operator=(Logger &&) = delete;

private:
  Logger();
  ~Logger();
  void logThread();
  void push(std::unique_ptr<LogMessage> message);

  std::unique_ptr<LogOutput> output;
  std::atomic_bool printing;
  std::queue<std::unique_ptr<LogMessage>> queue;
  std::mutex mutex;
  std::condition_variable queue_cv;
  std::thread thread;
  std::atomic_bool switching_output;
};
} // namespace Utilities
#endif // UTILITIES_INCLUDE_LOGGER_H
