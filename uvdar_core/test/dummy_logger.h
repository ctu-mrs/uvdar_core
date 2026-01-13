#include <vector>
#include <string>
#include <uvdar_core/utils/i_logger.h>

class DummyLogger : public ILogger {
 public:
  std::vector<std::pair<LogLevel, std::string>> logs;

  void log(LogLevel level, const std::string& msg) override {
    logs.emplace_back(level, msg);
  }
};