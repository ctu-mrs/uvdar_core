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

class TestCout : public std::stringstream {
 public:
  ~TestCout() {
    std::cout << "\u001b[32m[          ] \u001b[33m" << str() << "\u001b[0m" << std::flush;
  }
};

#define TEST_COUT TestCout()