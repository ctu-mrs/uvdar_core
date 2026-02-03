#pragma once

#include <string>
#include <chrono>
#include <uvdar/utils/i_logger.h>

namespace uvdar {

class ScopeTimer {
 public:
  using clock = std::chrono::steady_clock;

  ScopeTimer(ILogger& logger) : logger_(logger), start_(clock::now()) {
  }

  int64_t stop() {
    const auto end      = clock::now();
    const auto duration = end - start_;
    return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
  }
  ~ScopeTimer() = default;

 private:
  ILogger& logger_;
  clock::time_point start_;
};

} // namespace uvdar