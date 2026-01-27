#pragma once

#include <string>
#include <chrono>
#include <uvdar_core/utils/i_logger.h>

class Timer {
 public:
  using clock = std::chrono::steady_clock;

  Timer(ILogger& logger) : logger_(logger), start_(clock::now()) {
  }

  ~Timer() {
    const auto end      = clock::now();
    const auto duration = end - start_;

    // clang-format off
    logger_.info("Elapsed time: "
                + std::to_string(
                    std::chrono::duration_cast<std::chrono::milliseconds>(duration).count()
                  ) 
                + "ms");
    // clang-format on
  }

 private:
  ILogger& logger_;
  clock::time_point start_;
};