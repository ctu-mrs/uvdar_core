#pragma once

namespace uvdar::blink_processor {

enum MatchStatus {
  SIGNAL_SIZE_CORRECT = 1,
  SIGNAL_INVALID      = -1, // empty or no match
  SIGNAL_TOO_SHORT    = -3
};

} // namespace uvdar::blink_processor