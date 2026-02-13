#pragma once

enum MatchStatus {
  SIGNAL_SIZE_CORRECT = 1,
  SIGNAL_INVALID      = -1, // empty or no match
  SIGNAL_TOO_SHORT    = -3
};
