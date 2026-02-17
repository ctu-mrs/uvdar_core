#pragma once

#include <vector>
#include <memory>
#include <chrono>

#include <Eigen/Dense>
#include <opencv2/core/types.hpp>

namespace uvdar::blink_processor {

struct Shift2d {
  double x{0.0};
  double y{0.0};
};

// --- Data Containers ---
/* SequenceConfig //{ */
struct SequenceConfig {
  int blinking_patterns_length{0};
  int stored_seq_len_factor{0};

  size_t getMaxSequenceLength() const;
};
//}

// --- View Structs ---
/* LocalSearchConfig //{ */
struct LocalSearchConfig {
  SequenceConfig seq;
  cv::Point2d max_px_shift{0.0, 0.0};
};
//}

/* PolyRegressionConfig //{ */
struct PolyRegressionConfig {
  int poly_order{0};
  double decay_factor{0.0};
  int min_prediction_tol_px{0};
  int conf_prob_percentage{0};
};
//}

/* ExtendedSearchConfig //{ */
struct ExtendedSearchConfig {
  SequenceConfig seq;
  PolyRegressionConfig poly_reg;
};
//}

/* VerificationConfig //{ */
struct VerificationConfig {
  SequenceConfig seq;
  int max_buffer_length{0};
  int max_consecutive_zeros{0};
  int allowed_BER_per_seq{0};

  bool hasValidBufferRatios(size_t current_pattern_size) const;
};
//}

/* AmiTrackerConfig //{ */
struct AmiTrackerConfig {
  LocalSearchConfig local;
  ExtendedSearchConfig extended;
  VerificationConfig verification;
};
//}

struct SignalMatcherConfig {
  SequenceConfig seq;
  int allowed_BER_per_seq{0};
};

// --- Owner ---
/* BlinkProcessorConfig //{ */
struct BlinkProcessorConfig {
  int allowed_BER_per_seq{0};
  int max_buffer_length{0};
  int max_consecutive_zeros{0};
  double poly_decay_factor{0.0};
  int poly_order{0};
  int min_prediction_tol_px{0};
  int conf_prob_percentage{0};
  cv::Point2d max_px_shift{0.0, 0.0};

  SequenceConfig seq;
  AmiTrackerConfig ami_tracker;
  SignalMatcherConfig signal_matcher;

  bool isConfigValid(size_t pattern_size) const;
  int getPatternLength() const;

  BlinkProcessorConfig& setPatternLength(int size);
  BlinkProcessorConfig& setPoly(PolyRegressionConfig cfg);
  BlinkProcessorConfig& setSequence(SequenceConfig cfg);
  BlinkProcessorConfig& setVerification(VerificationConfig cfg);
  BlinkProcessorConfig& setMaxShift(Shift2d shift);

 private:
  void updateChildConfigs_();
};
//}

} // namespace uvdar::blink_processor