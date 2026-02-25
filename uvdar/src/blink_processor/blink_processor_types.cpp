#include <uvdar/blink_processor/blink_processor_types.h>

namespace uvdar::blink_processor {

/* BlinkProcessorConfig::getSequenceConfig() //{ */
SequenceConfig BlinkProcessorConfig::getSequenceConfig() const {
  return seq;
}
//}

/* BlinkProcessorConfig::getPolyRegressionConfig() //{ */
PolyRegressionConfig BlinkProcessorConfig::getPolyRegressionConfig() const {
  PolyRegressionConfig cfg;
  cfg.poly_order              = poly_order;
  cfg.decay_factor            = poly_decay_factor;
  cfg.min_prediction_tol_px   = min_prediction_tol_px;
  cfg.conf_prob_percentage    = conf_prob_percentage;
  cfg.max_predict_interval_px = max_predict_interval_px;
  cfg.seq                     = seq;
  return cfg;
}
//}

/* BlinkProcessorConfig::getLocalSearchConfig() //{ */
LocalSearchConfig BlinkProcessorConfig::getLocalSearchConfig() const {
  LocalSearchConfig cfg;
  cfg.seq          = seq;
  cfg.max_px_shift = max_px_shift;
  return cfg;
}
//}

/* BlinkProcessorConfig::getExtendedSearchConfig() //{ */
ExtendedSearchConfig BlinkProcessorConfig::getExtendedSearchConfig() const {
  ExtendedSearchConfig cfg;
  cfg.seq      = seq;
  cfg.poly_reg = getPolyRegressionConfig();
  return cfg;
}
//}

/* BlinkProcessorConfig::getVerificationConfig() //{ */
VerificationConfig BlinkProcessorConfig::getVerificationConfig() const {
  VerificationConfig cfg;
  cfg.seq                   = seq;
  cfg.allowed_BER_per_seq   = allowed_BER_per_seq;
  cfg.max_buffer_length     = max_buffer_length;
  cfg.max_consecutive_zeros = max_consecutive_zeros;
  return cfg;
}
//}

/* BlinkProcessorConfig::getAmiTrackerConfig() //{ */
AmiTrackerConfig BlinkProcessorConfig::getAmiTrackerConfig() const {
  AmiTrackerConfig cfg;
  cfg.local        = getLocalSearchConfig();
  cfg.extended     = getExtendedSearchConfig();
  cfg.verification = getVerificationConfig();
  return cfg;
}
//}

/* BlinkProcessorConfig::getSignalMatcherConfig() //{ */
SignalMatcherConfig BlinkProcessorConfig::getSignalMatcherConfig() const {
  SignalMatcherConfig cfg;
  cfg.seq                 = seq;
  cfg.allowed_BER_per_seq = allowed_BER_per_seq;
  return cfg;
}
//}

/* BlinkProcessorConfig::isConfigValid //{ */
bool BlinkProcessorConfig::isConfigValid(size_t pattern_size) const {
  if (!max_consecutive_zeros) {
    return false;
  }

  double min_required_len = seq.stored_seq_len_factor * pattern_size;
  return min_required_len >= max_consecutive_zeros;
}
//}

/* BlinkProcessorConfig::getPatternLength //{ */
int BlinkProcessorConfig::getPatternLength() const {
  return seq.blinking_patterns_length;
}
//}

/* BlinkProcessorConfig::setPatternLength //{ */
BlinkProcessorConfig& BlinkProcessorConfig::setPatternLength(int size) {
  if (size < 0 || size > 32) {
    throw std::invalid_argument("Pattern length has to be in the range [0, 32].");
  }
  this->seq.blinking_patterns_length = size;

  return *this;
}
//}

/* BlinkProcessorConfig::setPoly //{ */
BlinkProcessorConfig& BlinkProcessorConfig::setPoly(PolyRegressionConfig cfg) {
  if (cfg.poly_order < 0 || cfg.poly_order > 4) {
    throw std::invalid_argument("Polynomial order has to be in the range [0, 4].");
  }
  if (cfg.conf_prob_percentage < 0 || cfg.conf_prob_percentage > 100) {
    throw std::invalid_argument("Confidence probability percentage has to be in the range [0, 100].");
  }
  if (cfg.max_predict_interval_px < 0) {
    throw std::invalid_argument("Max prediction interval has to be positive.");
  }

  this->poly_order              = cfg.poly_order;
  this->poly_decay_factor       = cfg.decay_factor;
  this->min_prediction_tol_px   = cfg.min_prediction_tol_px;
  this->conf_prob_percentage    = cfg.conf_prob_percentage;
  this->max_predict_interval_px = cfg.max_predict_interval_px;

  return *this;
}
//}

/* BlinkProcessorConfig::setSequence //{ */
BlinkProcessorConfig& BlinkProcessorConfig::setSequence(SequenceConfig cfg) {
  this->seq = cfg;

  return *this;
}
//}

/* BlinkProcessorConfig::setVerification //{ */
BlinkProcessorConfig& BlinkProcessorConfig::setVerification(VerificationConfig cfg) {
  this->allowed_BER_per_seq   = cfg.allowed_BER_per_seq;
  this->max_buffer_length     = cfg.max_buffer_length;
  this->max_consecutive_zeros = cfg.max_consecutive_zeros;

  return *this;
}
//}

/* BlinkProcessorConfig::setMaxShift //{ */
BlinkProcessorConfig& BlinkProcessorConfig::setMaxShift(Shift2d shift) {
  this->max_px_shift = cv::Point2d(shift.x, shift.y);

  return *this;
}
//}

/* SequenceConfig::getMaxSequenceLength() //{ */
size_t SequenceConfig::getMaxSequenceLength() const {
  return blinking_patterns_length * stored_seq_len_factor;
}
//}

/* VerificationConfig::hasValidBufferRatios //{ */
bool VerificationConfig::hasValidBufferRatios(size_t current_pattern_size) const {
  if (!max_consecutive_zeros)
    return false;

  double min_required_len = seq.stored_seq_len_factor * current_pattern_size;
  return min_required_len >= max_consecutive_zeros;
}
//}

} // namespace uvdar::blink_processor