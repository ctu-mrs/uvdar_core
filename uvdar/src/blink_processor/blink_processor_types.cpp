#include <uvdar/blink_processor/blink_processor_types.h>

namespace uvdar::blink_processor {

/* BlinkProcessorConfig constructor //{ */
BlinkProcessorConfig::BlinkProcessorConfig() {
  updateChildConfigs_();
}
//}

/* BlinkProcessorConfig copy constructor //{ */
BlinkProcessorConfig::BlinkProcessorConfig(const BlinkProcessorConfig& other)
    : allowed_BER_per_seq(other.allowed_BER_per_seq), max_buffer_length(other.max_buffer_length),
      max_consecutive_zeros(other.max_consecutive_zeros), poly_decay_factor(other.poly_decay_factor),
      poly_order(other.poly_order), min_prediction_tol_px(other.min_prediction_tol_px),
      conf_prob_percentage(other.conf_prob_percentage), max_px_shift(other.max_px_shift), seq(other.seq),
      ami_tracker(other.ami_tracker), signal_matcher(other.signal_matcher) {
  updateChildConfigs_();
}
//}

/* BlinkProcessorConfig move constructor //{ */
BlinkProcessorConfig::BlinkProcessorConfig(BlinkProcessorConfig&& other) noexcept
    : allowed_BER_per_seq(other.allowed_BER_per_seq), max_buffer_length(other.max_buffer_length),
      max_consecutive_zeros(other.max_consecutive_zeros), poly_decay_factor(other.poly_decay_factor),
      poly_order(other.poly_order), min_prediction_tol_px(other.min_prediction_tol_px),
      conf_prob_percentage(other.conf_prob_percentage), max_px_shift(std::move(other.max_px_shift)),
      seq(std::move(other.seq)), ami_tracker(std::move(other.ami_tracker)),
      signal_matcher(std::move(other.signal_matcher)) {
  updateChildConfigs_();
}
//}

/* BlinkProcessorConfig copy assignment //{ */
BlinkProcessorConfig& BlinkProcessorConfig::operator=(const BlinkProcessorConfig& other) {
  if (this == &other) {
    return *this;
  }

  allowed_BER_per_seq   = other.allowed_BER_per_seq;
  max_buffer_length     = other.max_buffer_length;
  max_consecutive_zeros = other.max_consecutive_zeros;
  poly_decay_factor     = other.poly_decay_factor;
  poly_order            = other.poly_order;
  min_prediction_tol_px = other.min_prediction_tol_px;
  conf_prob_percentage  = other.conf_prob_percentage;
  max_px_shift          = other.max_px_shift;
  seq                   = other.seq;
  ami_tracker           = other.ami_tracker;
  signal_matcher        = other.signal_matcher;

  updateChildConfigs_();
  return *this;
}
//}

/* BlinkProcessorConfig move assignment //{ */
BlinkProcessorConfig& BlinkProcessorConfig::operator=(BlinkProcessorConfig&& other) noexcept {
  if (this == &other) {
    return *this;
  }

  allowed_BER_per_seq   = other.allowed_BER_per_seq;
  max_buffer_length     = other.max_buffer_length;
  max_consecutive_zeros = other.max_consecutive_zeros;
  poly_decay_factor     = other.poly_decay_factor;
  poly_order            = other.poly_order;
  min_prediction_tol_px = other.min_prediction_tol_px;
  conf_prob_percentage  = other.conf_prob_percentage;
  max_px_shift          = std::move(other.max_px_shift);
  seq                   = std::move(other.seq);
  ami_tracker           = std::move(other.ami_tracker);
  signal_matcher        = std::move(other.signal_matcher);

  updateChildConfigs_();
  return *this;
}
//}

/* BlinkProcessorConfig::updateChildConfigs //{ */
void BlinkProcessorConfig::updateChildConfigs_() {
  ami_tracker.local.seq        = seq;
  ami_tracker.extended.seq     = seq;
  ami_tracker.verification.seq = seq;
  signal_matcher.seq           = seq;

  ami_tracker.local.max_px_shift = max_px_shift;

  ami_tracker.extended.poly_reg.min_prediction_tol_px = min_prediction_tol_px;
  ami_tracker.extended.poly_reg.poly_order            = poly_order;
  ami_tracker.extended.poly_reg.decay_factor          = poly_decay_factor;
  ami_tracker.extended.poly_reg.conf_prob_percentage  = conf_prob_percentage;

  ami_tracker.verification.max_buffer_length     = max_buffer_length;
  ami_tracker.verification.max_consecutive_zeros = max_consecutive_zeros;
  ami_tracker.verification.allowed_BER_per_seq   = allowed_BER_per_seq;

  signal_matcher.allowed_BER_per_seq = allowed_BER_per_seq;
}
//}

/* SequenceConfig::getMaxSequenceLength //{ */
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

/* BlinkProcessorConfig::isConfigValid //{ */
bool BlinkProcessorConfig::isConfigValid(size_t pattern_size) const {
  return ami_tracker.verification.hasValidBufferRatios(pattern_size);
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

  updateChildConfigs_();
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

  this->poly_order            = cfg.poly_order;
  this->poly_decay_factor     = cfg.decay_factor;
  this->min_prediction_tol_px = cfg.min_prediction_tol_px;
  this->conf_prob_percentage  = cfg.conf_prob_percentage;

  updateChildConfigs_();
  return *this;
}
//}

/* BlinkProcessorConfig::setSequence //{ */
BlinkProcessorConfig& BlinkProcessorConfig::setSequence(SequenceConfig cfg) {
  this->seq = cfg;

  updateChildConfigs_();
  return *this;
}
//}

/* BlinkProcessorConfig::setVerification //{ */
BlinkProcessorConfig& BlinkProcessorConfig::setVerification(VerificationConfig cfg) {
  this->allowed_BER_per_seq   = cfg.allowed_BER_per_seq;
  this->max_buffer_length     = cfg.max_buffer_length;
  this->max_consecutive_zeros = cfg.max_consecutive_zeros;

  updateChildConfigs_();
  return *this;
}
//}

/* BlinkProcessorConfig::setMaxShift //{ */
BlinkProcessorConfig& BlinkProcessorConfig::setMaxShift(Shift2d shift) {
  this->max_px_shift = cv::Point2d(shift.x, shift.y);

  updateChildConfigs_();
  return *this;
}
//}

} // namespace uvdar::blink_processor