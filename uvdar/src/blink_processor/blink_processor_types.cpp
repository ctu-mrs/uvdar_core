#include <uvdar/blink_processor/blink_processor_types.h>

namespace uvdar::blink_processor {

/* BlinkProcessorConfig::updateChildConfigs //{ */
void BlinkProcessorConfig::updateChildConfigs_() {
  ami_tracker.local.seq    = seq;
  ami_tracker.extended.seq = seq;
  signal_matcher.seq       = seq;

  ami_tracker.local.max_px_shift = max_px_shift;

  ami_tracker.extended.poly_reg.min_prediction_tol_px = min_prediction_tol_px;
  ami_tracker.extended.poly_reg.poly_order            = poly_order;
  ami_tracker.extended.poly_reg.decay_factor          = poly_decay_factor;

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
  this->seq.blinking_patterns_length = size;
  updateChildConfigs_();
  return *this;
}
//}

BlinkProcessorConfig& BlinkProcessorConfig::setPoly(PolyRegressionConfig cfg) {
  this->poly_order                    = cfg.poly_order;
  this->poly_decay_factor             = cfg.decay_factor;
  this->min_prediction_tol_px         = cfg.min_prediction_tol_px;
  this->conf_prob_percentage          = cfg.conf_prob_percentage;
  this->ami_tracker.extended.poly_reg = cfg;
  updateChildConfigs_();
  return *this;
}

BlinkProcessorConfig& BlinkProcessorConfig::setSequence(SequenceConfig cfg) {
  this->seq = cfg;
  updateChildConfigs_();
  return *this;
}

BlinkProcessorConfig& BlinkProcessorConfig::setVerification(VerificationConfig cfg) {
  this->allowed_BER_per_seq   = cfg.allowed_BER_per_seq;
  this->max_buffer_length     = cfg.max_buffer_length;
  this->max_consecutive_zeros = cfg.max_consecutive_zeros;

  this->ami_tracker.verification = cfg;
  updateChildConfigs_();
  return *this;
}

} // namespace uvdar::blink_processor