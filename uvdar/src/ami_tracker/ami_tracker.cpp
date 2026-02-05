#include <uvdar/ami_tracker/ami_tracker.h>

namespace uvdar::ami {

/* AmiTracker constructor //{ */
AmiTracker::AmiTracker(AmiTrackerConfig cfg, ILogger& logger) : cfg_(std::move(cfg)), logger_(logger) {
}
//}

/* setSequences //{ */
bool AmiTracker::setSequences(const std::vector<Sequence>& sequences) {
  blinking_patterns_ = sequences;

  signal_matcher_ = std::make_unique<SignalMatcher>(blinking_patterns_, cfg_.allowed_BER_per_seq);
  if (blinking_patterns_.size() == 0) {
    logger_.error("[UVDARAmiTracker]: Provided blinking patterns are empty!");
    return false;
  }

  if ((cfg_.stored_seq_len_factor * blinking_patterns_[0].size()) < cfg_.max_zeros_consecutive) {
    logger_.error("[UVDARAmiTracker]: The wanted number of consecutive zeros is higher than the possible sequence "
                  "length in the buffer! Sequence cannot be set.");
    return false;
  }
  return true;
}
//}

/* setFrameRate //{ */
void AmiTracker::setFrameRate(const double input) {
  if (input > 1.0) {
    frame_rate_ = input;
  } else {
    logger_.warn("[UVDARAmiTracker]: Cannot set non-positive frame rate! Ignoring the command");
  }
}
//}

/* processBuffer //{ */
void AmiTracker::processBuffer(std::vector<PointState>& current_frame) {
  findClosestPixelAndInsert_(current_frame);
  cleanPotentialBuffer_();
}
//}

/* findClosestPixelAndInsert_ //{ */
void AmiTracker::findClosestPixelAndInsert_(std::vector<PointState>& current_frame) {
}
//}

/* cleanPotentialBuffer_ //{ */
void AmiTracker::cleanPotentialBuffer_() {
}
//}

} // namespace uvdar::ami