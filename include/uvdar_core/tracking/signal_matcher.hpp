#pragma once

#include <stdexcept>
#include <utility>
#include <vector>

namespace uvdar_core::tracking {

/**
 * @brief Match an observed ON/OFF blink history against cyclic marker signatures.
 *
 * The matcher is shared by all tracker backends because marker identity is a
 * property of the blinking sequence, not of the association strategy used to
 * build the track.
 */
class SignalMatcher {
public:
    /**
     * @brief Create matcher from supported sequences and allowed bit errors.
     */
    SignalMatcher(std::vector<std::vector<bool>> sequences, int allowed_ber_per_seq)
        : allowed_ber_per_seq_(allowed_ber_per_seq)
        , sequences_(std::move(sequences))
    {
        if (sequences_.empty()) {
            throw std::invalid_argument("[tracker] SignalMatcher - sequence list is empty.");
        }
        sequence_size_ = static_cast<int>(sequences_[0].size());
        if (sequence_size_ <= 0) {
            throw std::invalid_argument("[tracker] SignalMatcher - empty sequence provided.");
        }
        for (const auto& sequence : sequences_) {
            if (static_cast<int>(sequence.size()) != sequence_size_) {
                throw std::invalid_argument("[tracker] SignalMatcher - all sequences must have equal length.");
            }
        }

        for (auto& sequence : sequences_) {
            const auto duplicated = sequence;
            sequence.insert(sequence.end(), duplicated.begin(), duplicated.end() - 1);
        }
    }

    /**
     * @brief Return the matching sequence index, or a negative value on failure.
     *
     * Matching is cyclic Hamming distance against every phase offset. A sequence
     * is accepted when bit errors are <= allowed_BER_per_seq.
     */
    int matchSignal(const std::vector<bool>& signal) const
    {
        if (sequence_size_ <= 0) {
            return -2;
        }

        const int valid_size = checkSequenceSize(signal);
        if (valid_size != 1) {
            return valid_size;
        }

        for (int sequence_id = 0; sequence_id < static_cast<int>(sequences_.size()); ++sequence_id) {
            for (int offset = 0; offset < sequence_size_; ++offset) {
                int errors = 0;
                for (int signal_id = 0; signal_id < static_cast<int>(signal.size()); ++signal_id) {
                    if (sequences_[sequence_id].at(offset + signal_id) != signal.at(signal_id)) {
                        ++errors;
                    }
                    if (errors > allowed_ber_per_seq_) {
                        break;
                    }
                }
                if (errors <= allowed_ber_per_seq_) {
                    return sequence_id;
                }
            }
        }
        return -1;
    }

private:
    /**
     * @brief Validate minimum signal history length before cyclic matching.
     */
    int checkSequenceSize(const std::vector<bool>& signal) const
    {
        if (signal.empty()) {
            return -1;
        }
        if (static_cast<int>(signal.size()) < sequence_size_) {
            return -3;
        }
        return 1;
    }

    int sequence_size_ = 0;
    int allowed_ber_per_seq_ = 0;
    std::vector<std::vector<bool>> sequences_;
};

} // namespace uvdar_core::tracking
