#ifndef AMT_SIGNAL_MATCHER_H
#define AMT_SIGNAL_MATCHER_H

#include <stdexcept>
#include <vector>

namespace uvdar_core::tracking::ami {

/**
 * @brief Match current blinker bit sequence against configured blink signatures.
 */
class SignalMatcher {
public:
    /**
     * @brief Create matcher from supported sequences and allowed BER.
     * @param sequences Blink templates to match against.
     * @param allowed_ber_per_seq Accepted bit error count per sequence.
     */
    SignalMatcher(std::vector<std::vector<bool>> sequences, int allowed_ber_per_seq)
        : allowed_BER_per_seq_(allowed_ber_per_seq)
        , sequences_(std::move(sequences))
    {
        if (sequences_.empty()) {
            throw std::invalid_argument("[tracker] SignalMatcher - sequence list is empty.");
        }
        sequence_size_ = static_cast<int>(sequences_[0].size());
        if (sequence_size_ <= 0) {
            throw std::invalid_argument("[tracker] SignalMatcher - empty sequence provided.");
        }
        for (auto& sequence : sequences_) {
            const auto duplicated = sequence;
            sequence.insert(sequence.end(), duplicated.begin(), duplicated.end() - 1);
        }
    }

    /**
     * @brief Match a sampled bit signal against configured templates.
     * @param signal Sampled ON/OFF sequence.
     * @return sequence index on match, negative value if no match.
     */
    int matchSignal(const std::vector<bool>& signal) const
    {
        if (sequence_size_ <= 0) {
            return -1;
        }
        const int valid_size = check_seq_size(signal);
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
                    if (errors > allowed_BER_per_seq_) {
                        break;
                    }
                }
                if (errors <= allowed_BER_per_seq_) {
                    return sequence_id;
                }
            }
        }
        return -1;
    }

private:
    /**
     * @brief Validate signal length before matching.
     */
    int check_seq_size(const std::vector<bool>& signal) const
    {
        if (sequence_size_ <= 0) {
            return -2;
        }
        if (signal.empty()) {
            return -1;
        }
        if (static_cast<int>(signal.size()) < sequence_size_) {
            return -3;
        }
        return 1;
    }

    int sequence_size_ = 0;
    int allowed_BER_per_seq_ = 0;
    std::vector<std::vector<bool>> sequences_;
};

} // namespace uvdar_core::tracking::ami

#endif // AMT_SIGNAL_MATCHER_H
