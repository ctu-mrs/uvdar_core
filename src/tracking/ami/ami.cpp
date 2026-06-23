#include "uvdar_core/tracking/ami/ami.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "uvdar_core/tracking/sequence_buffer.hpp"
#include "uvdar_core/tracking/time_weights.hpp"

namespace uvdar_core::tracking::ami {

/**
 * @brief Construct tracker with selected tuning parameters.
 */
AMI::AMI(const ParamsAMI& i_params)
    : params_ami_(i_params)
{
}

/**
 * @brief Configure blinking templates used for matching.
 */
void AMI::setupSequenceMatcher(std::vector<std::vector<bool>> i_sequences)
{
    if (i_sequences.empty()) {
        throw std::invalid_argument("[tracker] AMI - no sequences provided.");
    }
    if (i_sequences[0].empty()) {
        throw std::invalid_argument("[tracker] AMI - first sequence is empty.");
    }
    const std::size_t sequence_size = i_sequences[0].size();
    for (std::size_t index = 0; index < i_sequences.size(); ++index) {
        if (i_sequences[index].empty()) {
            throw std::invalid_argument("[tracker] AMI - sequence " + std::to_string(index) + " is empty.");
        }
        if (i_sequences[index].size() != sequence_size) {
            throw std::invalid_argument("[tracker] AMI - all sequences must have equal length.");
        }
    }

    sequences_ = std::move(i_sequences);
    matcher_ = std::make_unique<uvdar_core::tracking::SignalMatcher>(sequences_, params_ami_.allowed_BER_per_seq);

    if (params_ami_.stored_seq_len_factor * static_cast<int>(sequences_[0].size()) < params_ami_.max_zeros_consecutive) {
        throw std::invalid_argument("[tracker] AMI - max_zeros_consecutive does not fit the configured sequence buffer.");
    }
}

/**
 * @brief Convert and append new detections, then prune old tracks.
 */
void AMI::processBuffer(const ImagePointsWithCovariancesStamped& points)
{
    if (!matcher_) {
        return;
    }

    std::vector<PointState> current_frame;
    current_frame.reserve(points.points.size());
    for (const ImagePoint& point : points.points) {
        PointState state;
        state.position = Eigen::Vector2d(point.x, point.y);
        state.predicted_position = state.position;
        state.measurement_covariance = point.covariance;
        state.prediction_covariance = point.covariance;
        state.covariance = point.covariance;
        state.led_state = true;
        state.virtual_point = false;
        state.associated_with_detection = true;
        state.stamp = points.stamp;
        current_frame.push_back(state);
    }

    findClosestPixelAndInsert(current_frame, points.stamp);
    cleanPotentialBuffer();
}

/**
 * @brief Remove stale trajectories with too many consecutive virtual points.
 */
void AMI::cleanPotentialBuffer()
{
    for (auto it_seq = buffer_.begin(); it_seq != buffer_.end();) {
        const int number_zeros_till_seq_deleted = (params_ami_.max_zeros_consecutive + params_ami_.allowed_BER_per_seq);
        if (static_cast<int>((*it_seq)->size()) > number_zeros_till_seq_deleted
            && uvdar_core::tracking::hasOffRunLongerThan(**it_seq, number_zeros_till_seq_deleted)) {
            it_seq = buffer_.erase(it_seq);
            continue;
        }
        ++it_seq;
    }
}

/**
 * @brief Associate unmatched detections with active trajectories using nearest-neighbour search.
 */
void AMI::findClosestPixelAndInsert(std::vector<PointState>& current_frame, double stamp)
{
    std::vector<seqPointer> buffer_local = buffer_;

    for (auto seq = buffer_local.begin(); seq != buffer_local.end();) {
        if ((*seq)->empty()) {
            ++seq;
            continue;
        }

        const PointState& last_inserted = (*seq)->back();
        const Eigen::Vector2d max_shift(params_ami_.max_px_shift_x, params_ami_.max_px_shift_y);
        const Eigen::Vector2d box_left_top = last_inserted.position - max_shift;
        const Eigen::Vector2d box_right_bottom = last_inserted.position + max_shift;

        std::vector<PointState>::iterator it_2 = current_frame.end();
        double closest_distance = std::numeric_limits<double>::max();
        for (auto it = current_frame.begin(); it != current_frame.end(); ++it) {
            const double curr_dist = euclideanDistance((*it).position, last_inserted.position);
            if (curr_dist <= closest_distance) {
                closest_distance = curr_dist;
                it_2 = it;
            }
        }

        if (it_2 != current_frame.end()) {
            if (isInsideBox((*it_2).position, box_left_top, box_right_bottom)) {
                addPointToSequenceAndCheckLength(**seq, *it_2);
                seq = buffer_local.erase(seq);
                current_frame.erase(it_2);
            } else {
                ++seq;
            }
        } else {
            ++seq;
        }
    }

    (void)stamp;
    extendedSearch(current_frame, buffer_local);
}

/**
 * @brief Use the AMI polynomial window to continue tracks without a direct match.
 */
void AMI::extendedSearch(std::vector<PointState>& no_nn_current_frame, std::vector<seqPointer>& sequences_no_insert)
{
    if (!no_nn_current_frame.empty()) {
        const double insert_time = -1.0;

        for (auto it_seq = sequences_no_insert.begin(); it_seq != sequences_no_insert.end();) {
            if ((*it_seq)->empty()) {
                ++it_seq;
                continue;
            }

            std::vector<double> x;
            std::vector<double> y;
            std::vector<double> time;
            for (const auto& point : **it_seq) {
                if (point.led_state) {
                    x.push_back(point.position.x());
                    y.push_back(point.position.y());
                    time.push_back(point.stamp);
                }
            }

            PointState& last_point = (*it_seq)->end()[-1];
            PredictionStats x_predictions = selectStatisticsValues(x, time, insert_time);
            PredictionStats y_predictions = selectStatisticsValues(y, time, insert_time);
            if (!x_predictions.model_reg_computed || !y_predictions.model_reg_computed) {
                ++it_seq;
                continue;
            }

            last_point.x_statistics = x_predictions;
            last_point.y_statistics = y_predictions;
            const double x_predicted = last_point.x_statistics.predicted_coordinate;
            const double y_predicted = last_point.y_statistics.predicted_coordinate;

            last_point.x_statistics.confidence_interval = std::min(last_point.x_statistics.confidence_interval, params_ami_.max_px_shift_x * 2.0);
            last_point.y_statistics.confidence_interval = std::min(last_point.y_statistics.confidence_interval, params_ami_.max_px_shift_y * 2.0);
            last_point.x_statistics.confidence_interval = std::max(last_point.x_statistics.confidence_interval, params_ami_.max_px_shift_x);
            last_point.y_statistics.confidence_interval = std::max(last_point.y_statistics.confidence_interval, params_ami_.max_px_shift_x);

            const Eigen::Vector2d box_left_top(
                x_predicted - last_point.x_statistics.confidence_interval,
                y_predicted - last_point.y_statistics.confidence_interval);
            const Eigen::Vector2d box_right_bottom(
                x_predicted + last_point.x_statistics.confidence_interval,
                y_predicted + last_point.y_statistics.confidence_interval);

            double closest_distance = std::numeric_limits<double>::max();
            std::vector<PointState>::iterator selected_it = no_nn_current_frame.end();
            for (auto it_frame = no_nn_current_frame.begin(); it_frame != no_nn_current_frame.end(); ++it_frame) {
                const double curr_dist = euclideanDistance((*it_frame).position, last_point.position);
                if (curr_dist <= closest_distance) {
                    closest_distance = curr_dist;
                    selected_it = it_frame;
                }
            }

            if (selected_it != no_nn_current_frame.end()) {
                if (isInsideBox((*selected_it).position, box_left_top, box_right_bottom)) {
                    selected_it->x_statistics = last_point.x_statistics;
                    selected_it->y_statistics = last_point.y_statistics;
                    addPointToSequenceAndCheckLength(*(*it_seq), *selected_it);
                    no_nn_current_frame.erase(selected_it);
                    it_seq = sequences_no_insert.erase(it_seq);
                } else {
                    ++it_seq;
                }
            } else {
                ++it_seq;
            }
        }
    }

    // Tracks with no inserted detection get an OFF sample to preserve blink timing.
    for (auto seq : sequences_no_insert) {
        addVirtualPointToSequencesWithNoInsert(seq);
    }

    // Bound the total number of candidate tracks.
    if (params_ami_.max_buffer_length < static_cast<int>(buffer_.size())) {
        const int diff = static_cast<int>(buffer_.size()) - params_ami_.max_buffer_length;
        for (int i = 0; i < diff; ++i) {
            buffer_.erase(buffer_.end() - 1);
        }
        return;
    }

    // Unmatched detections become new candidate tracks.
    for (auto point : no_nn_current_frame) {
        std::vector<PointState> vect;
        vect.reserve(params_ami_.stored_seq_len_factor * static_cast<int>(sequences_[0].size()));
        vect.emplace_back(point);
        buffer_.emplace_back(std::make_shared<std::vector<PointState>>(vect));
    }
}

/**
 * @brief Check if a point is inside an axis-aligned image-plane window.
 */
bool AMI::isInsideBox(const Eigen::Vector2d& query, const Eigen::Vector2d& left_top, const Eigen::Vector2d& right_bottom) const
{
    return left_top.x() <= query.x() && query.x() <= right_bottom.x()
        && left_top.y() <= query.y() && query.y() <= right_bottom.y();
}

/**
 * @brief Euclidean distance in image space.
 */
double AMI::euclideanDistance(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2) const
{
    return (point1 - point2).norm();
}

/**
 * @brief Append point and enforce fixed maximum sequence length.
 */
void AMI::addPointToSequenceAndCheckLength(std::vector<PointState>& insert_seq, const PointState& point)
{
    const int max_size = static_cast<int>(sequences_[0].size()) * params_ami_.stored_seq_len_factor;
    uvdar_core::tracking::appendBounded(insert_seq, point, max_size);
}

/**
 * @brief Add an artificial zero point when no sample was matched.
 */
void AMI::addVirtualPointToSequencesWithNoInsert(seqPointer& seq)
{
    PointState virtual_point = seq->back();
    virtual_point.led_state = false;
    virtual_point.virtual_point = true;
    virtual_point.associated_with_detection = false;
    virtual_point.stamp = -1.0;
    addPointToSequenceAndCheckLength(*seq, virtual_point);
}

/**
 * @brief Create regression statistics for one axis.
 */
PredictionStats AMI::selectStatisticsValues(const std::vector<double>& values, const std::vector<double>& time, const double& insert_time)
{
    PredictionStats stats;
    stats.mean_independent = calcWeightedMean(time, calcNormalizedWeightVect(time));
    stats.target_time = insert_time;
    stats.reference_time = time.empty() ? insert_time : time.back();
    stats.model_reg_computed = false;

    int model_order = params_ami_.poly_order;
    if (!values.empty() && static_cast<int>(values.size()) < model_order) {
        model_order = static_cast<int>(values.size()) - 2;
    }

    if (static_cast<int>(values.size()) > 1 && model_order >= 0) {
        auto [coeff, predicted_vals_past] = polyReg(values, time, calcNormalizedWeightVect(time), model_order);
        stats.coeff = coeff;
        stats.predicted_vals_past = predicted_vals_past;

        const bool all_coeff_zero = std::all_of(coeff.begin(), coeff.end(), [](double coefficient) { return coefficient == 0.0; });
        if (!all_coeff_zero) {
            for (int i = 0; i < static_cast<int>(coeff.size()); ++i) {
                stats.predicted_coordinate += coeff[i] * std::pow(insert_time, i);
            }
        }
        stats.confidence_interval = confidenceInterval(stats, time, values, calcNormalizedWeightVect(time), static_cast<int>(params_ami_.conf_probab_percent));
        stats.model_reg_computed = true;
    }
    stats.extended_search = true;
    return stats;
}

/**
 * @brief Fit weighted model and return coefficients and predicted history.
 */
std::tuple<std::vector<double>, Eigen::VectorXd> AMI::polyReg(
    const std::vector<double>& coordinate,
    const std::vector<double>& time,
    const std::vector<double>& weights,
    const int model_order) const
{
    const int sample_count = static_cast<int>(time.size());
    Eigen::MatrixXd design_matrix(sample_count, model_order + 1);
    Eigen::VectorXd pixel_vect = Eigen::VectorXd::Map(coordinate.data(), coordinate.size());
    Eigen::VectorXd weight_vect = Eigen::VectorXd::Map(weights.data(), weights.size());
    Eigen::VectorXd result(model_order + 1);

    // Weighted polynomial least squares:
    // argmin_c || W^(1/2) (A c - y) ||^2, A_ij = t_i^j.
    Eigen::MatrixXd weight_mat = weight_vect.asDiagonal();
    weight_mat = weight_mat.cwiseSqrt();

    for (int index = 0; index < sample_count; ++index) {
        for (int degree = 0; degree < model_order + 1; ++degree) {
            design_matrix(index, degree) = (degree == 0) ? 1.0 : std::pow(time[index], degree);
        }
    }

    Eigen::MatrixXd weighted_design_mat = weight_mat * design_matrix;
    Eigen::VectorXd weighted_pixel_vect = weight_mat * pixel_vect;
    result = weighted_design_mat.householderQr().solve(weighted_pixel_vect);

    std::vector<double> coeff;
    coeff.reserve(result.size());
    for (int i = 0; i < static_cast<int>(result.size()); ++i) {
        coeff.push_back(result[i]);
    }

    auto prediction = design_matrix * result;
    return { coeff, prediction };
}

/**
 * @brief Build time-decayed normalized weights.
 */
std::vector<double> AMI::calcNormalizedWeightVect(const std::vector<double>& time) const
{
    return uvdar_core::tracking::normalizedExponentialWeights(time, params_ami_.decay_factor, true);
}

/**
 * @brief Weighted mean of a sample vector.
 */
double AMI::calcWeightedMean(const std::vector<double>& values, const std::vector<double>& weights) const
{
    if (weights.size() != values.size()) {
        return -1;
    }

    double weighted_sum = 0.0;
    for (std::size_t i = 0; i < values.size(); ++i) {
        weighted_sum += values[i] * weights[i];
    }
    return weighted_sum;
}

/**
 * @brief Compute weighted sum of squared residuals.
 */
double AMI::calcWSSR(const Eigen::VectorXd& predictions, const std::vector<double>& values, const std::vector<double>& weights) const
{
    double sum_squared_residuals = 0.0;
    for (int i = 0; i < static_cast<int>(values.size()); ++i) {
        sum_squared_residuals += weights[i] * std::pow(predictions[i] - values[i], 2);
    }
    return sum_squared_residuals;
}

/**
 * @brief Compute prediction confidence interval in pixels.
 */
double AMI::confidenceInterval(
    const PredictionStats& stats,
    const std::vector<double>& time,
    const std::vector<double>& values,
    const std::vector<double> weights,
    const int& wanted_percentage) const
{
    const double wssr = calcWSSR(stats.predicted_vals_past, values, weights);
    const int n = static_cast<int>(values.size());
    const int dof = n - static_cast<int>(stats.coeff.size());

    if (stats.mean_independent == -1.0 || dof <= 0) {
        return -1.0;
    }

    // Student-t prediction interval from weighted residual variance.
    const double unb_estimate_error_var = wssr / dof;
    double var_time = 0.0;
    for (const double t : time) {
        var_time += std::pow(t - stats.mean_independent, 2);
    }

    const double standard_error = std::sqrt(unb_estimate_error_var + (1 + 1 / n + ((stats.target_time - stats.mean_independent) / var_time)));
    const double percentage_scaled = static_cast<double>(wanted_percentage) / 100.0;
    const double percentage_two_sided = (1 - percentage_scaled) / 2 + percentage_scaled;

    boost::math::students_t dist(dof);
    const double t = quantile(dist, percentage_two_sided);
    return t * standard_error;
}

/**
 * @brief Keep latest samples and drop initial part so only relevant signal remains.
 */
std::vector<PointState> AMI::processSequenceBasic(const seqPointer& sequence, const std::vector<bool>& original_sequence) const
{
    return uvdar_core::tracking::trailingSamples(*sequence, original_sequence.size());
}

/**
 * @brief Evaluate all active sequences and perform final AMI matching.
 */
std::vector<TrackResult> AMI::getResults() const
{
    constexpr int frame_length = 16;
    constexpr bool com_mode = false;
    std::vector<TrackResult> results;

    if (params_ami_.debug) {
        std::cout << "[AMI] Retrieved signals:{\n";
    }

    for (const auto sequence : buffer_) {
        std::vector<bool> led_states;
        std::vector<int> msg_frame(frame_length, 0);
        std::vector<PointState> selected;

        if (com_mode) {
            std::vector<bool> header_mask = { true, true, true, true };

            if (static_cast<int>(sequence->size()) > static_cast<int>(header_mask.size())) {
                bool matched = false;
                for (int i = static_cast<int>(sequence->size()) - 1; i >= static_cast<int>(header_mask.size()); --i) {
                    int match_count = 0;
                    for (int j = 0; j < static_cast<int>(header_mask.size()); ++j) {
                        if ((*sequence)[i - j].led_state == header_mask[static_cast<int>(header_mask.size()) - j - 1]) {
                            ++match_count;
                        }
                    }

                    if (match_count == static_cast<int>(header_mask.size())) {
                        matched = true;
                        const int bits_on_right = static_cast<int>(sequence->size()) - i - 1;
                        const int bits_on_left = i - static_cast<int>(header_mask.size()) + 1;
                        if (bits_on_right >= static_cast<int>(sequences_[0].size())) {
                            if (bits_on_right >= static_cast<int>(sequences_[0].size()) + frame_length) {
                                msg_frame.clear();
                            } else if (bits_on_left >= frame_length) {
                                msg_frame.clear();
                            }
                        } else if (bits_on_left >= frame_length + static_cast<int>(sequences_[0].size())) {
                            msg_frame.clear();
                        }

                        for (int k = 0; k < static_cast<int>(sequences_[0].size()); ++k) {
                            if (bits_on_right >= static_cast<int>(sequences_[0].size())) {
                                selected.push_back((*sequence)[i + 1 + k]);
                            } else if (bits_on_left >= frame_length + static_cast<int>(sequences_[0].size())) {
                                selected.push_back((*sequence)[i - 4 + 1 - frame_length - static_cast<int>(sequences_[0].size()) + k]);
                            } else {
                                selected = processSequenceBasic(sequence, sequences_[0]);
                                break;
                            }
                        }

                        for (int k = 0; k < frame_length; ++k) {
                            if (bits_on_right >= static_cast<int>(sequences_[0].size())) {
                                if (bits_on_right >= static_cast<int>(sequences_[0].size()) + frame_length) {
                                    msg_frame.push_back(static_cast<int>(
                                        (*sequence)[i + 1 + static_cast<int>(sequences_[0].size()) + k].led_state));
                                } else if (bits_on_left >= frame_length) {
                                    msg_frame.push_back(static_cast<int>((*sequence)[i - 4 + 1 - frame_length + k].led_state));
                                }
                            } else if (bits_on_left >= frame_length + static_cast<int>(sequences_[0].size())) {
                                msg_frame.push_back(static_cast<int>((*sequence)[i - 4 + 1 - frame_length + k].led_state));
                            }
                        }
                        break;
                    }
                }

                if (!matched) {
                    selected = processSequenceBasic(sequence, sequences_[0]);
                }
            } else {
                selected = *sequence;
            }
        } else {
            selected = processSequenceBasic(sequence, sequences_[0]);
        }

        for (const auto point : selected) {
            led_states.push_back(point.led_state);
        }

        if (params_ami_.debug) {
            std::cout << "[ ";
            for (const bool state : led_states) {
                std::cout << (state ? "1," : "0,");
            }
            std::cout << "]\n";
        }

        const int id = matcher_ ? matcher_->matchSignal(led_states) : -2;
        if (selected.empty()) {
            continue;
        }
        TrackResult result;
        result.state = selected.back();
        result.id = id;
        results.push_back(result);
    }

    if (params_ami_.debug) {
        std::cout << "}\n";
    }

    return results;
}

/**
 * @brief Destructor.
 */
AMI::~AMI() = default;

} // namespace uvdar_core::tracking::ami
