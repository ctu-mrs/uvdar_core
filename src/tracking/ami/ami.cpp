#include "uvdar_core/tracking/ami/ami.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

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

    sequences_ = std::move(i_sequences);
    matcher_ = std::make_unique<SignalMatcher>(sequences_, params_ami_.allowed_BER_per_seq);

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
    if (points.points.empty()) {
        return;
    }

    std::vector<PointState> current_frame;
    current_frame.reserve(points.points.size());
    for (const Point2D& point_time_stamp : points.points) {
        PointState state;
        state.px_cord = point_time_stamp;
        state.led_state = true;
        state.stamp = points.stamp;
        current_frame.push_back(state);
    }

    findClosestPixelAndInsert(current_frame);
    cleanPotentialBuffer();
}

/**
 * @brief Remove stale trajectories with too many consecutive virtual points.
 */
void AMI::cleanPotentialBuffer()
{
    for (auto it_seq = buffer_.begin(); it_seq != buffer_.end();) {
        bool deleted = false;
        const int number_zeros_till_seq_deleted = (params_ami_.max_zeros_consecutive + params_ami_.allowed_BER_per_seq);
        if (static_cast<int>((*it_seq)->size()) > number_zeros_till_seq_deleted) {
            int cnt = 0;
            for (const auto frame_state : *(*it_seq)) {
                if (!frame_state.led_state) {
                    ++cnt;
                    if (cnt > number_zeros_till_seq_deleted) {
                        break;
                    }
                } else {
                    cnt = 0;
                }
            }
            if (cnt > number_zeros_till_seq_deleted) {
                deleted = true;
                it_seq = buffer_.erase(it_seq);
                continue;
            }
        }
        if (!deleted) {
            ++it_seq;
        }
    }
}

/**
 * @brief Associate unmatched detections with active trajectories using nearest-neighbour search.
 */
void AMI::findClosestPixelAndInsert(std::vector<PointState>& current_frame)
{
    std::vector<seqPointer> buffer_local = buffer_;

    for (auto seq = buffer_local.begin(); seq != buffer_local.end();) {
        if ((*seq)->empty()) {
            ++seq;
            continue;
        }

        const PointState& last_inserted = (*seq)->back();
        const Point2D bb_left_top = last_inserted.px_cord - params_ami_.max_px_shift;
        const Point2D bb_right_bottom = last_inserted.px_cord + params_ami_.max_px_shift;

        std::vector<PointState>::iterator it_2 = current_frame.end();
        double closest_distance = std::numeric_limits<double>::max();
        for (auto it = current_frame.begin(); it != current_frame.end(); ++it) {
            const double curr_dist = euclideanDistance((*it).px_cord, last_inserted.px_cord);
            if (curr_dist <= closest_distance) {
                closest_distance = curr_dist;
                it_2 = it;
            }
        }

        if (it_2 != current_frame.end()) {
            if (isInsideBB((*it_2).px_cord, bb_left_top, bb_right_bottom)) {
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

    extendedSearch(current_frame, buffer_local);
}

/**
 * @brief Use generalized extrapolation to continue tracks where no direct match exists.
 */
void AMI::extendedSearch(std::vector<PointState>& no_nn_current_frame, std::vector<seqPointer>& sequences_no_insert)
{
    if (!no_nn_current_frame.empty()) {
        const double insert_time = no_nn_current_frame.front().stamp;

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
                    x.push_back(point.px_cord.x);
                    y.push_back(point.px_cord.y);
                    time.push_back(point.stamp);
                }
            }

            PointState& last_point = (*it_seq)->end()[-1];
            PredictionStats x_predictions = selectStatisticsValues(x, time, insert_time);
            PredictionStats y_predictions = selectStatisticsValues(y, time, insert_time);
            if (!x_predictions.poly_reg_computed || !y_predictions.poly_reg_computed) {
                ++it_seq;
                continue;
            }

            last_point.x_statistics = x_predictions;
            last_point.y_statistics = y_predictions;
            const double x_predicted = last_point.x_statistics.predicted_coordinate;
            const double y_predicted = last_point.y_statistics.predicted_coordinate;

            last_point.x_statistics.confidence_interval = std::min(last_point.x_statistics.confidence_interval * 2.0, static_cast<double>(params_ami_.max_px_shift.x * 2));
            last_point.y_statistics.confidence_interval = std::min(last_point.y_statistics.confidence_interval * 2.0, static_cast<double>(params_ami_.max_px_shift.y * 2));
            last_point.x_statistics.confidence_interval = std::max(last_point.x_statistics.confidence_interval, static_cast<double>(params_ami_.max_px_shift.x));
            last_point.y_statistics.confidence_interval = std::max(last_point.y_statistics.confidence_interval, static_cast<double>(params_ami_.max_px_shift.y));

            const Point2D bb_left_top = Point2D(
                static_cast<int>(std::floor(x_predicted - last_point.x_statistics.confidence_interval)),
                static_cast<int>(std::floor(y_predicted - last_point.y_statistics.confidence_interval)));
            const Point2D bb_right_bottom = Point2D(
                static_cast<int>(std::ceil(x_predicted + last_point.x_statistics.confidence_interval)),
                static_cast<int>(std::ceil(y_predicted + last_point.y_statistics.confidence_interval)));

            double closest_distance = std::numeric_limits<double>::max();
            std::vector<PointState>::iterator selected_it = no_nn_current_frame.end();
            for (auto it_frame = no_nn_current_frame.begin(); it_frame != no_nn_current_frame.end(); ++it_frame) {
                const double curr_dist = euclideanDistance((*it_frame).px_cord, last_point.px_cord);
                if (curr_dist <= closest_distance) {
                    closest_distance = curr_dist;
                    selected_it = it_frame;
                }
            }

            if (selected_it != no_nn_current_frame.end()) {
                if (isInsideBB((*selected_it).px_cord, bb_left_top, bb_right_bottom)) {
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

    // for sequences with no newly inserted point, add virtual point
    for (auto seq : sequences_no_insert) {
        addVirtualPointToSequencesWithNoInsert(seq);
    }

    // delete the sequences that are over the max_buffer_length. Elements are deleted from the back.
    if (params_ami_.max_buffer_length < static_cast<int>(buffer_.size())) {
        const int diff = static_cast<int>(buffer_.size()) - params_ami_.max_buffer_length;
        for (int i = 0; i < diff; ++i) {
            buffer_.erase(buffer_.end() - 1);
        }
        return;
    }

    // for the points, still no NN found -> start new sequence
    for (auto point : no_nn_current_frame) {
        std::vector<PointState> vect;
        vect.reserve(params_ami_.stored_seq_len_factor * static_cast<int>(sequences_[0].size()));
        vect.emplace_back(point);
        buffer_.emplace_back(std::make_shared<std::vector<PointState>>(vect));
    }
}

/**
 * @brief Check if a point is inside an axis-aligned integer window.
 */
bool AMI::isInsideBB(const Point2D& query, const Point2D& left_top, const Point2D& right_bottom) const
{
    return left_top.x <= query.x && query.x <= right_bottom.x && left_top.y <= query.y && query.y <= right_bottom.y;
}

/**
 * @brief Euclidean distance in image space.
 */
double AMI::euclideanDistance(const Point2D& point1, const Point2D& point2) const
{
    return std::sqrt(std::pow(point1.x - point2.x, 2) + std::pow(point1.y - point2.y, 2));
}

/**
 * @brief Append point and enforce fixed maximum sequence length.
 */
void AMI::addPointToSequenceAndCheckLength(std::vector<PointState>& insert_seq, const PointState& point)
{
    insert_seq.push_back(point);
    const int max_size = static_cast<int>(sequences_[0].size()) * params_ami_.stored_seq_len_factor;
    if (static_cast<int>(insert_seq.size()) > max_size) {
        insert_seq.erase(insert_seq.begin());
    }
}

/**
 * @brief Add an artificial zero point when no sample was matched.
 */
void AMI::addVirtualPointToSequencesWithNoInsert(seqPointer& seq)
{
    PointState virtual_point = seq->back();
    virtual_point.led_state = false;
    addPointToSequenceAndCheckLength(*seq, virtual_point);
}

/**
 * @brief Create regression statistics for one axis.
 */
PredictionStats AMI::selectStatisticsValues(const std::vector<double>& values, const std::vector<double>& time, const double& insert_time)
{
    PredictionStats stats;
    stats.mean_independent = calcWeightedMean(time, calcNormalizedWeightVect(time));
    stats.time_pred = insert_time;
    stats.poly_reg_computed = false;

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
        stats.poly_reg_computed = true;
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
    std::vector<double> weights;
    weights.reserve(time.size());
    if (time.empty()) {
        weights.push_back(1.0);
        return weights;
    }

    const double reference_time = time.back();
    double sum = 0.0;

    for (int i = 0; i < static_cast<int>(time.size()); ++i) {
        const double time_dist = reference_time - time[i];
        const double weight = std::exp(-params_ami_.decay_factor * time_dist);
        sum += weight;
        weights.push_back(weight);
    }

    for (double& weight : weights) {
        weight /= sum;
    }
    return weights;
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

    const double unb_estimate_error_var = wssr / dof;
    double var_time = 0.0;
    for (const double t : time) {
        var_time += std::pow(t - stats.mean_independent, 2);
    }

    const double standard_error = std::sqrt(unb_estimate_error_var + (1 + 1.0 / n + ((stats.time_pred - stats.mean_independent) / var_time)));
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
    std::vector<PointState> selected;
    if (static_cast<int>(sequence->size()) > static_cast<int>(original_sequence.size())) {
        const int diff = static_cast<int>(sequence->size()) - static_cast<int>(original_sequence.size());
        for (int i = diff; i < static_cast<int>(sequence->size()); ++i) {
            selected.push_back((*sequence)[i]);
        }
    } else {
        selected = *sequence;
    }
    return selected;
}

/**
 * @brief Evaluate all active sequences and perform final AMI matching.
 */
std::vector<std::pair<std::pair<PointState, int>, std::vector<int>>> AMI::getResults()
{
    constexpr int frame_length = 16;
    constexpr bool com_mode = false;
    std::vector<std::pair<std::pair<PointState, int>, std::vector<int>>> retrieved_signals;

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
        PointState last_element;
        if (!selected.empty()) {
            last_element = selected.back();
        }
        retrieved_signals.push_back(std::make_pair(std::make_pair(last_element, id), msg_frame));
    }

    if (params_ami_.debug) {
        std::cout << "}\n";
    }

    return retrieved_signals;
}

/**
 * @brief Destructor.
 */
AMI::~AMI() = default;

} // namespace uvdar_core::tracking::ami
