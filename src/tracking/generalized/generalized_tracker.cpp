#include "uvdar_core/tracking/generalized/generalized_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

/*
 * Generalized tracker vs. AMI tracker
 *
 * What stays the same:
 * - Both trackers maintain a dynamic buffer of candidate blinking-marker tracks.
 *   Each buffer entry is a time series of ON/OFF point states for one marker
 *   hypothesis.
 * - Both trackers run the same high-level frame pipeline:
 *   1. convert detector points into internal point states,
 *   2. perform a fast local association against active tracks,
 *   3. run an extended model-based search for tracks missed locally,
 *   4. append virtual OFF states for tracks without a detection,
 *   5. remove stale tracks and start new tracks from unmatched detections.
 * - Both trackers ignore OFF states when fitting the motion model. OFF states
 *   preserve blink history and temporary occlusions, but they are not trusted
 *   as measured image locations.
 * - Both trackers use exponentially decayed time weights, so recent detections
 *   influence the fitted motion model more strongly than old detections.
 * - Both trackers reuse the AMI SignalMatcher to assign marker IDs from the
 *   trailing ON/OFF sequence.
 * - Both trackers keep per-track history bounded by the configured sequence
 *   length factor and remove tracks with too many consecutive OFF states.
 *
 * What is different:
 * - AMI stores image positions as integer pixels. The generalized tracker stores
 *   positions as Eigen::Vector2d so subpixel detector outputs and continuous
 *   predictions can be passed forward without rounding.
 * - AMI accepts point detections without carrying their detector covariance
 *   through the tracker. The generalized tracker consumes and preserves 2D
 *   detector covariance for every associated detection.
 * - AMI local search uses an axis-aligned max-pixel-shift window around the last
 *   point. The generalized tracker still respects max pixel shifts, but also
 *   gates candidates with Mahalanobis distance using the current state and
 *   measurement covariance.
 * - AMI extended search produces confidence intervals mainly for search-window
 *   sizing. The generalized tracker also turns fitted-model uncertainty into a
 *   prediction covariance intended for downstream pose estimation.
 * - AMI combines measurement and prediction information implicitly. The
 *   generalized tracker keeps measurement_covariance, prediction_covariance, and
 *   a conservative combined covariance in the output state.
 * - AMI fits on absolute timestamps. The generalized tracker fits the same
 *   kind of weighted motion model around a reference time at the newest sample,
 *   keeping basis powers numerically smaller.
 * - AMI fills the legacy fields of TrackedBlinker. The generalized backend
 *   fills the same shared message plus covariance and uncertainty fields for
 *   the next pose-estimation stage.
 * - AMI exposes AMI-specific parameter names and types. The generalized tracker
 *   uses ParamsGeneralized and a separate namespace/path so it can evolve
 *   without changing the legacy AMI implementation.
 */

namespace uvdar_core::tracking::generalized {

namespace {

constexpr double kMinimumVariance = 1.0e-6;

double validVarianceOrFallback(double variance, double fallback)
{
    const double sanitized_fallback = std::isfinite(fallback) && fallback > 0.0 ? fallback : kMinimumVariance;
    if (!std::isfinite(variance) || variance <= 0.0) {
        return std::max(sanitized_fallback, kMinimumVariance);
    }
    return std::max(variance, kMinimumVariance);
}

double validNonNegativeVarianceOrZero(double variance)
{
    if (!std::isfinite(variance) || variance < 0.0) {
        return 0.0;
    }
    return variance;
}

Eigen::VectorXd basisRow(double tau, int order)
{
    Eigen::VectorXd row(order + 1);
    row(0) = 1.0;
    for (int degree = 1; degree <= order; ++degree) {
        row(degree) = row(degree - 1) * tau;
    }
    return row;
}

} // namespace

GeneralizedTracker::GeneralizedTracker(const ParamsGeneralized& params)
    : params_(params)
{
}

GeneralizedTracker::~GeneralizedTracker() = default;

void GeneralizedTracker::setupSequenceMatcher(std::vector<std::vector<bool>> sequences)
{
    if (sequences.empty()) {
        throw std::invalid_argument("[tracker] GeneralizedTracker - no sequences provided.");
    }
    if (sequences[0].empty()) {
        throw std::invalid_argument("[tracker] GeneralizedTracker - first sequence is empty.");
    }
    const std::size_t sequence_size = sequences[0].size();
    for (std::size_t index = 0; index < sequences.size(); ++index) {
        if (sequences[index].empty()) {
            throw std::invalid_argument("[tracker] GeneralizedTracker - sequence " + std::to_string(index) + " is empty.");
        }
        if (sequences[index].size() != sequence_size) {
            throw std::invalid_argument("[tracker] GeneralizedTracker - all sequences must have equal length.");
        }
    }

    sequences_ = std::move(sequences);
    matcher_ = std::make_unique<uvdar_core::tracking::ami::SignalMatcher>(sequences_, params_.allowed_BER_per_seq);

    if (params_.stored_seq_len_factor * static_cast<int>(sequences_[0].size()) < params_.max_zeros_consecutive) {
        throw std::invalid_argument("[tracker] GeneralizedTracker - max_zeros_consecutive does not fit the configured sequence buffer.");
    }
}

void GeneralizedTracker::processBuffer(const ImagePointsWithCovariancesStamped& points)
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
        // Normalize detector covariance before it enters any gates or output state.
        state.measurement_covariance = regularizeCovariance(point.covariance);
        state.prediction_covariance = addProcessNoise(state.measurement_covariance);
        state.covariance = state.measurement_covariance;
        state.led_state = true;
        state.virtual_point = false;
        state.associated_with_detection = true;
        state.stamp = points.stamp;
        current_frame.push_back(state);
    }

    // Work on a pointer copy so matched tracks can be removed from this frame's search only.
    std::vector<SeqPointer> unmatched_sequences = buffer_;
    localSearch(current_frame, unmatched_sequences);
    extendedSearch(current_frame, unmatched_sequences, points.stamp);
    cleanPotentialBuffer();
}

void GeneralizedTracker::localSearch(std::vector<PointState>& current_frame, std::vector<SeqPointer>& unmatched_sequences)
{
    const double gate_limit = params_.association_gate_sigma * params_.association_gate_sigma;

    for (auto seq = unmatched_sequences.begin(); seq != unmatched_sequences.end();) {
        if ((*seq)->empty()) {
            ++seq;
            continue;
        }

        const PointState& last_inserted = (*seq)->back();
        auto selected = current_frame.end();
        double best_score = std::numeric_limits<double>::max();
        const Covariance2D last_position_covariance = last_inserted.associated_with_detection
            ? last_inserted.measurement_covariance
            : last_inserted.prediction_covariance;

        for (auto point = current_frame.begin(); point != current_frame.end(); ++point) {
            // Local search trusts the last state, but widens the gate with detector uncertainty.
            Covariance2D gate_covariance = addCovariances(last_position_covariance, point->measurement_covariance);
            gate_covariance = addProcessNoise(gate_covariance);
            const double score = mahalanobisSquared(point->position, last_inserted.position, gate_covariance);
            const Eigen::Vector2d delta = (point->position - last_inserted.position).cwiseAbs();
            const double half_width_x = std::max(params_.max_px_shift_x, params_.association_gate_sigma * std::sqrt(gate_covariance.c00));
            const double half_width_y = std::max(params_.max_px_shift_y, params_.association_gate_sigma * std::sqrt(gate_covariance.c11));

            if (score <= gate_limit && delta.x() <= half_width_x && delta.y() <= half_width_y && score < best_score) {
                best_score = score;
                selected = point;
            }
        }

        if (selected == current_frame.end()) {
            ++seq;
            continue;
        }

        selected->predicted_position = last_inserted.position;
        selected->prediction_covariance = addProcessNoise(last_position_covariance);
        // Publish a conservative state covariance for the next estimator.
        selected->covariance = addCovariances(selected->measurement_covariance, selected->prediction_covariance);
        selected->x_statistics.extended_search = false;
        selected->y_statistics.extended_search = false;
        addPointToSequenceAndCheckLength(**seq, *selected);
        current_frame.erase(selected);
        seq = unmatched_sequences.erase(seq);
    }
}

void GeneralizedTracker::extendedSearch(std::vector<PointState>& unmatched_points, std::vector<SeqPointer>& unmatched_sequences, double stamp)
{
    const double gate_limit = params_.association_gate_sigma * params_.association_gate_sigma;

    for (auto seq = unmatched_sequences.begin(); seq != unmatched_sequences.end();) {
        if ((*seq)->empty()) {
            ++seq;
            continue;
        }

        // Extended search uses the fitted model when the fixed local gate misses.
        PointState prediction;
        const bool predicted = predictSequence(**seq, stamp, prediction);
        if (!predicted) {
            ++seq;
            continue;
        }

        auto selected = unmatched_points.end();
        double best_score = std::numeric_limits<double>::max();
        for (auto point = unmatched_points.begin(); point != unmatched_points.end(); ++point) {
            // Candidate detections must agree with both the model prediction and measurement noise.
            Covariance2D gate_covariance = addCovariances(prediction.prediction_covariance, point->measurement_covariance);
            gate_covariance = addProcessNoise(gate_covariance);
            const double score = mahalanobisSquared(point->position, prediction.predicted_position, gate_covariance);
            const Eigen::Vector2d delta = (point->position - prediction.predicted_position).cwiseAbs();
            const double half_width_x = std::max(prediction.x_statistics.confidence_interval, params_.max_px_shift_x);
            const double half_width_y = std::max(prediction.y_statistics.confidence_interval, params_.max_px_shift_y);

            if (score <= gate_limit && delta.x() <= half_width_x && delta.y() <= half_width_y && score < best_score) {
                best_score = score;
                selected = point;
            }
        }

        if (selected == unmatched_points.end()) {
            ++seq;
            continue;
        }

        selected->predicted_position = prediction.predicted_position;
        selected->prediction_covariance = prediction.prediction_covariance;
        // Keep both source covariances and the propagated combined covariance in the state.
        selected->covariance = addCovariances(selected->measurement_covariance, selected->prediction_covariance);
        selected->x_statistics = prediction.x_statistics;
        selected->y_statistics = prediction.y_statistics;
        selected->x_statistics.extended_search = true;
        selected->y_statistics.extended_search = true;
        addPointToSequenceAndCheckLength(**seq, *selected);
        unmatched_points.erase(selected);
        seq = unmatched_sequences.erase(seq);
    }

    // Tracks without a detection survive as OFF states using the best predicted position.
    for (const SeqPointer& sequence : unmatched_sequences) {
        addVirtualPointToSequence(sequence, stamp);
    }

    // Remaining detections are new track hypotheses.
    for (const PointState& point : unmatched_points) {
        startSequence(point);
    }
}

void GeneralizedTracker::cleanPotentialBuffer()
{
    for (std::size_t index = 0; index < buffer_.size();) {
        const SeqPointer& sequence = buffer_[index];
        int consecutive_zeros = 0;
        for (auto point = sequence->rbegin(); point != sequence->rend(); ++point) {
            if (!point->led_state) {
                ++consecutive_zeros;
            } else {
                break;
            }
        }

        if (consecutive_zeros > params_.max_zeros_consecutive) {
            buffer_.erase(buffer_.begin() + static_cast<std::ptrdiff_t>(index));
            track_ids_.erase(track_ids_.begin() + static_cast<std::ptrdiff_t>(index));
            continue;
        }
        ++index;
    }

    while (params_.max_buffer_length > 0 && static_cast<int>(buffer_.size()) > params_.max_buffer_length) {
        buffer_.erase(buffer_.begin());
        track_ids_.erase(track_ids_.begin());
    }
}

void GeneralizedTracker::addPointToSequenceAndCheckLength(std::vector<PointState>& sequence, const PointState& point)
{
    sequence.push_back(point);
    const int max_size = static_cast<int>(sequences_[0].size()) * params_.stored_seq_len_factor;
    if (static_cast<int>(sequence.size()) > max_size) {
        sequence.erase(sequence.begin());
    }
}

void GeneralizedTracker::addVirtualPointToSequence(const SeqPointer& sequence, double stamp)
{
    if (!sequence || sequence->empty()) {
        return;
    }

    PointState virtual_point;
    if (!predictSequence(*sequence, stamp, virtual_point)) {
        virtual_point = sequence->back();
        virtual_point.predicted_position = virtual_point.position;
        const Covariance2D last_position_covariance = virtual_point.associated_with_detection
            ? virtual_point.measurement_covariance
            : virtual_point.prediction_covariance;
        virtual_point.prediction_covariance = addProcessNoise(last_position_covariance);
        virtual_point.covariance = virtual_point.prediction_covariance;
        virtual_point.stamp = stamp;
    }

    virtual_point.position = virtual_point.predicted_position;
    virtual_point.measurement_covariance = virtual_point.prediction_covariance;
    virtual_point.covariance = virtual_point.prediction_covariance;
    virtual_point.led_state = false;
    virtual_point.virtual_point = true;
    virtual_point.associated_with_detection = false;
    virtual_point.stamp = stamp;
    addPointToSequenceAndCheckLength(*sequence, virtual_point);
}

void GeneralizedTracker::startSequence(const PointState& point)
{
    std::vector<PointState> sequence;
    sequence.reserve(static_cast<std::size_t>(params_.stored_seq_len_factor * static_cast<int>(sequences_[0].size())));
    sequence.push_back(point);
    buffer_.push_back(std::make_shared<std::vector<PointState>>(std::move(sequence)));
    track_ids_.push_back(next_track_id_++);
}

bool GeneralizedTracker::predictSequence(const std::vector<PointState>& sequence, double target_time, PointState& prediction) const
{
    std::vector<double> x_values;
    std::vector<double> y_values;
    std::vector<double> times;
    std::vector<double> x_variances;
    std::vector<double> y_variances;

    for (const PointState& point : sequence) {
        if (!point.led_state) {
            // OFF samples carry continuity, but not a measured marker location for fitting.
            continue;
        }

        x_values.push_back(point.position.x());
        y_values.push_back(point.position.y());
        times.push_back(point.stamp);
        const Covariance2D covariance = regularizeCovariance(point.measurement_covariance);
        x_variances.push_back(validVarianceOrFallback(covariance.c00, params_.default_measurement_variance));
        y_variances.push_back(validVarianceOrFallback(covariance.c11, params_.default_measurement_variance));
    }

    if (x_values.size() < 2 || y_values.size() < 2) {
        return false;
    }

    PredictionStats x_stats = selectStatisticsValues(x_values, times, x_variances, target_time);
    PredictionStats y_stats = selectStatisticsValues(y_values, times, y_variances, target_time);
    if (!x_stats.model_reg_computed || !y_stats.model_reg_computed) {
        return false;
    }

    prediction = sequence.back();
    prediction.predicted_position = Eigen::Vector2d(x_stats.predicted_coordinate, y_stats.predicted_coordinate);
    prediction.prediction_covariance = addProcessNoise(Covariance2D {
        std::max(x_stats.prediction_variance, kMinimumVariance),
        0.0,
        0.0,
        std::max(y_stats.prediction_variance, kMinimumVariance),
    });
    prediction.covariance = prediction.prediction_covariance;
    prediction.x_statistics = x_stats;
    prediction.y_statistics = y_stats;
    prediction.stamp = target_time;
    return true;
}

PredictionStats GeneralizedTracker::selectStatisticsValues(
    const std::vector<double>& values,
    const std::vector<double>& times,
    const std::vector<double>& variances,
    double target_time) const
{
    PredictionStats stats;
    stats.target_time = target_time;
    stats.reference_time = times.empty() ? target_time : times.back();
    stats.mean_independent = stats.reference_time;

    if (values.size() < 2 || values.size() != times.size() || values.size() != variances.size()) {
        return stats;
    }

    const int sample_count = static_cast<int>(values.size());
    const int model_order = std::max(0, std::min(params_.model_order, sample_count - 1));
    const int parameter_count = model_order + 1;

    Eigen::MatrixXd design(sample_count, parameter_count);
    for (int row = 0; row < sample_count; ++row) {
        // Center time at the newest sample to avoid large powers of absolute ROS time.
        const double tau = times[row] - stats.reference_time;
        for (int degree = 0; degree < parameter_count; ++degree) {
            design(row, degree) = degree == 0 ? 1.0 : design(row, degree - 1) * tau;
        }
    }

    const std::vector<double> temporal_weights = calcNormalizedWeightVect(times);
    Eigen::VectorXd weights(sample_count);
    Eigen::VectorXd observed(sample_count);
    double mean_measurement_variance = 0.0;
    for (int index = 0; index < sample_count; ++index) {
        const double variance = validVarianceOrFallback(variances[index], params_.default_measurement_variance);
        // Recent and precise detections influence the model more strongly.
        weights(index) = temporal_weights[index] / variance;
        observed(index) = values[index];
        mean_measurement_variance += variance;
    }
    mean_measurement_variance /= static_cast<double>(sample_count);

    const Eigen::MatrixXd weight_matrix = weights.asDiagonal();
    const Eigen::MatrixXd weighted_design = weight_matrix.cwiseSqrt() * design;
    const Eigen::VectorXd weighted_observed = weight_matrix.cwiseSqrt() * observed;
    const Eigen::VectorXd coeff = weighted_design.householderQr().solve(weighted_observed);

    stats.predicted_vals_past = design * coeff;
    stats.coeff.reserve(static_cast<std::size_t>(coeff.size()));
    for (int index = 0; index < coeff.size(); ++index) {
        stats.coeff.push_back(coeff(index));
    }

    const Eigen::VectorXd target_row = basisRow(target_time - stats.reference_time, model_order);
    stats.predicted_coordinate = target_row.dot(coeff);

    const Eigen::VectorXd residual = observed - stats.predicted_vals_past;
    const double weighted_rss = residual.transpose() * weight_matrix * residual;
    const int dof = sample_count - parameter_count;
    const double residual_variance = dof > 0 ? weighted_rss / static_cast<double>(dof) : mean_measurement_variance;

    const Eigen::MatrixXd normal = design.transpose() * weight_matrix * design;
    const Eigen::MatrixXd normal_inverse = normal.completeOrthogonalDecomposition().solve(Eigen::MatrixXd::Identity(parameter_count, parameter_count));
    const double model_variance = (target_row.transpose() * normal_inverse * target_row)(0, 0);
    // Prediction variance covers fit residuals and uncertainty in the fitted coefficients.
    stats.prediction_variance = std::max(residual_variance * (1.0 + model_variance), kMinimumVariance);

    const double percentage_scaled = std::clamp(params_.conf_probab_percent / 100.0, 0.0, 0.999999);
    const double percentage_two_sided = (1.0 - percentage_scaled) / 2.0 + percentage_scaled;
    double quantile_value = 1.96;
    if (dof > 0) {
        boost::math::students_t dist(dof);
        quantile_value = quantile(dist, percentage_two_sided);
    } else {
        boost::math::normal dist;
        quantile_value = quantile(dist, percentage_two_sided);
    }
    stats.confidence_interval = quantile_value * std::sqrt(stats.prediction_variance);
    stats.model_reg_computed = true;
    stats.extended_search = true;
    return stats;
}

std::vector<double> GeneralizedTracker::calcNormalizedWeightVect(const std::vector<double>& times) const
{
    std::vector<double> weights;
    weights.reserve(times.size());
    if (times.empty()) {
        return weights;
    }

    const double reference_time = times.back();
    double sum = 0.0;
    for (const double time : times) {
        const double time_distance = reference_time - time;
        const double weight = std::exp(-params_.decay_factor * std::max(0.0, time_distance));
        weights.push_back(weight);
        sum += weight;
    }

    if (sum <= 0.0) {
        const double uniform = 1.0 / static_cast<double>(weights.size());
        std::fill(weights.begin(), weights.end(), uniform);
        return weights;
    }

    for (double& weight : weights) {
        weight /= sum;
    }
    return weights;
}

double GeneralizedTracker::mahalanobisSquared(const Eigen::Vector2d& query, const Eigen::Vector2d& center, const Covariance2D& covariance) const
{
    const Eigen::Vector2d delta = query - center;
    Eigen::Matrix2d matrix = regularizeCovariance(covariance).matrix();
    const double determinant = matrix.determinant();
    if (std::abs(determinant) < kMinimumVariance) {
        matrix(0, 0) += validVarianceOrFallback(0.0, params_.default_measurement_variance);
        matrix(1, 1) += validVarianceOrFallback(0.0, params_.default_measurement_variance);
    }
    return delta.transpose() * matrix.inverse() * delta;
}

Covariance2D GeneralizedTracker::regularizeCovariance(const Covariance2D& covariance) const
{
    Eigen::Matrix2d matrix = covariance.matrix();
    matrix = 0.5 * (matrix + matrix.transpose());
    matrix(0, 0) = validVarianceOrFallback(matrix(0, 0), params_.default_measurement_variance);
    matrix(1, 1) = validVarianceOrFallback(matrix(1, 1), params_.default_measurement_variance);
    if (!std::isfinite(matrix(0, 1)) || !std::isfinite(matrix(1, 0))) {
        matrix(0, 1) = 0.0;
        matrix(1, 0) = 0.0;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(matrix);
    if (solver.info() != Eigen::Success) {
        matrix.setZero();
        matrix(0, 0) = validVarianceOrFallback(0.0, params_.default_measurement_variance);
        matrix(1, 1) = validVarianceOrFallback(0.0, params_.default_measurement_variance);
        return Covariance2D::fromMatrix(matrix);
    }

    Eigen::Vector2d eigenvalues = solver.eigenvalues();
    eigenvalues(0) = std::max(eigenvalues(0), kMinimumVariance);
    eigenvalues(1) = std::max(eigenvalues(1), kMinimumVariance);
    matrix = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
    return Covariance2D::fromMatrix(matrix);
}

Covariance2D GeneralizedTracker::addProcessNoise(const Covariance2D& covariance) const
{
    Eigen::Matrix2d matrix = regularizeCovariance(covariance).matrix();
    const double process_noise_variance = validNonNegativeVarianceOrZero(params_.process_noise_variance);
    matrix(0, 0) += process_noise_variance;
    matrix(1, 1) += process_noise_variance;
    return Covariance2D::fromMatrix(matrix);
}

Covariance2D GeneralizedTracker::addCovariances(const Covariance2D& left, const Covariance2D& right) const
{
    return Covariance2D::fromMatrix(regularizeCovariance(left).matrix() + regularizeCovariance(right).matrix());
}

std::vector<PointState> GeneralizedTracker::processSequenceBasic(const SeqPointer& sequence, const std::vector<bool>& original_sequence) const
{
    std::vector<PointState> selected;
    if (!sequence) {
        return selected;
    }

    if (static_cast<int>(sequence->size()) > static_cast<int>(original_sequence.size())) {
        const int diff = static_cast<int>(sequence->size()) - static_cast<int>(original_sequence.size());
        for (int index = diff; index < static_cast<int>(sequence->size()); ++index) {
            selected.push_back((*sequence)[index]);
        }
    } else {
        selected = *sequence;
    }
    return selected;
}

std::vector<TrackResult> GeneralizedTracker::getResults() const
{
    std::vector<TrackResult> results;
    if (!matcher_) {
        return results;
    }

    results.reserve(buffer_.size());
    for (std::size_t index = 0; index < buffer_.size(); ++index) {
        const SeqPointer& sequence = buffer_[index];
        const std::vector<PointState> selected = processSequenceBasic(sequence, sequences_[0]);
        if (selected.empty()) {
            continue;
        }

        std::vector<bool> led_states;
        led_states.reserve(selected.size());
        for (const PointState& point : selected) {
            led_states.push_back(point.led_state);
        }

        TrackResult result;
        result.state = selected.back();
        result.id = matcher_->matchSignal(led_states);
        result.track_id = index < track_ids_.size() ? track_ids_[index] : 0;
        results.push_back(std::move(result));

        if (params_.debug) {
            std::cout << "[GeneralizedTracker] track=" << result.track_id << " id=" << result.id << " bits=";
            for (const bool state : led_states) {
                std::cout << (state ? '1' : '0');
            }
            std::cout << '\n';
        }
    }
    return results;
}

} // namespace uvdar_core::tracking::generalized
