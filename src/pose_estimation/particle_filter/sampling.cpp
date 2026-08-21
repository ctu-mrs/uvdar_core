#include "uvdar_core/pose_estimation/particle_filter/sampling.hpp"

#include <utility>

namespace uvdar_core::pose_estimation::particle_filter {

double randomUniform01(std::mt19937& generator)
{
    return std::uniform_real_distribution<double>(0.0, 1.0)(generator);
}

Eigen::Vector3d randomUnitVector(std::mt19937& generator)
{
    Eigen::Vector3d vector;
    do {
        vector = Eigen::Vector3d(
            std::uniform_real_distribution<double>(-1.0, 1.0)(generator),
            std::uniform_real_distribution<double>(-1.0, 1.0)(generator),
            std::uniform_real_distribution<double>(-1.0, 1.0)(generator));
    } while (vector.squaredNorm() < 1.0e-12);
    return vector.normalized();
}

std::vector<Hypothesis> generatePoseMutations(
    const Hypothesis& source,
    const int count,
    const double position_max_step,
    const double angle_max_step,
    std::mt19937& generator)
{
    std::vector<Hypothesis> output;
    output.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
        Hypothesis mutation = source;
        mutation.unique_id = static_cast<int>(generator());
        mutation.flag = HypothesisFlag::Neutral;
        mutation.pose.position += randomUnitVector(generator) * randomUniform01(generator) * position_max_step;
        mutation.pose.orientation = Eigen::AngleAxisd(
            randomUniform01(generator) * angle_max_step,
            randomUnitVector(generator))
            * source.pose.orientation;
        output.push_back(std::move(mutation));
    }
    return output;
}

std::vector<Hypothesis> generateVelocityMutations(
    const Hypothesis& source,
    const int count,
    const double velocity_max_step,
    std::mt19937& generator)
{
    std::vector<Hypothesis> output;
    output.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
        Hypothesis mutation = source;
        mutation.unique_id = static_cast<int>(generator());
        mutation.flag = HypothesisFlag::Neutral;
        mutation.twist.linear += randomUnitVector(generator) * randomUniform01(generator) * velocity_max_step;
        output.push_back(std::move(mutation));
    }
    return output;
}

} // namespace uvdar_core::pose_estimation::particle_filter
