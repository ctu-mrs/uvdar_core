#pragma once

#include <random>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/pose_estimation/particle_filter/types.hpp"

namespace uvdar_core::pose_estimation::particle_filter {

/**
 * @brief Draw a uniformly distributed scalar from [0, 1].
 */
double randomUniform01(std::mt19937& generator);

/**
 * @brief Draw a random unit direction by normalizing a cube sample.
 */
Eigen::Vector3d randomUnitVector(std::mt19937& generator);

/**
 * @brief Create bounded position-and-orientation mutations of one particle.
 */
std::vector<Hypothesis> generatePoseMutations(
    const Hypothesis& source,
    int count,
    double position_max_step,
    double angle_max_step,
    std::mt19937& generator);

/**
 * @brief Create bounded linear-velocity mutations of one particle.
 */
std::vector<Hypothesis> generateVelocityMutations(
    const Hypothesis& source,
    int count,
    double velocity_max_step,
    std::mt19937& generator);

} // namespace uvdar_core::pose_estimation::particle_filter
