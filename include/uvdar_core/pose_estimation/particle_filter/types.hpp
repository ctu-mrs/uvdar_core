#pragma once

#include <cstdint>
#include <list>
#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "uvdar_core/pose_estimation/types.hpp"

namespace uvdar_core::pose_estimation::particle_filter {

using uvdar_core::pose_estimation::LEDMarker;
using uvdar_core::pose_estimation::Pose;
using uvdar_core::pose_estimation::PoseMeasurement;
using uvdar_core::pose_estimation::TimedPoseMeasurements;
using uvdar_core::pose_estimation::TrackedPoint;
using uvdar_core::pose_estimation::Twist;
using uvdar_core::pose_estimation::quaternionToRpy;
using uvdar_core::pose_estimation::rpyToQuaternion;
using uvdar_core::pose_estimation::transformPose;

/**
 * @brief Integer image observation used by the particle-filter scoring model.
 *
 * The generalized tracker emits subpixel values, but this backend intentionally
 * rounds to the integer reprojection scorer.
 */
struct ImagePointIdentified {
    int id = -1;
    Eigen::Vector2i position = Eigen::Vector2i::Zero();
};

/**
 * @brief Observations grouped by target id and optionally by image distance.
 */
struct ImageCluster {
    int id = -1;
    std::vector<ImagePointIdentified> points;
};

/**
 * @brief Discrete particle status after reprojection scoring.
 */
enum class HypothesisFlag {
    Neutral,
    Unfit,
    Verified,
};

/**
 * @brief One particle: pose, simple linear velocity, and verification status.
 */
struct Hypothesis {
    int index = -1;
    Pose pose;
    Twist twist;
    HypothesisFlag flag = HypothesisFlag::Neutral;
    double observed = 0.0;
    double propagated = 0.0;
    int unique_id = 0;
};

/**
 * @brief Hypothesis set belonging to one target.
 *
 * verified_count is maintained incrementally so measurement extraction can
 * quickly decide whether a target has enough support.
 */
struct AssociatedHypotheses {
    std::list<Hypothesis> hypotheses;
    int target = -1;
    int verified_count = 0;

    /**
     * @brief Iterator access by list index for list-based hypothesis storage.
     */
    std::list<Hypothesis>::iterator at(int index)
    {
        auto it = hypotheses.begin();
        std::advance(it, index);
        return it;
    }

    /**
     * @brief Return particles currently accepted by reprojection thresholds.
     */
    std::vector<Hypothesis> verified() const
    {
        std::vector<Hypothesis> output;
        for (const auto& hypothesis : hypotheses) {
            if (hypothesis.flag == HypothesisFlag::Verified) {
                output.push_back(hypothesis);
            }
        }
        return output;
    }

    /**
     * @brief Add one hypothesis and maintain the verified counter.
     */
    void add(const Hypothesis& hypothesis)
    {
        hypotheses.push_back(hypothesis);
        if (hypothesis.flag == HypothesisFlag::Verified) {
            ++verified_count;
        }
    }

    /**
     * @brief Add a batch of hypotheses.
     */
    void add(const std::vector<Hypothesis>& new_hypotheses)
    {
        for (const auto& hypothesis : new_hypotheses) {
            add(hypothesis);
        }
    }

    /**
     * @brief Erase a hypothesis while keeping verified_count consistent.
     */
    std::list<Hypothesis>::iterator erase(std::list<Hypothesis>::iterator it)
    {
        if (it->flag == HypothesisFlag::Verified) {
            --verified_count;
        }
        return hypotheses.erase(it);
    }

    /**
     * @brief Remove all particles that failed the unfit reprojection threshold.
     */
    void removeUnfit()
    {
        for (auto it = hypotheses.begin(); it != hypotheses.end();) {
            if (it->flag == HypothesisFlag::Unfit) {
                it = erase(it);
            } else {
                ++it;
            }
        }
    }

    /**
     * @brief Mark a particle as verified by the low reprojection-error gate.
     */
    void setVerified(std::list<Hypothesis>::iterator it)
    {
        if (it->flag != HypothesisFlag::Verified) {
            ++verified_count;
        }
        it->flag = HypothesisFlag::Verified;
    }

    /**
     * @brief Mark a particle as rejected by the high reprojection-error gate.
     */
    void setUnfit(std::list<Hypothesis>::iterator it)
    {
        if (it->flag == HypothesisFlag::Verified) {
            --verified_count;
        }
        it->flag = HypothesisFlag::Unfit;
    }

    /**
     * @brief Return a particle to tentative status between the two gates.
     */
    void setNeutral(std::list<Hypothesis>::iterator it)
    {
        if (it->flag == HypothesisFlag::Verified) {
            --verified_count;
        }
        it->flag = HypothesisFlag::Neutral;
    }
};

} // namespace uvdar_core::pose_estimation::particle_filter
