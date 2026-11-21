#ifndef HEXAPOD_CORE_LOCOMOTION_SWING_TRAJECTORY_GENERATOR_HPP_
#define HEXAPOD_CORE_LOCOMOTION_SWING_TRAJECTORY_GENERATOR_HPP_

#include "hexapod_core/math/point_3d.hpp"
#include "hexapod_core/math/quintic_spline.hpp"

namespace hexapod::core {

struct SwingConfig {
    double stepHeight{0.035};          // Clearance above highest anchor point in meters (35 mm)
    double probeVelocity{0.050};       // Downward probing velocity at landing in m/s (50 mm/s)
    double apexRatio{0.50};            // Ratio of swing duration where clearance apex occurs (0.0 < ratio < 1.0)
    double maxProbingDistance{0.050};  // Maximum downward probing depth below nominal target (meters)
};

struct TrajectoryPoint {
    Point3D position{};
    Point3D velocity{};
    Point3D acceleration{};
};

// Generates C^2-continuous 3D swing foot trajectories with vertical liftoff,
// configurable clearance, vertical approach, non-zero probing velocity,
// and dynamic C^2 replanning capabilities from instantaneous states.
class SwingTrajectoryGenerator {
public:
    explicit SwingTrajectoryGenerator(const SwingConfig& config = SwingConfig{}) noexcept;

    // Resets and plans a full nominal swing trajectory from liftoff to target
    void reset(const Point3D& liftoffPosition, const Point3D& targetPosition, double swingDurationSec) noexcept;

    // Evaluates the continuous C^2 trajectory state at elapsed time t
    [[nodiscard]] TrajectoryPoint evaluate(double elapsedTimeSec) const noexcept;

    // Replans dynamically from an instantaneous dynamic state (e.g. reflex reaction upon obstacle collision)
    void replan(const TrajectoryPoint& currentState, const Point3D& newTarget,
                double remainingDurationSec, double stepHeightBoost) noexcept;

    [[nodiscard]] const Point3D& getLiftoffPosition() const noexcept { return liftoffPosition_; }
    [[nodiscard]] const Point3D& getTargetPosition() const noexcept { return targetPosition_; }
    [[nodiscard]] double getSwingDuration() const noexcept { return swingDurationSec_; }
    [[nodiscard]] double getApexHeight() const noexcept { return apexZ_; }
    [[nodiscard]] const SwingConfig& getConfig() const noexcept { return config_; }
    void setConfig(const SwingConfig& config) noexcept { config_ = config; }

    [[nodiscard]] bool isProbing(double elapsedTimeSec) const noexcept {
        return elapsedTimeSec >= swingDurationSec_;
    }

private:
    void planTrajectory(const TrajectoryPoint& startState, const Point3D& targetPos,
                        double durationSec, double apexHeight) noexcept;

    SwingConfig config_{};
    Point3D liftoffPosition_{};
    Point3D targetPosition_{};
    double swingDurationSec_{0.5};
    double apexTimeSec_{0.25};
    double apexZ_{0.0};

    QuinticHermiteSpline splineX_{};
    QuinticHermiteSpline splineY_{};
    QuinticHermiteSpline splineZAscent_{};
    QuinticHermiteSpline splineZDescent_{};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_LOCOMOTION_SWING_TRAJECTORY_GENERATOR_HPP_
