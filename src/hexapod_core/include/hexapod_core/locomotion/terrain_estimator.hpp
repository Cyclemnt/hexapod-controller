#ifndef HEXAPOD_CORE_LOCOMOTION_TERRAIN_ESTIMATOR_HPP_
#define HEXAPOD_CORE_LOCOMOTION_TERRAIN_ESTIMATOR_HPP_

#include <span>
#include <vector>

#include "hexapod_core/locomotion/body_pose_command.hpp"
#include "hexapod_core/math/point_3d.hpp"

namespace hexapod::core {

struct TerrainEstimatorConfig {
    double nominalHeight{0.120};      // Nominal standing height h in meters (positive)
    double imuWeight{0.30};           // IMU fusion weight [0.0 = pure terrain fit, 1.0 = pure IMU]
    double maxTiltAngleRad{0.35};     // Maximum compensation tilt angle (~20 deg)
    double maxComShiftMeters{0.035};  // Maximum horizontal CoM shift (35 mm)
};

struct TerrainEstimate {
    double a{0.0};                     // dz/dx
    double b{0.0};                     // dz/dy
    double c{-0.120};                  // Height offset
    double terrainPitch{0.0};          // atan(a)
    double terrainRoll{0.0};           // atan(b)
    Point3D comShift{};                // CoM compensation shift (dx, dy, 0)
    BodyPoseCommand attitudeCompensation{};  // Recommended chassis compensation pose
    bool valid{false};                 // True if >= 3 non-collinear stance feet were fitted
};

// Analytical least-squares plane estimator (z = ax + by + c) and IMU slope stabilizer
class TerrainEstimator {
public:
    explicit TerrainEstimator(const TerrainEstimatorConfig& config = TerrainEstimatorConfig{}) noexcept;

    // Estimates terrain plane from stance foot coordinates and fuses with IMU orientation
    [[nodiscard]] TerrainEstimate estimate(std::span<const Point3D> stanceFeetBodyFrame,
                                          double imuRoll = 0.0, double imuPitch = 0.0, double imuYaw = 0.0) const noexcept;

    [[nodiscard]] const TerrainEstimatorConfig& getConfig() const noexcept { return config_; }
    void setConfig(const TerrainEstimatorConfig& config) noexcept { config_ = config; }

private:
    TerrainEstimatorConfig config_{};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_LOCOMOTION_TERRAIN_ESTIMATOR_HPP_
