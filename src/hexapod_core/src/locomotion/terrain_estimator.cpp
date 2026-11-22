#include "hexapod_core/locomotion/terrain_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace hexapod::core {

TerrainEstimator::TerrainEstimator(const TerrainEstimatorConfig& config) noexcept
    : config_(config) {}

TerrainEstimate TerrainEstimator::estimate(std::span<const Point3D> stanceFeetBodyFrame,
                                           double imuRoll, double imuPitch, double imuYaw) const noexcept {
    (void)imuYaw;  // Yaw does not tilt the gravity vector relative to planar slopes
    TerrainEstimate result{};
    result.c = -config_.nominalHeight;

    const size_t n = stanceFeetBodyFrame.size();
    if (n < 3) {
        result.valid = false;
        result.comShift = Point3D{0.0, 0.0, 0.0};
        result.attitudeCompensation = BodyPoseCommand{};
        return result;
    }

    const double nDouble = static_cast<double>(n);
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    double sxx = 0.0;
    double syy = 0.0;
    double sxy = 0.0;
    double sxz = 0.0;
    double syz = 0.0;

    for (const auto& pt : stanceFeetBodyFrame) {
        sx += pt.x;
        sy += pt.y;
        sz += pt.z;
        sxx += pt.x * pt.x;
        syy += pt.y * pt.y;
        sxy += pt.x * pt.y;
        sxz += pt.x * pt.z;
        syz += pt.y * pt.z;
    }

    // Solve 3x3 linear system M * [a, b, c]^T = V analytically via Cramer's rule
    // M = [Sxx, Sxy, Sx; Sxy, Syy, Sy; Sx, Sy, N]
    const double m00 = syy * nDouble - sy * sy;
    const double m01 = sxy * nDouble - sx * sy;
    const double m02 = sxy * sy - syy * sx;

    const double det = sxx * m00 - sxy * m01 + sx * m02;

    if (std::abs(det) < 1e-9) {
        // Degenerate geometry (e.g. collinear foot contacts)
        result.valid = false;
        result.c = sz / nDouble;
        return result;
    }

    const double detA = sxz * m00 - sxy * (syz * nDouble - sz * sy) + sx * (syz * sy - syy * sz);
    const double detB = sxx * (syz * nDouble - sz * sy) - sxz * m01 + sx * (sxy * sz - syz * sx);
    const double detC = sxx * (syy * sz - syz * sy) - sxy * (sxy * sz - sx * syz) + sxz * (sxy * sy - syy * sx);

    result.a = detA / det;
    result.b = detB / det;
    result.c = detC / det;
    result.valid = true;

    // Terrain slope angles (REP-103: dz/dx > 0 is pitch down = negative pitch, dz/dy > 0 is roll up = positive roll)
    result.terrainPitch = -std::atan(result.a);
    result.terrainRoll = +std::atan(result.b);

    // CoM horizontal shift on slopes calculated from IMU orientation relative to gravity:
    // When nose is up (imuPitch < 0), gravity pulls towards -X => comShift.x = -h * tan(imuPitch)
    // When left side is up (imuRoll > 0), gravity pulls towards -Y => comShift.y = +h * tan(imuRoll)
    const double h = config_.nominalHeight;
    const double maxShift = config_.maxComShiftMeters;
    result.comShift.x = std::clamp(-h * std::tan(imuPitch), -maxShift, maxShift);
    result.comShift.y = std::clamp(+h * std::tan(imuRoll), -maxShift, maxShift);
    result.comShift.z = 0.0;

    // Fused attitude compensation
    const double weight = std::clamp(config_.imuWeight, 0.0, 1.0);
    const double fusedPitch = (1.0 - weight) * result.terrainPitch + weight * imuPitch;
    const double fusedRoll = (1.0 - weight) * result.terrainRoll + weight * imuRoll;

    const double maxTilt = config_.maxTiltAngleRad;
    result.attitudeCompensation.roll = std::clamp(-fusedRoll, -maxTilt, maxTilt);
    result.attitudeCompensation.pitch = std::clamp(-fusedPitch, -maxTilt, maxTilt);
    result.attitudeCompensation.yaw = 0.0;
    result.attitudeCompensation.x = result.comShift.x;
    result.attitudeCompensation.y = result.comShift.y;
    result.attitudeCompensation.z = 0.0;

    return result;
}

}  // namespace hexapod::core
