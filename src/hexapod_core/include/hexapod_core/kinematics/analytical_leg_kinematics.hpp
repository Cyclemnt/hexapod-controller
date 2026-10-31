#ifndef HEXAPOD_CORE_KINEMATICS_ANALYTICAL_LEG_KINEMATICS_HPP_
#define HEXAPOD_CORE_KINEMATICS_ANALYTICAL_LEG_KINEMATICS_HPP_

#include <optional>

#include "hexapod_core/kinematics/i_leg_kinematics.hpp"
#include "hexapod_core/kinematics/joint_types.hpp"
#include "hexapod_core/math/point_3d.hpp"

namespace hexapod::core {

class AnalyticalLegKinematics final : public ILegKinematics {
public:
    explicit AnalyticalLegKinematics(const LegDimensions& dimensions) noexcept;

    [[nodiscard]] Point3D forwardKinematics(const LegJointAngles& jointAngles) const noexcept override;
    [[nodiscard]] std::optional<LegJointAngles> inverseKinematics(const Point3D& targetInLegFrame) const noexcept override;
    [[nodiscard]] bool isReachable(const Point3D& targetInLegFrame) const noexcept override;
    [[nodiscard]] const LegDimensions& getDimensions() const noexcept override { return dimensions_; }

private:
    LegDimensions dimensions_;
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_KINEMATICS_ANALYTICAL_LEG_KINEMATICS_HPP_

