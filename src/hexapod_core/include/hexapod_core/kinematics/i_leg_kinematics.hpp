#ifndef HEXAPOD_CORE_KINEMATICS_I_LEG_KINEMATICS_HPP_
#define HEXAPOD_CORE_KINEMATICS_I_LEG_KINEMATICS_HPP_

#include <optional>

#include "hexapod_core/kinematics/joint_types.hpp"
#include "hexapod_core/math/point_3d.hpp"

namespace hexapod::core {

class ILegKinematics {
public:
    virtual ~ILegKinematics() = default;

    [[nodiscard]] virtual Point3D forwardKinematics(const LegJointAngles& jointAngles) const noexcept = 0;
    [[nodiscard]] virtual std::optional<LegJointAngles> inverseKinematics(const Point3D& targetInLegFrame) const noexcept = 0;
    [[nodiscard]] virtual bool isReachable(const Point3D& targetInLegFrame) const noexcept = 0;
    [[nodiscard]] virtual const LegDimensions& getDimensions() const noexcept = 0;
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_KINEMATICS_I_LEG_KINEMATICS_HPP_

