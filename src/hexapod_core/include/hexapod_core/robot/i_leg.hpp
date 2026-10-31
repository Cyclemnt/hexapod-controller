#ifndef HEXAPOD_CORE_ROBOT_I_LEG_HPP_
#define HEXAPOD_CORE_ROBOT_I_LEG_HPP_

#include <memory>

#include "hexapod_core/actuator/i_joint.hpp"
#include "hexapod_core/kinematics/i_leg_kinematics.hpp"
#include "hexapod_core/kinematics/joint_types.hpp"
#include "hexapod_core/math/point_3d.hpp"
#include "hexapod_core/robot/leg_mounting.hpp"

namespace hexapod::core {

class ILeg {
public:
    virtual ~ILeg() = default;

    [[nodiscard]] virtual uint8_t getId() const noexcept = 0;
    [[nodiscard]] virtual const LegMounting& getMounting() const noexcept = 0;
    [[nodiscard]] virtual const ILegKinematics& getKinematics() const noexcept = 0;

    // Command target foot position in local leg frame
    virtual bool setTargetInLegFrame(const Point3D& targetInLegFrame) = 0;

    // Command target foot position in body/chassis frame
    virtual bool setTargetInBodyFrame(const Point3D& targetInBodyFrame) = 0;

    // Measured foot position from joint sensor feedback (forward kinematics)
    [[nodiscard]] virtual Point3D getCurrentFootInLegFrame() const = 0;
    [[nodiscard]] virtual Point3D getCurrentFootInBodyFrame() const = 0;

    // Target foot position
    [[nodiscard]] virtual Point3D getTargetFootInLegFrame() const noexcept = 0;
    [[nodiscard]] virtual Point3D getTargetFootInBodyFrame() const noexcept = 0;

    // Joint accessors
    [[nodiscard]] virtual std::shared_ptr<IJoint> getCoxaJoint() const noexcept = 0;
    [[nodiscard]] virtual std::shared_ptr<IJoint> getFemurJoint() const noexcept = 0;
    [[nodiscard]] virtual std::shared_ptr<IJoint> getTibiaJoint() const noexcept = 0;

    [[nodiscard]] virtual LegJointAngles getTargetJointAngles() const noexcept = 0;
    [[nodiscard]] virtual LegJointAngles getPresentJointAngles() const noexcept = 0;
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_ROBOT_I_LEG_HPP_

