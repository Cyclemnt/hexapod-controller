#ifndef HEXAPOD_CORE_ROBOT_LEG_HPP_
#define HEXAPOD_CORE_ROBOT_LEG_HPP_

#include <memory>
#include <utility>

#include "hexapod_core/actuator/i_joint.hpp"
#include "hexapod_core/kinematics/i_leg_kinematics.hpp"
#include "hexapod_core/kinematics/joint_types.hpp"
#include "hexapod_core/math/point_3d.hpp"
#include "hexapod_core/math/transform_3d.hpp"
#include "hexapod_core/robot/i_leg.hpp"
#include "hexapod_core/robot/leg_mounting.hpp"

namespace hexapod::core {

class Leg final : public ILeg {
public:
    Leg(uint8_t id, const LegMounting& mounting, std::unique_ptr<ILegKinematics> kinematics,
        std::shared_ptr<IJoint> coxaJoint, std::shared_ptr<IJoint> femurJoint, std::shared_ptr<IJoint> tibiaJoint) noexcept;

    [[nodiscard]] uint8_t getId() const noexcept override { return id_; }
    [[nodiscard]] const LegMounting& getMounting() const noexcept override { return mounting_; }
    [[nodiscard]] const ILegKinematics& getKinematics() const noexcept override { return *kinematics_; }

    bool setTargetInLegFrame(const Point3D& targetInLegFrame) override;
    bool setTargetInBodyFrame(const Point3D& targetInBodyFrame) override;

    [[nodiscard]] Point3D getCurrentFootInLegFrame() const override;
    [[nodiscard]] Point3D getCurrentFootInBodyFrame() const override;

    [[nodiscard]] Point3D getTargetFootInLegFrame() const noexcept override { return targetFootInLegFrame_; }
    [[nodiscard]] Point3D getTargetFootInBodyFrame() const noexcept override { return targetFootInBodyFrame_; }

    [[nodiscard]] std::shared_ptr<IJoint> getCoxaJoint() const noexcept override { return coxaJoint_; }
    [[nodiscard]] std::shared_ptr<IJoint> getFemurJoint() const noexcept override { return femurJoint_; }
    [[nodiscard]] std::shared_ptr<IJoint> getTibiaJoint() const noexcept override { return tibiaJoint_; }

    [[nodiscard]] LegJointAngles getTargetJointAngles() const noexcept override;
    [[nodiscard]] LegJointAngles getPresentJointAngles() const noexcept override;

private:
    uint8_t id_{0};
    LegMounting mounting_{};
    Transform3D mountingTransform_{};
    std::unique_ptr<ILegKinematics> kinematics_{};

    std::shared_ptr<IJoint> coxaJoint_{};
    std::shared_ptr<IJoint> femurJoint_{};
    std::shared_ptr<IJoint> tibiaJoint_{};

    Point3D targetFootInLegFrame_{};
    Point3D targetFootInBodyFrame_{};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_ROBOT_LEG_HPP_

