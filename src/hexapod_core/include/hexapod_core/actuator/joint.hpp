#ifndef HEXAPOD_CORE_ACTUATOR_JOINT_HPP_
#define HEXAPOD_CORE_ACTUATOR_JOINT_HPP_

#include <cstdint>

#include "hexapod_core/actuator/i_joint.hpp"
#include "hexapod_core/kinematics/joint_types.hpp"

namespace hexapod::core {

class Joint final : public IJoint {
public:
    Joint(uint8_t id, const JointLimits& limits, const JointCalibration& calibration) noexcept;

    [[nodiscard]] uint8_t getId() const noexcept override { return id_; }

    void setTargetAngle(double angleRad) override;
    [[nodiscard]] double getTargetAngle() const noexcept override { return targetAngleRad_; }

    void setPresentState(double angleRad, double velocityRadPerSec, double effortLoad) override;
    [[nodiscard]] double getPresentAngle() const noexcept override { return presentAngleRad_; }
    [[nodiscard]] double getPresentVelocity() const noexcept override { return presentVelocityRadPerSec_; }
    [[nodiscard]] double getPresentEffort() const noexcept override { return presentEffortLoad_; }

    [[nodiscard]] uint16_t radiansToTicks(double angleRad) const noexcept override;
    [[nodiscard]] double ticksToRadians(uint16_t ticks) const noexcept override;

    [[nodiscard]] const JointLimits& getLimits() const noexcept override { return limits_; }
    [[nodiscard]] const JointCalibration& getCalibration() const noexcept override { return calibration_; }

private:
    uint8_t id_{0};
    JointLimits limits_{};
    JointCalibration calibration_{};

    double targetAngleRad_{0.0};
    double presentAngleRad_{0.0};
    double presentVelocityRadPerSec_{0.0};
    double presentEffortLoad_{0.0};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_ACTUATOR_JOINT_HPP_

