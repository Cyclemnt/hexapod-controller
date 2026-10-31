#ifndef HEXAPOD_CORE_ACTUATOR_I_JOINT_HPP_
#define HEXAPOD_CORE_ACTUATOR_I_JOINT_HPP_

#include <cstdint>

#include "hexapod_core/kinematics/joint_types.hpp"

namespace hexapod::core {

class IJoint {
public:
    virtual ~IJoint() = default;

    [[nodiscard]] virtual uint8_t getId() const noexcept = 0;

    virtual void setTargetAngle(double angleRad) = 0;
    [[nodiscard]] virtual double getTargetAngle() const noexcept = 0;

    virtual void setPresentState(double angleRad, double velocityRadPerSec, double effortLoad) = 0;
    [[nodiscard]] virtual double getPresentAngle() const noexcept = 0;
    [[nodiscard]] virtual double getPresentVelocity() const noexcept = 0;
    [[nodiscard]] virtual double getPresentEffort() const noexcept = 0;

    [[nodiscard]] virtual uint16_t radiansToTicks(double angleRad) const noexcept = 0;
    [[nodiscard]] virtual double ticksToRadians(uint16_t ticks) const noexcept = 0;

    [[nodiscard]] virtual const JointLimits& getLimits() const noexcept = 0;
    [[nodiscard]] virtual const JointCalibration& getCalibration() const noexcept = 0;
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_ACTUATOR_I_JOINT_HPP_

