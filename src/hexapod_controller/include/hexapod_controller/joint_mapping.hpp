#ifndef HEXAPOD_CONTROLLER_JOINT_MAPPING_HPP_
#define HEXAPOD_CONTROLLER_JOINT_MAPPING_HPP_

#include <array>
#include <string>
#include <vector>

#include "hexapod_core/kinematics/joint_types.hpp"

namespace hexapod::controller {

struct JointAxisConfig {
    double multiplier{1.0};
    double offsetRad{0.0};
};

struct LegJointConfig {
    JointAxisConfig coxa{-1.0, 0.0};
    JointAxisConfig femur{-1.0, 0.0};
    JointAxisConfig tibia{1.0, 0.0};
};

class UrdfJointMapper {
public:
    UrdfJointMapper() noexcept;
    explicit UrdfJointMapper(const std::array<LegJointConfig, 6>& legConfigs) noexcept;

    [[nodiscard]] std::vector<double> mapToUrdf(const std::array<core::LegJointAngles, 6>& kinematicAngles) const noexcept;
    [[nodiscard]] static const std::array<std::string, 18>& getJointNames() noexcept;

    void setLegConfig(size_t legIndex, const LegJointConfig& config) noexcept;
    [[nodiscard]] const LegJointConfig& getLegConfig(size_t legIndex) const noexcept;

private:
    std::array<LegJointConfig, 6> legConfigs_{};
};

}  // namespace hexapod::controller

#endif  // HEXAPOD_CONTROLLER_JOINT_MAPPING_HPP_
