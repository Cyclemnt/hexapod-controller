#include "hexapod_controller/joint_mapping.hpp"

namespace hexapod::controller {

namespace {

const std::array<std::string, 18> kJointNames = {
    "leg_0_coxa_joint", "leg_0_femur_joint", "leg_0_tibia_joint",
    "leg_1_coxa_joint", "leg_1_femur_joint", "leg_1_tibia_joint",
    "leg_2_coxa_joint", "leg_2_femur_joint", "leg_2_tibia_joint",
    "leg_3_coxa_joint", "leg_3_femur_joint", "leg_3_tibia_joint",
    "leg_4_coxa_joint", "leg_4_femur_joint", "leg_4_tibia_joint",
    "leg_5_coxa_joint", "leg_5_femur_joint", "leg_5_tibia_joint"
};

}  // namespace

UrdfJointMapper::UrdfJointMapper() noexcept {
    for (size_t i = 0; i < 6; ++i) {
        legConfigs_[i] = LegJointConfig{
            .coxa = JointAxisConfig{.multiplier = -1.0, .offsetRad = 0.0},
            .femur = JointAxisConfig{.multiplier = -1.0, .offsetRad = 0.0},
            .tibia = JointAxisConfig{.multiplier = 1.0, .offsetRad = 0.0}
        };
    }
}

UrdfJointMapper::UrdfJointMapper(const std::array<LegJointConfig, 6>& legConfigs) noexcept : legConfigs_(legConfigs) {}

std::vector<double> UrdfJointMapper::mapToUrdf(const std::array<core::LegJointAngles, 6>& kinematicAngles) const noexcept {
    std::vector<double> urdfAngles(18, 0.0);
    for (size_t leg = 0; leg < 6; ++leg) {
        const auto& angles = kinematicAngles[leg];
        const auto& config = legConfigs_[leg];

        urdfAngles[leg * 3 + 0] = config.coxa.multiplier * angles.coxa + config.coxa.offsetRad;
        urdfAngles[leg * 3 + 1] = config.femur.multiplier * angles.femur + config.femur.offsetRad;
        urdfAngles[leg * 3 + 2] = config.tibia.multiplier * angles.tibia + config.tibia.offsetRad;
    }
    return urdfAngles;
}

const std::array<std::string, 18>& UrdfJointMapper::getJointNames() noexcept {
    return kJointNames;
}

void UrdfJointMapper::setLegConfig(size_t legIndex, const LegJointConfig& config) noexcept {
    if (legIndex < 6) {
        legConfigs_[legIndex] = config;
    }
}

const LegJointConfig& UrdfJointMapper::getLegConfig(size_t legIndex) const noexcept {
    return legConfigs_[legIndex < 6 ? legIndex : 0];
}

}  // namespace hexapod::controller
