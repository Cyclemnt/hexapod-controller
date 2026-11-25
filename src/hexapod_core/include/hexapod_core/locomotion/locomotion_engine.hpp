#ifndef HEXAPOD_CORE_LOCOMOTION_LOCOMOTION_ENGINE_HPP_
#define HEXAPOD_CORE_LOCOMOTION_LOCOMOTION_ENGINE_HPP_

#include <array>
#include <memory>
#include <vector>

#include "hexapod_core/locomotion/body_pose_command.hpp"
#include "hexapod_core/locomotion/gait_scheduler.hpp"
#include "hexapod_core/locomotion/leg_locomotion_controller.hpp"
#include "hexapod_core/locomotion/terrain_estimator.hpp"
#include "hexapod_core/math/point_3d.hpp"
#include "hexapod_core/robot/i_leg.hpp"

namespace hexapod::core {

enum class LocomotionState : uint8_t {
    kSitting = 0,
    kStanding = 1,
    kLocomotion = 2
};

struct LocomotionEngineConfig {
    double sittingHeight{-0.060};             // -60 mm in body frame
    double targetStandingHeight{-0.120};      // -120 mm in body frame
    double stanceRadius{0.135};               // Nominal foot reach in leg frame (meters)
    double standingTransitionTimeSec{2.0};    // Time to stand up from sitting
    double idleVelocityThreshold{0.005};      // Velocity norm threshold below which robot is idle
    double idleSettlingTimeSec{0.25};         // Settling duration with zero velocity before returning to standing
    GaitConfig gaitConfig{};
    LegLocomotionConfig legConfig{};
    TerrainEstimatorConfig terrainConfig{};
};

// Global locomotion orchestrator managing the 6 legs, gait scheduler,
// terrain estimator, state machine (Sitting -> Standing -> Locomotion),
// posture superposition, and automatic idle return.
class LocomotionEngine {
public:
    explicit LocomotionEngine(std::array<std::shared_ptr<ILeg>, 6> legs,
                              const LocomotionEngineConfig& config = LocomotionEngineConfig{}) noexcept;

    // Factory method using default HexapodGeometry
    [[nodiscard]] static std::unique_ptr<LocomotionEngine> createDefault(
        const LocomotionEngineConfig& config = LocomotionEngineConfig{});

    // Control cycle update step (dt in seconds)
    void update(double dt) noexcept;

    // State machine commands
    void requestStanding() noexcept;
    void requestSitting() noexcept;

    // Control inputs
    void setVelocityCommand(const Point3D& linearVel, const Point3D& angularVel) noexcept;
    void setBodyPoseCommand(const BodyPoseCommand& pose) noexcept;
    void setImuOrientation(double rollRad, double pitchRad, double yawRad) noexcept;

    // Queries
    [[nodiscard]] LocomotionState getState() const noexcept { return state_; }
    [[nodiscard]] bool isSitting() const noexcept { return state_ == LocomotionState::kSitting; }
    [[nodiscard]] bool isStanding() const noexcept { return state_ == LocomotionState::kStanding; }
    [[nodiscard]] bool isLocomoting() const noexcept { return state_ == LocomotionState::kLocomotion; }

    [[nodiscard]] const LegLocomotionController& getLegController(size_t legIndex) const noexcept;
    [[nodiscard]] LegLocomotionController& getLegController(size_t legIndex) noexcept;
    [[nodiscard]] std::shared_ptr<ILeg> getLeg(size_t legIndex) const noexcept { return legs_.at(legIndex); }

    [[nodiscard]] const GaitScheduler& getGaitScheduler() const noexcept { return gaitScheduler_; }
    [[nodiscard]] GaitScheduler& getGaitScheduler() noexcept { return gaitScheduler_; }

    [[nodiscard]] const TerrainEstimator& getTerrainEstimator() const noexcept { return terrainEstimator_; }
    [[nodiscard]] const TerrainEstimate& getLatestTerrainEstimate() const noexcept { return latestTerrainEstimate_; }

    [[nodiscard]] const LocomotionEngineConfig& getConfig() const noexcept { return config_; }
    void setConfig(const LocomotionEngineConfig& config) noexcept;

    [[nodiscard]] Point3D getNominalFootInBody(size_t legIndex) const noexcept;

private:
    void initNominalFeet() noexcept;
    void updateSitting(double dt) noexcept;
    void updateStanding(double dt) noexcept;
    void updateLocomotion(double dt) noexcept;

    LocomotionEngineConfig config_{};
    std::array<std::shared_ptr<ILeg>, 6> legs_{};
    std::vector<LegLocomotionController> legControllers_{};

    GaitScheduler gaitScheduler_{};
    TerrainEstimator terrainEstimator_{};
    TerrainEstimate latestTerrainEstimate_{};

    LocomotionState state_{LocomotionState::kSitting};
    double transitionProgress_{0.0};
    bool isTransitioning_{false};
    bool transitionToStanding_{true};

    Point3D cmdLinearVel_{0.0, 0.0, 0.0};
    Point3D cmdAngularVel_{0.0, 0.0, 0.0};
    BodyPoseCommand userBodyPose_{};
    double imuRoll_{0.0};
    double imuPitch_{0.0};
    double imuYaw_{0.0};

    double idleTimer_{0.0};
    BodyPoseCommand previousEffPose_{};
    std::array<Point3D, 6> nominalFeetInBody_{};
    std::array<Point3D, 6> lastGroundFeetNominal_{};
    std::array<uint8_t, 6> stopCenteringSteps_{};
    double filteredTerrainRoll_{0.0};
    double filteredTerrainPitch_{0.0};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_LOCOMOTION_LOCOMOTION_ENGINE_HPP_
