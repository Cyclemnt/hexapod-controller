#ifndef HEXAPOD_CORE_LOCOMOTION_CONTACT_DETECTOR_HPP_
#define HEXAPOD_CORE_LOCOMOTION_CONTACT_DETECTOR_HPP_

#include "hexapod_core/robot/i_leg.hpp"

namespace hexapod::core {

struct ContactDetectorConfig {
    double effortThreshold{1.0};                 // Moderate joint effort threshold (Nm)
    double trackingErrorThresholdRad{0.06};      // Significant tracking error threshold (~3.5 deg)
    double criticalEffortThreshold{1.8};         // Critical effort threshold indicating direct hard impact (Nm)
    double minSwingDurationSec{0.08};            // Blanking window after liftoff (seconds)
    double obstacleEffortThreshold{1.5};         // Effort threshold on coxa/femur during swing forward
};

// Proprioceptive contact and collision detector based on joint load and tracking error
class ContactDetector {
public:
    explicit ContactDetector(const ContactDetectorConfig& config = ContactDetectorConfig{}) noexcept;

    // Detects ground/bump contact during descending swing or probing
    [[nodiscard]] bool detectTouchdown(const ILeg& leg, double swingElapsedTime) const noexcept;

    // Detects obstacle impact during early / ascending swing
    [[nodiscard]] bool detectObstacle(const ILeg& leg, double swingElapsedTime, double apexTimeSec) const noexcept;

    [[nodiscard]] const ContactDetectorConfig& getConfig() const noexcept { return config_; }
    void setConfig(const ContactDetectorConfig& config) noexcept { config_ = config; }

private:
    ContactDetectorConfig config_{};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_LOCOMOTION_CONTACT_DETECTOR_HPP_
