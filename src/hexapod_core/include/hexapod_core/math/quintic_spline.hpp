#ifndef HEXAPOD_CORE_MATH_QUINTIC_SPLINE_HPP_
#define HEXAPOD_CORE_MATH_QUINTIC_SPLINE_HPP_

#include <algorithm>

namespace hexapod::core {

// Single-axis C^2 quintic Hermite spline segment
class QuinticHermiteSpline {
public:
    constexpr QuinticHermiteSpline() noexcept = default;

    constexpr QuinticHermiteSpline(double p0, double v0, double a0,
                                   double p1, double v1, double a1,
                                   double duration) noexcept {
        setBoundary(p0, v0, a0, p1, v1, a1, duration);
    }

    constexpr void setBoundary(double p0, double v0, double a0,
                               double p1, double v1, double a1,
                               double duration) noexcept {
        duration_ = (duration > 1e-6) ? duration : 1e-6;
        p0_ = p0;
        v0_ = v0 * duration_;
        a0_ = a0 * duration_ * duration_;
        p1_ = p1;
        v1_ = v1 * duration_;
        a1_ = a1 * duration_ * duration_;
    }

    [[nodiscard]] constexpr double evaluatePosition(double t) const noexcept {
        const double u = std::clamp(t / duration_, 0.0, 1.0);
        return p0_ * h0(u) + v0_ * h1(u) + a0_ * h2(u) + a1_ * h3(u) + v1_ * h4(u) + p1_ * h5(u);
    }

    [[nodiscard]] constexpr double evaluateVelocity(double t) const noexcept {
        const double u = std::clamp(t / duration_, 0.0, 1.0);
        const double du = p0_ * h0d(u) + v0_ * h1d(u) + a0_ * h2d(u) + a1_ * h3d(u) + v1_ * h4d(u) + p1_ * h5d(u);
        return du / duration_;
    }

    [[nodiscard]] constexpr double evaluateAcceleration(double t) const noexcept {
        const double u = std::clamp(t / duration_, 0.0, 1.0);
        const double ddu = p0_ * h0dd(u) + v0_ * h1dd(u) + a0_ * h2dd(u) + a1_ * h3dd(u) + v1_ * h4dd(u) + p1_ * h5dd(u);
        return ddu / (duration_ * duration_);
    }

    [[nodiscard]] constexpr double duration() const noexcept { return duration_; }
    [[nodiscard]] constexpr double startPosition() const noexcept { return p0_; }
    [[nodiscard]] constexpr double targetPosition() const noexcept { return p1_; }

private:
    static constexpr double h0(double u) noexcept { return 1.0 - 10.0 * u * u * u + 15.0 * u * u * u * u - 6.0 * u * u * u * u * u; }
    static constexpr double h1(double u) noexcept { return u - 6.0 * u * u * u + 8.0 * u * u * u * u - 3.0 * u * u * u * u * u; }
    static constexpr double h2(double u) noexcept { return 0.5 * u * u - 1.5 * u * u * u + 1.5 * u * u * u * u - 0.5 * u * u * u * u * u; }
    static constexpr double h3(double u) noexcept { return 0.5 * u * u * u - u * u * u * u + 0.5 * u * u * u * u * u; }
    static constexpr double h4(double u) noexcept { return -4.0 * u * u * u + 7.0 * u * u * u * u - 3.0 * u * u * u * u * u; }
    static constexpr double h5(double u) noexcept { return 10.0 * u * u * u - 15.0 * u * u * u * u + 6.0 * u * u * u * u * u; }

    static constexpr double h0d(double u) noexcept { return -30.0 * u * u + 60.0 * u * u * u - 30.0 * u * u * u * u; }
    static constexpr double h1d(double u) noexcept { return 1.0 - 18.0 * u * u + 32.0 * u * u * u - 15.0 * u * u * u * u; }
    static constexpr double h2d(double u) noexcept { return u - 4.5 * u * u + 6.0 * u * u * u - 2.5 * u * u * u * u; }
    static constexpr double h3d(double u) noexcept { return 1.5 * u * u - 4.0 * u * u * u + 2.5 * u * u * u * u; }
    static constexpr double h4d(double u) noexcept { return -12.0 * u * u + 28.0 * u * u * u - 15.0 * u * u * u * u; }
    static constexpr double h5d(double u) noexcept { return 30.0 * u * u - 60.0 * u * u * u + 30.0 * u * u * u * u; }

    static constexpr double h0dd(double u) noexcept { return -60.0 * u + 180.0 * u * u - 120.0 * u * u * u; }
    static constexpr double h1dd(double u) noexcept { return -36.0 * u + 96.0 * u * u - 60.0 * u * u * u; }
    static constexpr double h2dd(double u) noexcept { return 1.0 - 9.0 * u + 18.0 * u * u - 10.0 * u * u * u; }
    static constexpr double h3dd(double u) noexcept { return 3.0 * u - 12.0 * u * u + 10.0 * u * u * u; }
    static constexpr double h4dd(double u) noexcept { return -24.0 * u + 84.0 * u * u - 60.0 * u * u * u; }
    static constexpr double h5dd(double u) noexcept { return 60.0 * u - 180.0 * u * u + 120.0 * u * u * u; }

    double p0_{0.0};
    double v0_{0.0};
    double a0_{0.0};
    double p1_{0.0};
    double v1_{0.0};
    double a1_{0.0};
    double duration_{1.0};
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_MATH_QUINTIC_SPLINE_HPP_
