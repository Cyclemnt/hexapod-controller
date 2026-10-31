#ifndef HEXAPOD_CORE_MATH_POINT_3D_HPP_
#define HEXAPOD_CORE_MATH_POINT_3D_HPP_

#include <cmath>

namespace hexapod::core {

struct Point3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};

    constexpr Point3D() noexcept = default;
    constexpr Point3D(double xVal, double yVal, double zVal) noexcept : x(xVal), y(yVal), z(zVal) {}

    [[nodiscard]] double norm() const noexcept {
        return std::sqrt(x * x + y * y + z * z);
    }

    [[nodiscard]] constexpr double normSquared() const noexcept {
        return x * x + y * y + z * z;
    }

    [[nodiscard]] constexpr double dot(const Point3D& other) const noexcept {
        return x * other.x + y * other.y + z * other.z;
    }

    constexpr Point3D operator+(const Point3D& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }

    constexpr Point3D operator-(const Point3D& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }

    constexpr Point3D operator*(double scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    constexpr Point3D operator/(double scalar) const noexcept {
        return {x / scalar, y / scalar, z / scalar};
    }

    constexpr bool operator==(const Point3D& other) const noexcept = default;
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_MATH_POINT_3D_HPP_

