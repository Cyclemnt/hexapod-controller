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

    [[nodiscard]] constexpr Point3D cross(const Point3D& other) const noexcept {
        return {
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        };
    }

    constexpr Point3D operator+(const Point3D& other) const noexcept {
        return {x + other.x, y + other.y, z + other.z};
    }

    constexpr Point3D operator-(const Point3D& other) const noexcept {
        return {x - other.x, y - other.y, z - other.z};
    }

    constexpr Point3D operator-() const noexcept {
        return {-x, -y, -z};
    }

    constexpr Point3D operator*(double scalar) const noexcept {
        return {x * scalar, y * scalar, z * scalar};
    }

    friend constexpr Point3D operator*(double scalar, const Point3D& point) noexcept {
        return point * scalar;
    }

    constexpr Point3D operator/(double scalar) const noexcept {
        return {x / scalar, y / scalar, z / scalar};
    }

    constexpr Point3D& operator+=(const Point3D& other) noexcept {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    constexpr Point3D& operator-=(const Point3D& other) noexcept {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    constexpr Point3D& operator*=(double scalar) noexcept {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    constexpr bool operator==(const Point3D& other) const noexcept = default;
};

}  // namespace hexapod::core

#endif  // HEXAPOD_CORE_MATH_POINT_3D_HPP_

