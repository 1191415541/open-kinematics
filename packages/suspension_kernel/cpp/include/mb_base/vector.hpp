#pragma once

/// The kernel's vector, matrix and quaternion types (MB_BASE).
///
/// These are dependency-free: a `Vec3` is three doubles and a `Quat` is
/// four, and the arithmetic that operates on them is declared in
/// `mb_base/vector.hpp`'s companion translation unit.  A non-suspension
/// product reuses them as they are.

#include <cmath>

namespace axle_kernel {

struct Vec3 {
    double x{0.0}, y{0.0}, z{0.0};
    Vec3() = default;
    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
};

struct Mat3 {
    double a[3][3]{};
};

struct Quat {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};
};

double norm(const Vec3& a);
bool inverse3(const Mat3& m, Mat3& out);
Mat3 operator*(const Mat3& a, const Mat3& b);
Mat3 operator*(const Mat3& m, double s);
Mat3 operator+(const Mat3& a, const Mat3& b);
Mat3 operator-(const Mat3& a, const Mat3& b);
Vec3 operator*(const Mat3& a, const Vec3& v);

} // namespace axle_kernel
