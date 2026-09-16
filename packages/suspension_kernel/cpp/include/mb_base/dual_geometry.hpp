#pragma once

/// The dual-number geometry (MB_BASE).
///
/// `DVec3`/`DMat3`/`DQuat` mirror `Vec3`/`Mat3`/`Quat` with every component a
/// `DirectionalScalar`.  The direction-carrying state and scratch types are
/// here too: they are what a directional pass is handed, and they depend on the
/// geometry rather than on the model.

#include "mb_base/vector.hpp"
#include "mb_base/dual.hpp"
#include <vector>

namespace axle_kernel {

struct DVec3 {
    DirectionalScalar x{}, y{}, z{};

    DVec3() = default;
    DVec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    DVec3(
        const DirectionalScalar& x_, const DirectionalScalar& y_,
        const DirectionalScalar& z_
    ) : x(x_), y(y_), z(z_) {}

    Vec3 value() const;
};

struct DMat3 {
    DirectionalScalar a[3][3]{};
};

struct DQuat {
    DirectionalScalar w{1.0}, x{}, y{}, z{};

    DQuat() = default;
    DQuat(
        const DirectionalScalar& w_, const DirectionalScalar& x_,
        const DirectionalScalar& y_, const DirectionalScalar& z_
    ) : w(w_), x(x_), y(y_), z(z_) {}
};

struct DirectionalState {
    std::vector<Vec3> dr, dtheta, dv, domega;
    std::vector<double> dsx, dsy;
};

struct DirectionalRoadProfile {
    DirectionalScalar height{};
    DirectionalScalar slope{};
};

struct DirectionalForceScratch {
    std::vector<Vec3> force;
    std::vector<Vec3> torque;
};

DVec3 operator+(const DVec3& a, const DVec3& b);
DVec3 operator-(const DVec3& a, const DVec3& b);
DVec3 operator-(const DVec3& a);
DVec3 operator*(const DVec3& a, const DirectionalScalar& b);
DVec3 operator*(const DVec3& a, double b);
DVec3 operator/(const DVec3& a, const DirectionalScalar& b);
DMat3 operator+(const DMat3& a, const DMat3& b);
DMat3 operator-(const DMat3& a, const DMat3& b);
DMat3 operator*(const DMat3& a, const DirectionalScalar& b);
DMat3 operator*(const DMat3& a, double b);
DMat3 operator*(const DMat3& a, const DMat3& b);
DVec3 operator*(const DMat3& a, const DVec3& v);

} // namespace axle_kernel
