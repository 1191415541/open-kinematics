#pragma once

/// The free functions of the `mb_dual` module.
///
/// Split out of the former `mb_base/functions.hpp` aggregate at subtask 03.
/// The declarations are grouped by the module that defines them.

#include "mb_config/prelude.hpp"
#include "mb_dual/dual.hpp"
#include "mb_dual/dual_geometry.hpp"

namespace axle_kernel {

DirectionalScalar operator+( const DirectionalScalar& a, const DirectionalScalar& b );

DirectionalScalar operator-( const DirectionalScalar& a, const DirectionalScalar& b );

DirectionalScalar operator-(const DirectionalScalar& a);

DirectionalScalar operator*( const DirectionalScalar& a, const DirectionalScalar& b );

DirectionalScalar operator/( const DirectionalScalar& a, const DirectionalScalar& b );

DVec3 operator+(const DVec3& a, const DVec3& b);

DVec3 operator-(const DVec3& a, const DVec3& b);

DVec3 operator-(const DVec3& a);

DVec3 operator*(const DVec3& a, const DirectionalScalar& b);

DVec3 operator*(const DVec3& a, double b);

DVec3 operator/(const DVec3& a, const DirectionalScalar& b);

DirectionalScalar d_dot(const DVec3& a, const DVec3& b);

DVec3 d_cross(const DVec3& a, const DVec3& b);

DirectionalScalar d_norm(const DVec3& a);

DVec3 d_normalized(const DVec3& a, bool& smooth);

DMat3 d_identity3();

Mat3 so3_left_jacobian(const Vec3& phi);

DMat3 d_transpose(const DMat3& m);

DMat3 operator+(const DMat3& a, const DMat3& b);

DMat3 operator-(const DMat3& a, const DMat3& b);

DMat3 operator*(const DMat3& a, const DirectionalScalar& b);

DMat3 operator*(const DMat3& a, double b);

DMat3 d_skew(const DVec3& v);

DMat3 d_outer(const DVec3& a, const DVec3& b);

DVec3 d_row_times(const DVec3& e, const DMat3& m);

DMat3 operator*(const DMat3& a, const DMat3& b);

DVec3 operator*(const DMat3& a, const DVec3& v);

DQuat d_qmul(const DQuat& a, const DQuat& b);

DQuat d_qconj(const DQuat& q);

DQuat d_qnormalize(const DQuat& q);

DMat3 d_qmat(const DQuat& q_);

DVec3 d_cardan_xyz_from_rotation(const DMat3& rotation, bool& smooth);

DVec3 d_cardan_xyz_rate( const DVec3& angles, const DVec3& relative_omega, bool& smooth );

DQuat d_body_quaternion(const Quat& q, const Vec3& dtheta);

DVec3 d_qlog(DQuat q, bool& smooth);

DMat3 d_log_left_jacobian_inverse(const DVec3& phi, bool& smooth);

DVec3 d_rotate( const Quat& q, const Vec3& local, const Vec3& dtheta );

DirectionalScalar interpolated_curve_directional( const std::vector<double>& x, const std::vector<double>& y, const DirectionalScalar& value, bool& smooth, bool akima = false );

} // namespace axle_kernel
