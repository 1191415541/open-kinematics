#pragma once

/// The free functions of the `mb_numeric` module.
///
/// Split out of the former `mb_base/functions.hpp` aggregate at subtask 03.
/// The declarations are grouped by the module that defines them.

#include "mb_config/prelude.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_numeric/monotone_cubic.hpp"
#include "mb_config/constants.hpp"

namespace axle_kernel {

double dot(const Vec3& a, const Vec3& b);

Vec3 cross(const Vec3& a, const Vec3& b);

double norm(const Vec3& a);

Vec3 normalized(const Vec3& a);

double determinant(const Mat3& m);

bool inverse3(const Mat3& m, Mat3& out);

bool finite_symmetric(const Mat3& m);

bool symmetric_positive_definite(const Mat3& m);

Mat3 transpose(const Mat3& m);

Mat3 operator*(const Mat3& a, const Mat3& b);

Mat3 operator*(const Mat3& m, double s);

Mat3 operator+(const Mat3& a, const Mat3& b);

Mat3 operator-(const Mat3& a, const Mat3& b);

Mat3 identity3();

Vec3 operator*(const Mat3& a, const Vec3& v);

Mat3 skew(const Vec3& v);

Mat3 outer(const Vec3& a, const Vec3& b);

Vec3 row_times(const Vec3& e, const Mat3& m);

Quat qmul(const Quat& a, const Quat& b);

Quat qconj(const Quat& q);

double qnorm(const Quat& q);

double qdot(const Quat& a, const Quat& b);

Quat qnegated(const Quat& q);

Quat qnormalize(Quat q);

Quat normalized_continuous(const Quat& raw, const Quat& reference);

bool unit_quaternion(const Quat& q, double tolerance = 1e-8);

Mat3 qmat(const Quat& q_);

Quat qexp(const Vec3& theta);

Vec3 qlog(Quat q);

Vec3 rotate(const Quat& q, const Vec3& v);

double interpolate_curve( const std::vector<double>& x, const std::vector<double>& y, double value );

std::vector<double> akima_curve_slopes( const std::vector<double>& x, const std::vector<double>& y );

std::pair<double, double> interpolate_akima_curve( const std::vector<double>& x, const std::vector<double>& y, double value );

double integrate_akima_segment( double y0, double y1, double slope0, double slope1, double h, double lower_u, double upper_u );

double integrate_akima_curve_from_zero( const std::vector<double>& x, const std::vector<double>& y, double value );

double integrate_curve_interval( const std::vector<double>& x, const std::vector<double>& y, double lower, double upper );

double integrate_curve_from_zero( const std::vector<double>& x, const std::vector<double>& y, double value );

Vec3 cardan_xyz_from_rotation(const Mat3& rotation);

Vec3 cardan_xyz_rate(const Vec3& angles, const Vec3& relative_omega);

CurveValue monotone_cubic( const std::vector<double>& curve, double x );

Mat3 log_left_jacobian_inverse(const Vec3& phi);

} // namespace axle_kernel
