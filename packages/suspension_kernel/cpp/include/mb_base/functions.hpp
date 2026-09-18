#pragma once

/// The free functions of the `mb_base` module.
///
/// K6 moved these declarations here out of the transitional aggregate
/// `kernel_internal.hpp`, which was deleted once every translation unit
/// included the header of its own module (`MODULES.md` section 4).  The
/// declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_base/prelude.hpp"
#include "mb_base/vector.hpp"
#include "mb_base/monotone_cubic.hpp"
#include "mb_base/constants.hpp"
#include "mb_base/diagnostics.hpp"
#include "mb_base/env.hpp"
#include "mb_base/util.hpp"
#include "mb_base/dual.hpp"
#include "mb_base/dual_geometry.hpp"
#include "mb_base/prelude.hpp"


namespace axle_kernel {
DirectionalScalar operator+( const DirectionalScalar& a, const DirectionalScalar& b );

DirectionalScalar operator-( const DirectionalScalar& a, const DirectionalScalar& b );

DirectionalScalar operator-(const DirectionalScalar& a);

DirectionalScalar operator*( const DirectionalScalar& a, const DirectionalScalar& b );

DirectionalScalar operator/( const DirectionalScalar& a, const DirectionalScalar& b );

bool profiling_enabled();

bool runtime_jacobian_validation_enabled();

bool static_debug_enabled();

int linearization_reuse_limit(int default_limit);

int newton_jacobian_refresh_period();

int linear_solver_threads();

bool blocked_lu_enabled();

bool blocked_lu_parallel_enabled();

bool lu_equilibration_enabled();

bool sparse_gmres_enabled();

int sparse_gmres_restart();

int sparse_gmres_max_iterations();

bool sparse_lu_enabled();

bool mkl_dense_enabled();

bool mkl_pardiso_enabled();

bool mkl_pardiso_full_enabled();

bool mkl_pardiso_debug_enabled();

bool mkl_pardiso_matching_enabled();

int mkl_pardiso_pivot_perturbation();

int mkl_pardiso_ordering();

int mkl_pardiso_threads();

int blocked_lu_block_size();

bool exact_fiala_relaxation_enabled();

bool light_fiala_relaxation_enabled();

bool acceleration_predictor_enabled(bool default_enabled = false);

bool acceleration_schur_probe_enabled();

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
