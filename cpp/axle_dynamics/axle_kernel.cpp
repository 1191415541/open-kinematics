#include "axle_kernel.hpp"

#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstddef>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

constexpr double kEps = 1e-12;
constexpr double kPi = 3.141592653589793238462643383279502884;
// 与 PAC2002 .tir 的 [INCLINATION_ANGLE_RANGE] 一致；范围外的参数
// 外推没有标定依据，且会把外倾相关曲率项放大到非物理区间。
constexpr double kPac2002CamberLimit = 0.26181;
constexpr int kStatePerBody = 19;
constexpr int kTireOutputWidth = 15;
constexpr int kConstraintOutputWidth = 6;
constexpr int kSpringOutputWidth = 7;
constexpr int kBushingOutputWidth = 12;
constexpr int kAntiRollOutputWidth = 3;
constexpr int kSteeringOutputWidth = 4;
constexpr int kDiagnosticsWidth = 16;
// Optional aggregate rows written after the per-sample diagnostics when the
// caller provides two extra rows and enables SUSPENSION_AXLE_PROFILE.
constexpr int kPerformanceWidth = 23;
constexpr int kEnergyOutputWidth = 21;
constexpr int kContactEventOutputWidth = 3;

using PerformanceClock = std::chrono::steady_clock;

struct PerformanceCounters {
    std::atomic<std::uint64_t> residual_calls{0};
    std::atomic<std::uint64_t> residual_nanoseconds{0};
    std::atomic<std::uint64_t> constraint_jacobian_calls{0};
    std::atomic<std::uint64_t> constraint_jacobian_nanoseconds{0};
    std::atomic<std::uint64_t> force_evaluations{0};
    std::atomic<std::uint64_t> force_nanoseconds{0};
    std::atomic<std::uint64_t> mass_inverse_calls{0};
    std::atomic<std::uint64_t> mass_inverse_nanoseconds{0};
    std::atomic<std::uint64_t> reaction_nanoseconds{0};
    std::atomic<std::uint64_t> linear_factorizations{0};
    std::atomic<std::uint64_t> linear_factorization_nanoseconds{0};
    std::atomic<std::uint64_t> linear_solves{0};
    std::atomic<std::uint64_t> linear_solve_nanoseconds{0};
    std::atomic<std::uint64_t> line_search_trials{0};
    std::atomic<std::uint64_t> accepted_steps{0};
    std::atomic<std::uint64_t> rejected_attempts{0};
    std::atomic<std::uint64_t> newton_iterations{0};
    std::atomic<std::uint64_t> analytic_jacobian_columns{0};
    std::atomic<std::uint64_t> finite_difference_jacobian_columns{0};
    std::atomic<std::uint64_t> nonsmooth_fallback_columns{0};
    std::atomic<std::uint64_t> analytic_jacobian_nanoseconds{0};
    std::atomic<std::uint64_t> finite_difference_jacobian_nanoseconds{0};
};

struct ScopedPerformanceTimer {
    std::atomic<std::uint64_t>* destination{nullptr};
    PerformanceClock::time_point started{};

    explicit ScopedPerformanceTimer(
        std::atomic<std::uint64_t>* destination_
    ) : destination(destination_) {
        if (destination != nullptr) started = PerformanceClock::now();
    }

    ~ScopedPerformanceTimer() {
        if (destination == nullptr) return;
        const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
            PerformanceClock::now() - started
        ).count();
        destination->fetch_add(
            static_cast<std::uint64_t>(elapsed), std::memory_order_relaxed
        );
    }
};

bool profiling_enabled() {
    const char* value = std::getenv("SUSPENSION_AXLE_PROFILE");
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

bool runtime_jacobian_validation_enabled() {
    const char* value = std::getenv("SUSPENSION_AXLE_VALIDATE_JACOBIAN");
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

bool static_debug_enabled() {
    const char* value = std::getenv("SUSPENSION_AXLE_DEBUG_STATIC");
    return value != nullptr && value[0] != '\0' && value[0] != '0';
}

struct ContactEventRecord {
    double time{0.0};
    int tire_index{-1};
    int transition{0};
};

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

double dot(const Vec3& a, const Vec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
Vec3 normalized(const Vec3& a) {
    const double n = norm(a);
    return n > kEps ? a / n : Vec3{0.0, 0.0, 0.0};
}

struct Mat3 {
    double a[3][3]{};
};

double determinant(const Mat3& m) {
    return
        m.a[0][0] * (m.a[1][1] * m.a[2][2] - m.a[1][2] * m.a[2][1]) -
        m.a[0][1] * (m.a[1][0] * m.a[2][2] - m.a[1][2] * m.a[2][0]) +
        m.a[0][2] * (m.a[1][0] * m.a[2][1] - m.a[1][1] * m.a[2][0]);
}

bool inverse3(const Mat3& m, Mat3& out) {
    double scale = 0.0;
    Mat3 scaled{};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            if (!std::isfinite(m.a[row][column])) return false;
            scale = std::max(scale, std::abs(m.a[row][column]));
        }
    }
    if (!(scale > 0.0) || !std::isfinite(scale)) return false;
    const double inv_scale = 1.0/scale;
    if (!std::isfinite(inv_scale)) return false;
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            scaled.a[row][column] = m.a[row][column]*inv_scale;
        }
    }
    const double d = determinant(scaled);
    if (!std::isfinite(d) || std::abs(d) <= 1e-14) return false;
    // Invert the normalized matrix first.  Multiplying by 1/scale at the end
    // keeps small SI inertias (for example 1e-6 kg*m^2) away from an absolute
    // determinant threshold and avoids underflow in the raw determinant.
    out.a[0][0] = (scaled.a[1][1]*scaled.a[2][2]
                   -scaled.a[1][2]*scaled.a[2][1])/d*inv_scale;
    out.a[0][1] = (scaled.a[0][2]*scaled.a[2][1]
                   -scaled.a[0][1]*scaled.a[2][2])/d*inv_scale;
    out.a[0][2] = (scaled.a[0][1]*scaled.a[1][2]
                   -scaled.a[0][2]*scaled.a[1][1])/d*inv_scale;
    out.a[1][0] = (scaled.a[1][2]*scaled.a[2][0]
                   -scaled.a[1][0]*scaled.a[2][2])/d*inv_scale;
    out.a[1][1] = (scaled.a[0][0]*scaled.a[2][2]
                   -scaled.a[0][2]*scaled.a[2][0])/d*inv_scale;
    out.a[1][2] = (scaled.a[0][2]*scaled.a[1][0]
                   -scaled.a[0][0]*scaled.a[1][2])/d*inv_scale;
    out.a[2][0] = (scaled.a[1][0]*scaled.a[2][1]
                   -scaled.a[1][1]*scaled.a[2][0])/d*inv_scale;
    out.a[2][1] = (scaled.a[0][1]*scaled.a[2][0]
                   -scaled.a[0][0]*scaled.a[2][1])/d*inv_scale;
    out.a[2][2] = (scaled.a[0][0]*scaled.a[1][1]
                   -scaled.a[0][1]*scaled.a[1][0])/d*inv_scale;
    return true;
}

bool finite_symmetric(const Mat3& m) {
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column) {
            if (!std::isfinite(m.a[row][column])) return false;
        }
    }
    const double symmetry = std::max({
        std::abs(m.a[0][1]-m.a[1][0]),
        std::abs(m.a[0][2]-m.a[2][0]),
        std::abs(m.a[1][2]-m.a[2][1])
    });
    return symmetry <= 1e-10;
}

bool symmetric_positive_definite(const Mat3& m) {
    if (!finite_symmetric(m)) return false;
    const double scale = std::max({
        std::abs(m.a[0][0]), std::abs(m.a[1][1]), std::abs(m.a[2][2])
    });
    if (scale <= 1e-14) return false;
    const double inv_scale = 1.0/scale;
    const double a00 = m.a[0][0]*inv_scale;
    const double a01 = m.a[0][1]*inv_scale;
    const double a11 = m.a[1][1]*inv_scale;
    const double a02 = m.a[0][2]*inv_scale;
    const double a12 = m.a[1][2]*inv_scale;
    const double a22 = m.a[2][2]*inv_scale;
    const double d1 = a00;
    const double d2 = a00*a11-a01*a01;
    const double d3 =
        a00*(a11*a22-a12*a12)
        -a01*(a01*a22-a12*a02)
        +a02*(a01*a12-a11*a02);
    return d1 > 1e-12 && d2 > 1e-12 && d3 > 1e-12;
}

Mat3 transpose(const Mat3& m) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.a[i][j] = m.a[j][i];
    return r;
}

Mat3 operator*(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) r.a[i][j] += a.a[i][k] * b.a[k][j];
        }
    }
    return r;
}

Mat3 operator*(const Mat3& m, double s) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.a[i][j] = m.a[i][j]*s;
    return r;
}

Mat3 operator+(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.a[i][j] = a.a[i][j]+b.a[i][j];
    return r;
}

Mat3 operator-(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.a[i][j] = a.a[i][j]-b.a[i][j];
    return r;
}

Mat3 identity3() {
    Mat3 r{};
    r.a[0][0] = r.a[1][1] = r.a[2][2] = 1.0;
    return r;
}

Vec3 operator*(const Mat3& a, const Vec3& v) {
    return {
        a.a[0][0]*v.x + a.a[0][1]*v.y + a.a[0][2]*v.z,
        a.a[1][0]*v.x + a.a[1][1]*v.y + a.a[1][2]*v.z,
        a.a[2][0]*v.x + a.a[2][1]*v.y + a.a[2][2]*v.z
    };
}

Mat3 skew(const Vec3& v) {
    Mat3 r{};
    r.a[0][1] = -v.z; r.a[0][2] =  v.y;
    r.a[1][0] =  v.z; r.a[1][2] = -v.x;
    r.a[2][0] = -v.y; r.a[2][1] =  v.x;
    return r;
}

Mat3 outer(const Vec3& a, const Vec3& b) {
    Mat3 r{};
    const double av[3] = {a.x, a.y, a.z};
    const double bv[3] = {b.x, b.y, b.z};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) r.a[i][j] = av[i]*bv[j];
    return r;
}

// Row-vector times matrix: returns the vector w with w_j = sum_i e_i * M_ij.
Vec3 row_times(const Vec3& e, const Mat3& m) { return transpose(m) * e; }

struct Quat {
    double w{1.0}, x{0.0}, y{0.0}, z{0.0};
};

Quat qmul(const Quat& a, const Quat& b) {
    return {
        a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
        a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
        a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
        a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w
    };
}

Quat qconj(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }

double qnorm(const Quat& q) {
    return std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

double qdot(const Quat& a, const Quat& b) {
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

Quat qnegated(const Quat& q) {
    return {-q.w, -q.x, -q.y, -q.z};
}

Quat qnormalize(Quat q) {
    const double n = qnorm(q);
    if (!std::isfinite(n) || n <= kEps) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan, nan, nan};
    }
    q.w /= n; q.x /= n; q.y /= n; q.z /= n;
    return q;
}

Quat normalized_continuous(const Quat& raw, const Quat& reference) {
    const double n = qnorm(raw);
    if (!std::isfinite(n) || std::abs(n - 1.0) > 1e-10) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan, nan, nan};
    }
    Quat result = qnormalize(raw);
    if (qdot(result, reference) < 0.0) result = qnegated(result);
    return result;
}

bool unit_quaternion(const Quat& q, double tolerance = 1e-8) {
    const double n = qnorm(q);
    return std::isfinite(n) && std::abs(n - 1.0) <= tolerance;
}

Mat3 qmat(const Quat& q_) {
    const Quat q = qnormalize(q_);
    Mat3 r{};
    r.a[0][0] = 1-2*(q.y*q.y+q.z*q.z);
    r.a[0][1] = 2*(q.x*q.y-q.z*q.w);
    r.a[0][2] = 2*(q.x*q.z+q.y*q.w);
    r.a[1][0] = 2*(q.x*q.y+q.z*q.w);
    r.a[1][1] = 1-2*(q.x*q.x+q.z*q.z);
    r.a[1][2] = 2*(q.y*q.z-q.x*q.w);
    r.a[2][0] = 2*(q.x*q.z-q.y*q.w);
    r.a[2][1] = 2*(q.y*q.z+q.x*q.w);
    r.a[2][2] = 1-2*(q.x*q.x+q.y*q.y);
    return r;
}

Quat qexp(const Vec3& theta) {
    const double a = norm(theta);
    if (a < 1e-10) return qnormalize({1.0, 0.5*theta.x, 0.5*theta.y, 0.5*theta.z});
    const double s = std::sin(0.5*a) / a;
    return qnormalize({std::cos(0.5*a), s*theta.x, s*theta.y, s*theta.z});
}

Vec3 qlog(Quat q) {
    q = qnormalize(q);
    if (q.w < 0.0) { q.w=-q.w; q.x=-q.x; q.y=-q.y; q.z=-q.z; }
    const double s = std::sqrt(q.x*q.x+q.y*q.y+q.z*q.z);
    if (s < 1e-10) return {2*q.x, 2*q.y, 2*q.z};
    const double a = 2.0*std::atan2(s, q.w);
    return {a*q.x/s, a*q.y/s, a*q.z/s};
}

Vec3 rotate(const Quat& q, const Vec3& v) { return qmat(q) * v; }

struct Body {
    double mass{0.0};
    Mat3 inertia_body{};
    Vec3 r{};
    Quat q{};
    Vec3 v{};
    Vec3 omega{};
    bool fixed{false};
};

struct Constraint {
    int type{AXLE_SPHERICAL};
    int a{-1}, b{-1};
    Vec3 pa{}, pb{}, axis_a{0,0,1}, axis_b{0,0,1};
    Vec3 axis_a_secondary{0,1,0}, axis_b_secondary{1,0,0};
    double convel_angle_target{0.0};
    int row{0};
};

struct CoordinateCoupler {
    int joint_a{-1};
    int coordinate_a{0};
    double scale_a{0.0};
    int joint_b{-1};
    int coordinate_b{0};
    double scale_b{0.0};
    Quat reference_rotation_a{};
    Quat reference_rotation_b{};
    double reference_translation_a{0.0};
    double reference_translation_b{0.0};
    int row{0};
};

struct Spring {
    int a{-1}, b{-1};
    Vec3 pa{}, pb{};
    double k{0.0}, c_compression{0.0}, c_rebound{0.0}, free_length{0.0};
    double minimum_length{std::numeric_limits<double>::quiet_NaN()};
    double maximum_length{std::numeric_limits<double>::quiet_NaN()};
    double compression_stop_k{0.0}, compression_stop_c{0.0};
    double rebound_stop_k{0.0}, rebound_stop_c{0.0};
    // Optional measured damper curve, strictly increasing in velocity.  When
    // present it replaces the two constant coefficients entirely; a real shock
    // is neither linear nor symmetric about zero velocity, so approximating one
    // by a pair of constants would be a fit rather than the measured element.
    std::vector<double> damper_velocity;
    std::vector<double> damper_force;
    // 可选源曲线：弹簧使用压缩挠度，限位块使用穿透量。
    std::vector<double> elastic_deflection;
    std::vector<double> elastic_force;
    std::vector<double> compression_stop_penetration;
    std::vector<double> compression_stop_force;
    std::vector<double> rebound_stop_penetration;
    std::vector<double> rebound_stop_force;
};

// Piecewise-linear interpolation with constant extrapolation beyond the ends.
// Returns the force the curve applies for the given extension rate.
double interpolate_curve(
    const std::vector<double>& x, const std::vector<double>& y, double value
) {
    const std::size_t n = x.size();
    if (n == 0) return 0.0;
    if (n == 1 || value <= x.front()) return y.front();
    if (value >= x.back()) return y.back();
    std::size_t high = 1;
    while (high < n && x[high] < value) ++high;
    const double x0 = x[high-1], x1 = x[high];
    const double y0 = y[high-1], y1 = y[high];
    const double span = x1 - x0;
    if (span <= 0.0) return y1;
    return y0 + (y1 - y0) * ((value - x0) / span);
}

std::vector<double> akima_curve_slopes(
    const std::vector<double>& x, const std::vector<double>& y
) {
    const std::size_t n = x.size();
    std::vector<double> slope(n, 0.0);
    if (n < 2) return slope;
    std::vector<double> secant(n-1, 0.0);
    for (std::size_t i = 0; i+1 < n; ++i) {
        secant[i] = (y[i+1]-y[i])/(x[i+1]-x[i]);
    }
    if (n == 2) {
        slope[0] = secant[0];
        slope[1] = secant[0];
        return slope;
    }
    // Four endpoint secants extend the standard Akima five-point stencil.
    std::vector<double> extended(n+3, 0.0);
    for (std::size_t i = 0; i < secant.size(); ++i) {
        extended[i+2] = secant[i];
    }
    extended[1] = 2.0*extended[2]-extended[3];
    extended[0] = 2.0*extended[1]-extended[2];
    extended[n+1] = 2.0*extended[n]-extended[n-1];
    extended[n+2] = 2.0*extended[n+1]-extended[n];
    for (std::size_t i = 0; i < n; ++i) {
        const double w_left = std::abs(extended[i+3]-extended[i+2]);
        const double w_right = std::abs(extended[i+1]-extended[i]);
        const double denominator = w_left+w_right;
        slope[i] = denominator > 0.0
            ? (w_left*extended[i+1]+w_right*extended[i+2])/denominator
            : 0.5*(extended[i+1]+extended[i+2]);
    }
    return slope;
}

std::pair<double, double> interpolate_akima_curve(
    const std::vector<double>& x, const std::vector<double>& y, double value
) {
    if (x.empty() || y.empty()) return {0.0, 0.0};
    if (x.size() == 1 || value <= x.front()) return {y.front(), 0.0};
    if (value >= x.back()) return {y.back(), 0.0};
    const std::vector<double> slopes = akima_curve_slopes(x, y);
    std::size_t high = 1;
    while (high < x.size() && x[high] < value) ++high;
    const double x0 = x[high-1], x1 = x[high];
    const double y0 = y[high-1], y1 = y[high];
    const double h = x1-x0;
    const double u = (value-x0)/h;
    const double u2 = u*u;
    const double u3 = u2*u;
    const double h00 = 2.0*u3-3.0*u2+1.0;
    const double h10 = u3-2.0*u2+u;
    const double h01 = -2.0*u3+3.0*u2;
    const double h11 = u3-u2;
    const double result = h00*y0+h10*h*slopes[high-1]
        +h01*y1+h11*h*slopes[high];
    const double derivative =
        ((6.0*u2-6.0*u)/h)*y0
        +(3.0*u2-4.0*u+1.0)*slopes[high-1]
        +((-6.0*u2+6.0*u)/h)*y1
        +(3.0*u2-2.0*u)*slopes[high];
    return {result, derivative};
}

double integrate_akima_segment(
    double y0, double y1, double slope0, double slope1, double h,
    double lower_u, double upper_u
) {
    const auto primitive = [=](double u) {
        const double u2 = u*u;
        const double u3 = u2*u;
        const double u4 = u3*u;
        return y0*(0.5*u4-u3+u)
            +h*slope0*(0.25*u4-(2.0/3.0)*u3+0.5*u2)
            +y1*(-0.5*u4+u3)
            +h*slope1*(0.25*u4-(1.0/3.0)*u3);
    };
    return h*(primitive(upper_u)-primitive(lower_u));
}

double integrate_akima_curve_from_zero(
    const std::vector<double>& x, const std::vector<double>& y, double value
) {
    if (x.empty() || y.empty() || value == 0.0) return 0.0;
    const std::vector<double> slopes = akima_curve_slopes(x, y);
    const double lower = std::min(0.0, value);
    const double upper = std::max(0.0, value);
    double total = 0.0;
    double left = lower;
    while (left < upper) {
        if (left < x.front()) {
            const double right = std::min(upper, x.front());
            total += y.front()*(right-left);
            left = right;
            continue;
        }
        if (left >= x.back()) {
            total += y.back()*(upper-left);
            break;
        }
        std::size_t high = 1;
        while (high < x.size() && x[high] <= left) ++high;
        const double right = std::min(upper, x[high]);
        const double h = x[high]-x[high-1];
        total += integrate_akima_segment(
            y[high-1], y[high], slopes[high-1], slopes[high], h,
            (left-x[high-1])/h, (right-x[high-1])/h
        );
        if (right <= left) break;
        left = right;
    }
    return value >= 0.0 ? total : -total;
}

double integrate_curve_interval(
    const std::vector<double>& x, const std::vector<double>& y,
    double lower, double upper
) {
    if (x.empty() || y.empty() || upper <= lower) return 0.0;
    double total = 0.0;
    double left = lower;
    while (left < upper) {
        double right = upper;
        if (left < x.front()) {
            right = std::min(right, x.front());
            total += y.front()*(right-left);
        } else if (left >= x.back()) {
            total += y.back()*(right-left);
        } else {
            std::size_t high = 1;
            while (high < x.size() && x[high] <= left) ++high;
            right = std::min(right, x[high]);
            const double left_force = interpolate_curve(x, y, left);
            const double right_force = interpolate_curve(x, y, right);
            total += 0.5*(left_force+right_force)*(right-left);
        }
        if (right <= left) break;
        left = right;
    }
    return total;
}

double integrate_curve_from_zero(
    const std::vector<double>& x, const std::vector<double>& y,
    double value
) {
    if (value >= 0.0) return integrate_curve_interval(x, y, 0.0, value);
    return -integrate_curve_interval(x, y, value, 0.0);
}

struct Bushing {
    int a{-1}, b{-1};
    Vec3 pa{}, pb{};
    Quat frame_a{}, frame_b{}, reference{};
    Vec3 reference_translation{};
    std::array<double, 36> stiffness{};
    std::array<double, 36> damping{};
    std::array<double, 6> preload{};
    int rotation_coordinates{VEHICLE_BUSHING_ROTATION_VECTOR};
    int force_curve_interpolation{0};
    std::array<std::vector<double>, 6> elastic_coordinate;
    std::array<std::vector<double>, 6> elastic_force;
};

std::pair<double, double> bushing_curve_value_slope(
    const Bushing& bushing, std::size_t axis, double value
) {
    const auto& x = bushing.elastic_coordinate[axis];
    const auto& y = bushing.elastic_force[axis];
    if (bushing.force_curve_interpolation == 1) {
        return interpolate_akima_curve(x, y, value);
    }
    if (x.empty() || y.empty()) return {0.0, 0.0};
    const double result = interpolate_curve(x, y, value);
    double slope = 0.0;
    if (x.size() > 1) {
        std::size_t high = 1;
        if (value > x.front() && value < x.back()) {
            while (high < x.size() && x[high] < value) ++high;
        } else if (value >= x.back()) {
            high = x.size()-1;
        }
        slope = (y[high]-y[high-1])/(x[high]-x[high-1]);
    }
    return {result, slope};
}

double integrate_bushing_curve_from_zero(
    const Bushing& bushing, std::size_t axis, double value
) {
    if (bushing.force_curve_interpolation == 1) {
        return integrate_akima_curve_from_zero(
            bushing.elastic_coordinate[axis],
            bushing.elastic_force[axis], value
        );
    }
    return integrate_curve_from_zero(
        bushing.elastic_coordinate[axis],
        bushing.elastic_force[axis], value
    );
}

Vec3 cardan_xyz_from_rotation(const Mat3& rotation) {
    const double cosine_y = std::hypot(rotation.a[0][0], rotation.a[0][1]);
    if (cosine_y <= 1e-10) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan, nan};
    }
    return {
        std::atan2(-rotation.a[1][2], rotation.a[2][2]),
        std::atan2(rotation.a[0][2], cosine_y),
        std::atan2(-rotation.a[0][1], rotation.a[0][0])
    };
}

Vec3 cardan_xyz_rate(const Vec3& angles, const Vec3& relative_omega) {
    const double sine_x = std::sin(angles.x);
    const double cosine_x = std::cos(angles.x);
    const double sine_y = std::sin(angles.y);
    const double cosine_y = std::cos(angles.y);
    if (std::abs(cosine_y) <= 1e-10) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return {nan, nan, nan};
    }
    const double y_rate =
        cosine_x*relative_omega.y+sine_x*relative_omega.z;
    const double z_rate =
        (-sine_x*relative_omega.y+cosine_x*relative_omega.z)/cosine_y;
    return {
        relative_omega.x-sine_y*z_rate,
        y_rate,
        z_rate
    };
}

struct AntiRollBar {
    int a{-1}, b{-1};
    Vec3 axis_a{0, 0, 1};
    Quat reference{};
    double stiffness{0.0}, damping{0.0};
};

struct Tire {
    int body{-1};
    // `body` is the spinning wheel body that receives the contact wrench.
    // `frame_body` is the non-spinning carrier used for tire axes. The default
    // keeps the stable axle ABI semantics.
    int frame_body{-1};
    int drive_torque_body{-1};
    int drive_torque_reaction_body{-1};
    Vec3 center{};
    Vec3 frame_center{};
    Vec3 drive_torque_axis{};
    Vec3 spin_axis{0, 1, 0};
    Vec3 forward_axis{1, 0, 0};
    double radius{0.0}, maximum_compression{0.0}, k{0.0}, c{0.0};
    double mu_longitudinal{0.0}, mu_lateral{0.0};
    double brush_k_longitudinal{0.0}, brush_k_lateral{0.0};
    double relaxation_length_longitudinal{0.0};
    double relaxation_length_lateral{0.0};
    double detached_relaxation{0.0};
    int model_kind{VEHICLE_TIRE_NATIVE_BRUSH};
    std::array<double, VEHICLE_PAC2002_PARAMETER_COUNT> pac2002_parameters{};
};

struct AerodynamicDrag {
    int body{-1};
    Vec3 application_point{};
    Vec3 forward_axis{1.0, 0.0, 0.0};
    double coefficient{0.0};
};

struct StaticRotationGauge {
    int body{-1};
    Vec3 axis_world{0.0, 0.0, 1.0};
    int pivot{-1};
};

enum Pac2002ParameterIndex {
    PAC_FNOMIN = 0,
    PAC_PCX1,
    PAC_PDX1,
    PAC_PDX2,
    PAC_PKX1,
    PAC_PKX2,
    PAC_PEX1,
    PAC_PEX2,
    PAC_PHX1,
    PAC_PHX2,
    PAC_PVX1,
    PAC_PVX2,
    PAC_PCY1,
    PAC_PDY1,
    PAC_PDY2,
    PAC_PKY1,
    PAC_PKY2,
    PAC_PEY1,
    PAC_PEY2,
    PAC_PHY1,
    PAC_PHY2,
    PAC_PVY1,
    PAC_PVY2,
    PAC_PDX3,
    PAC_PEX3,
    PAC_PEX4,
    PAC_PKX3,
    PAC_PDY3,
    PAC_PEY3,
    PAC_PEY4,
    PAC_PKY3,
    PAC_PHY3,
    PAC_PVY3,
    PAC_PVY4,
    PAC_RBX1,
    PAC_RBX2,
    PAC_RCX1,
    PAC_REX1,
    PAC_REX2,
    PAC_RHX1,
    PAC_RBY1,
    PAC_RBY2,
    PAC_RBY3,
    PAC_RCY1,
    PAC_REY1,
    PAC_REY2,
    PAC_RHY1,
    PAC_RHY2,
    PAC_RVY1,
    PAC_RVY2,
    PAC_RVY3,
    PAC_RVY4,
    PAC_RVY5,
    PAC_RVY6,
    PAC_PTX1,
    PAC_PTX2,
    PAC_PTX3,
    PAC_PTY1,
    PAC_PTY2,
    PAC_QBZ1,
    PAC_QBZ2,
    PAC_QBZ3,
    PAC_QBZ4,
    PAC_QBZ5,
    PAC_QBZ9,
    PAC_QBZ10,
    PAC_QCZ1,
    PAC_QDZ1,
    PAC_QDZ2,
    PAC_QDZ3,
    PAC_QDZ4,
    PAC_QDZ6,
    PAC_QDZ7,
    PAC_QDZ8,
    PAC_QDZ9,
    PAC_QEZ1,
    PAC_QEZ2,
    PAC_QEZ3,
    PAC_QEZ4,
    PAC_QEZ5,
    PAC_QHZ1,
    PAC_QHZ2,
    PAC_QHZ3,
    PAC_QHZ4,
    PAC_SSZ1,
    PAC_SSZ2,
    PAC_SSZ3,
    PAC_SSZ4,
    PAC_LCX,
    PAC_LCY,
    PAC_LEX,
    PAC_LEY,
    PAC_LFZO,
    PAC_LGAX,
    PAC_LGAY,
    PAC_LGAZ,
    PAC_LHX,
    PAC_LHY,
    PAC_LKX,
    PAC_LKY,
    PAC_LKYG,
    PAC_LMUX,
    PAC_LMUY,
    PAC_LMX,
    PAC_LMY,
    PAC_LRES,
    PAC_LS,
    PAC_LSGAL,
    PAC_LSGKP,
    PAC_LTR,
    PAC_LVMX,
    PAC_LVX,
    PAC_LVY,
    PAC_LVYKA,
    PAC_LXAL,
    PAC_LYKA,
    PAC_LIP,
    PAC_IP,
    PAC_IP_NOM,
    PAC_PPX1,
    PAC_PPX2,
    PAC_PPX3,
    PAC_PPX4,
    PAC_PPY1,
    PAC_PPY2,
    PAC_PPY3,
    PAC_PPY4,
    PAC_QPX1,
    PAC_QPZ1,
    PAC_QPZ2,
    PAC_QSX1,
    PAC_QSX2,
    PAC_QSX3,
    PAC_QSX4,
    PAC_QSX5,
    PAC_QSX7,
    PAC_QSX8,
    PAC_QSX9,
    PAC_QSX10,
    PAC_QSX11,
    PAC_QSY1,
    PAC_QSY2,
    PAC_QSY3,
    PAC_QSY4,
    PAC_QSY5,
    PAC_QSY6,
    PAC_QSY7,
    PAC_QSY8,
    PAC_QTZ1,
    PAC_LCZ,
    PAC_BREFF,
    PAC_DREFF,
    PAC_FREFF,
    PAC_QREO,
    PAC_QV1,
    PAC_QV2,
    PAC_QFCX1,
    PAC_QFCY1,
    PAC_QFCG1,
    PAC_QFZ1,
    PAC_QFZ2,
    PAC_QFZ3,
    PAC_QPFZ1,
    PAC_LONGVL,
    PAC_VXLOW,
    PAC_LGYR,
    PAC_MBELT
};

// Fiala 参数复用车辆扩展数组的前 14 个槽位；仅在模型类型为 FIALA
// 时解释，PAC2002 的历史布局和数值不变。
enum FialaParameterIndex {
    FIALA_CSLIP = 0,
    FIALA_CALPHA = 1,
    FIALA_CGAMMA = 2,
    FIALA_MGAMMA = 3,
    FIALA_CSPIN = 4,
    FIALA_UMIN = 5,
    FIALA_UMAX = 6,
    FIALA_RELAX_LENGTH_X = 7,
    FIALA_RELAX_LENGTH_Y = 8,
    FIALA_WIDTH = 9,
    FIALA_ROLLING_RESISTANCE = 10,
    FIALA_LOW_SPEED_THRESHOLD = 11,
    FIALA_DAMP_X = 12,
    FIALA_DAMP_Y = 13
};

double pac2002_parameter(const Tire& tire, int index, double fallback);
double fiala_parameter(const Tire& tire, int index, double fallback);
void fiala_forces(
    const Tire& tire, double kappa, double alpha, double normal_force,
    double& fx, double& fy, double& mz
);
double pac2002_positive_scale(
    const Tire& tire, int index, double fallback
);
double pac2002_pressure_difference(const Tire& tire);

double pac2002_effective_rolling_radius(
    const Tire& tire, double penetration, double spin_rate
) {
    const double radius = std::max(tire.radius, 1e-9);
    const double nominal_load = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double vertical_scale = pac2002_positive_scale(tire, PAC_LCZ, 1.0);
    const double vertical_stiffness = std::max(tire.k*vertical_scale, 1e-9);
    const double nominal_deflection = nominal_load/vertical_stiffness;
    const double normalized_deflection = std::max(penetration, 0.0)
        /std::max(nominal_deflection, 1e-9);
    const double speed_reference = std::max(
        pac2002_parameter(tire, PAC_LONGVL, 16.6), 1e-9
    );
    const double speed_growth = pac2002_parameter(tire, PAC_QV1, 0.0)
        *radius*std::pow(spin_rate*radius/speed_reference, 2.0);
    const double radius_correction = nominal_deflection* (
        pac2002_parameter(tire, PAC_DREFF, 0.27)
            *std::atan(pac2002_parameter(tire, PAC_BREFF, 8.4)
                *normalized_deflection)
        +pac2002_parameter(tire, PAC_FREFF, 0.07)*normalized_deflection
    );
    const double result = radius*pac2002_parameter(tire, PAC_QREO, 1.0)
        +speed_growth-radius_correction;
    return std::max(result, 1e-9);
}

double pac2002_vertical_force(
    const Tire& tire, double penetration, double penetration_rate,
    double camber
) {
    if (penetration <= 0.0) return 0.0;
    const double radius = std::max(tire.radius, 1e-9);
    const double nominal_load = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double load_scale = pac2002_positive_scale(tire, PAC_LCZ, 1.0);
    double qfz1 = pac2002_parameter(tire, PAC_QFZ1, 0.0);
    if (std::abs(qfz1) <= 1e-12) {
        qfz1 = tire.k*radius/(nominal_load*load_scale);
    }
    const double normalized_penetration = penetration/radius;
    const double camber_scale = camber*camber
        *pac2002_parameter(tire, PAC_QFZ3, 0.0);
    const double pressure_scale = 1.0
        +pac2002_parameter(tire, PAC_QPFZ1, 0.0)
            *pac2002_pressure_difference(tire);
    const double elastic_force = nominal_load*load_scale*pressure_scale* (
        qfz1*normalized_penetration
        +pac2002_parameter(tire, PAC_QFZ2, 0.0)
            *normalized_penetration*normalized_penetration
        +camber_scale*normalized_penetration
    );
    return std::max(0.0, elastic_force+tire.c*penetration_rate);
}

double pac2002_pure_force(
    const Tire& tire, double slip, double normal_force, bool lateral,
    double camber
);
double pac2002_force_limit(
    const Tire& tire, double normal_force, bool lateral, double camber
);
double pac2002_relaxation_length(
    const Tire& tire, double normal_force, bool lateral, double camber
);
double pac2002_gyroscopic_moment(
    const Tire& tire, double normal_force, double camber,
    double effective_lateral_slip, double effective_lateral_slip_rate,
    double rolling_radius, double spin_rate
);

bool pac2002_has_combined_slip_terms(const Tire& tire);
double pac2002_combined_longitudinal_force(
    const Tire& tire, double kappa, double alpha, double normal_force,
    double camber, double pure_force
);
double pac2002_combined_lateral_force(
    const Tire& tire, double kappa, double alpha, double normal_force,
    double camber, double pure_force
);
bool pac2002_has_aligning_moment_terms(const Tire& tire);
double pac2002_aligning_moment(
    const Tire& tire, double kappa, double alpha, double normal_force,
    double camber, double fx, double fy
);
bool pac2002_has_overturning_moment_terms(const Tire& tire);
bool pac2002_has_rolling_resistance_terms(const Tire& tire);

double pac2002_overturning_moment(
    const Tire& tire, double fy_source, double normal_force, double camber
);
double pac2002_rolling_resistance_moment(
    const Tire& tire, double fx_source, double normal_force, double camber,
    double longitudinal_speed
);

struct SteeringActuator {
    int type{VEHICLE_STEERING_ROTATION};
    int body{-1};
    int reaction_body{-1};
    Vec3 point_local{};
    Vec3 reaction_point_local{};
    Vec3 axis_local{0.0, 0.0, 1.0};
    // Relative body orientation at zero steering, expressed from the
    // reaction-body frame to the steerable-body frame.
    Quat reference{};
    double stiffness{0.0};
    double damping{0.0};
    const double* target_angle{nullptr};
    const double* target_rate{nullptr};
    int constraint_row{-1};
};

struct RoadProfile {
    int kind{0};
    double origin_x{0.0};
    double origin_z{0.0};
    double amplitude{0.0};
    double wavelength{1.0};
    double phase{0.0};
    double bump_start{0.0};
    double bump_length{1.0};
    std::array<double, 4> corner_scale{1.0, 1.0, 1.0, 1.0};
};

struct Model {
    std::vector<Body> bodies;
    std::vector<Constraint> constraints;
    std::vector<CoordinateCoupler> coordinate_couplers;
    std::vector<Spring> springs;
    std::vector<Bushing> bushings;
    std::vector<AntiRollBar> anti_roll_bars;
    std::vector<Tire> tires;
    std::vector<AerodynamicDrag> aerodynamic_drags;
    std::vector<SteeringActuator> steering_actuators;
    std::vector<StaticRotationGauge> static_rotation_gauges;
    RoadProfile road_profile{};
    const double* vehicle_brake_torque{nullptr};
    int static_gauge_body{-1};
    std::uint32_t static_gauge_dof_mask{0};
    bool static_trim_then_release{false};
    double initial_state_angle_tolerance{0.0};
    std::vector<Vec3> release_velocity;
    std::vector<Vec3> release_omega;
    std::vector<int> free_body;
    std::vector<int> body_to_free;
    int rows{0};
    int ndof{0};
};

struct State {
    std::vector<Vec3> r, v, a, omega, alpha;
    std::vector<Quat> q;
    std::vector<double> tire_sx, tire_sy;
    std::vector<double> tire_sx_dot, tire_sy_dot;
};

struct SampleInput {
    std::vector<double> body_wrench, road_z, road_v, torque, brake_torque;
    std::vector<double> steering_target, steering_target_rate,
        steering_target_acceleration;
};

double road_profile_height(
    const Model& model, const State& state, std::size_t tire_index
);
double road_profile_slope(
    const Model& model, const State& state, std::size_t tire_index
);

void set_error(char* buffer, std::size_t capacity, const std::string& text) {
    if (!buffer || capacity == 0) return;
    std::snprintf(buffer, capacity, "%s", text.c_str());
}

int constraint_rows(int type) {
    if (type == AXLE_SPHERICAL) return 3;
    if (type == AXLE_REVOLUTE || type == AXLE_PRISMATIC) return 5;
    if (type == AXLE_FIXED) return 6;
    if (type == AXLE_UNIVERSAL || type == AXLE_CYLINDRICAL ||
        type == AXLE_CONVEL) return 4;
    if (type == AXLE_INPLANE) return 1;
    return -1;
}

Vec3 state_point(const State& state, int body, const Vec3& local) {
    return state.r[body] + rotate(state.q[body], local);
}

Vec3 state_point_velocity(const State& state, int body, const Vec3& local) {
    const Vec3 arm = rotate(state.q[body], local);
    return state.v[body] + cross(state.omega[body], arm);
}

int tire_frame_body(const Tire& tire) {
    return tire.frame_body >= 0 ? tire.frame_body : tire.body;
}

Vec3 tire_frame_center(const Tire& tire) {
    return tire.frame_body >= 0 ? tire.frame_center : tire.center;
}

int tire_center_body(const Tire& tire) {
    // Adams evaluates transient tire kinematics at the wheel/spindle center.
    // The carrier remains the orientation frame, but its center velocity can
    // differ from the wheel center when suspension compliance is active.
    return tire.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE
        || tire.model_kind == VEHICLE_TIRE_FIALA
        ? tire.body : tire_frame_body(tire);
}

Vec3 tire_center_local(const Tire& tire) {
    return tire.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE
        || tire.model_kind == VEHICLE_TIRE_FIALA
        ? tire.center : tire_frame_center(tire);
}

Vec3 tire_relative_spin_omega(const State& state, const Tire& tire) {
    Vec3 omega = state.omega[tire.body];
    const int frame_body = tire_frame_body(tire);
    if (frame_body != tire.body) omega -= state.omega[frame_body];
    return omega;
}

double joint_coordinate_value(
    const Model& model, const State& state, int joint_index, int coordinate,
    double reference_translation, const Quat& reference_rotation
) {
    const Constraint& joint = model.constraints[
        static_cast<std::size_t>(joint_index)
    ];
    if (coordinate == 0) {
        const Quat relative = qmul(
            qconj(state.q[joint.a]), state.q[joint.b]
        );
        const Vec3 delta = qlog(qmul(qconj(reference_rotation), relative));
        return dot(normalized(joint.axis_a), delta);
    }
    const Vec3 axis = normalized(rotate(state.q[joint.a], joint.axis_a));
    const Vec3 separation = state_point(state, joint.a, joint.pa)
        - state_point(state, joint.b, joint.pb);
    return dot(axis, separation) - reference_translation;
}

Vec3 steering_axis_reference(const SteeringActuator& actuator) {
    return normalized(rotate(actuator.reference, actuator.axis_local));
}

double steering_rotation_coordinate(
    const SteeringActuator& actuator, const State& state
) {
    const Quat reaction_q = actuator.reaction_body >= 0
        ? state.q[actuator.reaction_body] : Quat{};
    const Quat relative = qmul(
        qconj(reaction_q), state.q[actuator.body]
    );
    const Vec3 error_rotation = qlog(
        qmul(qconj(actuator.reference), relative)
    );
    return dot(error_rotation, actuator.axis_local);
}

bool prescribed_steering(const SteeringActuator& actuator) {
    return actuator.type == VEHICLE_STEERING_PRESCRIBED_ROTATION ||
        actuator.type == VEHICLE_STEERING_PRESCRIBED_TRANSLATION;
}

double steering_translation_coordinate(
    const SteeringActuator& actuator, const State& state
) {
    const Vec3 body_point = state_point(
        state, actuator.body, actuator.point_local
    );
    const Vec3 reaction_point = actuator.reaction_body >= 0
        ? state_point(
            state, actuator.reaction_body, actuator.reaction_point_local
        )
        : Vec3{};
    const Vec3 axis_world = normalized(
        actuator.reaction_body >= 0
            ? rotate(state.q[actuator.reaction_body], actuator.axis_local)
            : actuator.axis_local
    );
    return dot(axis_world, body_point-reaction_point);
}

// The reference vector used to complete a frame perpendicular to `axis`. Both
// the residual and the analytic Jacobian must call this, or the Jacobian would
// linearize a different frame than the residual defines.
Vec3 perpendicular_reference(const Vec3& axis) {
    return std::abs(axis.x) < 0.8 ? Vec3{1,0,0} : Vec3{0,1,0};
}

std::vector<double> constraint_residual(
    const Model& model, const State& state, const SampleInput* input = nullptr
) {
    std::vector<double> out(model.rows, 0.0);
    for (const auto& c : model.constraints) {
        const Vec3 pa = state_point(state, c.a, c.pa);
        const Vec3 pb = state_point(state, c.b, c.pb);
        const Vec3 dp = pa - pb;
        int k = c.row;
        if (c.type == AXLE_SPHERICAL || c.type == AXLE_REVOLUTE ||
            c.type == AXLE_FIXED || c.type == AXLE_UNIVERSAL ||
            c.type == AXLE_CONVEL) {
            out[k++] = dp.x; out[k++] = dp.y; out[k++] = dp.z;
        }
        if (c.type == AXLE_FIXED) {
            const Vec3 dr = qlog(qmul(qconj(state.q[c.a]), state.q[c.b]));
            out[k++] = dr.x; out[k++] = dr.y; out[k++] = dr.z;
        } else if (c.type == AXLE_REVOLUTE) {
            const Vec3 aa = normalized(rotate(state.q[c.a], c.axis_a));
            const Vec3 ab = normalized(rotate(state.q[c.b], c.axis_b));
            Vec3 e1 = perpendicular_reference(aa);
            e1 = normalized(e1 - aa*dot(e1, aa));
            const Vec3 e2 = cross(aa, e1);
            out[k++] = dot(ab, e1);
            out[k++] = dot(ab, e2);
        } else if (c.type == AXLE_UNIVERSAL) {
            // The two cross-axes stay perpendicular; both spin freely.
            const Vec3 aa = normalized(rotate(state.q[c.a], c.axis_a));
            const Vec3 ab = normalized(rotate(state.q[c.b], c.axis_b));
            out[k++] = dot(aa, ab);
        } else if (c.type == AXLE_CONVEL) {
            const Vec3 xa = normalized(rotate(state.q[c.a], c.axis_a));
            const Vec3 ya = normalized(
                rotate(state.q[c.a], c.axis_a_secondary)
            );
            const Vec3 yb = normalized(rotate(state.q[c.b], c.axis_b));
            const Vec3 xb = normalized(
                rotate(state.q[c.b], c.axis_b_secondary)
            );
            out[k++] = dot(xa, yb) + dot(ya, xb)
                - c.convel_angle_target;
        } else if (c.type == AXLE_CYLINDRICAL) {
            // Both bodies share one axis line: the offset carries no component
            // perpendicular to the axis, and the axes stay parallel.  Sliding
            // along and spinning about the axis remain free.
            const Vec3 aa = normalized(rotate(state.q[c.a], c.axis_a));
            const Vec3 ab = normalized(rotate(state.q[c.b], c.axis_b));
            Vec3 e1 = perpendicular_reference(aa);
            e1 = normalized(e1 - aa*dot(e1, aa));
            const Vec3 e2 = cross(aa, e1);
            out[k++] = dot(dp, e1);
            out[k++] = dot(dp, e2);
            out[k++] = dot(ab, e1);
            out[k++] = dot(ab, e2);
        } else if (c.type == AXLE_INPLANE) {
            // Body B's point stays in the plane through body A's point whose
            // normal is axis_a.
            const Vec3 aa = normalized(rotate(state.q[c.a], c.axis_a));
            out[k++] = dot(dp, aa);
        } else if (c.type == AXLE_PRISMATIC) {
            const Vec3 aa = normalized(rotate(state.q[c.a], c.axis_a));
            const Vec3 ab = normalized(rotate(state.q[c.b], c.axis_b));
            Vec3 e1 = perpendicular_reference(aa);
            e1 = normalized(e1 - aa*dot(e1, aa));
            const Vec3 e2 = cross(aa, e1);
            out[k++] = dot(dp, e1);
            out[k++] = dot(dp, e2);
            // 棱柱副只约束两轴平行以及绕轴相对转角为零，不要求两刚体
            // 的局部坐标系完全重合，以支持源模型中的非共线安装姿态。
            out[k++] = dot(ab, e1);
            out[k++] = dot(ab, e2);
            const Vec3 relative_rotation = qlog(
                qmul(qconj(state.q[c.a]), state.q[c.b])
            );
            out[k++] = dot(normalized(c.axis_a), relative_rotation);
        }
    }
    for (const auto& coupler : model.coordinate_couplers) {
        out[static_cast<std::size_t>(coupler.row)] =
            coupler.scale_a * joint_coordinate_value(
                model, state, coupler.joint_a, coupler.coordinate_a,
                coupler.reference_translation_a, coupler.reference_rotation_a
            )
            + coupler.scale_b * joint_coordinate_value(
                model, state, coupler.joint_b, coupler.coordinate_b,
                coupler.reference_translation_b, coupler.reference_rotation_b
            );
    }
    for (std::size_t actuator_index = 0;
         actuator_index < model.steering_actuators.size();
         ++actuator_index) {
        const auto& actuator = model.steering_actuators[actuator_index];
        if (!prescribed_steering(actuator)) continue;
        const double target = input != nullptr &&
                actuator_index < input->steering_target.size()
            ? input->steering_target[actuator_index]
            : 0.0;
        const double coordinate =
            actuator.type == VEHICLE_STEERING_PRESCRIBED_TRANSLATION
                ? steering_translation_coordinate(actuator, state)
                : steering_rotation_coordinate(actuator, state);
        out[static_cast<std::size_t>(actuator.constraint_row)] =
            coordinate-target;
    }
    return out;
}

void perturb_pose(State& state, const Model& model, const std::vector<double>& dy, double scale) {
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        Vec3 dr{scale*dy[6*fi], scale*dy[6*fi+1], scale*dy[6*fi+2]};
        Vec3 dtheta{scale*dy[6*fi+3], scale*dy[6*fi+4], scale*dy[6*fi+5]};
        state.r[bi] += dr;
        state.q[bi] = normalized_continuous(
            qmul(qexp(dtheta), state.q[bi]), state.q[bi]
        );
    }
}

std::vector<double> constraint_jacobian_central_difference(
    const Model& model, const State& state
) {
    // Reference implementation. Central differences on the 6-dimensional world
    // tangent increment. A one-sided difference leaves an O(1e-7) error in J,
    // which reappears as a residual floor through the reaction and GGL terms
    // and stalls the Newton solve on ordinary mechanisms; the central step
    // keeps that error near 1e-10 for the same 1e-6 perturbation. This is now
    // used only by the startup self-check, because rebuilding it inside every
    // residual evaluation dominated the whole solve.
    constexpr double h = 1e-6;
    std::vector<double> J(model.rows * model.ndof, 0.0);
    for (int j = 0; j < model.ndof; ++j) {
        std::vector<double> dy(model.ndof, 0.0);
        dy[j] = 1.0;
        State plus = state;
        State minus = state;
        perturb_pose(plus, model, dy, h);
        perturb_pose(minus, model, dy, -h);
        const auto rp = constraint_residual(model, plus);
        const auto rm = constraint_residual(model, minus);
        for (int i = 0; i < model.rows; ++i) {
            J[i*model.ndof+j] = (rp[i]-rm[i])/(2.0*h);
        }
    }
    return J;
}

// Inverse of the left Jacobian of the SO(3) logarithm:
// Log(Exp(w) R) = Log(R) + Jl^-1(phi) w + O(w^2),  phi = Log(R).
Mat3 log_left_jacobian_inverse(const Vec3& phi) {
    const Mat3 s = skew(phi);
    const double angle = norm(phi);
    // The [phi]x^2 coefficient is 1/angle^2 - (1+cos)/(2*angle*sin), whose
    // series is 1/12 + angle^2/720; use the limit where the closed form
    // cancels catastrophically.
    double quadratic = 1.0/12.0;
    if (angle > 1e-4) {
        quadratic = 1.0/(angle*angle)
            - (1.0+std::cos(angle))/(2.0*angle*std::sin(angle));
    }
    return identity3() - s*0.5 + (s*s)*quadratic;
}

// Closed-form dPhi/dy on the world tangent increment
// (delta r, delta theta), where the pose update is
// r <- r + delta r and R <- Exp(delta theta) R.
std::vector<double> constraint_jacobian(const Model& model, const State& state) {
    std::vector<double> J(model.rows * model.ndof, 0.0);
    const Mat3 eye = identity3();

    auto add_row = [&](int row, int body, const Vec3& translation,
                       const Vec3& rotation) {
        const int fi = model.body_to_free[body];
        if (fi < 0) return;
        double* out = &J[static_cast<std::size_t>(row)*model.ndof + 6*fi];
        out[0] += translation.x; out[1] += translation.y; out[2] += translation.z;
        out[3] += rotation.x;    out[4] += rotation.y;    out[5] += rotation.z;
    };
    auto add_block = [&](int row0, int body, const Mat3& translation,
                         const Mat3& rotation) {
        for (int i = 0; i < 3; ++i) {
            add_row(
                row0+i, body,
                {translation.a[i][0], translation.a[i][1], translation.a[i][2]},
                {rotation.a[i][0], rotation.a[i][1], rotation.a[i][2]}
            );
        }
    };

    for (const auto& c : model.constraints) {
        const Mat3 ra = qmat(state.q[c.a]);
        const Vec3 arm_a = rotate(state.q[c.a], c.pa);
        const Vec3 arm_b = rotate(state.q[c.b], c.pb);
        const Vec3 dp = state_point(state, c.a, c.pa)
            - state_point(state, c.b, c.pb);
        const Mat3 zero{};
        int k = c.row;

        // d(pa - pb): d(R*local)/d(delta theta) = -[R*local]x.
        if (c.type == AXLE_SPHERICAL || c.type == AXLE_REVOLUTE ||
            c.type == AXLE_FIXED || c.type == AXLE_UNIVERSAL ||
            c.type == AXLE_CONVEL) {
            add_block(k, c.a, eye, skew(arm_a)*(-1.0));
            add_block(k, c.b, eye*(-1.0), skew(arm_b));
            k += 3;
        }

        // Relative rotation rows share one derivation. With
        // Rrel = Ra^T Rb, a world increment on either body is a left
        // perturbation of Rrel by Ra^T (delta theta_b - delta theta_a).
        auto add_relative_rotation = [&](int row0) {
            const Vec3 phi = qlog(qmul(qconj(state.q[c.a]), state.q[c.b]));
            const Mat3 m = log_left_jacobian_inverse(phi) * transpose(ra);
            add_block(row0, c.a, zero, m*(-1.0));
            add_block(row0, c.b, zero, m);
        };
        auto add_relative_rotation_row = [&](int row0, const Vec3& axis_local) {
            const Vec3 phi = qlog(qmul(qconj(state.q[c.a]), state.q[c.b]));
            const Mat3 m = log_left_jacobian_inverse(phi) * transpose(ra);
            const Vec3 row = row_times(normalized(axis_local), m);
            add_row(row0, c.a, {}, row*(-1.0));
            add_row(row0, c.b, {}, row);
        };

        if (c.type == AXLE_FIXED) {
            add_relative_rotation(k);
        } else if (c.type == AXLE_UNIVERSAL) {
            // d(aa . ab) = ab . d(aa) + aa . d(ab)
            const Vec3 aa = normalized(rotate(state.q[c.a], c.axis_a));
            const Vec3 ab = normalized(rotate(state.q[c.b], c.axis_b));
            add_row(k, c.a, {}, row_times(ab, skew(aa)*(-1.0)));
            add_row(k, c.b, {}, row_times(aa, skew(ab)*(-1.0)));
        } else if (c.type == AXLE_CONVEL) {
            const Vec3 xa = normalized(rotate(state.q[c.a], c.axis_a));
            const Vec3 ya = normalized(
                rotate(state.q[c.a], c.axis_a_secondary)
            );
            const Vec3 yb = normalized(rotate(state.q[c.b], c.axis_b));
            const Vec3 xb = normalized(
                rotate(state.q[c.b], c.axis_b_secondary)
            );
            add_row(
                k, c.a, {},
                row_times(yb, skew(xa)*(-1.0))
                    + row_times(xb, skew(ya)*(-1.0))
            );
            add_row(
                k, c.b, {},
                row_times(xa, skew(yb)*(-1.0))
                    + row_times(ya, skew(xb)*(-1.0))
            );
        } else if (c.type == AXLE_INPLANE) {
            // d(dp . aa) = aa . d(dp) + dp . d(aa)
            const Vec3 aa = normalized(rotate(state.q[c.a], c.axis_a));
            add_row(
                k, c.a, aa,
                row_times(aa, skew(arm_a)*(-1.0))
                    + row_times(dp, skew(aa)*(-1.0))
            );
            add_row(k, c.b, aa*(-1.0), row_times(aa, skew(arm_b)));
        } else if (c.type == AXLE_REVOLUTE || c.type == AXLE_PRISMATIC ||
                   c.type == AXLE_CYLINDRICAL) {
            // Frame perpendicular to the body-a axis, and its derivative.
            const Vec3 aa = normalized(rotate(state.q[c.a], c.axis_a));
            const Vec3 g = perpendicular_reference(aa);
            const Vec3 u = g - aa*dot(g, aa);
            const double u_norm = norm(u);
            const Vec3 e1 = u * (1.0/u_norm);
            const Vec3 e2 = cross(aa, e1);
            // Rotation preserves length, so normalizing commutes with it and
            // d(aa)/d(delta theta_a) = -[aa]x.
            const Mat3 d_aa = skew(aa)*(-1.0);
            const Mat3 d_u = (eye*dot(g, aa) + outer(aa, g))*(-1.0);
            const Mat3 d_e1 =
                ((eye - outer(e1, e1))*(1.0/u_norm)) * d_u * d_aa;
            const Mat3 d_e2 = skew(e1)*(-1.0)*d_aa + skew(aa)*d_e1;

            if (c.type == AXLE_REVOLUTE) {
                const Vec3 ab = normalized(rotate(state.q[c.b], c.axis_b));
                const Mat3 d_ab = skew(ab)*(-1.0);
                // d(ab . e_i) = e_i . d(ab) + ab . d(e_i)
                add_row(k,   c.b, {}, row_times(e1, d_ab));
                add_row(k,   c.a, {}, row_times(ab, d_e1));
                add_row(k+1, c.b, {}, row_times(e2, d_ab));
                add_row(k+1, c.a, {}, row_times(ab, d_e2));
            } else if (c.type == AXLE_CYLINDRICAL) {
                // Two offset rows, then two axis-parallelism rows.
                add_row(
                    k, c.a, e1,
                    row_times(e1, skew(arm_a)*(-1.0)) + row_times(dp, d_e1)
                );
                add_row(k, c.b, e1*(-1.0), row_times(e1, skew(arm_b)));
                add_row(
                    k+1, c.a, e2,
                    row_times(e2, skew(arm_a)*(-1.0)) + row_times(dp, d_e2)
                );
                add_row(k+1, c.b, e2*(-1.0), row_times(e2, skew(arm_b)));
                const Vec3 ab = normalized(rotate(state.q[c.b], c.axis_b));
                const Mat3 d_ab = skew(ab)*(-1.0);
                add_row(k+2, c.b, {}, row_times(e1, d_ab));
                add_row(k+2, c.a, {}, row_times(ab, d_e1));
                add_row(k+3, c.b, {}, row_times(e2, d_ab));
                add_row(k+3, c.a, {}, row_times(ab, d_e2));
            } else {
                // d(dp . e_i) = e_i . d(dp) + dp . d(e_i)
                add_row(
                    k, c.a, e1,
                    row_times(e1, skew(arm_a)*(-1.0)) + row_times(dp, d_e1)
                );
                add_row(k, c.b, e1*(-1.0), row_times(e1, skew(arm_b)));
                add_row(
                    k+1, c.a, e2,
                    row_times(e2, skew(arm_a)*(-1.0)) + row_times(dp, d_e2)
                );
                add_row(k+1, c.b, e2*(-1.0), row_times(e2, skew(arm_b)));
                const Vec3 ab = normalized(rotate(state.q[c.b], c.axis_b));
                const Mat3 d_ab = skew(ab)*(-1.0);
                add_row(k+2, c.b, {}, row_times(e1, d_ab));
                add_row(k+2, c.a, {}, row_times(ab, d_e1));
                add_row(k+3, c.b, {}, row_times(e2, d_ab));
                add_row(k+3, c.a, {}, row_times(ab, d_e2));
                add_relative_rotation_row(k+4, c.axis_a);
            }
        }
    }
    for (const auto& coupler : model.coordinate_couplers) {
        auto add_joint_coordinate = [&](int row, int joint_index, int coordinate,
                                        const Quat& reference_rotation,
                                        double scale) {
            const Constraint& joint = model.constraints[
                static_cast<std::size_t>(joint_index)
            ];
            const Mat3 ra = qmat(state.q[joint.a]);
            const Vec3 arm_a = rotate(state.q[joint.a], joint.pa);
            const Vec3 arm_b = rotate(state.q[joint.b], joint.pb);
            if (coordinate == 0) {
                const Quat relative = qmul(
                    qconj(state.q[joint.a]), state.q[joint.b]
                );
                const Quat delta_rotation = qmul(
                    qconj(reference_rotation), relative
                );
                const Vec3 phi = qlog(delta_rotation);
                const Mat3 map = log_left_jacobian_inverse(phi)
                    * transpose(qmat(reference_rotation)) * transpose(ra);
                const Vec3 row_value = row_times(
                    normalized(joint.axis_a), map
                ) * scale;
                add_row(row, joint.a, {}, row_value*(-1.0));
                add_row(row, joint.b, {}, row_value);
                return;
            }
            const Vec3 axis = normalized(
                rotate(state.q[joint.a], joint.axis_a)
            );
            const Vec3 separation = state_point(state, joint.a, joint.pa)
                - state_point(state, joint.b, joint.pb);
            const Mat3 d_axis = skew(axis)*(-1.0);
            add_row(
                row, joint.a, axis*scale,
                (row_times(axis, skew(arm_a)*(-1.0))
                    + row_times(separation, d_axis))*scale
            );
            add_row(
                row, joint.b, axis*(-scale),
                row_times(axis, skew(arm_b))*scale
            );
        };
        add_joint_coordinate(
            coupler.row, coupler.joint_a, coupler.coordinate_a,
            coupler.reference_rotation_a, coupler.scale_a
        );
        add_joint_coordinate(
            coupler.row, coupler.joint_b, coupler.coordinate_b,
            coupler.reference_rotation_b, coupler.scale_b
        );
    }
    for (const auto& actuator : model.steering_actuators) {
        if (!prescribed_steering(actuator)) continue;
        if (actuator.type == VEHICLE_STEERING_PRESCRIBED_TRANSLATION) {
            const Vec3 body_arm = rotate(
                state.q[actuator.body], actuator.point_local
            );
            const Vec3 body_point = state.r[actuator.body]+body_arm;
            Vec3 reaction_arm{};
            Vec3 reaction_point{};
            Vec3 axis = normalized(actuator.axis_local);
            if (actuator.reaction_body >= 0) {
                reaction_arm = rotate(
                    state.q[actuator.reaction_body],
                    actuator.reaction_point_local
                );
                reaction_point =
                    state.r[actuator.reaction_body]+reaction_arm;
                axis = normalized(rotate(
                    state.q[actuator.reaction_body], actuator.axis_local
                ));
            }
            const Vec3 separation = body_point-reaction_point;
            add_row(
                actuator.constraint_row, actuator.body, axis,
                row_times(axis, skew(body_arm)*(-1.0))
            );
            if (actuator.reaction_body >= 0) {
                add_row(
                    actuator.constraint_row, actuator.reaction_body,
                    axis*(-1.0),
                    row_times(separation, skew(axis)*(-1.0))
                        +row_times(axis, skew(reaction_arm))
                );
            }
            continue;
        }
        const Quat reaction_q = actuator.reaction_body >= 0
            ? state.q[actuator.reaction_body] : Quat{};
        const Mat3 reaction_matrix = qmat(reaction_q);
        const Quat relative = qmul(
            qconj(reaction_q), state.q[actuator.body]
        );
        const Vec3 phi = qlog(
            qmul(qconj(actuator.reference), relative)
        );
        const Mat3 map = log_left_jacobian_inverse(phi)
            * transpose(qmat(actuator.reference))
            * transpose(reaction_matrix);
        const Vec3 row_value = row_times(actuator.axis_local, map);
        add_row(actuator.constraint_row, actuator.body, {}, row_value);
        if (actuator.reaction_body >= 0) {
            add_row(
                actuator.constraint_row, actuator.reaction_body, {},
                row_value*(-1.0)
            );
        }
    }
    return J;
}

// The analytic Jacobian above is a hand derivation, so every run verifies it
// against central differences at the initial state and refuses to continue on
// a mismatch. One reference evaluation per run is negligible next to the
// hundreds of thousands of analytic evaluations it replaces.
bool analytic_constraint_jacobian_matches_reference(
    const Model& model, const State& state, double& error
) {
    const auto analytic = constraint_jacobian(model, state);
    const auto reference = constraint_jacobian_central_difference(model, state);
    double scale = 1.0;
    double worst = 0.0;
    for (std::size_t i = 0; i < reference.size(); ++i) {
        scale = std::max(scale, std::abs(reference[i]));
        worst = std::max(worst, std::abs(analytic[i]-reference[i]));
    }
    error = worst;
    return worst <= 1e-6*scale;
}


Vec3 add_force_on_body(
    std::vector<Vec3>& force, std::vector<Vec3>& torque,
    const Model& model, const State& state, int body, const Vec3& point_local,
    const Vec3& f_world) {
    if (body < 0 || model.bodies[body].fixed) return {};
    const Vec3 arm = rotate(state.q[body], point_local);
    force[body] += f_world;
    torque[body] += cross(arm, f_world);
    return f_world;
}

void add_torque_on_body(
    std::vector<Vec3>& torque, const Model& model, int body, const Vec3& tau_world
) {
    if (body < 0 || model.bodies[body].fixed) return;
    torque[body] += tau_world;
}

std::array<double, 6> mat6_mul(
    const std::array<double, 36>& matrix, const std::array<double, 6>& vector
) {
    std::array<double, 6> result{};
    for (int row = 0; row < 6; ++row) {
        for (int col = 0; col < 6; ++col) {
            result[row] += matrix[static_cast<std::size_t>(row * 6 + col)] * vector[col];
        }
    }
    return result;
}

void project_brush_state(
    const Tire& tire, double normal_force, double sx, double sy,
    double& projected_sx, double& projected_sy, double& trial_utilization
) {
    projected_sx = sx;
    projected_sy = sy;
    trial_utilization = 0.0;
    if (normal_force <= 0.0) return;
    const double longitudinal_limit =
        tire.mu_longitudinal * normal_force;
    const double lateral_limit = tire.mu_lateral * normal_force;
    const double normalized_x =
        tire.brush_k_longitudinal * sx / longitudinal_limit;
    const double normalized_y =
        tire.brush_k_lateral * sy / lateral_limit;
    trial_utilization = std::sqrt(
        normalized_x*normalized_x + normalized_y*normalized_y
    );
    if (trial_utilization >= 1.0) {
        projected_sx /= trial_utilization;
        projected_sy /= trial_utilization;
    }
}

std::array<double, 6> bushing_deformation(
    const Bushing& bushing, const Model& model, const State& state,
    std::array<double, 6>& rate
) {
    const Vec3 pa = state_point(state, bushing.a, bushing.pa);
    const Vec3 pb = state_point(state, bushing.b, bushing.pb);
    const Vec3 va = state_point_velocity(state, bushing.a, bushing.pa);
    const Vec3 vb = state_point_velocity(state, bushing.b, bushing.pb);
    const Quat qfa = qmul(state.q[bushing.a], bushing.frame_a);
    const Quat qfb = qmul(state.q[bushing.b], bushing.frame_b);
    const Mat3 rfa = qmat(qfa);
    const Mat3 rt = transpose(rfa);
    const Vec3 rel = rt * (pb - pa);
    const Vec3 rel_v = rt * (vb - va);
    const Vec3 omega_a = rt * state.omega[bushing.a];
    const Vec3 omega_b = rt * state.omega[bushing.b];
    const Vec3 rel_rate = rel_v - cross(omega_a, rel);
    const Vec3 rel_omega = omega_b - omega_a;
    const Quat qrel = qmul(qconj(qfa), qfb);
    const Quat qdelta = qmul(qconj(bushing.reference), qrel);
    const Vec3 rotation = bushing.rotation_coordinates ==
            VEHICLE_BUSHING_CARDAN_XYZ
        ? cardan_xyz_from_rotation(qmat(qdelta))
        : qlog(qdelta);
    const Vec3 rotation_rate = bushing.rotation_coordinates ==
            VEHICLE_BUSHING_CARDAN_XYZ
        ? cardan_xyz_rate(rotation, rel_omega)
        : rel_omega;
    const Vec3 translation = rel - bushing.reference_translation;
    std::array<double, 6> deformation{
        translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z
    };
    rate = {
        rel_rate.x, rel_rate.y, rel_rate.z,
        rotation_rate.x, rotation_rate.y, rotation_rate.z
    };
    (void)model;
    return deformation;
}

struct StaticContactOverride {
    const std::vector<int>* active{nullptr};
    const std::vector<double>* compression{nullptr};
    const std::vector<double>* compression_derivative{nullptr};
    // 静态载荷递增因子。显式外载和保守内部力元共同按同一因子递增。
    double external_load_scale{1.0};
    double internal_force_scale{1.0};
};

struct EnergyRates {
    double external_power{0.0};
    double road_power{0.0};
    double drive_power{0.0};
    double damper_dissipation{0.0};
    double friction_dissipation{0.0};
    double contact_dissipation{0.0};
};

struct EnergyStorage {
    double gravity{0.0};
    double spring{0.0};
    double stop{0.0};
    double bushing{0.0};
    double anti_roll{0.0};
    double tire_normal{0.0};
    double tire_brush{0.0};

    double total() const {
        return gravity+spring+stop+bushing+anti_roll+
            tire_normal+tire_brush;
    }
};

void external_force_vector(
    const Model& model, const State& state, const SampleInput& input,
    double gravity_x, double gravity_y, double gravity_z,
    std::vector<double>& tire_forces, std::vector<double>& tire_brush_derivatives,
    std::vector<double>& tire_output, double& potential, double& external_power,
    double& dissipation, std::vector<double>& generalized_force,
    std::vector<double>* spring_component_output = nullptr,
    std::vector<double>* bushing_component_output = nullptr,
    std::vector<double>* anti_roll_component_output = nullptr,
    const StaticContactOverride* static_contact = nullptr,
    EnergyRates* energy_rates = nullptr,
    EnergyStorage* energy_storage = nullptr,
    // When set, only the quantities the tire brush ODE needs are produced.
    // The caller must use nothing but `tire_brush_derivatives`; the returned
    // generalized force, energies and component outputs are not populated.
    bool brush_only = false,
    std::vector<Vec3>* force_workspace = nullptr,
    std::vector<Vec3>* torque_workspace = nullptr,
    // Newton residuals need force and tire-state derivatives, but do not need
    // public tire/component outputs or energy bookkeeping.  Keep force
    // assembly identical while omitting those observer-side calculations.
    bool dynamics_only = false) {
    const int n = static_cast<int>(model.bodies.size());
    std::vector<Vec3> local_force;
    std::vector<Vec3> local_torque;
    std::vector<Vec3>& force = force_workspace != nullptr
        ? *force_workspace : local_force;
    std::vector<Vec3>& torque = torque_workspace != nullptr
        ? *torque_workspace : local_torque;
    if (!brush_only) {
        force.assign(static_cast<std::size_t>(n), Vec3{});
        torque.assign(static_cast<std::size_t>(n), Vec3{});
    }
    potential = 0.0;
    external_power = 0.0;
    dissipation = 0.0;
    if (energy_rates) *energy_rates = EnergyRates{};
    if (energy_storage) *energy_storage = EnergyStorage{};
    const double external_load_scale = static_contact == nullptr
        ? 1.0 : static_contact->external_load_scale;
    const double internal_force_scale = static_contact == nullptr
        ? 1.0 : static_contact->internal_force_scale;
    const bool record_output = !brush_only && !dynamics_only;
    const bool record_energy = !dynamics_only;
    if (!brush_only) {
        for (int i = 0; i < n; ++i) {
            if (input.body_wrench.size() >= static_cast<std::size_t>(6 * n)) {
                force[i] = {
                    external_load_scale * input.body_wrench[static_cast<std::size_t>(6 * i)],
                    external_load_scale * input.body_wrench[static_cast<std::size_t>(6 * i + 1)],
                    external_load_scale * input.body_wrench[static_cast<std::size_t>(6 * i + 2)]
                };
                torque[i] = {
                    external_load_scale * input.body_wrench[static_cast<std::size_t>(6 * i + 3)],
                    external_load_scale * input.body_wrench[static_cast<std::size_t>(6 * i + 4)],
                    external_load_scale * input.body_wrench[static_cast<std::size_t>(6 * i + 5)]
                };
                if (record_energy) {
                    const double applied_power =
                        dot(force[i], state.v[i]) +
                        dot(torque[i], state.omega[i]);
                    external_power += applied_power;
                    if (energy_rates) {
                        energy_rates->external_power += applied_power;
                    }
                }
            }
        }
        for (const AerodynamicDrag& drag : model.aerodynamic_drags) {
            const Vec3 axis = normalized(rotate(
                state.q[drag.body], drag.forward_axis
            ));
            const Vec3 point_velocity = state_point_velocity(
                state, drag.body, drag.application_point
            );
            const double longitudinal_speed = dot(point_velocity, axis);
            const Vec3 drag_force = axis * (
                -external_load_scale * drag.coefficient
                * std::abs(longitudinal_speed)
                * longitudinal_speed
            );
            add_force_on_body(
                force, torque, model, state, drag.body,
                drag.application_point, drag_force
            );
            if (record_energy) {
                const double applied_power = dot(drag_force, point_velocity);
                external_power += applied_power;
                if (energy_rates) energy_rates->external_power += applied_power;
            }
        }
        for (int i = 0; i < n; ++i) {
            if (!model.bodies[i].fixed) {
                force[i].x += external_load_scale * model.bodies[i].mass * gravity_x;
                force[i].y += external_load_scale * model.bodies[i].mass * gravity_y;
                force[i].z += external_load_scale * model.bodies[i].mass * gravity_z;
                if (record_energy) {
                    const double gravity_energy = -model.bodies[i].mass * (
                        external_load_scale * gravity_x * state.r[i].x
                        + external_load_scale * gravity_y * state.r[i].y
                        + external_load_scale * gravity_z * state.r[i].z
                    );
                    potential += gravity_energy;
                    if (energy_storage) {
                        energy_storage->gravity += gravity_energy;
                    }
                }
            }
        }
    }
    tire_forces.assign(model.tires.size(), 0.0);
    tire_brush_derivatives.assign(model.tires.size() * 2, 0.0);
    if (record_output) {
        tire_output.assign(model.tires.size() * kTireOutputWidth, 0.0);
    }
    if (spring_component_output) {
        spring_component_output->assign(
            model.springs.size()*kSpringOutputWidth, 0.0
        );
    }
    if (bushing_component_output) {
        bushing_component_output->assign(
            model.bushings.size()*kBushingOutputWidth, 0.0
        );
    }
    if (anti_roll_component_output) {
        anti_roll_component_output->assign(
            model.anti_roll_bars.size()*kAntiRollOutputWidth, 0.0
        );
    }
    for (std::size_t i = 0; i < model.springs.size() && !brush_only; ++i) {
        const Spring& s = model.springs[i];
        const Vec3 pa = state_point(state, s.a, s.pa);
        const Vec3 pb = state_point(state, s.b, s.pb);
        const Vec3 va = state_point_velocity(state, s.a, s.pa);
        const Vec3 vb = state_point_velocity(state, s.b, s.pb);
        const Vec3 d = pb-pa;
        const double L = norm(d);
        if (L < 1e-10) continue;
        const Vec3 e = d/L;
        const double dL = dot(vb-va, e);
        const double compression = s.free_length - L;
        const double damping = dL < 0.0 ? s.c_compression : s.c_rebound;
        const bool has_elastic_curve = !s.elastic_deflection.empty();
        const double elastic_force = has_elastic_curve
            ? interpolate_curve(s.elastic_deflection, s.elastic_force, compression)
            : s.k*compression;
        // A measured curve gives the force directly; the constant-coefficient
        // form is the special case used when no curve is supplied.
        const bool has_curve = !s.damper_velocity.empty();
        const double damping_force = has_curve
            ? -interpolate_curve(s.damper_velocity, s.damper_force, dL)
            : -damping*dL;
        double compression_stop_elastic_force = 0.0;
        double compression_stop_damping_force = 0.0;
        double rebound_stop_elastic_force = 0.0;
        double rebound_stop_damping_force = 0.0;
        double fscalar = elastic_force + damping_force;
        // A measured curve carries a gas preload: a non-zero force at zero
        // velocity that is conservative, not dissipative.  Splitting it off
        // leaves the velocity-dependent remainder, whose power is what the
        // damper actually removes from the system.  Charging the preload to
        // dissipation would report energy input whenever the rod moves against
        // it, which is a bookkeeping error rather than a physical one.
        const double preload_force = has_curve
            ? -interpolate_curve(s.damper_velocity, s.damper_force, 0.0)
            : 0.0;
        if (record_energy) {
            const double damper_dissipation =
                has_curve ? -(damping_force - preload_force)*dL : damping*dL*dL;
            dissipation += damper_dissipation;
            if (energy_rates) {
                energy_rates->damper_dissipation += damper_dissipation;
            }
        }
        if (std::isfinite(s.minimum_length) && L < s.minimum_length) {
            const double penetration = s.minimum_length - L;
            compression_stop_elastic_force = s.compression_stop_penetration.empty()
                ? s.compression_stop_k * penetration
                : interpolate_curve(
                    s.compression_stop_penetration,
                    s.compression_stop_force,
                    penetration
                );
            if (dL < 0.0) {
                compression_stop_damping_force =
                    -s.compression_stop_c * dL;
            }
            fscalar += compression_stop_elastic_force
                + compression_stop_damping_force;
            if (record_energy && dL < 0.0) {
                const double stop_dissipation =
                    s.compression_stop_c*dL*dL;
                dissipation += stop_dissipation;
                if (energy_rates) {
                    energy_rates->damper_dissipation += stop_dissipation;
                }
            }
            if (record_energy) {
                const double stop_energy =
                    s.compression_stop_penetration.empty()
                        ? 0.5*s.compression_stop_k*penetration*penetration
                        : integrate_curve_from_zero(
                            s.compression_stop_penetration,
                            s.compression_stop_force,
                            penetration
                        );
                potential += stop_energy;
                if (energy_storage) energy_storage->stop += stop_energy;
            }
        }
        if (std::isfinite(s.maximum_length) && L > s.maximum_length) {
            const double penetration = L - s.maximum_length;
            rebound_stop_elastic_force = s.rebound_stop_penetration.empty()
                ? -s.rebound_stop_k * penetration
                : -interpolate_curve(
                    s.rebound_stop_penetration,
                    s.rebound_stop_force,
                    penetration
                );
            if (dL > 0.0) {
                rebound_stop_damping_force =
                    -s.rebound_stop_c * dL;
            }
            fscalar += rebound_stop_elastic_force
                + rebound_stop_damping_force;
            if (record_energy && dL > 0.0) {
                const double stop_dissipation =
                    s.rebound_stop_c*dL*dL;
                dissipation += stop_dissipation;
                if (energy_rates) {
                    energy_rates->damper_dissipation += stop_dissipation;
                }
            }
            if (record_energy) {
                const double stop_energy =
                    s.rebound_stop_penetration.empty()
                        ? 0.5*s.rebound_stop_k*penetration*penetration
                        : integrate_curve_from_zero(
                            s.rebound_stop_penetration,
                            s.rebound_stop_force,
                            penetration
                        );
                potential += stop_energy;
                if (energy_storage) energy_storage->stop += stop_energy;
            }
        }
        fscalar *= internal_force_scale;
        const Vec3 f = e*fscalar;
        add_force_on_body(force, torque, model, state, s.b, s.pb, f);
        add_force_on_body(force, torque, model, state, s.a, s.pa, f*(-1.0));
        if (record_energy) {
            const double spring_energy = has_elastic_curve
                ? integrate_curve_from_zero(
                    s.elastic_deflection, s.elastic_force, compression
                )
                : 0.5*s.k*compression*compression;
            potential += spring_energy;
            if (energy_storage) energy_storage->spring += spring_energy;
        }
        if (spring_component_output) {
            const std::size_t offset = i*kSpringOutputWidth;
            (*spring_component_output)[offset] = L;
            (*spring_component_output)[offset+1] = dL;
            (*spring_component_output)[offset+2] = elastic_force;
            (*spring_component_output)[offset+3] = damping_force;
            (*spring_component_output)[offset+4] =
                compression_stop_elastic_force;
            (*spring_component_output)[offset+5] =
                rebound_stop_elastic_force;
            (*spring_component_output)[offset+6] = fscalar;
        }
    }
    for (std::size_t bushing_index = 0;
         bushing_index < model.bushings.size() && !brush_only; ++bushing_index) {
        const Bushing& b = model.bushings[bushing_index];
        const Vec3 pa = state_point(state, b.a, b.pa);
        const Vec3 pb = state_point(state, b.b, b.pb);
        std::array<double, 6> rate{};
        const auto deformation = bushing_deformation(b, model, state, rate);
        auto elastic = mat6_mul(b.stiffness, deformation);
        for (int i = 0; i < 6; ++i) {
            const std::size_t axis = static_cast<std::size_t>(i);
            if (!b.elastic_coordinate[axis].empty()) {
                elastic[axis] = bushing_curve_value_slope(
                    b, axis, deformation[axis]
                ).first;
            }
        }
        const auto viscous = mat6_mul(b.damping, rate);
        std::array<double, 6> wrench_local{};
        for (int i = 0; i < 6; ++i) {
            wrench_local[i] = internal_force_scale * (
                b.preload[static_cast<std::size_t>(i)]
                - elastic[static_cast<std::size_t>(i)]
                - viscous[static_cast<std::size_t>(i)]
            );
        }
        const Quat qfa = qmul(state.q[b.a], b.frame_a);
        const Mat3 rfa = qmat(qfa);
        const Vec3 f_world = rfa * Vec3{wrench_local[0], wrench_local[1], wrench_local[2]};
        const Vec3 t_world = rfa * Vec3{wrench_local[3], wrench_local[4], wrench_local[5]};
        // FIELD reports the reaction wrench at the other marker.  The two
        // marker points are generally distinct, so the reaction torque must
        // also transfer the force through the marker-to-marker arm.
        const Vec3 marker_arm = pb-pa;
        add_force_on_body(force, torque, model, state, b.b, b.pb, f_world);
        add_force_on_body(force, torque, model, state, b.a, b.pa, f_world * (-1.0));
        add_torque_on_body(torque, model, b.b, t_world);
        add_torque_on_body(
            torque, model, b.a,
            (t_world+cross(marker_arm, f_world))*(-1.0)
        );
        if (record_energy) {
            double bushing_energy = 0.0;
            for (int i = 0; i < 6; ++i) {
                const std::size_t axis = static_cast<std::size_t>(i);
                bushing_energy += b.elastic_coordinate[axis].empty()
                    ? 0.5*deformation[axis]*elastic[axis]
                    : integrate_bushing_curve_from_zero(
                        b, axis, deformation[axis]
                    );
                bushing_energy -= b.preload[axis]*deformation[axis];
            }
            potential += bushing_energy;
            if (energy_storage) energy_storage->bushing += bushing_energy;
            for (int i = 0; i < 6; ++i) {
                const double bushing_dissipation =
                    rate[static_cast<std::size_t>(i)]
                    * viscous[static_cast<std::size_t>(i)];
                dissipation += bushing_dissipation;
                if (energy_rates) {
                    energy_rates->damper_dissipation +=
                        bushing_dissipation;
                }
            }
        }
        if (bushing_component_output) {
            const std::size_t offset =
                bushing_index*kBushingOutputWidth;
            for (int i = 0; i < 6; ++i) {
                (*bushing_component_output)[offset+i] =
                    deformation[static_cast<std::size_t>(i)];
                (*bushing_component_output)[offset+6+i] =
                    wrench_local[static_cast<std::size_t>(i)];
            }
        }
    }
    for (std::size_t bar_index = 0;
         bar_index < model.anti_roll_bars.size() && !brush_only; ++bar_index) {
        const AntiRollBar& bar = model.anti_roll_bars[bar_index];
        const Quat qrel = qmul(qconj(state.q[bar.a]), state.q[bar.b]);
        const Vec3 phi = qlog(qmul(qconj(bar.reference), qrel));
        const Vec3 axis_world = normalized(rotate(state.q[bar.a], bar.axis_a));
        const Vec3 axis_a_world = rotate(state.q[bar.a], bar.axis_a);
        const double angle = dot(phi, bar.axis_a);
        const double rate = dot(axis_a_world, state.omega[bar.b] - state.omega[bar.a]);
        const double tau = internal_force_scale * (
            -bar.stiffness * angle - bar.damping * rate
        );
        add_torque_on_body(torque, model, bar.b, axis_world * tau);
        add_torque_on_body(torque, model, bar.a, axis_world * (-tau));
        if (record_energy) {
            const double anti_roll_energy =
                0.5 * bar.stiffness * angle * angle;
            potential += anti_roll_energy;
            if (energy_storage) {
                energy_storage->anti_roll += anti_roll_energy;
            }
            const double bar_dissipation = bar.damping * rate * rate;
            dissipation += bar_dissipation;
            if (energy_rates) {
                energy_rates->damper_dissipation += bar_dissipation;
            }
        }
        if (anti_roll_component_output) {
            const std::size_t offset =
                bar_index*kAntiRollOutputWidth;
            (*anti_roll_component_output)[offset] = angle;
            (*anti_roll_component_output)[offset+1] = rate;
            (*anti_roll_component_output)[offset+2] = tau;
        }
    }
    for (std::size_t steering_index = 0;
         steering_index < model.steering_actuators.size() && !brush_only;
         ++steering_index) {
        const SteeringActuator& actuator =
            model.steering_actuators[steering_index];
        const int reaction = actuator.reaction_body;
        const double target = steering_index < input.steering_target.size()
            ? input.steering_target[steering_index] : 0.0;
        const double target_rate =
            steering_index < input.steering_target_rate.size()
                ? input.steering_target_rate[steering_index] : 0.0;
        if (prescribed_steering(actuator)) continue;
        if (actuator.type == VEHICLE_STEERING_TRANSLATION) {
            const Vec3 body_point = state_point(
                state, actuator.body, actuator.point_local
            );
            const Vec3 reaction_point = reaction >= 0
                ? state_point(state, reaction, actuator.reaction_point_local)
                : Vec3{};
            const Vec3 body_velocity = state_point_velocity(
                state, actuator.body, actuator.point_local
            );
            const Vec3 reaction_velocity = reaction >= 0
                ? state_point_velocity(
                    state, reaction, actuator.reaction_point_local
                )
                : Vec3{};
            const Vec3 axis_world = normalized(
                reaction >= 0
                    ? rotate(state.q[reaction], actuator.axis_local)
                    : actuator.axis_local
            );
            const Vec3 relative_position = body_point - reaction_point;
            const Vec3 relative_velocity = body_velocity - reaction_velocity;
            const double displacement = dot(axis_world, relative_position);
            const double rate = dot(axis_world, relative_velocity);
            const double force_value = actuator.stiffness * (target-displacement)
                + actuator.damping * (target_rate-rate);
            const Vec3 force_value_world = axis_world * force_value;
            add_force_on_body(
                force, torque, model, state, actuator.body,
                actuator.point_local, force_value_world
            );
            add_force_on_body(
                force, torque, model, state, reaction,
                actuator.reaction_point_local, force_value_world * (-1.0)
            );
            if (record_energy) {
                const double steering_power =
                    dot(force_value_world, relative_velocity);
                external_power += steering_power;
                if (energy_rates) {
                    energy_rates->external_power += steering_power;
                }
            }
        } else {
            const Quat reaction_q = reaction >= 0
                ? state.q[reaction] : Quat{};
            const Vec3 axis_reference =
                rotate(actuator.reference, actuator.axis_local);
            const Quat relative = qmul(
                qconj(reaction_q), state.q[actuator.body]
            );
            const Vec3 error_rotation = qlog(
                qmul(qconj(actuator.reference), relative)
            );
            const Vec3 axis_world = normalized(
                rotate(reaction_q, axis_reference)
            );
            const double angle = dot(error_rotation, axis_reference);
            const Vec3 relative_omega = state.omega[actuator.body]
                - (reaction >= 0 ? state.omega[reaction] : Vec3{});
            const double rate = dot(axis_world, relative_omega);
            const double torque_value = actuator.stiffness * (target-angle)
                + actuator.damping * (target_rate-rate);
            const Vec3 torque_value_world = axis_world * torque_value;
            add_torque_on_body(
                torque, model, actuator.body, torque_value_world
            );
            add_torque_on_body(
                torque, model, reaction, torque_value_world * (-1.0)
            );
            if (record_energy) {
                const double steering_power =
                    dot(torque_value_world, relative_omega);
                external_power += steering_power;
                if (energy_rates) {
                    energy_rates->external_power += steering_power;
                }
            }
        }
    }
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& t = model.tires[i];
        const int frame_body = tire_frame_body(t);
        const int center_body = tire_center_body(t);
        const Vec3 center_local = tire_center_local(t);
        const Vec3 center = state_point(state, center_body, center_local);
        const Vec3 vc = state_point_velocity(state, center_body, center_local);
        const double road_height =
            (i < input.road_z.size() ? input.road_z[i] : 0.0)
            + road_profile_height(model, state, i);
        const double road_slope = road_profile_slope(model, state, i);
        const double road_v =
            (i < input.road_v.size() ? input.road_v[i] : 0.0)
            + road_slope * vc.x;
        const Vec3 normal{0.0, 0.0, 1.0};
        double delta = t.radius + road_height - center.z;
        const double sx = i < state.tire_sx.size() ? state.tire_sx[i] : 0.0;
        const double sy = i < state.tire_sy.size() ? state.tire_sy[i] : 0.0;
        double delta_dot = road_v - vc.z;
        Vec3 forward = rotate(state.q[frame_body], t.forward_axis);
        forward.z = 0.0;
        forward = normalized(forward);
        if (norm(forward) <= kEps) {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            tire_forces[i] = nan;
            tire_brush_derivatives[2*i] = nan;
            tire_brush_derivatives[2*i+1] = nan;
            continue;
        }
        const Vec3 lateral = normalized(cross(normal, forward));
        const Vec3 spin_axis = normalized(
            rotate(state.q[frame_body], t.spin_axis)
        );
        const bool pac2002 =
            t.model_kind == VEHICLE_TIRE_PAC2002_PURE_SLIP
            || t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE;
        const bool fiala = t.model_kind == VEHICLE_TIRE_FIALA;
        // Adams reports Fiala wheel spin relative to the upright.  The
        // carrier angular velocity must not enter the rolling speed.
        const Vec3 spin_omega = fiala
            ? tire_relative_spin_omega(state, t)
            : state.omega[t.body];
        const double spin_rate = dot(spin_omega, spin_axis);
        double loaded_radius = std::max(
            t.radius-std::max(delta, 0.0), 1e-9
        );
        if (fiala && static_contact == nullptr) {
            // The default Adams Fiala contact is the intersection of the
            // wheel plane and the road plane. Its radial loaded radius is
            // therefore the vertical gap divided by the radial direction's
            // vertical projection, not the vertical gap itself.
            const double axis_normal = dot(spin_axis, normal);
            const double vertical_projection = std::max(
                std::sqrt(std::max(1.0-axis_normal*axis_normal, 0.0)),
                1e-9
            );
            const Vec3 axis_rate = cross(state.omega[frame_body], spin_axis);
            const double axis_normal_rate = dot(axis_rate, normal);
            const double projection_rate = -axis_normal*axis_normal_rate
                / vertical_projection;
            const double vertical_gap = center.z-road_height;
            const double vertical_gap_rate = vc.z-road_v;
            loaded_radius = std::max(
                vertical_gap/vertical_projection,
                1e-9
            );
            delta = t.radius-loaded_radius;
            delta_dot = -(
                vertical_gap_rate/vertical_projection
                - vertical_gap*projection_rate
                    /(vertical_projection*vertical_projection)
            );
        }
        const double rolling_radius = t.model_kind ==
            VEHICLE_TIRE_PAC2002_ADAMS_SOURCE
            ? pac2002_effective_rolling_radius(t, delta, spin_rate)
            : ((pac2002 || fiala) ? loaded_radius : t.radius);
        // Adams primitive brush 用未加载半径把 GFORCE 力施加在轮心下方；
        // PAC2002 保留其加载半径接触点，滚动速度另用有效滚动半径。
        const double force_application_radius = pac2002 || fiala
            ? loaded_radius
            : t.radius;
        Vec3 patch_arm = normal * (-force_application_radius);
        if ((pac2002 || fiala) && static_contact == nullptr) {
            // A cambered tire intersects the road along its radial plane, not
            // directly below the wheel center. loaded_radius is the radial
            // wheel-plane loaded radius; divide it by the radial direction's
            // vertical projection to keep the floating contact point on the
            // road, as used by the Adams tire FMA marker.
            Vec3 radial_down = normalized(
                (normal-spin_axis*dot(spin_axis, normal))*(-1.0)
            );
            const double vertical_projection = std::max(
                -dot(radial_down, normal), 1e-9
            );
            patch_arm = radial_down * (
                force_application_radius/vertical_projection
            );
        }
        const Vec3 rolling_arm = (pac2002 || fiala)
            ? patch_arm
            : normal * (-rolling_radius);
        const Vec3 patch_velocity = vc + cross(
            state.omega[t.body], rolling_arm
        );
        const Vec3 road_velocity{0.0, 0.0, road_v};
        const Vec3 relative_patch_velocity = patch_velocity - road_velocity;
        // Adams PAC2002 defines longitudinal slip from the wheel-center
        // forward speed and the scalar wheel spin times effective radius.
        // Using omega x a world-vertical arm introduces an erroneous
        // cos(camber) factor into the rolling speed.
        const double vx = (t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE
            || fiala)
            ? dot(vc-road_velocity, forward)-spin_rate*rolling_radius
            : dot(relative_patch_velocity, forward);
        const double vy = dot(relative_patch_velocity, lateral);
        const std::size_t output_offset = i*kTireOutputWidth;
        if (record_output) {
            tire_output[output_offset+0] = 0.0;
            tire_output[output_offset+1] = -delta;
            tire_output[output_offset+2] = std::max(0.0, delta);
            tire_output[output_offset+3] = -delta_dot;
            tire_output[output_offset+7] = vx;
            tire_output[output_offset+8] = vy;
            tire_output[output_offset+10] = sx;
            tire_output[output_offset+11] = sy;
        }
        const bool static_active =
            static_contact != nullptr && static_contact->active != nullptr &&
            static_contact->compression != nullptr &&
            i < static_contact->active->size() &&
            (*static_contact->active)[i] != 0;
        const double static_delta =
            static_active && i < static_contact->compression->size()
                ? (*static_contact->compression)[i]
                : delta;
        if (!static_active && (static_contact != nullptr || delta <= 0.0)) {
            tire_brush_derivatives[2*i] = -sx / t.detached_relaxation;
            tire_brush_derivatives[2*i+1] = -sy / t.detached_relaxation;
            continue;
        }
        const double fn = static_active
            ? internal_force_scale * (pac2002
                ? pac2002_vertical_force(t, static_delta, 0.0, 0.0)
                : std::max(0.0, t.k*static_delta))
            : pac2002
                ? pac2002_vertical_force(t, delta, delta_dot, 0.0)
                : std::max(0.0, t.k*delta+t.c*delta_dot);
        tire_forces[i] = fn;
        if (fn <= 0.0) {
            tire_brush_derivatives[2*i] = -sx / t.detached_relaxation;
            tire_brush_derivatives[2*i+1] = -sy / t.detached_relaxation;
            if (brush_only) continue;
            if (record_energy) {
                const double normal_energy = 0.5*t.k*delta*delta;
                potential += normal_energy;
                if (energy_storage) {
                    energy_storage->tire_normal += normal_energy;
                }
            }
            continue;
        }
        if (static_active) {
            if (brush_only) continue;
            const Mat3 body_rotation = qmat(state.q[t.body]);
            // During static trim the wheel center is constrained to the
            // carrier center. Expressing this equivalent contact point on
            // the force body keeps the KKT extension consistent off the
            // constraint manifold; the converged equilibrium is unchanged.
            const Vec3 contact_center = state_point(state, t.body, t.center);
            const Vec3 contact_point = contact_center + patch_arm;
            const Vec3 contact_local = transpose(body_rotation) * (
                contact_point - state.r[t.body]
            );
            add_force_on_body(
                force, torque, model, state, t.body, contact_local,
                normal * fn
            );
            if (record_output) {
                tire_output[output_offset+0] = 1.0;
                tire_output[output_offset+4] = fn;
                tire_output[output_offset+9] = 0.0;
            }
            if (record_energy) {
                const double normal_energy =
                    0.5*t.k*static_delta*static_delta;
                potential += normal_energy;
                if (energy_storage) {
                    energy_storage->tire_normal += normal_energy;
                }
            }
            continue;
        }
        const double rolling_speed = std::abs(dot(vc, forward));
        if (t.model_kind == VEHICLE_TIRE_FIALA) {
            const double slip_speed = std::max(
                rolling_speed,
                std::max(fiala_parameter(t, FIALA_LOW_SPEED_THRESHOLD, 1e-3), 1e-3)
            );
            const double longitudinal_slip = std::clamp(-vx/slip_speed, -1.0, 1.0);
            const double lateral_slip = std::clamp(
                std::atan2(vy, slip_speed), -0.5*kPi+0.01, 0.5*kPi-0.01
            );
            const double relax_x = std::max(
                fiala_parameter(t, FIALA_RELAX_LENGTH_X, t.relaxation_length_longitudinal), 1e-6
            );
            const double relax_y = std::max(
                fiala_parameter(t, FIALA_RELAX_LENGTH_Y, t.relaxation_length_lateral), 1e-6
            );
            tire_brush_derivatives[2*i] = rolling_speed/relax_x*(longitudinal_slip-sx);
            tire_brush_derivatives[2*i+1] = rolling_speed/relax_y*(lateral_slip-sy);
            if (brush_only) continue;
            double fx = 0.0, fy = 0.0, aligning_moment = 0.0;
            fiala_forces(t, sx, sy, fn, fx, fy, aligning_moment);
            const double umax = std::max(fiala_parameter(t, FIALA_UMAX, 1.0), 1e-9);
            const double utilization = std::sqrt(
                std::pow(fx/(umax*fn), 2.0)+std::pow(fy/(umax*fn), 2.0)
            );
            const Mat3 body_rotation = qmat(state.q[t.body]);
            const Vec3 contact_point = center + patch_arm;
            const Vec3 contact_local = transpose(body_rotation) * (contact_point-state.r[t.body]);
            add_force_on_body(force, torque, model, state, t.body, contact_local,
                forward*fx+lateral*fy+normal*fn);
            add_torque_on_body(torque, model, t.body,
                normal*aligning_moment + lateral*(-fiala_parameter(t, FIALA_ROLLING_RESISTANCE, 0.0)*fn));
            if (record_output) {
                tire_output[output_offset+0] = 1.0;
                tire_output[output_offset+4] = fn;
                tire_output[output_offset+5] = fx;
                tire_output[output_offset+6] = fy;
                tire_output[output_offset+9] = utilization;
                tire_output[output_offset+10] = sx;
                tire_output[output_offset+11] = sy;
                tire_output[output_offset+14] = aligning_moment;
            }
            continue;
        }
        if (t.model_kind == VEHICLE_TIRE_PAC2002_PURE_SLIP
            || t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
            // Adams 高性能轮胎把 USE_MODE=14 的松弛状态保存在局部轮胎
            // 求解器内；input array 状态数为 0 只表示不通过 GSE 暴露状态。
            const double slip_speed = std::max(rolling_speed, 1e-3);
            const double longitudinal_slip = std::clamp(
                -vx/slip_speed, -1.0, 1.0
            );
            const double lateral_slip = std::clamp(
                std::atan2(vy, slip_speed),
                -0.5*kPi+0.01, 0.5*kPi-0.01
            );
            const Vec3 spin_axis = normalized(
                rotate(state.q[frame_body], t.spin_axis)
            );
            const double camber = std::clamp(
                std::atan2(
                    dot(spin_axis, normal), dot(spin_axis, lateral)
                ),
                -kPac2002CamberLimit,
                kPac2002CamberLimit
            );
            const double effective_longitudinal_slip = sx;
            const double effective_lateral_slip = sy;
            const double relaxation_length_longitudinal =
                pac2002_relaxation_length(t, fn, false, camber);
            const double relaxation_length_lateral =
                pac2002_relaxation_length(t, fn, true, camber);
            const double longitudinal_relaxation =
                rolling_speed / relaxation_length_longitudinal;
            const double lateral_relaxation =
                rolling_speed / relaxation_length_lateral;
            tire_brush_derivatives[2*i] = longitudinal_relaxation
                *(longitudinal_slip-sx);
            tire_brush_derivatives[2*i+1] = lateral_relaxation
                *(lateral_slip-sy);
            if (brush_only) continue;
            if (record_output) {
                tire_output[output_offset+10] = effective_longitudinal_slip;
                tire_output[output_offset+11] = effective_lateral_slip;
            }
            double fx = pac2002_pure_force(
                t, effective_longitudinal_slip, fn, false, camber
            );
            double fy = pac2002_pure_force(
                t, effective_lateral_slip, fn, true, camber
            );
            double aligning_moment = 0.0;
            double overturning_moment = 0.0;
            double rolling_resistance_moment = 0.0;
            double utilization = 0.0;
            if (pac2002_has_combined_slip_terms(t)) {
                // 源 PAC2002 参数包含 RBX/RBY/RVY 联合滑移项时，使用
                // Pacejka 联合滑移缩放；未提供这些项的模型仍使用原有
                // 摩擦椭圆，避免用默认零值制造新的力律。
                fx = pac2002_combined_longitudinal_force(
                    t, effective_longitudinal_slip,
                    effective_lateral_slip, fn, camber, fx
                );
                fy = pac2002_combined_lateral_force(
                    t, effective_longitudinal_slip,
                    effective_lateral_slip, fn, camber, fy
                );
                // 联合滑移路径不再用摩擦椭圆截断力，但仍报告相对
                // PAC 峰值的实际利用率，便于识别源参数造成的过载。
                const double limit_x = pac2002_force_limit(
                    t, fn, false, camber
                );
                const double limit_y = pac2002_force_limit(
                    t, fn, true, camber
                );
                utilization = std::sqrt(
                    std::pow(fx/limit_x, 2)+std::pow(fy/limit_y, 2)
                );
            } else {
                const double limit_x = pac2002_force_limit(
                    t, fn, false, camber
                );
                const double limit_y = pac2002_force_limit(
                    t, fn, true, camber
                );
                utilization = std::sqrt(
                    std::pow(fx/limit_x, 2)+std::pow(fy/limit_y, 2)
                );
                if (utilization > 1.0) {
                    fx /= utilization;
                    fy /= utilization;
                }
            }
            aligning_moment = pac2002_aligning_moment(
                t, effective_longitudinal_slip,
                effective_lateral_slip, fn, camber, fx, fy
            );
            if (t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
                aligning_moment += pac2002_gyroscopic_moment(
                    t, fn, camber, effective_lateral_slip,
                    tire_brush_derivatives[2*i+1], rolling_radius, spin_rate
                );
            }
            overturning_moment = pac2002_overturning_moment(
                t, fy, fn, camber
            );
            rolling_resistance_moment = pac2002_rolling_resistance_moment(
                t, fx, fn, camber, rolling_speed
            );
            // Adams reports the rolling moment at the loaded contact
            // reference while PAC2002 evaluates the rolling resistance at
            // the effective rolling radius.  Transfer the longitudinal
            // force between those two radii before applying the ISO moment.
            if (t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
                rolling_resistance_moment +=
                    -fx*(rolling_radius-loaded_radius);
            }
            const Mat3 body_rotation = qmat(state.q[t.body]);
            const Vec3 contact_point = center + patch_arm;
            const Vec3 contact_local = transpose(body_rotation) * (
                contact_point - state.r[t.body]
            );
            const Vec3 contact_force = forward*fx + lateral*fy + normal*fn;
            add_force_on_body(
                force, torque, model, state, t.body, contact_local,
                contact_force
            );
            add_torque_on_body(
                torque, model, t.body,
                forward*overturning_moment
                    +lateral*rolling_resistance_moment
                    +normal*aligning_moment
            );
            if (record_output) {
                tire_output[output_offset+0] = 1.0;
                tire_output[output_offset+4] = fn;
                tire_output[output_offset+5] = fx;
                tire_output[output_offset+6] = fy;
                tire_output[output_offset+9] = utilization;
                tire_output[output_offset+12] = overturning_moment;
                tire_output[output_offset+13] = rolling_resistance_moment;
                tire_output[output_offset+14] = aligning_moment;
            }
            if (record_energy) {
                const double normal_energy = 0.5*t.k*delta*delta;
                potential += normal_energy;
                if (energy_storage) {
                    energy_storage->tire_normal += normal_energy;
                }
            }
            if (record_energy && fn > 0.0) {
                const double contact_dissipation =
                    t.c*delta_dot*delta_dot;
                dissipation += contact_dissipation;
                if (energy_rates) {
                    energy_rates->contact_dissipation +=
                        contact_dissipation;
                }
                const double friction_dissipation = std::max(
                    0.0, -(fx*vx+fy*vy)
                );
                dissipation += friction_dissipation;
                if (energy_rates) {
                    energy_rates->friction_dissipation +=
                        friction_dissipation;
                }
                const double road_power = fn*road_v;
                external_power += road_power;
                if (energy_rates) energy_rates->road_power += road_power;
            }
            continue;
        }
        const double rolling_relaxation_longitudinal =
            rolling_speed / t.relaxation_length_longitudinal;
        const double rolling_relaxation_lateral =
            rolling_speed / t.relaxation_length_lateral;
        tire_brush_derivatives[2*i] =
            vx - rolling_relaxation_longitudinal * sx;
        tire_brush_derivatives[2*i+1] =
            vy - rolling_relaxation_lateral * sy;
        if (brush_only) continue;
        double projected_sx = sx;
        double projected_sy = sy;
        double trial_utilization = 0.0;
        project_brush_state(
            t, fn, sx, sy, projected_sx, projected_sy, trial_utilization
        );
        const double fx = -t.brush_k_longitudinal * projected_sx;
        const double fy = -t.brush_k_lateral * projected_sy;
        const double utilization = std::sqrt(
            std::pow(fx/(t.mu_longitudinal*fn), 2)
            + std::pow(fy/(t.mu_lateral*fn), 2)
        );
        const Mat3 body_rotation = qmat(state.q[t.body]);
        const Vec3 contact_point = center + patch_arm;
        const Vec3 contact_local = transpose(body_rotation) * (
            contact_point - state.r[t.body]
        );
        const Vec3 contact_force = forward * fx + lateral * fy + normal * fn;
        add_force_on_body(force, torque, model, state, t.body, contact_local, contact_force);
        if (record_output) {
            tire_output[output_offset+0] = 1.0;
            tire_output[output_offset+4] = fn;
            tire_output[output_offset+5] = fx;
            tire_output[output_offset+6] = fy;
            tire_output[output_offset+9] = utilization;
        }
        if (record_energy) {
            const double normal_energy = 0.5*t.k*delta*delta;
            const double brush_energy =
                0.5*t.brush_k_longitudinal*projected_sx*projected_sx
                +0.5*t.brush_k_lateral*projected_sy*projected_sy;
            potential += normal_energy+brush_energy;
            if (energy_storage) {
                energy_storage->tire_normal += normal_energy;
                energy_storage->tire_brush += brush_energy;
            }
        }
        if (record_energy && fn > 0.0) {
            const double contact_dissipation =
                t.c * delta_dot * delta_dot;
            dissipation += contact_dissipation;
            if (energy_rates) {
                energy_rates->contact_dissipation +=
                    contact_dissipation;
            }
            const double brush_energy_rate =
                t.brush_k_longitudinal*projected_sx*tire_brush_derivatives[2*i]
                + t.brush_k_lateral*projected_sy*tire_brush_derivatives[2*i+1];
            const double friction_dissipation =
                std::max(
                    0.0,
                    -(fx*vx + fy*vy) - brush_energy_rate
                );
            dissipation += friction_dissipation;
            if (energy_rates) {
                energy_rates->friction_dissipation +=
                    friction_dissipation;
            }
            const double road_power = fn * road_v;
            external_power += road_power;
            if (energy_rates) {
                energy_rates->road_power += road_power;
            }
        }
        (void)trial_utilization;
    }
    for (std::size_t i = 0; i < model.tires.size() && !brush_only; ++i) {
        const Tire& t = model.tires[i];
        const int frame_body = tire_frame_body(t);
        const Vec3 tire_axis = normalized(
            rotate(state.q[frame_body], t.spin_axis)
        );
        const double axial_rate = dot(
            tire_axis, tire_relative_spin_omega(state, t)
        );
        const double drive_torque = i < input.torque.size()
            ? input.torque[i] : 0.0;
        const bool mapped_drive = t.drive_torque_body >= 0;
        const int drive_body = mapped_drive ? t.drive_torque_body : t.body;
        const Vec3 drive_axis = mapped_drive
            ? normalized(rotate(
                state.q[drive_body], t.drive_torque_axis
            ))
            : tire_axis;
        add_torque_on_body(
            torque, model, drive_body, drive_axis*drive_torque
        );
        add_torque_on_body(
            torque, model, t.drive_torque_reaction_body,
            drive_axis*(-drive_torque)
        );
        const double drive_rate = mapped_drive
            ? dot(
                drive_axis,
                state.omega[drive_body]
                    - (t.drive_torque_reaction_body >= 0
                        ? state.omega[t.drive_torque_reaction_body]
                        : Vec3{})
            )
            : axial_rate;

        const double brake_magnitude = i < input.brake_torque.size()
            ? input.brake_torque[i] : 0.0;
        double brake_torque = 0.0;
        if (brake_magnitude > 0.0) {
            if (axial_rate > kEps) {
                brake_torque = -brake_magnitude;
            } else if (axial_rate < -kEps) {
                brake_torque = brake_magnitude;
            }
        }
        add_torque_on_body(
            torque, model, t.body, tire_axis*brake_torque
        );
        if (record_energy) {
            const double actuator_power =
                drive_torque*drive_rate + brake_torque*axial_rate;
            external_power += actuator_power;
            if (energy_rates) {
                energy_rates->drive_power += actuator_power;
            }
        }
    }
    if (!brush_only) {
        generalized_force.assign(static_cast<std::size_t>(model.ndof), 0.0);
        for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
            const int bi = model.free_body[fi];
            const Body& b = model.bodies[bi];
            const Mat3 R = qmat(state.q[bi]);
            const Mat3 Iw = R * b.inertia_body * transpose(R);
            const Vec3 gyro = cross(state.omega[bi], Iw*state.omega[bi]);
            const Vec3 F = force[bi];
            const Vec3 T = torque[bi];
            generalized_force[6*fi] = F.x;
            generalized_force[6*fi+1] = F.y;
            generalized_force[6*fi+2] = F.z;
            generalized_force[6*fi+3] = T.x - gyro.x;
            generalized_force[6*fi+4] = T.y - gyro.y;
            generalized_force[6*fi+5] = T.z - gyro.z;
        }
    }
}

double pac2002_parameter(
    const Tire& tire, int index, double fallback
) {
    const double value = tire.pac2002_parameters[
        static_cast<std::size_t>(index)
    ];
    return std::isfinite(value) ? value : fallback;
}

double fiala_parameter(const Tire& tire, int index, double fallback) {
    const double value = tire.pac2002_parameters[static_cast<std::size_t>(index)];
    return std::isfinite(value) ? value : fallback;
}

void fiala_forces(
    const Tire& tire, double kappa, double alpha, double normal_force,
    double& fx, double& fy, double& mz
) {
    fx = fy = mz = 0.0;
    if (normal_force <= 0.0) return;
    const double cslip = std::max(fiala_parameter(tire, FIALA_CSLIP, 1000.0), 1e-9);
    const double calpha = std::max(fiala_parameter(tire, FIALA_CALPHA, 800.0), 1e-9);
    const double umin = std::max(fiala_parameter(tire, FIALA_UMIN, 0.9), 0.0);
    const double umax = std::max(fiala_parameter(tire, FIALA_UMAX, 1.0), umin);
    const double tan_alpha = std::tan(alpha);
    const double ss = std::min(1.0, std::sqrt(kappa*kappa + tan_alpha*tan_alpha));
    const double uf = (umax-(umax-umin)*ss)*normal_force;
    if (uf <= 0.0) return;
    const double sign_k = kappa < 0.0 ? -1.0 : (kappa > 0.0 ? 1.0 : 0.0);
    const double abs_k = std::abs(kappa);
    if (abs_k <= uf/(2.0*cslip)) {
        fx = cslip*kappa;
    } else if (sign_k != 0.0) {
        fx = sign_k*(uf-(uf*uf)/(4.0*abs_k*cslip));
    }
    const double critical_alpha = std::atan(3.0*uf/calpha);
    const double sign_alpha = alpha < 0.0 ? -1.0 : (alpha > 0.0 ? 1.0 : 0.0);
    if (std::abs(alpha) <= critical_alpha) {
        const double h = 1.0-calpha*std::abs(tan_alpha)/(3.0*uf);
        fy = -uf*(1.0-h*h*h)*sign_alpha;
        mz = uf*fiala_parameter(tire, FIALA_WIDTH, 0.235)*(1.0-h)*h*h*h*sign_alpha;
    } else {
        fy = -uf*sign_alpha;
    }
}

double pac2002_sign(double value) {
    return value < 0.0 ? -1.0 : 1.0;
}

double pac2002_positive_scale(
    const Tire& tire, int index, double fallback
) {
    const double value = pac2002_parameter(tire, index, fallback);
    return value > 0.0 ? value : fallback;
}

double pac2002_reference_load(const Tire& tire) {
    return std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0)
            *pac2002_positive_scale(tire, PAC_LFZO, 1.0),
        1e-9
    );
}

double pac2002_load_difference(const Tire& tire, double normal_force) {
    const double reference_load = pac2002_reference_load(tire);
    return (normal_force-reference_load)/reference_load;
}

double pac2002_pressure_difference(const Tire& tire) {
    const double nominal_pressure = std::max(
        pac2002_parameter(tire, PAC_IP_NOM, 200000.0)
            *pac2002_positive_scale(tire, PAC_LIP, 1.0),
        1e-9
    );
    return (
        pac2002_parameter(tire, PAC_IP, 200000.0)-nominal_pressure
    )/nominal_pressure;
}

double pac2002_peak_force(
    const Tire& tire, double normal_force, bool lateral, double camber
) {
    if (normal_force <= 0.0) return 0.0;
    const double dfz = pac2002_load_difference(tire, normal_force);
    const double dpi = pac2002_pressure_difference(tire);
    if (lateral) {
        const double gamma_y = camber
            *pac2002_parameter(tire, PAC_LGAY, 1.0);
        const double camber_factor =
            1.0 + pac2002_parameter(tire, PAC_PDY3, 0.0)*gamma_y*gamma_y;
        const double mu =
            (pac2002_parameter(tire, PAC_PDY1, 1.0)
                + pac2002_parameter(tire, PAC_PDY2, 0.0)*dfz)
            *(1.0+pac2002_parameter(tire, PAC_PPY3, 0.0)*dpi
                +pac2002_parameter(tire, PAC_PPY4, 0.0)*dpi*dpi)
            *camber_factor*pac2002_parameter(tire, PAC_LMUY, 1.0);
        return std::abs(mu*normal_force);
    }
    const double gamma_x = camber
        *pac2002_parameter(tire, PAC_LGAX, 1.0);
    const double camber_factor =
        1.0-pac2002_parameter(tire, PAC_PDX3, 0.0)*gamma_x*gamma_x;
    const double mu =
        (pac2002_parameter(tire, PAC_PDX1, 1.0)
            + pac2002_parameter(tire, PAC_PDX2, 0.0)*dfz)
        *(1.0+pac2002_parameter(tire, PAC_PPX3, 0.0)*dpi
            +pac2002_parameter(tire, PAC_PPX4, 0.0)*dpi*dpi)
        *camber_factor*pac2002_parameter(tire, PAC_LMUX, 1.0);
    return std::abs(mu*normal_force);
}

double pac2002_pure_force(
    const Tire& tire, double slip, double normal_force, bool lateral,
    double camber
) {
    if (normal_force <= 0.0) return 0.0;
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double reference_load = pac2002_reference_load(tire);
    const double dfz = pac2002_load_difference(tire, normal_force);
    const double dpi = pac2002_pressure_difference(tire);
    const double c = lateral
        ? pac2002_parameter(tire, PAC_PCY1, 1.3)
            *pac2002_parameter(tire, PAC_LCY, 1.0)
        : pac2002_parameter(tire, PAC_PCX1, 1.65)
            *pac2002_parameter(tire, PAC_LCX, 1.0);
    const double d = pac2002_peak_force(tire, normal_force, lateral, camber);
    if (d <= 0.0 || c <= 0.0) return 0.0;
    double stiffness = 0.0;
    double e = 0.0;
    double sh = 0.0;
    double sv = 0.0;
    if (lateral) {
        const double gamma_y = camber
            *pac2002_parameter(tire, PAC_LGAY, 1.0);
        const double pky2 = pac2002_parameter(tire, PAC_PKY2, 0.0);
        const double ky0 = std::abs(pky2) > 1e-12
            ? pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
                * nominal
                *(1.0+pac2002_parameter(tire, PAC_PPY1, 0.0)*dpi)
                * std::sin(2.0*std::atan(normal_force/(
                    pky2*reference_load
                    *(1.0+pac2002_parameter(tire, PAC_PPY2, 0.0)*dpi)
                )))
                *pac2002_parameter(tire, PAC_LFZO, 1.0)
                *pac2002_parameter(tire, PAC_LMUY, 1.0)
            : normal_force
                * pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0);
        stiffness = ky0 * (
            1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)*std::abs(gamma_y)
        );
        e = (pac2002_parameter(tire, PAC_PEY1, 0.0)
            + pac2002_parameter(tire, PAC_PEY2, 0.0)*dfz)
            *pac2002_parameter(tire, PAC_LEY, 1.0);
        e *= 1.0-
            (pac2002_parameter(tire, PAC_PEY3, 0.0)
                + pac2002_parameter(tire, PAC_PEY4, 0.0)*gamma_y)
            *pac2002_sign(
                slip
                +(pac2002_parameter(tire, PAC_PHY1, 0.0)
                    +pac2002_parameter(tire, PAC_PHY2, 0.0)*dfz)
                    *pac2002_parameter(tire, PAC_LHY, 1.0)
                +pac2002_parameter(tire, PAC_PHY3, 0.0)*gamma_y
                    *pac2002_parameter(tire, PAC_LKYG, 1.0)
            );
        sh = (pac2002_parameter(tire, PAC_PHY1, 0.0)
            + pac2002_parameter(tire, PAC_PHY2, 0.0)*dfz
            )*pac2002_parameter(tire, PAC_LHY, 1.0)
            +pac2002_parameter(tire, PAC_PHY3, 0.0)*gamma_y
                *pac2002_parameter(tire, PAC_LKYG, 1.0);
        sv = normal_force * (
            pac2002_parameter(tire, PAC_PVY1, 0.0)
                + pac2002_parameter(tire, PAC_PVY2, 0.0)*dfz
                + (
                    pac2002_parameter(tire, PAC_PVY3, 0.0)
                    + pac2002_parameter(tire, PAC_PVY4, 0.0)*dfz
                )*gamma_y*pac2002_parameter(tire, PAC_LKYG, 1.0)
        )*pac2002_parameter(tire, PAC_LVY, 1.0)
            *pac2002_parameter(tire, PAC_LMUY, 1.0);
    } else {
        const double gamma_x = camber
            *pac2002_parameter(tire, PAC_LGAX, 1.0);
        stiffness = normal_force * (
            pac2002_parameter(tire, PAC_PKX1, 120000.0/4850.0)
                + pac2002_parameter(tire, PAC_PKX2, 0.0)*dfz
        ) * std::exp(pac2002_parameter(tire, PAC_PKX3, 0.0)*dfz)
            *(1.0+pac2002_parameter(tire, PAC_PPX1, 0.0)*dpi
                +pac2002_parameter(tire, PAC_PPX2, 0.0)*dpi*dpi)
            *pac2002_parameter(tire, PAC_LKX, 1.0);
        e = (pac2002_parameter(tire, PAC_PEX1, 0.0)
            + pac2002_parameter(tire, PAC_PEX2, 0.0)*dfz
            + pac2002_parameter(tire, PAC_PEX3, 0.0)*dfz*dfz)
            *pac2002_parameter(tire, PAC_LEX, 1.0);
        sh = (pac2002_parameter(tire, PAC_PHX1, 0.0)
            + pac2002_parameter(tire, PAC_PHX2, 0.0)*dfz)
            *pac2002_parameter(tire, PAC_LHX, 1.0);
        e *= 1.0-pac2002_parameter(tire, PAC_PEX4, 0.0)
            *pac2002_sign(slip+sh);
        sv = normal_force * (
            pac2002_parameter(tire, PAC_PVX1, 0.0)
                + pac2002_parameter(tire, PAC_PVX2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LVX, 1.0)
            *pac2002_parameter(tire, PAC_LMUX, 1.0);
        (void)gamma_x;
    }
    e = std::min(1.0, e);
    const double b = stiffness/(c*d+0.1);
    const auto raw = [&](double value) {
        const double argument = std::clamp(
            b*(value+sh), -0.5*kPi+0.01, 0.5*kPi-0.01
        );
        return d*std::sin(
            c*std::atan(argument-e*(argument-std::atan(argument)))
        )+sv;
    };
    if (tire.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
        // Adams/Chrono 的 PAC2002 基础输出包含 Svx/Svy；只有用户
        // PAC 模式才保留项目历史上的零滑移校准约定。
        return raw(slip);
    }
    return raw(slip)-raw(0.0);
}

double pac2002_force_limit(
    const Tire& tire, double normal_force, bool lateral, double camber
) {
    return std::max(
        pac2002_peak_force(tire, normal_force, lateral, camber), 1e-12
    );
}

bool pac2002_has_combined_slip_terms(const Tire& tire) {
    // RBX1/RBY1 是两条联合滑移缩放曲线的幅值。其余参数即使被填充，
    // 没有对应幅值也不应改变历史默认的摩擦椭圆行为。
    return std::abs(pac2002_parameter(tire, PAC_RBX1, 0.0)) > 1e-12
        || std::abs(pac2002_parameter(tire, PAC_RBY1, 0.0)) > 1e-12;
}

double pac2002_safe_combined_ratio(
    double numerator, double denominator
) {
    if (!std::isfinite(numerator) || !std::isfinite(denominator)
        || std::abs(denominator) <= 1e-12) {
        return 1.0;
    }
    const double result = numerator/denominator;
    return std::isfinite(result) ? result : 1.0;
}

double pac2002_combined_longitudinal_force(
    const Tire& tire, double kappa, double alpha, double normal_force,
    double camber, double pure_force
) {
    if (normal_force <= 0.0) return 0.0;
    const double dfz = pac2002_load_difference(tire, normal_force);
    const double sh = pac2002_parameter(tire, PAC_RHX1, 0.0);
    const double c = pac2002_parameter(tire, PAC_RCX1, 1.0);
    const double e = std::min(
        1.0,
        pac2002_parameter(tire, PAC_REX1, 0.0)
            +pac2002_parameter(tire, PAC_REX2, 0.0)*dfz
    );
    const double b = std::abs(
        pac2002_parameter(tire, PAC_RBX1, 0.0)
        *std::cos(std::atan(
            pac2002_parameter(tire, PAC_RBX2, 0.0)*kappa
        ))*pac2002_parameter(tire, PAC_LXAL, 1.0)
    );
    const double shifted_alpha = alpha+sh;
    const auto combined_shape = [b, c, e](double value) {
        const double argument = std::clamp(
            b*value, -0.5*kPi+0.01, 0.5*kPi-0.01
        );
        return std::cos(
            c*std::atan(
                argument-e*(argument-std::atan(argument))
            )
        );
    };
    const double scale = pac2002_safe_combined_ratio(
        combined_shape(shifted_alpha), combined_shape(sh)
    );
    (void)camber;
    return pure_force*scale;
}

double pac2002_combined_lateral_force(
    const Tire& tire, double kappa, double alpha, double normal_force,
    double camber, double pure_force
) {
    if (normal_force <= 0.0) return 0.0;
    const double dfz = pac2002_load_difference(tire, normal_force);
    const double sh = pac2002_parameter(tire, PAC_RHY1, 0.0)
        +pac2002_parameter(tire, PAC_RHY2, 0.0)*dfz;
    const double c = pac2002_parameter(tire, PAC_RCY1, 1.0);
    const double e = std::min(
        1.0,
        pac2002_parameter(tire, PAC_REY1, 0.0)
            +pac2002_parameter(tire, PAC_REY2, 0.0)*dfz
    );
    const double b = pac2002_parameter(tire, PAC_RBY1, 0.0)
        *std::cos(std::atan(
            pac2002_parameter(tire, PAC_RBY2, 0.0)
            *(alpha-pac2002_parameter(tire, PAC_RBY3, 0.0))
        ))*pac2002_parameter(tire, PAC_LYKA, 1.0);
    const double shifted_kappa = kappa+sh;
    const auto combined_shape = [b, c, e](double value) {
        const double argument = std::clamp(
            b*value, -0.5*kPi+0.01, 0.5*kPi-0.01
        );
        return std::cos(
            c*std::atan(
                argument-e*(argument-std::atan(argument))
            )
        );
    };
    const double scale = pac2002_safe_combined_ratio(
        combined_shape(shifted_kappa), combined_shape(sh)
    );
    const double lateral_peak = pac2002_force_limit(
        tire, normal_force, true, camber
    );
    const double velocity_offset = lateral_peak * (
        pac2002_parameter(tire, PAC_RVY1, 0.0)
        +pac2002_parameter(tire, PAC_RVY2, 0.0)*dfz
        +pac2002_parameter(tire, PAC_RVY3, 0.0)*camber
    ) * std::cos(std::atan(
        pac2002_parameter(tire, PAC_RVY4, 0.0)*alpha
    ))*pac2002_parameter(tire, PAC_LVYKA, 1.0);
    const double combined_offset = velocity_offset * std::sin(
        pac2002_parameter(tire, PAC_RVY5, 0.0)*std::atan(
            pac2002_parameter(tire, PAC_RVY6, 0.0)*kappa
        )
    );
    return pure_force*scale+combined_offset;
}

double pac2002_lateral_stiffness(
    const Tire& tire, double normal_force, double camber
) {
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double reference_load = pac2002_reference_load(tire);
    const double dpi = pac2002_pressure_difference(tire);
    const double dfz = pac2002_load_difference(tire, normal_force);
    const double gamma_y = camber
        *pac2002_parameter(tire, PAC_LGAY, 1.0);
    const double pky2 = pac2002_parameter(tire, PAC_PKY2, 0.0);
    const double ky0 = std::abs(pky2) > 1e-12
        ? pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
            *nominal*(1.0+pac2002_parameter(tire, PAC_PPY1, 0.0)*dpi)
            *std::sin(2.0*std::atan(normal_force/(
                pky2*reference_load
                *(1.0+pac2002_parameter(tire, PAC_PPY2, 0.0)*dpi)
            )))
            *pac2002_parameter(tire, PAC_LFZO, 1.0)
            *pac2002_parameter(tire, PAC_LMUY, 1.0)
        : normal_force
            *pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
            *pac2002_parameter(tire, PAC_LFZO, 1.0)
            *pac2002_parameter(tire, PAC_LMUY, 1.0);
    (void)dfz;
    return ky0*(1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)
        *std::abs(gamma_y));
}

double pac2002_longitudinal_stiffness(
    const Tire& tire, double normal_force
) {
    const double dfz = pac2002_load_difference(tire, normal_force);
    return normal_force*(
        pac2002_parameter(tire, PAC_PKX1, 120000.0/4850.0)
        +pac2002_parameter(tire, PAC_PKX2, 0.0)*dfz
    )*std::exp(pac2002_parameter(tire, PAC_PKX3, 0.0)*dfz)
        *(1.0+pac2002_parameter(tire, PAC_PPX1, 0.0)
            *pac2002_pressure_difference(tire)
            +pac2002_parameter(tire, PAC_PPX2, 0.0)
                *std::pow(pac2002_pressure_difference(tire), 2.0))
        *pac2002_parameter(tire, PAC_LKX, 1.0);
}

double pac2002_aligning_moment(
    const Tire& tire, double kappa, double alpha, double normal_force,
    double camber, double fx, double fy
) {
    if (normal_force <= 0.0 || !pac2002_has_aligning_moment_terms(tire)) {
        return 0.0;
    }
    const double reference_load = pac2002_reference_load(tire);
    const double dfz = pac2002_load_difference(tire, normal_force);
    const double dpi = pac2002_pressure_difference(tire);
    const double ky = pac2002_lateral_stiffness(tire, normal_force, camber);
    if (std::abs(ky) <= 1e-12) return 0.0;
    const double kx = pac2002_longitudinal_stiffness(tire, normal_force);
    const double gamma_y = camber*pac2002_parameter(tire, PAC_LGAY, 1.0);
    const double gamma_z = camber*pac2002_parameter(tire, PAC_LGAZ, 1.0);
    const double shy = (
        pac2002_parameter(tire, PAC_PHY1, 0.0)
        +pac2002_parameter(tire, PAC_PHY2, 0.0)*dfz
    )*pac2002_parameter(tire, PAC_LHY, 1.0)
        +pac2002_parameter(tire, PAC_PHY3, 0.0)*gamma_y
            *pac2002_parameter(tire, PAC_LKYG, 1.0);
    const double svy = normal_force* (
        (
            pac2002_parameter(tire, PAC_PVY1, 0.0)
            +pac2002_parameter(tire, PAC_PVY2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LVY, 1.0)
        +(
            pac2002_parameter(tire, PAC_PVY3, 0.0)
            +pac2002_parameter(tire, PAC_PVY4, 0.0)*dfz
        )*gamma_y*pac2002_parameter(tire, PAC_LKYG, 1.0)
    )*pac2002_parameter(tire, PAC_LMUY, 1.0);
    const double sht = pac2002_parameter(tire, PAC_QHZ1, 0.0)
        +pac2002_parameter(tire, PAC_QHZ2, 0.0)*dfz
        +(
            pac2002_parameter(tire, PAC_QHZ3, 0.0)
            +pac2002_parameter(tire, PAC_QHZ4, 0.0)*dfz
        )*gamma_z;
    const double alpha_r = alpha+shy+svy/ky;
    const double alpha_t = alpha+sht;
    const double ct = pac2002_parameter(tire, PAC_QCZ1, 1.0);
    if (ct <= 0.0) return 0.0;
    const double bt = std::abs(
        pac2002_parameter(tire, PAC_QBZ1, 0.0)
        +pac2002_parameter(tire, PAC_QBZ2, 0.0)*dfz
        +pac2002_parameter(tire, PAC_QBZ3, 0.0)*dfz*dfz
    )*std::abs(
        1.0
        +pac2002_parameter(tire, PAC_QBZ4, 0.0)*gamma_z
        +pac2002_parameter(tire, PAC_QBZ5, 0.0)*std::abs(gamma_z)
    )*pac2002_parameter(tire, PAC_LKY, 1.0)
        /std::max(pac2002_parameter(tire, PAC_LMUY, 1.0), 1e-9);
    const double et = std::min(
        1.0,
        (
            pac2002_parameter(tire, PAC_QEZ1, 0.0)
            +pac2002_parameter(tire, PAC_QEZ2, 0.0)*dfz
            +pac2002_parameter(tire, PAC_QEZ3, 0.0)*dfz*dfz
        )*(
            1.0+(
                pac2002_parameter(tire, PAC_QEZ4, 0.0)
                +pac2002_parameter(tire, PAC_QEZ5, 0.0)*gamma_z
            )*(2.0/kPi)*std::atan(bt*ct*alpha_t)
        )
    );
    const double dt = normal_force* (
        pac2002_parameter(tire, PAC_QDZ1, 0.0)
        +pac2002_parameter(tire, PAC_QDZ2, 0.0)*dfz
    )* (
        1.0
        +pac2002_parameter(tire, PAC_QDZ3, 0.0)*gamma_z
        +pac2002_parameter(tire, PAC_QDZ4, 0.0)*gamma_z*gamma_z
    )*tire.radius/reference_load
        *(1.0-pac2002_parameter(tire, PAC_QPZ1, 0.0)*dpi)
        *pac2002_parameter(tire, PAC_LTR, 1.0);
    const auto trail = [&](double value) {
        const double argument = std::clamp(
            bt*value, -0.5*kPi+0.01, 0.5*kPi-0.01
        );
        return dt*std::cos(
            ct*std::atan(argument-et*(argument-std::atan(argument)))
        )*std::cos(alpha);
    };
    const double br = pac2002_parameter(tire, PAC_QBZ9, 0.0)
        *pac2002_parameter(tire, PAC_LKY, 1.0)
        /std::max(pac2002_parameter(tire, PAC_LMUY, 1.0), 1e-9)
        +pac2002_parameter(tire, PAC_QBZ10, 0.0)
            *ky/(ct*pac2002_force_limit(tire, normal_force, true, camber)+0.1)
            *ct;
    const double dr = normal_force* (
        (
            pac2002_parameter(tire, PAC_QDZ6, 0.0)
            +pac2002_parameter(tire, PAC_QDZ7, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LRES, 1.0)
        +(
            pac2002_parameter(tire, PAC_QDZ8, 0.0)
            +pac2002_parameter(tire, PAC_QDZ9, 0.0)*dfz
        )*(1.0+pac2002_parameter(tire, PAC_QPZ2, 0.0)*dpi)*gamma_z
    )*tire.radius*pac2002_parameter(tire, PAC_LMUY, 1.0);
    // fy uses the imported tire convention while this expression is assembled
    // in the PAC2002 moment convention. Mz is converted to ISO at the return.
    double mz = trail(alpha_r)*fy
        -dr*std::cos(std::atan(br*alpha_r))*std::cos(alpha);
    if (pac2002_has_combined_slip_terms(tire)) {
        const double lateral_peak = pac2002_force_limit(
            tire, normal_force, true, camber
        );
        const double velocity_offset = lateral_peak* (
            pac2002_parameter(tire, PAC_RVY1, 0.0)
            +pac2002_parameter(tire, PAC_RVY2, 0.0)*dfz
            +pac2002_parameter(tire, PAC_RVY3, 0.0)*camber
        )*std::cos(std::atan(
            pac2002_parameter(tire, PAC_RVY4, 0.0)*alpha
        ));
        const double svyk = -velocity_offset*std::sin(
            pac2002_parameter(tire, PAC_RVY5, 0.0)*std::atan(
                pac2002_parameter(tire, PAC_RVY6, 0.0)*kappa
            )
        );
        const double alpha_teq = std::atan(std::sqrt(
            std::pow(std::tan(alpha_t), 2.0)
            +std::pow(kx/ky*kappa, 2.0)
        ))*pac2002_sign(kappa);
        const double alpha_req = std::atan(std::sqrt(
            std::pow(std::tan(alpha_r), 2.0)
            +std::pow(kx/ky*kappa, 2.0)
        ))*pac2002_sign(alpha_r);
        const double s = (
            pac2002_parameter(tire, PAC_SSZ1, 0.0)
            -pac2002_parameter(tire, PAC_SSZ2, 0.0)*fy/reference_load
            +(
                pac2002_parameter(tire, PAC_SSZ3, 0.0)
                +pac2002_parameter(tire, PAC_SSZ4, 0.0)*dfz
            )*gamma_z
        )*tire.radius*pac2002_parameter(tire, PAC_LS, 1.0);
        mz = trail(alpha_teq)*(fy-svyk)
            -dr*std::cos(std::atan(br*alpha_req))*std::cos(alpha)
            -s*fx;
    }
    return std::isfinite(mz) ? -mz : 0.0;
}

double pac2002_overturning_moment(
    const Tire& tire, double fy_source, double normal_force, double camber
) {
    if (normal_force <= 0.0 || !pac2002_has_overturning_moment_terms(tire)) {
        return 0.0;
    }
    const double reference_load = pac2002_reference_load(tire);
    const double pressure_difference = pac2002_pressure_difference(tire);
    const double fy_ratio = fy_source/reference_load;
    const double load_ratio = normal_force/reference_load;
    const double bracket =
        pac2002_parameter(tire, PAC_QSX3, 0.0)*fy_ratio
        +pac2002_parameter(tire, PAC_QSX4, 0.0)
            *std::cos(pac2002_parameter(tire, PAC_QSX5, 0.0)
                *std::atan(load_ratio*load_ratio))
            *std::sin(
                pac2002_parameter(tire, PAC_QSX7, 0.0)*camber
                +pac2002_parameter(tire, PAC_QSX8, 0.0)
                    *std::atan(pac2002_parameter(tire, PAC_QSX9, 0.0)
                        *fy_ratio)
            )
        +(
            pac2002_parameter(tire, PAC_QSX10, 0.0)
                *std::atan(pac2002_parameter(tire, PAC_QSX11, 0.0)
                    *load_ratio)
            -pac2002_parameter(tire, PAC_QSX2, 0.0)
                *(1.0+pac2002_parameter(tire, PAC_QPX1, 0.0)
                    *pressure_difference)
        )*camber
        +pac2002_parameter(tire, PAC_QSX1, 0.0)
            *pac2002_parameter(tire, PAC_LVMX, 1.0);
    const double moment = normal_force*tire.radius*bracket
        *pac2002_parameter(tire, PAC_LMX, 1.0);
    return std::isfinite(moment) ? moment : 0.0;
}

double pac2002_rolling_resistance_moment(
    const Tire& tire, double fx_source, double normal_force, double camber,
    double longitudinal_speed
) {
    if (normal_force <= 0.0 || !pac2002_has_rolling_resistance_terms(tire)) {
        return 0.0;
    }
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double pressure_ratio = std::max(
        pac2002_parameter(tire, PAC_IP, 200000.0)
            /std::max(pac2002_parameter(tire, PAC_IP_NOM, 200000.0), 1e-9),
        1e-12
    );
    const double reference_speed = std::sqrt(9.81*tire.radius);
    const double speed_ratio = longitudinal_speed
        /std::max(reference_speed, 1e-9);
    const double load_ratio = normal_force/nominal;
    const double bracket =
        pac2002_parameter(tire, PAC_QSY1, 0.0)
        +pac2002_parameter(tire, PAC_QSY2, 0.0)*fx_source/nominal
        +pac2002_parameter(tire, PAC_QSY3, 0.0)*std::abs(speed_ratio)
        +pac2002_parameter(tire, PAC_QSY4, 0.0)*std::pow(speed_ratio, 4.0)
        +pac2002_parameter(tire, PAC_QSY5, 0.0)*camber*camber
        +pac2002_parameter(tire, PAC_QSY6, 0.0)*camber*camber*load_ratio;
    const double load_scale = std::pow(
        std::max(load_ratio, 1e-12), pac2002_parameter(tire, PAC_QSY7, 0.0)
    );
    const double pressure_scale = std::pow(
        pressure_ratio, pac2002_parameter(tire, PAC_QSY8, 0.0)
    );
    const double moment = normal_force*tire.radius*bracket*load_scale
        *pressure_scale*pac2002_parameter(tire, PAC_LMY, 1.0);
    // PAC2002 的 My 为 SAE 轮胎坐标约定；native 侧向轴与 SAE y 轴相反。
    return std::isfinite(moment) ? -moment : 0.0;
}

bool pac2002_has_overturning_moment_terms(const Tire& tire) {
    for (const int index : {
             PAC_QSX1, PAC_QSX2, PAC_QSX3, PAC_QSX4, PAC_QSX5,
             PAC_QSX7, PAC_QSX8, PAC_QSX9, PAC_QSX10, PAC_QSX11
         }) {
        if (std::abs(pac2002_parameter(tire, index, 0.0)) > 1e-12) {
            return true;
        }
    }
    return false;
}

bool pac2002_has_rolling_resistance_terms(const Tire& tire) {
    for (const int index : {
             PAC_QSY1, PAC_QSY2, PAC_QSY3, PAC_QSY4,
             PAC_QSY5, PAC_QSY6, PAC_QSY7, PAC_QSY8
         }) {
        if (std::abs(pac2002_parameter(tire, index, 0.0)) > 1e-12) {
            return true;
        }
    }
    return false;
}

bool pac2002_has_aligning_moment_terms(const Tire& tire) {
    for (const int index : {
             PAC_QDZ1, PAC_QDZ2, PAC_QDZ3, PAC_QDZ4,
             PAC_QDZ6, PAC_QDZ7, PAC_QDZ8, PAC_QDZ9,
             PAC_SSZ1, PAC_SSZ2, PAC_SSZ3, PAC_SSZ4
         }) {
        if (std::abs(pac2002_parameter(tire, index, 0.0)) > 1e-12) {
            return true;
        }
    }
    return false;
}

double pac2002_relaxation_length(
    const Tire& tire, double normal_force, bool lateral, double camber
) {
    if (normal_force <= 0.0) {
        return lateral
            ? tire.relaxation_length_lateral
            : tire.relaxation_length_longitudinal;
    }
    const double reference_load = pac2002_reference_load(tire);
    const double dfz = pac2002_load_difference(tire, normal_force);
    double length = 0.0;
    if (lateral) {
        const double pty2 = pac2002_parameter(tire, PAC_PTY2, 1.0);
        const double denominator = std::max(
            std::abs(pty2*reference_load), 1e-9
        );
        length = pac2002_parameter(tire, PAC_PTY1, 1.0)
            * std::sin(2.0*std::atan(normal_force/denominator))
            *(1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)
                *std::abs(camber))
            *tire.radius*pac2002_positive_scale(tire, PAC_LFZO, 1.0)
            *pac2002_positive_scale(tire, PAC_LSGAL, 1.0);
    } else {
        length = normal_force * (
            pac2002_parameter(tire, PAC_PTX1, 1.0)
                + pac2002_parameter(tire, PAC_PTX2, 0.0)*dfz
        ) * std::exp(pac2002_parameter(tire, PAC_PTX3, 0.0)*dfz)
            * tire.radius/reference_load
            *pac2002_positive_scale(tire, PAC_LSGKP, 1.0);
    }
    if (tire.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
        return std::max(length, 1e-9);
    }
    return std::max(
        length,
        lateral
            ? tire.relaxation_length_lateral
            : tire.relaxation_length_longitudinal
    );
}

double pac2002_gyroscopic_moment(
    const Tire& tire, double normal_force, double camber,
    double effective_lateral_slip, double effective_lateral_slip_rate,
    double rolling_radius, double spin_rate
) {
    const double coefficient = pac2002_parameter(tire, PAC_QTZ1, 0.0)
        *pac2002_parameter(tire, PAC_LGYR, 1.0)
        *pac2002_parameter(tire, PAC_MBELT, 0.0);
    if (std::abs(coefficient) <= 1e-12 || normal_force <= 0.0 ||
        rolling_radius <= 0.0) {
        return 0.0;
    }
    const double sigma_alpha = pac2002_relaxation_length(
        tire, normal_force, true, camber
    );
    const double tangent = std::tan(effective_lateral_slip);
    const double lateral_deflection_rate = sigma_alpha
        *(1.0+tangent*tangent)*effective_lateral_slip_rate;
    const double result = coefficient*(rolling_radius*spin_rate)
        *lateral_deflection_rate;
    return std::isfinite(result) ? result : 0.0;
}

double road_profile_height(
    const Model& model, const State& state, std::size_t tire_index
) {
    const RoadProfile& profile = model.road_profile;
    if (profile.kind == 0 || tire_index >= model.tires.size()) return 0.0;
    const Tire& tire = model.tires[tire_index];
    const double x = state_point(
        state, tire_center_body(tire), tire_center_local(tire)
    ).x;
    const double corner_scale = tire_index < profile.corner_scale.size()
        ? profile.corner_scale[tire_index] : 1.0;
    const double amplitude = profile.amplitude * corner_scale;
    const double distance = x-profile.origin_x-profile.bump_start;
    double height = profile.origin_z;
    if (profile.kind == 2) {
        const double angle = 2.0*kPi*(x-profile.origin_x)
            / profile.wavelength + profile.phase;
        height += amplitude*std::sin(angle);
    } else if (profile.kind == 3 || profile.kind == 5) {
        if (distance > 0.0 && distance < profile.bump_length) {
            const double ratio = distance/profile.bump_length;
            height += 0.5*amplitude*(1.0-std::cos(2.0*kPi*ratio));
        }
    } else if (profile.kind == 4) {
        for (const auto& term : std::array<Vec3, 3>{
                 Vec3{1.0, 1.0, 0.0},
                 Vec3{2.7, 0.6, 1.2},
                 Vec3{5.1, 0.35, 2.4}
             }) {
            const double angle = 2.0*kPi*term.x*(x-profile.origin_x)
                / profile.wavelength + profile.phase + term.z;
            height += amplitude*term.y*std::sin(angle);
        }
    }
    return height;
}

double road_profile_slope(
    const Model& model, const State& state, std::size_t tire_index
) {
    const RoadProfile& profile = model.road_profile;
    if (profile.kind == 0 || tire_index >= model.tires.size()) return 0.0;
    const Tire& tire = model.tires[tire_index];
    const double x = state_point(
        state, tire_center_body(tire), tire_center_local(tire)
    ).x;
    const double corner_scale = tire_index < profile.corner_scale.size()
        ? profile.corner_scale[tire_index] : 1.0;
    const double amplitude = profile.amplitude * corner_scale;
    const double distance = x-profile.origin_x-profile.bump_start;
    if (profile.kind == 2) {
        const double wave_number = 2.0*kPi/profile.wavelength;
        const double angle = wave_number*(x-profile.origin_x)
            + profile.phase;
        return amplitude*wave_number*std::cos(angle);
    }
    if (profile.kind == 3 || profile.kind == 5) {
        if (distance > 0.0 && distance < profile.bump_length) {
            return amplitude*kPi/profile.bump_length
                *std::sin(2.0*kPi*distance/profile.bump_length);
        }
        return 0.0;
    }
    if (profile.kind == 4) {
        double slope = 0.0;
        for (const auto& term : std::array<Vec3, 3>{
                 Vec3{1.0, 1.0, 0.0},
                 Vec3{2.7, 0.6, 1.2},
                 Vec3{5.1, 0.35, 2.4}
             }) {
            const double wave_number = 2.0*kPi*term.x
                / profile.wavelength;
            const double angle = wave_number*(x-profile.origin_x)
                + profile.phase + term.z;
            slope += amplitude*term.y*wave_number*std::cos(angle);
        }
        return slope;
    }
    return 0.0;
}

// A single-direction forward derivative used by the Newton assembly.  Unlike
// the old residual perturbation, this carries the chain rule through the force
// element formulas without evaluating the residual at a second state.  The
// value member is also used for branch selection, so the active physical
// branch remains identical to external_force_vector.
struct DirectionalScalar {
    double value{0.0};
    double derivative{0.0};

    DirectionalScalar() = default;
    DirectionalScalar(double value_, double derivative_ = 0.0)
        : value(value_), derivative(derivative_) {}

    DirectionalScalar& operator+=(const DirectionalScalar& other) {
        value += other.value;
        derivative += other.derivative;
        return *this;
    }
    DirectionalScalar& operator-=(const DirectionalScalar& other) {
        value -= other.value;
        derivative -= other.derivative;
        return *this;
    }
};

DirectionalScalar operator+(
    const DirectionalScalar& a, const DirectionalScalar& b
) {
    return {a.value+b.value, a.derivative+b.derivative};
}
DirectionalScalar operator-(
    const DirectionalScalar& a, const DirectionalScalar& b
) {
    return {a.value-b.value, a.derivative-b.derivative};
}
DirectionalScalar operator-(const DirectionalScalar& a) {
    return {-a.value, -a.derivative};
}
DirectionalScalar operator*(
    const DirectionalScalar& a, const DirectionalScalar& b
) {
    return {
        a.value*b.value,
        a.derivative*b.value+a.value*b.derivative
    };
}
DirectionalScalar operator/(
    const DirectionalScalar& a, const DirectionalScalar& b
) {
    const double denominator = b.value*b.value;
    return {
        a.value/b.value,
        (a.derivative*b.value-a.value*b.derivative)/denominator
    };
}
DirectionalScalar d_sqrt(const DirectionalScalar& a) {
    const double root = std::sqrt(a.value);
    return {root, a.derivative/(2.0*root)};
}
DirectionalScalar d_sin(const DirectionalScalar& a) {
    return {std::sin(a.value), std::cos(a.value)*a.derivative};
}
DirectionalScalar d_cos(const DirectionalScalar& a) {
    return {std::cos(a.value), -std::sin(a.value)*a.derivative};
}
DirectionalScalar d_tan(const DirectionalScalar& a) {
    const double cosine = std::cos(a.value);
    return {
        std::tan(a.value),
        a.derivative/(cosine*cosine)
    };
}
DirectionalScalar d_exp(const DirectionalScalar& a) {
    const double value = std::exp(a.value);
    return {value, value*a.derivative};
}
DirectionalScalar d_pow_positive(
    const DirectionalScalar& a, double exponent, bool& smooth
) {
    if (a.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double value = std::pow(a.value, exponent);
    return {value, value*exponent*a.derivative/a.value};
}
DirectionalScalar d_atan2(
    const DirectionalScalar& y, const DirectionalScalar& x
) {
    const double denominator = x.value*x.value+y.value*y.value;
    return {
        std::atan2(y.value, x.value),
        (x.value*y.derivative-y.value*x.derivative)/denominator
    };
}
DirectionalScalar d_abs(const DirectionalScalar& a, bool& smooth) {
    constexpr double kJacobianStep = 1e-7;
    const double trial_value = a.value + kJacobianStep*a.derivative;
    if (
        std::abs(a.value) <= 1e-12 ||
        a.value*trial_value <= 0.0
    ) smooth = false;
    const double sign = a.value < 0.0 ? -1.0 : 1.0;
    return {std::abs(a.value), sign*a.derivative};
}

double d_sign(const DirectionalScalar& a, bool& smooth) {
    if (std::abs(a.value) <= 1e-12) smooth = false;
    return pac2002_sign(a.value);
}

DirectionalScalar d_clamp(
    const DirectionalScalar& a, double lower, double upper, bool& smooth
) {
    constexpr double kJacobianStep = 1e-7;
    const double trial_value = a.value+kJacobianStep*a.derivative;
    if (
        a.value <= lower || a.value >= upper ||
        trial_value <= lower || trial_value >= upper
    ) {
        smooth = false;
    }
    if (a.value <= lower) return {lower, 0.0};
    if (a.value >= upper) return {upper, 0.0};
    return a;
}

void fiala_forces_directional(
    const Tire& tire,
    const DirectionalScalar& kappa,
    const DirectionalScalar& alpha,
    const DirectionalScalar& normal_force,
    DirectionalScalar& fx,
    DirectionalScalar& fy,
    DirectionalScalar& mz,
    bool& smooth
) {
    fx = fy = mz = {};
    if (normal_force.value <= 0.0) {
        smooth = false;
        return;
    }
    const double cslip = std::max(
        fiala_parameter(tire, FIALA_CSLIP, 1000.0), 1e-9
    );
    const double calpha = std::max(
        fiala_parameter(tire, FIALA_CALPHA, 800.0), 1e-9
    );
    const double umin = std::max(
        fiala_parameter(tire, FIALA_UMIN, 0.9), 0.0
    );
    const double umax = std::max(
        fiala_parameter(tire, FIALA_UMAX, 1.0), umin
    );
    const DirectionalScalar tan_alpha = d_tan(alpha);
    const DirectionalScalar combined_argument =
        kappa*kappa+tan_alpha*tan_alpha;
    DirectionalScalar combined_slip;
    if (combined_argument.value <= 1e-24) {
        smooth = false;
    } else {
        combined_slip = d_sqrt(combined_argument);
    }
    constexpr double kJacobianStep = 1e-7;
    const double trial_combined_slip =
        combined_slip.value+kJacobianStep*combined_slip.derivative;
    if (
        std::abs(combined_slip.value-1.0) <= 1e-12 ||
        (combined_slip.value-1.0)*(trial_combined_slip-1.0) <= 0.0
    ) {
        smooth = false;
    }
    const DirectionalScalar ss = combined_slip.value >= 1.0
        ? DirectionalScalar{1.0, 0.0}
        : combined_slip;
    const DirectionalScalar uf = (
        DirectionalScalar{umax}-(umax-umin)*ss
    )*normal_force;
    if (uf.value <= 0.0) {
        smooth = false;
        return;
    }

    const DirectionalScalar longitudinal_limit =
        uf/(2.0*cslip);
    const double abs_kappa = std::abs(kappa.value);
    const double trial_abs_kappa = abs_kappa +
        kJacobianStep*(kappa.value < 0.0 ? -kappa.derivative : kappa.derivative);
    if (
        std::abs(abs_kappa-longitudinal_limit.value) <= 1e-12 ||
        (abs_kappa-longitudinal_limit.value)
            *(trial_abs_kappa-longitudinal_limit.value) <= 0.0
    ) {
        smooth = false;
    }
    if (abs_kappa <= longitudinal_limit.value) {
        fx = cslip*kappa;
    } else if (abs_kappa > 1e-12) {
        const double sign_kappa = kappa.value < 0.0 ? -1.0 : 1.0;
        const DirectionalScalar abs_kappa_directional{
            abs_kappa,
            sign_kappa*kappa.derivative
        };
        fx = DirectionalScalar{sign_kappa}*(
            uf-uf*uf/(4.0*abs_kappa_directional*cslip)
        );
    } else {
        smooth = false;
    }

    const DirectionalScalar critical_alpha = d_atan2(
        3.0*uf/calpha, DirectionalScalar{1.0}
    );
    const DirectionalScalar tan_alpha_abs = d_abs(tan_alpha, smooth);
    const double abs_alpha = std::abs(alpha.value);
    const double trial_abs_alpha = abs_alpha+
        kJacobianStep*(alpha.value < 0.0 ? -alpha.derivative : alpha.derivative);
    if (
        std::abs(abs_alpha-critical_alpha.value) <= 1e-12 ||
        (abs_alpha-critical_alpha.value)
            *(trial_abs_alpha-critical_alpha.value) <= 0.0
    ) {
        smooth = false;
    }
    const double sign_alpha = d_sign(alpha, smooth);
    if (abs_alpha <= critical_alpha.value) {
        const DirectionalScalar h = DirectionalScalar{1.0}
            -calpha*tan_alpha_abs/(3.0*uf);
        const DirectionalScalar h3 = h*h*h;
        fy = -uf*(DirectionalScalar{1.0}-h3)*sign_alpha;
        mz = uf*fiala_parameter(tire, FIALA_WIDTH, 0.235)
            *(DirectionalScalar{1.0}-h)*h*h*h*sign_alpha;
    } else {
        fy = -uf*sign_alpha;
    }
}

DirectionalScalar pac2002_effective_rolling_radius_directional(
    const Tire& tire, const DirectionalScalar& penetration,
    const DirectionalScalar& spin_rate, bool& smooth
) {
    const double radius = std::max(tire.radius, 1e-9);
    const double nominal_load = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double vertical_scale = pac2002_positive_scale(tire, PAC_LCZ, 1.0);
    const double vertical_stiffness = std::max(tire.k*vertical_scale, 1e-9);
    const double nominal_deflection = nominal_load/vertical_stiffness;
    DirectionalScalar normalized_deflection;
    if (penetration.value <= 0.0) {
        if (std::abs(penetration.value) <= 1e-12) smooth = false;
        normalized_deflection = {};
    } else {
        normalized_deflection = penetration/
            std::max(nominal_deflection, 1e-9);
    }
    const double speed_reference = std::max(
        pac2002_parameter(tire, PAC_LONGVL, 16.6), 1e-9
    );
    const DirectionalScalar speed_ratio = spin_rate*radius/speed_reference;
    const DirectionalScalar speed_growth =
        pac2002_parameter(tire, PAC_QV1, 0.0)*radius
            *speed_ratio*speed_ratio;
    const DirectionalScalar radius_correction = nominal_deflection* (
        pac2002_parameter(tire, PAC_DREFF, 0.27)
            *d_atan2(
                pac2002_parameter(tire, PAC_BREFF, 8.4)
                    *normalized_deflection,
                {1.0, 0.0}
            )
        +pac2002_parameter(tire, PAC_FREFF, 0.07)
            *normalized_deflection
    );
    DirectionalScalar result = radius*
        pac2002_parameter(tire, PAC_QREO, 1.0)
        +speed_growth-radius_correction;
    if (result.value <= 1e-9) {
        smooth = false;
        result = {1e-9, 0.0};
    }
    return result;
}

DirectionalScalar pac2002_vertical_force_directional(
    const Tire& tire, const DirectionalScalar& penetration,
    const DirectionalScalar& penetration_rate,
    const DirectionalScalar& camber, bool& smooth
) {
    if (penetration.value <= 0.0) {
        if (std::abs(penetration.value) <= 1e-12) smooth = false;
        return {};
    }
    const double radius = std::max(tire.radius, 1e-9);
    const double nominal_load = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double load_scale = pac2002_positive_scale(tire, PAC_LCZ, 1.0);
    double qfz1 = pac2002_parameter(tire, PAC_QFZ1, 0.0);
    if (std::abs(qfz1) <= 1e-12) {
        qfz1 = tire.k*radius/(nominal_load*load_scale);
    }
    const DirectionalScalar normalized_penetration = penetration/radius;
    const DirectionalScalar camber_scale = camber*camber
        *pac2002_parameter(tire, PAC_QFZ3, 0.0);
    const double pressure_scale = 1.0
        +pac2002_parameter(tire, PAC_QPFZ1, 0.0)
            *pac2002_pressure_difference(tire);
    const DirectionalScalar elastic_force = nominal_load*load_scale
        *pressure_scale* (
            qfz1*normalized_penetration
            +pac2002_parameter(tire, PAC_QFZ2, 0.0)
                *normalized_penetration*normalized_penetration
            +camber_scale*normalized_penetration
        );
    const DirectionalScalar result = elastic_force+tire.c*penetration_rate;
    if (result.value <= 0.0) {
        smooth = false;
        return {};
    }
    return result;
}

DirectionalScalar pac2002_relaxation_length_directional(
    const Tire& tire, const DirectionalScalar& normal_force, bool lateral,
    const DirectionalScalar& camber, bool& smooth
) {
    const double reference_load = pac2002_reference_load(tire);
    const DirectionalScalar dfz =
        (normal_force-reference_load)/reference_load;
    DirectionalScalar length;
    if (lateral) {
        const double pty2 = pac2002_parameter(tire, PAC_PTY2, 1.0);
        const double denominator = std::max(
            std::abs(pty2*reference_load), 1e-9
        );
        length = pac2002_parameter(tire, PAC_PTY1, 1.0)
            *d_sin(2.0*d_atan2(
                normal_force, DirectionalScalar{denominator}
            ))*tire.radius
            *(
                1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)
                    *d_abs(
                        camber*pac2002_parameter(tire, PAC_LGAY, 1.0),
                        smooth
                    )
            )*pac2002_positive_scale(tire, PAC_LFZO, 1.0)
                *pac2002_positive_scale(tire, PAC_LSGAL, 1.0);
    } else {
        length = normal_force * (
            pac2002_parameter(tire, PAC_PTX1, 1.0)
            +pac2002_parameter(tire, PAC_PTX2, 0.0)*dfz
        ) * d_exp(pac2002_parameter(tire, PAC_PTX3, 0.0)*dfz)
            *tire.radius/reference_load
            *pac2002_positive_scale(tire, PAC_LSGKP, 1.0);
    }
    const double fallback = lateral
        ? tire.relaxation_length_lateral
        : tire.relaxation_length_longitudinal;
    if (tire.model_kind != VEHICLE_TIRE_PAC2002_ADAMS_SOURCE
        && length.value <= fallback) {
        smooth = false;
        return {fallback, 0.0};
    }
    if (length.value <= 0.0) {
        smooth = false;
        return {1e-9, 0.0};
    }
    return length;
}

DirectionalScalar pac2002_gyroscopic_moment_directional(
    const Tire& tire, const DirectionalScalar& normal_force,
    const DirectionalScalar& camber,
    const DirectionalScalar& effective_lateral_slip,
    const DirectionalScalar& effective_lateral_slip_rate,
    const DirectionalScalar& rolling_radius,
    const DirectionalScalar& spin_rate, bool& smooth
) {
    const double coefficient = pac2002_parameter(tire, PAC_QTZ1, 0.0)
        *pac2002_parameter(tire, PAC_LGYR, 1.0)
        *pac2002_parameter(tire, PAC_MBELT, 0.0);
    if (std::abs(coefficient) <= 1e-12 || normal_force.value <= 0.0 ||
        rolling_radius.value <= 0.0) {
        if (normal_force.value <= 0.0 || rolling_radius.value <= 0.0) {
            smooth = false;
        }
        return {};
    }
    const DirectionalScalar sigma_alpha =
        pac2002_relaxation_length_directional(
            tire, normal_force, true, camber, smooth
        );
    const DirectionalScalar sine = d_sin(effective_lateral_slip);
    const DirectionalScalar cosine = d_cos(effective_lateral_slip);
    if (std::abs(cosine.value) <= 1e-12) {
        smooth = false;
        return {};
    }
    const DirectionalScalar tangent = sine/cosine;
    const DirectionalScalar lateral_deflection_rate = sigma_alpha
        *(1.0+tangent*tangent)*effective_lateral_slip_rate;
    const DirectionalScalar result = coefficient
        *(rolling_radius*spin_rate)*lateral_deflection_rate;
    if (!std::isfinite(result.value) || !std::isfinite(result.derivative)) {
        smooth = false;
        return {};
    }
    return result;
}

DirectionalScalar pac2002_peak_force_directional(
    const Tire& tire, const DirectionalScalar& normal_force,
    bool lateral, const DirectionalScalar& camber, bool& smooth
) {
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double reference_load = pac2002_reference_load(tire);
    const DirectionalScalar dfz = (normal_force-reference_load)/reference_load;
    const double dpi = pac2002_pressure_difference(tire);
    DirectionalScalar mu;
    if (lateral) {
        const DirectionalScalar gamma_y = camber
            *pac2002_parameter(tire, PAC_LGAY, 1.0);
        mu = (
            pac2002_parameter(tire, PAC_PDY1, 1.0)
            +pac2002_parameter(tire, PAC_PDY2, 0.0)*dfz
        ) * (
            1.0+pac2002_parameter(tire, PAC_PDY3, 0.0)
                *gamma_y*gamma_y
        ) * (
            1.0+pac2002_parameter(tire, PAC_PPY3, 0.0)*dpi
                +pac2002_parameter(tire, PAC_PPY4, 0.0)*dpi*dpi
        )*pac2002_parameter(tire, PAC_LMUY, 1.0);
    } else {
        const DirectionalScalar gamma_x = camber
            *pac2002_parameter(tire, PAC_LGAX, 1.0);
        mu = (
            pac2002_parameter(tire, PAC_PDX1, 1.0)
            +pac2002_parameter(tire, PAC_PDX2, 0.0)*dfz
        ) * (
            1.0-pac2002_parameter(tire, PAC_PDX3, 0.0)
                *gamma_x*gamma_x
        ) * (
            1.0+pac2002_parameter(tire, PAC_PPX3, 0.0)*dpi
                +pac2002_parameter(tire, PAC_PPX4, 0.0)*dpi*dpi
        )*pac2002_parameter(tire, PAC_LMUX, 1.0);
    }
    return d_abs(mu*normal_force, smooth);
}

DirectionalScalar pac2002_pure_force_directional(
    const Tire& tire, const DirectionalScalar& slip,
    const DirectionalScalar& normal_force, bool lateral,
    const DirectionalScalar& camber, bool& smooth
) {
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double reference_load = pac2002_reference_load(tire);
    const DirectionalScalar dfz = (normal_force-reference_load)/reference_load;
    const double dpi = pac2002_pressure_difference(tire);
    const double c = lateral
        ? pac2002_parameter(tire, PAC_PCY1, 1.3)
            *pac2002_parameter(tire, PAC_LCY, 1.0)
        : pac2002_parameter(tire, PAC_PCX1, 1.65)
            *pac2002_parameter(tire, PAC_LCX, 1.0);
    const DirectionalScalar d = pac2002_peak_force_directional(
        tire, normal_force, lateral, camber, smooth
    );
    if (d.value <= 0.0 || c <= 0.0) {
        smooth = false;
        return {};
    }
    DirectionalScalar stiffness;
    DirectionalScalar e;
    DirectionalScalar sh;
    DirectionalScalar sv;
    if (lateral) {
        const DirectionalScalar gamma_y = camber
            *pac2002_parameter(tire, PAC_LGAY, 1.0);
        const double pky2 = pac2002_parameter(tire, PAC_PKY2, 0.0);
        const DirectionalScalar ky0 = std::abs(pky2) > 1e-12
            ? pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
                *nominal
                *(1.0+pac2002_parameter(tire, PAC_PPY1, 0.0)*dpi)
                *d_sin(2.0*d_atan2(
                    normal_force, DirectionalScalar{
                        pky2*reference_load
                        *(1.0+pac2002_parameter(tire, PAC_PPY2, 0.0)*dpi)
                    }
                ))
                *pac2002_parameter(tire, PAC_LFZO, 1.0)
                *pac2002_parameter(tire, PAC_LMUY, 1.0)
            : normal_force
                *pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
                *pac2002_parameter(tire, PAC_LFZO, 1.0)
                *pac2002_parameter(tire, PAC_LMUY, 1.0);
        stiffness = ky0 * (
            1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)
                *d_abs(gamma_y, smooth)
        );
        sh = (
            pac2002_parameter(tire, PAC_PHY1, 0.0)
            +pac2002_parameter(tire, PAC_PHY2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LHY, 1.0)
            +pac2002_parameter(tire, PAC_PHY3, 0.0)*gamma_y
                *pac2002_parameter(tire, PAC_LKYG, 1.0);
        const double sign = d_sign(slip+sh, smooth);
        e = (
            pac2002_parameter(tire, PAC_PEY1, 0.0)
            +pac2002_parameter(tire, PAC_PEY2, 0.0)*dfz
        ) * (
            1.0-(pac2002_parameter(tire, PAC_PEY3, 0.0)
                +pac2002_parameter(tire, PAC_PEY4, 0.0)*gamma_y)*sign
        )*pac2002_parameter(tire, PAC_LEY, 1.0);
        sv = normal_force * (
            (
                pac2002_parameter(tire, PAC_PVY1, 0.0)
                +pac2002_parameter(tire, PAC_PVY2, 0.0)*dfz
            )*pac2002_parameter(tire, PAC_LVY, 1.0)
            +(
                pac2002_parameter(tire, PAC_PVY3, 0.0)
                +pac2002_parameter(tire, PAC_PVY4, 0.0)*dfz
            )*gamma_y*pac2002_parameter(tire, PAC_LKYG, 1.0)
        )*pac2002_parameter(tire, PAC_LMUY, 1.0);
    } else {
        stiffness = normal_force * (
            pac2002_parameter(tire, PAC_PKX1, 120000.0/4850.0)
            +pac2002_parameter(tire, PAC_PKX2, 0.0)*dfz
        ) * d_exp(pac2002_parameter(tire, PAC_PKX3, 0.0)*dfz)
            *(1.0+pac2002_parameter(tire, PAC_PPX1, 0.0)*dpi
                +pac2002_parameter(tire, PAC_PPX2, 0.0)*dpi*dpi)
            *pac2002_parameter(tire, PAC_LKX, 1.0);
        sh = (
            pac2002_parameter(tire, PAC_PHX1, 0.0)
            +pac2002_parameter(tire, PAC_PHX2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LHX, 1.0);
        const double sign = d_sign(slip+sh, smooth);
        e = (
            pac2002_parameter(tire, PAC_PEX1, 0.0)
            +pac2002_parameter(tire, PAC_PEX2, 0.0)*dfz
            +pac2002_parameter(tire, PAC_PEX3, 0.0)*dfz*dfz
        ) * (
            1.0-pac2002_parameter(tire, PAC_PEX4, 0.0)*sign
        )*pac2002_parameter(tire, PAC_LEX, 1.0);
        sv = normal_force * (
            pac2002_parameter(tire, PAC_PVX1, 0.0)
                +pac2002_parameter(tire, PAC_PVX2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LVX, 1.0)
            *pac2002_parameter(tire, PAC_LMUX, 1.0);
    }
    if (e.value > 1.0) {
        smooth = false;
        e = {1.0, 0.0};
    }
    const DirectionalScalar b = stiffness/(c*d+0.1);
    const auto raw = [&](const DirectionalScalar& value) {
        const DirectionalScalar argument = d_clamp(
            b*(value+sh), -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
        );
        const DirectionalScalar arctangent = d_atan2(
            argument, DirectionalScalar{1.0}
        );
        return d* d_sin(c*(arctangent-e*(argument-arctangent)))+sv;
    };
    if (tire.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
        return raw(slip);
    }
    return raw(slip)-raw(DirectionalScalar{});
}

DirectionalScalar pac2002_force_limit_directional(
    const Tire& tire, const DirectionalScalar& normal_force, bool lateral,
    const DirectionalScalar& camber, bool& smooth
) {
    return pac2002_peak_force_directional(
        tire, normal_force, lateral, camber, smooth
    );
}

DirectionalScalar pac2002_combined_ratio_directional(
    const DirectionalScalar& numerator, const DirectionalScalar& denominator,
    bool& smooth
) {
    if (!std::isfinite(denominator.value)
        || std::abs(denominator.value) <= 1e-12) {
        smooth = false;
        return {1.0, 0.0};
    }
    return numerator/denominator;
}

DirectionalScalar pac2002_combined_longitudinal_force_directional(
    const Tire& tire, const DirectionalScalar& kappa,
    const DirectionalScalar& alpha, const DirectionalScalar& normal_force,
    const DirectionalScalar& camber, const DirectionalScalar& pure_force,
    bool& smooth
) {
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const DirectionalScalar dfz =
        (normal_force-pac2002_reference_load(tire))
        /pac2002_reference_load(tire);
    const double sh = pac2002_parameter(tire, PAC_RHX1, 0.0);
    const double c = pac2002_parameter(tire, PAC_RCX1, 1.0);
    DirectionalScalar e =
        pac2002_parameter(tire, PAC_REX1, 0.0)
            +pac2002_parameter(tire, PAC_REX2, 0.0)*dfz
        ;
    if (e.value > 1.0) {
        smooth = false;
        e = {1.0, 0.0};
    }
    const DirectionalScalar b = d_abs(
        pac2002_parameter(tire, PAC_RBX1, 0.0)
            *d_cos(d_atan2(
                pac2002_parameter(tire, PAC_RBX2, 0.0)*kappa,
                {1.0, 0.0}
            ))*pac2002_parameter(tire, PAC_LXAL, 1.0),
        smooth
    );
    const auto combined_shape = [&](const DirectionalScalar& value) {
        const DirectionalScalar argument = d_clamp(
            b*value, -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
        );
        const DirectionalScalar arctangent = d_atan2(
            argument, {1.0, 0.0}
        );
        return d_cos(c* (arctangent-e*(argument-arctangent)));
    };
    const DirectionalScalar scale = pac2002_combined_ratio_directional(
        combined_shape(alpha+sh), combined_shape(DirectionalScalar{sh}),
        smooth
    );
    (void)camber;
    return pure_force*scale;
}

DirectionalScalar pac2002_combined_lateral_force_directional(
    const Tire& tire, const DirectionalScalar& kappa,
    const DirectionalScalar& alpha, const DirectionalScalar& normal_force,
    const DirectionalScalar& camber, const DirectionalScalar& pure_force,
    bool& smooth
) {
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const DirectionalScalar dfz =
        (normal_force-pac2002_reference_load(tire))
        /pac2002_reference_load(tire);
    const DirectionalScalar sh = pac2002_parameter(tire, PAC_RHY1, 0.0)
        +pac2002_parameter(tire, PAC_RHY2, 0.0)*dfz;
    const double c = pac2002_parameter(tire, PAC_RCY1, 1.0);
    DirectionalScalar e =
        pac2002_parameter(tire, PAC_REY1, 0.0)
            +pac2002_parameter(tire, PAC_REY2, 0.0)*dfz
        ;
    if (e.value > 1.0) {
        smooth = false;
        e = {1.0, 0.0};
    }
    const DirectionalScalar b =
        pac2002_parameter(tire, PAC_RBY1, 0.0)
        *d_cos(d_atan2(
            pac2002_parameter(tire, PAC_RBY2, 0.0)
                *(alpha-pac2002_parameter(tire, PAC_RBY3, 0.0)),
            {1.0, 0.0}
        ))*pac2002_parameter(tire, PAC_LYKA, 1.0);
    const auto combined_shape = [&](const DirectionalScalar& value) {
        const DirectionalScalar argument = d_clamp(
            b*value, -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
        );
        const DirectionalScalar arctangent = d_atan2(
            argument, {1.0, 0.0}
        );
        return d_cos(c* (arctangent-e*(argument-arctangent)));
    };
    const DirectionalScalar scale = pac2002_combined_ratio_directional(
        combined_shape(kappa+sh), combined_shape(sh),
        smooth
    );
    const DirectionalScalar lateral_peak =
        pac2002_force_limit_directional(
            tire, normal_force, true, camber, smooth
        );
    const DirectionalScalar velocity_offset = lateral_peak* (
        pac2002_parameter(tire, PAC_RVY1, 0.0)
        +pac2002_parameter(tire, PAC_RVY2, 0.0)*dfz
        +pac2002_parameter(tire, PAC_RVY3, 0.0)*camber
    ) *d_cos(d_atan2(
        pac2002_parameter(tire, PAC_RVY4, 0.0)*alpha,
        {1.0, 0.0}
    ))*pac2002_parameter(tire, PAC_LVYKA, 1.0);
    const DirectionalScalar combined_offset = velocity_offset*d_sin(
        pac2002_parameter(tire, PAC_RVY5, 0.0)*d_atan2(
            pac2002_parameter(tire, PAC_RVY6, 0.0)*kappa,
            {1.0, 0.0}
        )
    );
    return pure_force*scale+combined_offset;
}

DirectionalScalar pac2002_aligning_moment_directional(
    const Tire& tire, const DirectionalScalar& kappa,
    const DirectionalScalar& alpha, const DirectionalScalar& normal_force,
    const DirectionalScalar& camber, const DirectionalScalar& fx,
    const DirectionalScalar& fy, bool& smooth
) {
    if (!pac2002_has_aligning_moment_terms(tire)) return {};
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double reference_load = pac2002_reference_load(tire);
    const DirectionalScalar dfz = (normal_force-reference_load)/reference_load;
    const double dpi = pac2002_pressure_difference(tire);
    const double pky2 = pac2002_parameter(tire, PAC_PKY2, 0.0);
    const DirectionalScalar ky0 = std::abs(pky2) > 1e-12
        ? pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
            *nominal*(1.0+pac2002_parameter(tire, PAC_PPY1, 0.0)*dpi)
            *d_sin(2.0*d_atan2(
                normal_force, DirectionalScalar{
                    pky2*reference_load
                    *(1.0+pac2002_parameter(tire, PAC_PPY2, 0.0)*dpi)
                }
            ))
            *pac2002_parameter(tire, PAC_LFZO, 1.0)
            *pac2002_parameter(tire, PAC_LMUY, 1.0)
        : normal_force
            *pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
            *pac2002_parameter(tire, PAC_LFZO, 1.0)
            *pac2002_parameter(tire, PAC_LMUY, 1.0);
    const DirectionalScalar ky = ky0* (
        1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)
            *d_abs(camber*pac2002_parameter(tire, PAC_LGAY, 1.0), smooth)
    );
    if (std::abs(ky.value) <= 1e-12) {
        smooth = false;
        return {};
    }
    const DirectionalScalar kx = normal_force* (
        pac2002_parameter(tire, PAC_PKX1, 120000.0/4850.0)
        +pac2002_parameter(tire, PAC_PKX2, 0.0)*dfz
    )*d_exp(pac2002_parameter(tire, PAC_PKX3, 0.0)*dfz)
        *(1.0+pac2002_parameter(tire, PAC_PPX1, 0.0)*dpi
            +pac2002_parameter(tire, PAC_PPX2, 0.0)*dpi*dpi)
        *pac2002_parameter(tire, PAC_LKX, 1.0);
    const DirectionalScalar gamma_y = camber
        *pac2002_parameter(tire, PAC_LGAY, 1.0);
    const DirectionalScalar gamma_z = camber
        *pac2002_parameter(tire, PAC_LGAZ, 1.0);
    const DirectionalScalar shy = (
        pac2002_parameter(tire, PAC_PHY1, 0.0)
        +pac2002_parameter(tire, PAC_PHY2, 0.0)*dfz
    )*pac2002_parameter(tire, PAC_LHY, 1.0)
        +pac2002_parameter(tire, PAC_PHY3, 0.0)*gamma_y
            *pac2002_parameter(tire, PAC_LKYG, 1.0);
    const DirectionalScalar svy = normal_force* (
        (
            pac2002_parameter(tire, PAC_PVY1, 0.0)
            +pac2002_parameter(tire, PAC_PVY2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LVY, 1.0)
        +(
            pac2002_parameter(tire, PAC_PVY3, 0.0)
            +pac2002_parameter(tire, PAC_PVY4, 0.0)*dfz
        )*gamma_y*pac2002_parameter(tire, PAC_LKYG, 1.0)
    )*pac2002_parameter(tire, PAC_LMUY, 1.0);
    const DirectionalScalar sht = pac2002_parameter(tire, PAC_QHZ1, 0.0)
        +pac2002_parameter(tire, PAC_QHZ2, 0.0)*dfz
        +(
            pac2002_parameter(tire, PAC_QHZ3, 0.0)
        +pac2002_parameter(tire, PAC_QHZ4, 0.0)*dfz
        )*gamma_z;
    const DirectionalScalar alpha_r = alpha+shy+svy/ky;
    const DirectionalScalar alpha_t = alpha+sht;
    const double ct = pac2002_parameter(tire, PAC_QCZ1, 1.0);
    if (ct <= 0.0) {
        smooth = false;
        return {};
    }
    const DirectionalScalar bt = d_abs(
        pac2002_parameter(tire, PAC_QBZ1, 0.0)
        +pac2002_parameter(tire, PAC_QBZ2, 0.0)*dfz
        +pac2002_parameter(tire, PAC_QBZ3, 0.0)*dfz*dfz,
        smooth
    )*d_abs(
        1.0
        +pac2002_parameter(tire, PAC_QBZ4, 0.0)*gamma_z
        +pac2002_parameter(tire, PAC_QBZ5, 0.0)*d_abs(gamma_z, smooth),
        smooth
    )*pac2002_parameter(tire, PAC_LKY, 1.0)
        /std::max(pac2002_parameter(tire, PAC_LMUY, 1.0), 1e-9);
    DirectionalScalar et = (
        pac2002_parameter(tire, PAC_QEZ1, 0.0)
        +pac2002_parameter(tire, PAC_QEZ2, 0.0)*dfz
        +pac2002_parameter(tire, PAC_QEZ3, 0.0)*dfz*dfz
    )*(
        1.0+(
            pac2002_parameter(tire, PAC_QEZ4, 0.0)
            +pac2002_parameter(tire, PAC_QEZ5, 0.0)*gamma_z
        )*(2.0/kPi)*d_atan2(
                bt*pac2002_parameter(tire, PAC_QCZ1, 1.0)*alpha_t,
                {1.0, 0.0}
            )
    );
    if (et.value > 1.0) {
        smooth = false;
        et = {1.0, 0.0};
    }
    const DirectionalScalar dt = normal_force* (
        pac2002_parameter(tire, PAC_QDZ1, 0.0)
        +pac2002_parameter(tire, PAC_QDZ2, 0.0)*dfz
    )* (
        1.0
        +pac2002_parameter(tire, PAC_QDZ3, 0.0)*gamma_z
        +pac2002_parameter(tire, PAC_QDZ4, 0.0)*gamma_z*gamma_z
    )*tire.radius/reference_load
        *(1.0-pac2002_parameter(tire, PAC_QPZ1, 0.0)*dpi)
        *pac2002_parameter(tire, PAC_LTR, 1.0);
    const auto trail = [&](const DirectionalScalar& value) {
        const DirectionalScalar argument = d_clamp(
            bt*value, -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
        );
        return dt*d_cos(
            ct*d_atan2(
                argument-et*(argument-d_atan2(argument, {1.0, 0.0})),
                {1.0, 0.0}
            )
        )*d_cos(alpha);
    };
    const DirectionalScalar dy = pac2002_force_limit_directional(
        tire, normal_force, true, camber, smooth
    );
    const DirectionalScalar by = ky/(ct*dy+0.1);
    const DirectionalScalar br = pac2002_parameter(tire, PAC_QBZ9, 0.0)
        *pac2002_parameter(tire, PAC_LKY, 1.0)
        /std::max(pac2002_parameter(tire, PAC_LMUY, 1.0), 1e-9)
        +pac2002_parameter(tire, PAC_QBZ10, 0.0)*by*ct;
    const DirectionalScalar dr = normal_force* (
        (
            pac2002_parameter(tire, PAC_QDZ6, 0.0)
            +pac2002_parameter(tire, PAC_QDZ7, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LRES, 1.0)
        +(
            pac2002_parameter(tire, PAC_QDZ8, 0.0)
            +pac2002_parameter(tire, PAC_QDZ9, 0.0)*dfz
        )*(1.0+pac2002_parameter(tire, PAC_QPZ2, 0.0)*dpi)*gamma_z
    )*tire.radius*pac2002_parameter(tire, PAC_LMUY, 1.0);
    DirectionalScalar mz = trail(alpha_r)*fy
        -dr*d_cos(d_atan2(br*alpha_r, {1.0, 0.0}))*d_cos(alpha);
    if (pac2002_has_combined_slip_terms(tire)) {
        const DirectionalScalar lateral_peak = dy;
        const DirectionalScalar velocity_offset = lateral_peak* (
            pac2002_parameter(tire, PAC_RVY1, 0.0)
            +pac2002_parameter(tire, PAC_RVY2, 0.0)*dfz
            +pac2002_parameter(tire, PAC_RVY3, 0.0)*camber
        )*d_cos(d_atan2(
            pac2002_parameter(tire, PAC_RVY4, 0.0)*alpha, {1.0, 0.0}
        ));
        const DirectionalScalar svyk = -velocity_offset*d_sin(
            pac2002_parameter(tire, PAC_RVY5, 0.0)*d_atan2(
                pac2002_parameter(tire, PAC_RVY6, 0.0)*kappa,
                {1.0, 0.0}
            )
        );
        const DirectionalScalar tan_t = d_sin(alpha_t)/d_cos(alpha_t);
        const DirectionalScalar tan_r = d_sin(alpha_r)/d_cos(alpha_r);
        const DirectionalScalar combined_tangent = d_sqrt(
            tan_t*tan_t+(kx/ky)*(kx/ky)*kappa*kappa
        );
        const DirectionalScalar combined_radial = d_sqrt(
            tan_r*tan_r+(kx/ky)*(kx/ky)*kappa*kappa
        );
        const double sign_kappa = d_sign(kappa, smooth);
        const double sign_alpha_r = d_sign(alpha_r, smooth);
        const DirectionalScalar alpha_teq = d_atan2(
            combined_tangent, {1.0, 0.0}
        )*sign_kappa;
        const DirectionalScalar alpha_req = d_atan2(
            combined_radial, {1.0, 0.0}
        )*sign_alpha_r;
        const DirectionalScalar s = (
            pac2002_parameter(tire, PAC_SSZ1, 0.0)
            -pac2002_parameter(tire, PAC_SSZ2, 0.0)*fy/reference_load
            +(
                pac2002_parameter(tire, PAC_SSZ3, 0.0)
                +pac2002_parameter(tire, PAC_SSZ4, 0.0)*dfz
            )*gamma_z
        )*tire.radius*pac2002_parameter(tire, PAC_LS, 1.0);
        mz = trail(alpha_teq)*(fy-svyk)
            -dr*d_cos(d_atan2(br*alpha_req, {1.0, 0.0}))*d_cos(alpha)
            -s*fx;
    }
    if (!std::isfinite(mz.value) || !std::isfinite(mz.derivative)) {
        smooth = false;
        return {};
    }
    return mz*(-1.0);
}

DirectionalScalar pac2002_overturning_moment_directional(
    const Tire& tire, const DirectionalScalar& fy_source,
    const DirectionalScalar& normal_force, const DirectionalScalar& camber,
    bool& smooth
) {
    if (!pac2002_has_overturning_moment_terms(tire)) return {};
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double reference_load = pac2002_reference_load(tire);
    const double pressure_difference = pac2002_pressure_difference(tire);
    const DirectionalScalar fy_ratio = fy_source/reference_load;
    const DirectionalScalar load_ratio = normal_force/reference_load;
    const DirectionalScalar bracket =
        pac2002_parameter(tire, PAC_QSX3, 0.0)*fy_ratio
        +pac2002_parameter(tire, PAC_QSX4, 0.0)
            *d_cos(pac2002_parameter(tire, PAC_QSX5, 0.0)
                *d_atan2(load_ratio*load_ratio, {1.0, 0.0}))
            *d_sin(
                pac2002_parameter(tire, PAC_QSX7, 0.0)*camber
                +pac2002_parameter(tire, PAC_QSX8, 0.0)
                    *d_atan2(
                        pac2002_parameter(tire, PAC_QSX9, 0.0)*fy_ratio,
                        {1.0, 0.0}
                    )
            )
        +(
            pac2002_parameter(tire, PAC_QSX10, 0.0)
                *d_atan2(
                    pac2002_parameter(tire, PAC_QSX11, 0.0)*load_ratio,
                    {1.0, 0.0}
                )
            -pac2002_parameter(tire, PAC_QSX2, 0.0)
                *(1.0+pac2002_parameter(tire, PAC_QPX1, 0.0)
                    *pressure_difference)
        )*camber
        +pac2002_parameter(tire, PAC_QSX1, 0.0)
            *pac2002_parameter(tire, PAC_LVMX, 1.0);
    const DirectionalScalar moment = normal_force*tire.radius*bracket
        *pac2002_parameter(tire, PAC_LMX, 1.0);
    if (!std::isfinite(moment.value) || !std::isfinite(moment.derivative)) {
        smooth = false;
        return {};
    }
    return moment;
}

DirectionalScalar pac2002_rolling_resistance_moment_directional(
    const Tire& tire, const DirectionalScalar& fx_source,
    const DirectionalScalar& normal_force, const DirectionalScalar& camber,
    const DirectionalScalar& longitudinal_speed, bool& smooth
) {
    if (!pac2002_has_rolling_resistance_terms(tire)) return {};
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double pressure_ratio = std::max(
        pac2002_parameter(tire, PAC_IP, 200000.0)
            /std::max(pac2002_parameter(tire, PAC_IP_NOM, 200000.0), 1e-9),
        1e-12
    );
    const double reference_speed = std::sqrt(9.81*tire.radius);
    const DirectionalScalar speed_ratio = longitudinal_speed/
        std::max(reference_speed, 1e-9);
    const DirectionalScalar load_ratio = normal_force/nominal;
    const DirectionalScalar bracket =
        pac2002_parameter(tire, PAC_QSY1, 0.0)
        +pac2002_parameter(tire, PAC_QSY2, 0.0)*fx_source/nominal
        +pac2002_parameter(tire, PAC_QSY3, 0.0)
            *d_abs(speed_ratio, smooth)
        +pac2002_parameter(tire, PAC_QSY4, 0.0)
            *speed_ratio*speed_ratio*speed_ratio*speed_ratio
        +pac2002_parameter(tire, PAC_QSY5, 0.0)*camber*camber
        +pac2002_parameter(tire, PAC_QSY6, 0.0)*camber*camber*load_ratio;
    const DirectionalScalar load_scale = d_pow_positive(
        load_ratio, pac2002_parameter(tire, PAC_QSY7, 0.0), smooth
    );
    const double pressure_scale = std::pow(
        pressure_ratio, pac2002_parameter(tire, PAC_QSY8, 0.0)
    );
    const DirectionalScalar moment = normal_force*tire.radius*bracket
        *load_scale*pressure_scale*pac2002_parameter(tire, PAC_LMY, 1.0);
    if (!std::isfinite(moment.value) || !std::isfinite(moment.derivative)) {
        smooth = false;
        return {};
    }
    // PAC2002 的 My 为 SAE 轮胎坐标约定；native 侧向轴与 SAE y 轴相反。
    return -moment;
}

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

DVec3 operator+(const DVec3& a, const DVec3& b) {
    return {a.x+b.x, a.y+b.y, a.z+b.z};
}
DVec3 operator-(const DVec3& a, const DVec3& b) {
    return {a.x-b.x, a.y-b.y, a.z-b.z};
}
DVec3 operator-(const DVec3& a) { return {-a.x, -a.y, -a.z}; }
DVec3 operator*(const DVec3& a, const DirectionalScalar& b) {
    return {a.x*b, a.y*b, a.z*b};
}
DVec3 operator*(const DVec3& a, double b) {
    return {a.x*b, a.y*b, a.z*b};
}
DVec3 operator/(const DVec3& a, const DirectionalScalar& b) {
    return {a.x/b, a.y/b, a.z/b};
}
DirectionalScalar d_dot(const DVec3& a, const DVec3& b) {
    return a.x*b.x+a.y*b.y+a.z*b.z;
}
DVec3 d_cross(const DVec3& a, const DVec3& b) {
    return {
        a.y*b.z-a.z*b.y,
        a.z*b.x-a.x*b.z,
        a.x*b.y-a.y*b.x
    };
}
DirectionalScalar d_norm(const DVec3& a) { return d_sqrt(d_dot(a, a)); }
DVec3 d_normalized(const DVec3& a, bool& smooth) {
    if (
        a.x.derivative == 0.0 && a.y.derivative == 0.0 &&
        a.z.derivative == 0.0
    ) {
        const Vec3 value = normalized(a.value());
        if (norm(a.value()) <= kEps) smooth = false;
        return {value.x, value.y, value.z};
    }
    const DirectionalScalar length = d_norm(a);
    if (length.value <= kEps) {
        smooth = false;
        return {};
    }
    return a/length;
}

struct DMat3 {
    DirectionalScalar a[3][3]{};
};

DMat3 d_identity3() {
    DMat3 r{};
    r.a[0][0] = r.a[1][1] = r.a[2][2] = 1.0;
    return r;
}

Mat3 so3_left_jacobian(const Vec3& phi) {
    const Mat3 s = skew(phi);
    const double angle = norm(phi);
    if (angle < 1e-6) {
        return identity3()+s*0.5+(s*s)*(1.0/6.0);
    }
    const double angle_squared = angle*angle;
    const double linear = (1.0-std::cos(angle))/angle_squared;
    const double quadratic =
        (angle-std::sin(angle))/(angle_squared*angle);
    return identity3()+s*linear+(s*s)*quadratic;
}
DMat3 d_transpose(const DMat3& m) {
    DMat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) r.a[i][j] = m.a[j][i];
    }
    return r;
}
DMat3 operator+(const DMat3& a, const DMat3& b) {
    DMat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) r.a[i][j] = a.a[i][j]+b.a[i][j];
    }
    return r;
}
DMat3 operator-(const DMat3& a, const DMat3& b) {
    DMat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) r.a[i][j] = a.a[i][j]-b.a[i][j];
    }
    return r;
}
DMat3 operator*(const DMat3& a, const DirectionalScalar& b) {
    DMat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) r.a[i][j] = a.a[i][j]*b;
    }
    return r;
}
DMat3 operator*(const DMat3& a, double b) { return a*DirectionalScalar(b); }
DMat3 d_skew(const DVec3& v) {
    DMat3 r{};
    r.a[0][1] = -v.z; r.a[0][2] = v.y;
    r.a[1][0] = v.z;  r.a[1][2] = -v.x;
    r.a[2][0] = -v.y; r.a[2][1] = v.x;
    return r;
}
DMat3 d_outer(const DVec3& a, const DVec3& b) {
    DMat3 r{};
    const DirectionalScalar av[3] = {a.x, a.y, a.z};
    const DirectionalScalar bv[3] = {b.x, b.y, b.z};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) r.a[i][j] = av[i]*bv[j];
    }
    return r;
}
DVec3 d_row_times(const DVec3& e, const DMat3& m) {
    const DMat3 transpose_m = d_transpose(m);
    return {
        transpose_m.a[0][0]*e.x+transpose_m.a[0][1]*e.y
            +transpose_m.a[0][2]*e.z,
        transpose_m.a[1][0]*e.x+transpose_m.a[1][1]*e.y
            +transpose_m.a[1][2]*e.z,
        transpose_m.a[2][0]*e.x+transpose_m.a[2][1]*e.y
            +transpose_m.a[2][2]*e.z
    };
}
DMat3 operator*(const DMat3& a, const DMat3& b) {
    DMat3 r{};
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) r.a[i][j] += a.a[i][k]*b.a[k][j];
        }
    }
    return r;
}
DVec3 operator*(const DMat3& a, const DVec3& v) {
    return {
        a.a[0][0]*v.x+a.a[0][1]*v.y+a.a[0][2]*v.z,
        a.a[1][0]*v.x+a.a[1][1]*v.y+a.a[1][2]*v.z,
        a.a[2][0]*v.x+a.a[2][1]*v.y+a.a[2][2]*v.z
    };
}

struct DQuat {
    DirectionalScalar w{1.0}, x{}, y{}, z{};

    DQuat() = default;
    DQuat(
        const DirectionalScalar& w_, const DirectionalScalar& x_,
        const DirectionalScalar& y_, const DirectionalScalar& z_
    ) : w(w_), x(x_), y(y_), z(z_) {}
};

DQuat d_qmul(const DQuat& a, const DQuat& b) {
    return {
        a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
        a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
        a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
        a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w
    };
}
DQuat d_qconj(const DQuat& q) { return {q.w, -q.x, -q.y, -q.z}; }
DQuat d_qnormalize(const DQuat& q) {
    const DirectionalScalar length = d_sqrt(
        q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z
    );
    return {q.w/length, q.x/length, q.y/length, q.z/length};
}
DMat3 d_qmat(const DQuat& q_) {
    if (
        q_.w.derivative == 0.0 && q_.x.derivative == 0.0 &&
        q_.y.derivative == 0.0 && q_.z.derivative == 0.0
    ) {
        const Mat3 value = qmat({q_.w.value, q_.x.value, q_.y.value, q_.z.value});
        DMat3 result{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                result.a[row][col] = {value.a[row][col], 0.0};
            }
        }
        return result;
    }
    const DQuat q = d_qnormalize(q_);
    DMat3 r{};
    r.a[0][0] = 1.0-2.0*(q.y*q.y+q.z*q.z);
    r.a[0][1] = 2.0*(q.x*q.y-q.z*q.w);
    r.a[0][2] = 2.0*(q.x*q.z+q.y*q.w);
    r.a[1][0] = 2.0*(q.x*q.y+q.z*q.w);
    r.a[1][1] = 1.0-2.0*(q.x*q.x+q.z*q.z);
    r.a[1][2] = 2.0*(q.y*q.z-q.x*q.w);
    r.a[2][0] = 2.0*(q.x*q.z-q.y*q.w);
    r.a[2][1] = 2.0*(q.y*q.z+q.x*q.w);
    r.a[2][2] = 1.0-2.0*(q.x*q.x+q.y*q.y);
    return r;
}

DVec3 d_cardan_xyz_from_rotation(const DMat3& rotation, bool& smooth) {
    const DirectionalScalar cosine_y = d_sqrt(
        rotation.a[0][0]*rotation.a[0][0]
        +rotation.a[0][1]*rotation.a[0][1]
    );
    if (cosine_y.value <= 1e-10) smooth = false;
    return {
        d_atan2(-rotation.a[1][2], rotation.a[2][2]),
        d_atan2(rotation.a[0][2], cosine_y),
        d_atan2(-rotation.a[0][1], rotation.a[0][0])
    };
}

DVec3 d_cardan_xyz_rate(
    const DVec3& angles, const DVec3& relative_omega, bool& smooth
) {
    const DirectionalScalar sine_x = d_sin(angles.x);
    const DirectionalScalar cosine_x = d_cos(angles.x);
    const DirectionalScalar sine_y = d_sin(angles.y);
    const DirectionalScalar cosine_y = d_cos(angles.y);
    if (std::abs(cosine_y.value) <= 1e-10) smooth = false;
    const DirectionalScalar y_rate =
        cosine_x*relative_omega.y+sine_x*relative_omega.z;
    const DirectionalScalar z_rate =
        (-sine_x*relative_omega.y+cosine_x*relative_omega.z)/cosine_y;
    return {
        relative_omega.x-sine_y*z_rate,
        y_rate,
        z_rate
    };
}

DQuat d_body_quaternion(const Quat& q, const Vec3& dtheta) {
    if (dtheta.x == 0.0 && dtheta.y == 0.0 && dtheta.z == 0.0) {
        return {{q.w, 0.0}, {q.x, 0.0}, {q.y, 0.0}, {q.z, 0.0}};
    }
    const Quat tangent = qmul({0.0, dtheta.x, dtheta.y, dtheta.z}, q);
    return {
        {q.w, 0.5*tangent.w}, {q.x, 0.5*tangent.x},
        {q.y, 0.5*tangent.y}, {q.z, 0.5*tangent.z}
    };
}

DVec3 d_qlog(DQuat q, bool& smooth) {
    if (
        q.w.derivative == 0.0 && q.x.derivative == 0.0 &&
        q.y.derivative == 0.0 && q.z.derivative == 0.0
    ) {
        if (std::abs(q.w.value) <= 1e-12) smooth = false;
        const Vec3 value = qlog({q.w.value, q.x.value, q.y.value, q.z.value});
        return {value.x, value.y, value.z};
    }
    q = d_qnormalize(q);
    if (q.w.value < 0.0) {
        q.w = -q.w; q.x = -q.x; q.y = -q.y; q.z = -q.z;
    }
    const DirectionalScalar sine = d_sqrt(q.x*q.x+q.y*q.y+q.z*q.z);
    if (std::abs(q.w.value) <= 1e-12) {
        smooth = false;
    }
    if (sine.value < 1e-10) return {2.0*q.x, 2.0*q.y, 2.0*q.z};
    const DirectionalScalar angle = 2.0*d_atan2(sine, q.w);
    return {angle*q.x/sine, angle*q.y/sine, angle*q.z/sine};
}

DMat3 d_log_left_jacobian_inverse(const DVec3& phi, bool& smooth) {
    if (
        phi.x.derivative == 0.0 && phi.y.derivative == 0.0 &&
        phi.z.derivative == 0.0
    ) {
        const double angle = norm(phi.value());
        if (std::abs(angle-1e-4) <= 1e-12) smooth = false;
        const Mat3 value = log_left_jacobian_inverse(phi.value());
        DMat3 result{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                result.a[row][col] = {value.a[row][col], 0.0};
            }
        }
        return result;
    }
    const DMat3 s = d_skew(phi);
    const DirectionalScalar angle = d_norm(phi);
    DirectionalScalar quadratic{1.0/12.0, 0.0};
    if (angle.value > 1e-4) {
        const DirectionalScalar sine = d_sin(angle);
        quadratic =
            1.0/(angle*angle)
            -(1.0+d_cos(angle))/(2.0*angle*sine);
    } else if (std::abs(angle.value-1e-4) <= 1e-12) {
        smooth = false;
    }
    return d_identity3()-s*0.5+(s*s)*quadratic;
}

DVec3 d_rotate(
    const Quat& q, const Vec3& local, const Vec3& dtheta
) {
    if (dtheta.x == 0.0 && dtheta.y == 0.0 && dtheta.z == 0.0) {
        const Vec3 value = rotate(q, local);
        return {value.x, value.y, value.z};
    }
    return d_qmat(d_body_quaternion(q, dtheta))*DVec3(local.x, local.y, local.z);
}

DVec3 d_state_point(
    const State& state, const std::vector<Vec3>& dr,
    const std::vector<Vec3>& dtheta, int body, const Vec3& local
) {
    const DVec3 arm = d_rotate(state.q[body], local, dtheta[body]);
    return {
        {state.r[body].x+arm.x.value, dr[body].x+arm.x.derivative},
        {state.r[body].y+arm.y.value, dr[body].y+arm.y.derivative},
        {state.r[body].z+arm.z.value, dr[body].z+arm.z.derivative}
    };
}

DVec3 d_state_point_velocity(
    const State& state, const std::vector<Vec3>& dr,
    const std::vector<Vec3>& dtheta, const std::vector<Vec3>& dv,
    const std::vector<Vec3>& domega, int body, const Vec3& local
) {
    if (
        dtheta[body].x == 0.0 && dtheta[body].y == 0.0 &&
        dtheta[body].z == 0.0 && dv[body].x == 0.0 &&
        dv[body].y == 0.0 && dv[body].z == 0.0 &&
        domega[body].x == 0.0 && domega[body].y == 0.0 &&
        domega[body].z == 0.0
    ) {
        const Vec3 value = state_point_velocity(state, body, local);
        return {value.x, value.y, value.z};
    }
    const DVec3 arm = d_rotate(state.q[body], local, dtheta[body]);
    const DVec3 omega{
        {state.omega[body].x, domega[body].x},
        {state.omega[body].y, domega[body].y},
        {state.omega[body].z, domega[body].z}
    };
    const DVec3 velocity{
        {state.v[body].x, dv[body].x},
        {state.v[body].y, dv[body].y},
        {state.v[body].z, dv[body].z}
    };
    (void)dr;
    return velocity+d_cross(omega, arm);
}

struct DirectionalState {
    std::vector<Vec3> dr, dtheta, dv, domega;
    std::vector<double> dsx, dsy;
};

struct DirectionalRoadProfile {
    DirectionalScalar height{};
    DirectionalScalar slope{};
};

DirectionalRoadProfile road_profile_directional(
    const Model& model, const State& state,
    const DirectionalState& direction, std::size_t tire_index,
    bool& smooth
) {
    const RoadProfile& profile = model.road_profile;
    if (profile.kind == 0 || tire_index >= model.tires.size()) return {};
    const Tire& tire = model.tires[tire_index];
    const DVec3 center = d_state_point(
        state, direction.dr, direction.dtheta,
        tire_center_body(tire), tire_center_local(tire)
    );
    const DirectionalScalar x = center.x;
    const double corner_scale = tire_index < profile.corner_scale.size()
        ? profile.corner_scale[tire_index] : 1.0;
    const double amplitude = profile.amplitude * corner_scale;
    DirectionalRoadProfile result;
    result.height = profile.origin_z;
    const DirectionalScalar distance =
        x-profile.origin_x-profile.bump_start;
    if (profile.kind == 2) {
        const double wave_number = 2.0*kPi/profile.wavelength;
        const DirectionalScalar angle =
            wave_number*(x-profile.origin_x)+profile.phase;
        result.height += amplitude*d_sin(angle);
        result.slope = amplitude*wave_number*d_cos(angle);
    } else if (profile.kind == 3 || profile.kind == 5) {
        if (distance.value > 0.0 && distance.value < profile.bump_length) {
            const DirectionalScalar angle =
                2.0*kPi*distance/profile.bump_length;
            result.height +=
                0.5*amplitude*(1.0-d_cos(angle));
            result.slope = amplitude*kPi/profile.bump_length*d_sin(angle);
        } else {
            if (
                std::abs(distance.value) <= 1e-12 ||
                std::abs(distance.value-profile.bump_length) <= 1e-12
            ) {
                smooth = false;
            }
        }
    } else if (profile.kind == 4) {
        for (const auto& term : std::array<Vec3, 3>{
                 Vec3{1.0, 1.0, 0.0},
                 Vec3{2.7, 0.6, 1.2},
                 Vec3{5.1, 0.35, 2.4}
             }) {
            const double wave_number = 2.0*kPi*term.x
                / profile.wavelength;
            const DirectionalScalar angle =
                wave_number*(x-profile.origin_x)
                + profile.phase + term.z;
            result.height += amplitude*term.y*d_sin(angle);
            result.slope += amplitude*term.y*wave_number*d_cos(angle);
        }
    }
    return result;
}

struct DirectionalForceScratch {
    std::vector<Vec3> force;
    std::vector<Vec3> torque;
};

bool directional_vec_active(const Vec3& value) {
    return value.x != 0.0 || value.y != 0.0 || value.z != 0.0;
}

bool directional_body_active(const DirectionalState& direction, int body) {
    return directional_vec_active(direction.dr[static_cast<std::size_t>(body)])
        || directional_vec_active(
            direction.dtheta[static_cast<std::size_t>(body)]
        )
        || directional_vec_active(direction.dv[static_cast<std::size_t>(body)])
        || directional_vec_active(
            direction.domega[static_cast<std::size_t>(body)]
        );
}

bool directional_orientation_active(
    const DirectionalState& direction, int body
) {
    return directional_vec_active(
        direction.dtheta[static_cast<std::size_t>(body)]
    );
}

bool directional_inertial_active(
    const DirectionalState& direction, int body
) {
    return directional_orientation_active(direction, body)
        || directional_vec_active(
            direction.domega[static_cast<std::size_t>(body)]
        );
}

void reset_directional_state(
    const Model& model, DirectionalState& direction
) {
    const std::size_t body_count = model.bodies.size();
    const std::size_t tire_count = model.tires.size();
    direction.dr.resize(body_count);
    direction.dtheta.resize(body_count);
    direction.dv.resize(body_count);
    direction.domega.resize(body_count);
    direction.dsx.resize(tire_count);
    direction.dsy.resize(tire_count);
    std::fill(direction.dr.begin(), direction.dr.end(), Vec3{});
    std::fill(direction.dtheta.begin(), direction.dtheta.end(), Vec3{});
    std::fill(direction.dv.begin(), direction.dv.end(), Vec3{});
    std::fill(direction.domega.begin(), direction.domega.end(), Vec3{});
    std::fill(direction.dsx.begin(), direction.dsx.end(), 0.0);
    std::fill(direction.dsy.begin(), direction.dsy.end(), 0.0);
}

void ensure_directional_state(
    const Model& model, DirectionalState& direction
) {
    reset_directional_state(model, direction);
}

void add_directional_force_at_arm(
    std::vector<Vec3>& force, std::vector<Vec3>& torque,
    const Model& model, int body, const DVec3& arm, const DVec3& f
) {
    if (body < 0 || model.bodies[body].fixed) return;
    force[body] += Vec3{f.x.derivative, f.y.derivative, f.z.derivative};
    const Vec3 dtorque =
        cross(arm.value(), Vec3{f.x.derivative, f.y.derivative, f.z.derivative})
        + cross(
            Vec3{arm.x.derivative, arm.y.derivative, arm.z.derivative},
            Vec3{f.x.value, f.y.value, f.z.value}
        );
    torque[body] += dtorque;
}

void add_directional_torque(
    std::vector<Vec3>& torque, const Model& model, int body,
    const DVec3& value
) {
    if (body < 0 || model.bodies[body].fixed) return;
    torque[body] += Vec3{value.x.derivative, value.y.derivative, value.z.derivative};
}

Vec3 DVec3::value() const {
    return {x.value, y.value, z.value};
}

DirectionalScalar interpolated_curve_directional(
    const std::vector<double>& x, const std::vector<double>& y,
    const DirectionalScalar& value, bool& smooth, bool akima = false
) {
    if (x.empty() || y.empty()) return {};
    if (akima) {
        const auto value_slope = interpolate_akima_curve(x, y, value.value);
        if (value.value <= x.front() || value.value >= x.back()) {
            smooth = false;
        }
        return {
            value_slope.first,
            value_slope.second*value.derivative
        };
    }
    const auto slopes_match = [&](std::size_t knot) {
        if (knot == 0 || knot+1 >= x.size()) return false;
        const double left_span = x[knot]-x[knot-1];
        const double right_span = x[knot+1]-x[knot];
        if (left_span <= 0.0 || right_span <= 0.0) return false;
        const double left_slope = (y[knot]-y[knot-1])/left_span;
        const double right_slope = (y[knot+1]-y[knot])/right_span;
        return std::abs(left_slope-right_slope) <=
            1.0e-10*std::max({1.0, std::abs(left_slope), std::abs(right_slope)});
    };
    double slope = 0.0;
    if (x.size() > 1 && value.value > x.front() && value.value < x.back()) {
        std::size_t high = 1;
        while (high < x.size() && x[high] < value.value) ++high;
        const double x0 = x[high-1], x1 = x[high];
        const double span = x1-x0;
        if (span > 0.0) slope = (y[high]-y[high-1])/span;
        if (std::abs(value.value-x[high-1]) <= 1e-12) {
            if (!slopes_match(high-1)) smooth = false;
        }
        if (std::abs(value.value-x[high]) <= 1e-12) {
            if (!slopes_match(high)) smooth = false;
        }
    } else if (x.size() > 1) {
        for (std::size_t knot = 0; knot < x.size(); ++knot) {
            if (std::abs(value.value-x[knot]) <= 1e-12 &&
                !slopes_match(knot)) {
                smooth = false;
            }
        }
    }
    return {interpolate_curve(x, y, value.value), slope*value.derivative};
}

bool external_force_directional(
    const Model& model, const State& state, const SampleInput& input,
    const DirectionalState& direction, std::vector<double>& generalized_force,
    std::vector<double>& tire_brush_derivatives, bool brush_only = false,
    const StaticContactOverride* static_contact = nullptr,
    DirectionalForceScratch* scratch = nullptr
) {
    const int body_count = static_cast<int>(model.bodies.size());
    const int n = model.ndof;
    bool smooth = true;
    std::vector<Vec3> local_force;
    std::vector<Vec3> local_torque;
    std::vector<Vec3>& force = scratch == nullptr
        ? local_force : scratch->force;
    std::vector<Vec3>& torque = scratch == nullptr
        ? local_torque : scratch->torque;
    force.resize(static_cast<std::size_t>(body_count));
    torque.resize(static_cast<std::size_t>(body_count));
    std::fill(force.begin(), force.end(), Vec3{});
    std::fill(torque.begin(), torque.end(), Vec3{});
    tire_brush_derivatives.assign(model.tires.size()*2, 0.0);
    const double external_load_scale = static_contact == nullptr
        ? 1.0 : static_contact->external_load_scale;
    const double internal_force_scale = static_contact == nullptr
        ? 1.0 : static_contact->internal_force_scale;

    if (!brush_only) {
        for (const AerodynamicDrag& drag : model.aerodynamic_drags) {
            if (!directional_body_active(direction, drag.body)) continue;
            const DVec3 axis = d_normalized(
                d_rotate(
                    state.q[drag.body], drag.forward_axis,
                    direction.dtheta[drag.body]
                ),
                smooth
            );
            const DVec3 point_velocity = d_state_point_velocity(
                state, direction.dr, direction.dtheta, direction.dv,
                direction.domega, drag.body, drag.application_point
            );
            const DirectionalScalar longitudinal_speed = d_dot(
                point_velocity, axis
            );
            // v*|v| is continuously differentiable at zero even though |v|
            // alone is not.  Differentiate the complete drag law so a
            // zero-velocity static trim keeps its valid zero tangent.
            const double drag_scale =
                -external_load_scale*drag.coefficient;
            const DirectionalScalar force_value{
                drag_scale*std::abs(longitudinal_speed.value)
                    *longitudinal_speed.value,
                drag_scale*2.0*std::abs(longitudinal_speed.value)
                    *longitudinal_speed.derivative
            };
            add_directional_force_at_arm(
                force, torque, model, drag.body,
                d_rotate(
                    state.q[drag.body], drag.application_point,
                    direction.dtheta[drag.body]
                ),
                axis * force_value
            );
        }
        for (std::size_t i = 0; i < model.springs.size(); ++i) {
            const Spring& s = model.springs[i];
            if (
                !directional_body_active(direction, s.a) &&
                !directional_body_active(direction, s.b)
            ) {
                continue;
            }
            const DVec3 pa = d_state_point(
                state, direction.dr, direction.dtheta, s.a, s.pa
            );
            const DVec3 pb = d_state_point(
                state, direction.dr, direction.dtheta, s.b, s.pb
            );
            const DVec3 va = d_state_point_velocity(
                state, direction.dr, direction.dtheta, direction.dv,
                direction.domega, s.a, s.pa
            );
            const DVec3 vb = d_state_point_velocity(
                state, direction.dr, direction.dtheta, direction.dv,
                direction.domega, s.b, s.pb
            );
            const DVec3 d = pb-pa;
            const DirectionalScalar length = d_norm(d);
            if (length.value < 1e-10) {
                smooth = false;
                continue;
            }
            const DVec3 e = d/length;
            const DirectionalScalar dlength = d_dot(vb-va, e);
            // The compression/rebound damping branch is non-smooth at zero
            // relative speed. Detect an actual crossing under the same
            // perturbation used by the remaining finite-difference columns.
            constexpr double kJacobianStep = 1e-7;
            const double trial_dlength =
                dlength.value + kJacobianStep*dlength.derivative;
            if (
                static_contact == nullptr &&
                (std::abs(dlength.value) <= 1e-12 ||
                 dlength.value*trial_dlength <= 0.0)
            ) smooth = false;
            const DirectionalScalar compression = s.free_length-length;
            const double damping =
                dlength.value < 0.0 ? s.c_compression : s.c_rebound;
            const DirectionalScalar elastic_force = s.elastic_deflection.empty()
                ? s.k*compression
                : interpolated_curve_directional(
                    s.elastic_deflection, s.elastic_force,
                    compression, smooth
                );
            const bool has_curve = !s.damper_velocity.empty();
            const DirectionalScalar damping_force = has_curve
                ? (static_contact != nullptr
                    ? -DirectionalScalar{
                        interpolate_curve(
                            s.damper_velocity, s.damper_force, dlength.value
                        )
                    }
                    : -interpolated_curve_directional(
                        s.damper_velocity, s.damper_force, dlength, smooth
                    ))
                : -damping*dlength;
            DirectionalScalar scalar_force = elastic_force+damping_force;
            if (
                std::isfinite(s.minimum_length) &&
                length.value < s.minimum_length
            ) {
                const DirectionalScalar penetration =
                    s.minimum_length-length;
                scalar_force += s.compression_stop_penetration.empty()
                    ? s.compression_stop_k*penetration
                    : interpolated_curve_directional(
                        s.compression_stop_penetration,
                        s.compression_stop_force,
                        penetration,
                        smooth
                    );
                if (dlength.value < 0.0) {
                    scalar_force += -s.compression_stop_c*dlength;
                }
            } else if (
                std::isfinite(s.minimum_length) &&
                std::abs(length.value-s.minimum_length) <= 1e-12
            ) {
                smooth = false;
            }
            if (
                std::isfinite(s.maximum_length) &&
                length.value > s.maximum_length
            ) {
                const DirectionalScalar penetration =
                    length-s.maximum_length;
                scalar_force += s.rebound_stop_penetration.empty()
                    ? -s.rebound_stop_k*penetration
                    : -interpolated_curve_directional(
                        s.rebound_stop_penetration,
                        s.rebound_stop_force,
                        penetration,
                        smooth
                    );
                if (dlength.value > 0.0) {
                    scalar_force += -s.rebound_stop_c*dlength;
                }
            } else if (
                std::isfinite(s.maximum_length) &&
                std::abs(length.value-s.maximum_length) <= 1e-12
            ) {
                smooth = false;
            }
            scalar_force = internal_force_scale*scalar_force;
            const DVec3 f = e*scalar_force;
            add_directional_force_at_arm(
                force, torque, model, s.b,
                d_rotate(state.q[s.b], s.pb, direction.dtheta[s.b]), f
            );
            add_directional_force_at_arm(
                force, torque, model, s.a,
                d_rotate(state.q[s.a], s.pa, direction.dtheta[s.a]), -f
            );
        }

        for (const Bushing& b : model.bushings) {
            if (
                !directional_body_active(direction, b.a) &&
                !directional_body_active(direction, b.b)
            ) {
                continue;
            }
            const DVec3 pa = d_state_point(
                state, direction.dr, direction.dtheta, b.a, b.pa
            );
            const DVec3 pb = d_state_point(
                state, direction.dr, direction.dtheta, b.b, b.pb
            );
            const DVec3 va = d_state_point_velocity(
                state, direction.dr, direction.dtheta, direction.dv,
                direction.domega, b.a, b.pa
            );
            const DVec3 vb = d_state_point_velocity(
                state, direction.dr, direction.dtheta, direction.dv,
                direction.domega, b.b, b.pb
            );
            const DQuat qfa = d_qmul(
                d_body_quaternion(state.q[b.a], direction.dtheta[b.a]),
                DQuat{{b.frame_a.w}, {b.frame_a.x}, {b.frame_a.y}, {b.frame_a.z}}
            );
            const DQuat qfb = d_qmul(
                d_body_quaternion(state.q[b.b], direction.dtheta[b.b]),
                DQuat{{b.frame_b.w}, {b.frame_b.x}, {b.frame_b.y}, {b.frame_b.z}}
            );
            const DMat3 rfa = d_qmat(qfa);
            const DMat3 rt = d_transpose(rfa);
            const DVec3 rel = rt*(pb-pa);
            const DVec3 rel_v = rt*(vb-va);
            const DVec3 omega_a = rt*DVec3{
                {state.omega[b.a].x, direction.domega[b.a].x},
                {state.omega[b.a].y, direction.domega[b.a].y},
                {state.omega[b.a].z, direction.domega[b.a].z}
            };
            const DVec3 omega_b = rt*DVec3{
                {state.omega[b.b].x, direction.domega[b.b].x},
                {state.omega[b.b].y, direction.domega[b.b].y},
                {state.omega[b.b].z, direction.domega[b.b].z}
            };
            const DVec3 rel_rate = rel_v-d_cross(omega_a, rel);
            const DVec3 rel_omega = omega_b-omega_a;
            const DQuat qrel = d_qmul(d_qconj(qfa), qfb);
            const DQuat qdelta = d_qmul(
                DQuat{{b.reference.w}, { -b.reference.x},
                       { -b.reference.y}, { -b.reference.z}},
                qrel
            );
            const DVec3 rotation = b.rotation_coordinates ==
                    VEHICLE_BUSHING_CARDAN_XYZ
                ? d_cardan_xyz_from_rotation(d_qmat(qdelta), smooth)
                : d_qlog(qdelta, smooth);
            const DVec3 rotation_rate = b.rotation_coordinates ==
                    VEHICLE_BUSHING_CARDAN_XYZ
                ? d_cardan_xyz_rate(rotation, rel_omega, smooth)
                : rel_omega;
            const std::array<DirectionalScalar, 6> deformation{
                rel.x-b.reference_translation.x,
                rel.y-b.reference_translation.y,
                rel.z-b.reference_translation.z,
                rotation.x, rotation.y, rotation.z
            };
            const std::array<DirectionalScalar, 6> rate{
                rel_rate.x, rel_rate.y, rel_rate.z,
                rotation_rate.x, rotation_rate.y, rotation_rate.z
            };
            std::array<DirectionalScalar, 6> wrench{};
            for (int i = 0; i < 6; ++i) {
                const std::size_t axis = static_cast<std::size_t>(i);
                DirectionalScalar elastic{}, viscous{};
                if (!b.elastic_coordinate[axis].empty()) {
                    elastic = interpolated_curve_directional(
                        b.elastic_coordinate[axis], b.elastic_force[axis],
                        deformation[axis], smooth,
                        b.force_curve_interpolation == 1
                    );
                } else {
                    for (int j = 0; j < 6; ++j) {
                        elastic = elastic + b.stiffness[static_cast<std::size_t>(i*6+j)]
                            * deformation[static_cast<std::size_t>(j)];
                    }
                }
                for (int j = 0; j < 6; ++j) {
                    viscous = viscous + b.damping[static_cast<std::size_t>(i*6+j)]
                        * rate[static_cast<std::size_t>(j)];
                }
                wrench[static_cast<std::size_t>(i)] = internal_force_scale * (
                    b.preload[static_cast<std::size_t>(i)]-elastic-viscous
                );
            }
            const DVec3 f_local{
                wrench[0], wrench[1], wrench[2]
            };
            const DVec3 t_local{
                wrench[3], wrench[4], wrench[5]
            };
            const DVec3 f_world = rfa*f_local;
            const DVec3 t_world = rfa*t_local;
            const DVec3 marker_arm = pb-pa;
            add_directional_force_at_arm(
                force, torque, model, b.b,
                d_rotate(state.q[b.b], b.pb, direction.dtheta[b.b]), f_world
            );
            add_directional_force_at_arm(
                force, torque, model, b.a,
                d_rotate(state.q[b.a], b.pa, direction.dtheta[b.a]), -f_world
            );
            add_directional_torque(torque, model, b.b, t_world);
            add_directional_torque(
                torque, model, b.a,
                (t_world+d_cross(marker_arm, f_world))*(-1.0)
            );
        }

        for (const AntiRollBar& bar : model.anti_roll_bars) {
            if (
                !directional_body_active(direction, bar.a) &&
                !directional_body_active(direction, bar.b)
            ) {
                continue;
            }
            const DQuat qa = d_body_quaternion(
                state.q[bar.a], direction.dtheta[bar.a]
            );
            const DQuat qb = d_body_quaternion(
                state.q[bar.b], direction.dtheta[bar.b]
            );
            const DQuat qrel = d_qmul(d_qconj(qa), qb);
            const DVec3 phi = d_qlog(
                d_qmul(
                    DQuat{{bar.reference.w}, {-bar.reference.x},
                           {-bar.reference.y}, {-bar.reference.z}},
                    qrel
                ), smooth
            );
            const DVec3 axis_world = d_normalized(
                d_qmat(qa)*DVec3(bar.axis_a.x, bar.axis_a.y, bar.axis_a.z),
                smooth
            );
            const DVec3 axis_a_world =
                d_qmat(qa)*DVec3(bar.axis_a.x, bar.axis_a.y, bar.axis_a.z);
            const DVec3 omega_a{
                {state.omega[bar.a].x, direction.domega[bar.a].x},
                {state.omega[bar.a].y, direction.domega[bar.a].y},
                {state.omega[bar.a].z, direction.domega[bar.a].z}
            };
            const DVec3 omega_b{
                {state.omega[bar.b].x, direction.domega[bar.b].x},
                {state.omega[bar.b].y, direction.domega[bar.b].y},
                {state.omega[bar.b].z, direction.domega[bar.b].z}
            };
            const DirectionalScalar angle = d_dot(
                phi, DVec3(bar.axis_a.x, bar.axis_a.y, bar.axis_a.z)
            );
            const DirectionalScalar rate = d_dot(
                axis_a_world, omega_b-omega_a
            );
            const DirectionalScalar tau = internal_force_scale * (
                -bar.stiffness*angle-bar.damping*rate
            );
            add_directional_torque(torque, model, bar.b, axis_world*tau);
            add_directional_torque(torque, model, bar.a, -(axis_world*tau));
        }

        for (std::size_t steering_index = 0;
             steering_index < model.steering_actuators.size();
             ++steering_index) {
            const SteeringActuator& actuator =
                model.steering_actuators[steering_index];
            const double target = steering_index < input.steering_target.size()
                ? input.steering_target[steering_index] : 0.0;
            const double target_rate =
                steering_index < input.steering_target_rate.size()
                    ? input.steering_target_rate[steering_index] : 0.0;
            if (prescribed_steering(actuator)) continue;
            if (actuator.type == VEHICLE_STEERING_TRANSLATION) {
                const bool body_active =
                    directional_body_active(direction, actuator.body);
                const bool reaction_active = actuator.reaction_body >= 0 &&
                    directional_body_active(direction, actuator.reaction_body);
                if (!body_active && !reaction_active) continue;
                const DVec3 body_point = d_state_point(
                    state, direction.dr, direction.dtheta,
                    actuator.body, actuator.point_local
                );
                DVec3 reaction_point{};
                if (actuator.reaction_body >= 0) {
                    reaction_point = d_state_point(
                        state, direction.dr, direction.dtheta,
                        actuator.reaction_body, actuator.reaction_point_local
                    );
                }
                const DVec3 body_velocity = d_state_point_velocity(
                    state, direction.dr, direction.dtheta, direction.dv,
                    direction.domega, actuator.body, actuator.point_local
                );
                DVec3 reaction_velocity{};
                if (actuator.reaction_body >= 0) {
                    reaction_velocity = d_state_point_velocity(
                        state, direction.dr, direction.dtheta, direction.dv,
                        direction.domega, actuator.reaction_body,
                        actuator.reaction_point_local
                    );
                }
                DVec3 axis_world{
                    actuator.axis_local.x,
                    actuator.axis_local.y,
                    actuator.axis_local.z
                };
                if (actuator.reaction_body >= 0) {
                    const DQuat reaction_q = d_body_quaternion(
                        state.q[actuator.reaction_body],
                        direction.dtheta[actuator.reaction_body]
                    );
                    axis_world = d_normalized(
                        d_qmat(reaction_q) * axis_world, smooth
                    );
                }
                const DVec3 relative_position = body_point-reaction_point;
                const DVec3 relative_velocity = body_velocity-reaction_velocity;
                const DirectionalScalar displacement = d_dot(
                    axis_world, relative_position
                );
                const DirectionalScalar rate = d_dot(
                    axis_world, relative_velocity
                );
                const DirectionalScalar force_value =
                    actuator.stiffness * (target-displacement)
                    + actuator.damping * (target_rate-rate);
                const DVec3 force_world = axis_world*force_value;
                add_directional_force_at_arm(
                    force, torque, model, actuator.body,
                    d_rotate(state.q[actuator.body], actuator.point_local,
                             direction.dtheta[actuator.body]),
                    force_world
                );
                if (actuator.reaction_body >= 0) {
                    add_directional_force_at_arm(
                        force, torque, model, actuator.reaction_body,
                        d_rotate(state.q[actuator.reaction_body],
                                 actuator.reaction_point_local,
                                 direction.dtheta[actuator.reaction_body]),
                        -force_world
                    );
                }
            } else {
                const bool body_active =
                    directional_orientation_active(direction, actuator.body);
                const bool reaction_active = actuator.reaction_body >= 0 &&
                    directional_orientation_active(
                        direction, actuator.reaction_body
                    );
                if (!body_active && !reaction_active) continue;
                const DQuat body_q = d_body_quaternion(
                    state.q[actuator.body], direction.dtheta[actuator.body]
                );
                const DQuat reaction_q = actuator.reaction_body >= 0
                    ? d_body_quaternion(
                        state.q[actuator.reaction_body],
                        direction.dtheta[actuator.reaction_body]
                    )
                    : DQuat{};
                const DQuat relative = d_qmul(d_qconj(reaction_q), body_q);
                const DQuat reference_conjugate{
                    actuator.reference.w,
                    -actuator.reference.x,
                    -actuator.reference.y,
                    -actuator.reference.z
                };
                const DVec3 error_rotation = d_qlog(
                    d_qmul(reference_conjugate, relative), smooth
                );
                const Vec3 axis_reference = rotate(
                    actuator.reference, actuator.axis_local
                );
                const DVec3 axis_world = d_normalized(
                    d_qmat(reaction_q) * DVec3(
                        axis_reference.x, axis_reference.y, axis_reference.z
                    ), smooth
                );
                const DirectionalScalar angle = d_dot(
                    error_rotation,
                    DVec3(
                        actuator.axis_local.x, actuator.axis_local.y,
                        actuator.axis_local.z
                    )
                );
                const DVec3 body_omega{
                    {state.omega[actuator.body].x,
                     direction.domega[actuator.body].x},
                    {state.omega[actuator.body].y,
                     direction.domega[actuator.body].y},
                    {state.omega[actuator.body].z,
                     direction.domega[actuator.body].z}
                };
                const DVec3 reaction_omega = actuator.reaction_body >= 0
                    ? DVec3{
                        {state.omega[actuator.reaction_body].x,
                         direction.domega[actuator.reaction_body].x},
                        {state.omega[actuator.reaction_body].y,
                         direction.domega[actuator.reaction_body].y},
                        {state.omega[actuator.reaction_body].z,
                         direction.domega[actuator.reaction_body].z}
                    }
                    : DVec3{};
                const DirectionalScalar rate = d_dot(
                    axis_world, body_omega-reaction_omega
                );
                const DirectionalScalar torque_value =
                    actuator.stiffness * (target-angle)
                    + actuator.damping * (target_rate-rate);
                add_directional_torque(
                    torque, model, actuator.body, axis_world*torque_value
                );
                add_directional_torque(
                    torque, model, actuator.reaction_body,
                    -(axis_world*torque_value)
                );
            }
        }
    }

    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& t = model.tires[i];
        const int frame_body = tire_frame_body(t);
        const int center_body = tire_center_body(t);
        const Vec3 center_local = tire_center_local(t);
        const bool body_active = directional_body_active(direction, t.body);
        const bool frame_active = directional_body_active(direction, frame_body);
        const bool center_active = directional_body_active(direction, center_body);
        const bool brush_active =
            i < direction.dsx.size() &&
            (direction.dsx[i] != 0.0 || direction.dsy[i] != 0.0);
        const bool static_active =
            static_contact != nullptr && static_contact->active != nullptr &&
            i < static_contact->active->size() &&
            (*static_contact->active)[i] != 0;
        const bool static_compression_active =
            static_contact != nullptr &&
            static_contact->compression_derivative != nullptr &&
            i < static_contact->compression_derivative->size() &&
            (*static_contact->compression_derivative)[i] != 0.0;
        if (!body_active && !frame_active && !center_active && !brush_active &&
            !static_compression_active) {
            continue;
        }
        const DVec3 center = d_state_point(
            state, direction.dr, direction.dtheta, center_body, center_local
        );
        const DVec3 normal{0.0, 0.0, 1.0};
        const bool pac2002 =
            t.model_kind == VEHICLE_TIRE_PAC2002_PURE_SLIP
            || t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE;
        const bool fiala = t.model_kind == VEHICLE_TIRE_FIALA;
        if (static_contact != nullptr) {
            if (!static_active) continue;
            const double compression =
                static_contact->compression != nullptr &&
                i < static_contact->compression->size()
                ? (*static_contact->compression)[i] : 0.0;
            const double compression_derivative =
                static_contact->compression_derivative != nullptr &&
                i < static_contact->compression_derivative->size()
                ? (*static_contact->compression_derivative)[i] : 0.0;
            const DirectionalScalar normal_force = internal_force_scale * (
                pac2002
                    ? pac2002_vertical_force_directional(
                        t,
                        {compression, compression_derivative},
                        {},
                        {},
                        smooth
                    )
                    : DirectionalScalar{
                        t.k*compression,
                        t.k*compression_derivative
                    }
            );
            if (normal_force.value < 0.0) continue;
            const DVec3 body_origin{
                {state.r[t.body].x, direction.dr[t.body].x},
                {state.r[t.body].y, direction.dr[t.body].y},
                {state.r[t.body].z, direction.dr[t.body].z}
            };
            const DVec3 contact_center = d_state_point(
                state, direction.dr, direction.dtheta, t.body, t.center
            );
            const DVec3 contact_arm =
                contact_center-body_origin+normal*(-t.radius);
            add_directional_force_at_arm(
                force, torque, model, t.body, contact_arm,
                normal*normal_force
            );
            continue;
        }
        const DVec3 vc = d_state_point_velocity(
            state, direction.dr, direction.dtheta, direction.dv,
            direction.domega, center_body, center_local
        );
        const DirectionalRoadProfile road_profile = road_profile_directional(
            model, state, direction, i, smooth
        );
        const DirectionalScalar road = road_profile.height + (
            i < input.road_z.size() ? DirectionalScalar{input.road_z[i]}
                                     : DirectionalScalar{}
        );
        const DirectionalScalar road_v = (
            i < input.road_v.size() ? DirectionalScalar{input.road_v[i]}
                                    : DirectionalScalar{}
        ) + road_profile.slope*vc.x;
        DirectionalScalar delta = t.radius+road-center.z;
        DirectionalScalar delta_dot = road_v-vc.z;
        const DVec3 forward_raw = d_rotate(
            state.q[frame_body], t.forward_axis, direction.dtheta[frame_body]
        );
        DVec3 forward = forward_raw;
        forward.z = DirectionalScalar{};
        forward = d_normalized(forward, smooth);
        const DVec3 lateral = d_normalized(d_cross(normal, forward), smooth);
        const DVec3 omega{
            {state.omega[t.body].x, direction.domega[t.body].x},
            {state.omega[t.body].y, direction.domega[t.body].y},
            {state.omega[t.body].z, direction.domega[t.body].z}
        };
        const DVec3 spin_axis = d_normalized(
            d_rotate(
                state.q[frame_body], t.spin_axis,
                direction.dtheta[frame_body]
            ),
            smooth
        );
        DirectionalScalar loaded_radius = t.radius-d_clamp(
            delta, 0.0, t.radius, smooth
        );
        if (fiala && static_contact == nullptr) {
            const DirectionalScalar axis_normal = d_dot(spin_axis, normal);
            DirectionalScalar projection = d_sqrt(
                DirectionalScalar{1.0}-axis_normal*axis_normal
            );
            if (projection.value <= 1e-9) {
                smooth = false;
                projection = {1e-9, 0.0};
            }
            const DVec3 frame_omega{
                {state.omega[frame_body].x, direction.domega[frame_body].x},
                {state.omega[frame_body].y, direction.domega[frame_body].y},
                {state.omega[frame_body].z, direction.domega[frame_body].z}
            };
            const DirectionalScalar axis_normal_rate = d_dot(
                d_cross(frame_omega, spin_axis), normal
            );
            const DirectionalScalar projection_rate = -axis_normal
                *axis_normal_rate/projection;
            const DirectionalScalar vertical_gap = center.z-road;
            const DirectionalScalar vertical_gap_rate{
                vc.z.value-road_v.value,
                vc.z.derivative-road_v.derivative
            };
            loaded_radius = vertical_gap/projection;
            delta = t.radius-loaded_radius;
            const DirectionalScalar loaded_radius_rate =
                vertical_gap_rate/projection
                -vertical_gap*projection_rate/(projection*projection);
            delta_dot = -loaded_radius_rate;
        }
        DirectionalScalar camber{};
        if (pac2002 && static_contact == nullptr) {
            camber = d_clamp(
                d_atan2(
                    d_dot(spin_axis, normal), d_dot(spin_axis, lateral)
                ),
                -kPac2002CamberLimit,
                kPac2002CamberLimit,
                smooth
            );
        }
        DVec3 spin_omega = omega;
        if (fiala && frame_body != t.body) {
            spin_omega = spin_omega-DVec3{
                {state.omega[frame_body].x, direction.domega[frame_body].x},
                {state.omega[frame_body].y, direction.domega[frame_body].y},
                {state.omega[frame_body].z, direction.domega[frame_body].z}
            };
        }
        const DirectionalScalar spin_rate = d_dot(spin_omega, spin_axis);
        const DirectionalScalar rolling_radius =
            t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE
            ? pac2002_effective_rolling_radius_directional(
                t, delta, spin_rate, smooth
            )
            : ((pac2002 || fiala)
                ? loaded_radius
                : DirectionalScalar{t.radius});
        // 方向导数路径必须与标量路径采用同一模型相关的施力半径。
        const DirectionalScalar force_application_radius = pac2002 || fiala
            ? loaded_radius
            : DirectionalScalar{t.radius};
        DVec3 patch_arm = normal*(-force_application_radius);
        if (pac2002 || fiala) {
            const DVec3 radial_down = d_normalized(
                (normal-spin_axis*d_dot(spin_axis, normal))*(-1.0),
                smooth
            );
            DirectionalScalar vertical_projection = -d_dot(
                radial_down, normal
            );
            if (vertical_projection.value <= 1e-9) {
                smooth = false;
                vertical_projection = {1e-9, 0.0};
            }
            patch_arm = radial_down * (
                force_application_radius/vertical_projection
            );
        }
        const DVec3 rolling_arm = (pac2002 || fiala)
            ? patch_arm
            : normal*(-rolling_radius);
        const DVec3 patch_velocity = vc+d_cross(omega, rolling_arm);
        const DVec3 relative_patch_velocity = patch_velocity-DVec3(0.0,0.0,road_v);
        const DirectionalScalar vx =
            (t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE || fiala)
            ? d_dot(vc-DVec3(0.0,0.0,road_v), forward)
                -spin_rate*rolling_radius
            : d_dot(relative_patch_velocity, forward);
        const DirectionalScalar vy = d_dot(relative_patch_velocity, lateral);
        const DirectionalScalar sx{
            i < state.tire_sx.size() ? state.tire_sx[i] : 0.0,
            i < direction.dsx.size() ? direction.dsx[i] : 0.0
        };
        const DirectionalScalar sy{
            i < state.tire_sy.size() ? state.tire_sy[i] : 0.0,
            i < direction.dsy.size() ? direction.dsy[i] : 0.0
        };
        if (delta.value <= 0.0) {
            if (std::abs(delta.value) <= 1e-12) smooth = false;
            const DirectionalScalar detached_x = -sx/t.detached_relaxation;
            const DirectionalScalar detached_y = -sy/t.detached_relaxation;
            tire_brush_derivatives[2*i] = detached_x.derivative;
            tire_brush_derivatives[2*i+1] = detached_y.derivative;
            continue;
        }
        const DirectionalScalar trial_normal = pac2002
            ? pac2002_vertical_force_directional(
                t, delta, delta_dot, camber, smooth
            )
            : t.k*delta+t.c*delta_dot;
        if (std::abs(trial_normal.value) <= 1e-12) smooth = false;
        if (trial_normal.value <= 0.0) {
            const DirectionalScalar detached_x = -sx/t.detached_relaxation;
            const DirectionalScalar detached_y = -sy/t.detached_relaxation;
            tire_brush_derivatives[2*i] = detached_x.derivative;
            tire_brush_derivatives[2*i+1] = detached_y.derivative;
            continue;
        }
        const DirectionalScalar normal_force = trial_normal;
            const DirectionalScalar rolling_speed = d_abs(
                d_dot(vc, forward), smooth
            );
            if (pac2002) {
            // 与标量分支相同，Adams 高性能轮胎的局部求解器包含
            // USE_MODE=14 的纵向和侧向一阶松弛状态。
            DirectionalScalar slip_speed = rolling_speed;
            if (slip_speed.value < 1e-3) {
                smooth = false;
                slip_speed = {1e-3, 0.0};
            }
            const DirectionalScalar longitudinal_slip = d_clamp(
                -vx/slip_speed, -1.0, 1.0, smooth
            );
            const DirectionalScalar lateral_slip = d_clamp(
                d_atan2(vy, slip_speed),
                -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
            );
            const DirectionalScalar effective_longitudinal_slip = sx;
            const DirectionalScalar effective_lateral_slip = sy;
            const DirectionalScalar relaxation_length_longitudinal =
                pac2002_relaxation_length_directional(
                    t, normal_force, false, camber, smooth
                );
            const DirectionalScalar relaxation_length_lateral =
                pac2002_relaxation_length_directional(
                    t, normal_force, true, camber, smooth
                );
            const DirectionalScalar longitudinal_relaxation =
                rolling_speed/relaxation_length_longitudinal;
            const DirectionalScalar lateral_relaxation =
                rolling_speed/relaxation_length_lateral;
            const DirectionalScalar longitudinal_state_rate =
                longitudinal_relaxation*(longitudinal_slip-sx);
            const DirectionalScalar lateral_state_rate =
                lateral_relaxation*(lateral_slip-sy);
            tire_brush_derivatives[2*i] = longitudinal_state_rate.derivative;
            tire_brush_derivatives[2*i+1] = lateral_state_rate.derivative;
            DirectionalScalar fx = pac2002_pure_force_directional(
                t, effective_longitudinal_slip, normal_force, false,
                camber, smooth
            );
            DirectionalScalar fy = pac2002_pure_force_directional(
                t, effective_lateral_slip, normal_force, true,
                camber, smooth
            );
            const bool combined_slip = pac2002_has_combined_slip_terms(t);
            if (combined_slip) {
                fx = pac2002_combined_longitudinal_force_directional(
                    t, effective_longitudinal_slip,
                    effective_lateral_slip, normal_force, camber, fx, smooth
                );
                fy = pac2002_combined_lateral_force_directional(
                    t, effective_longitudinal_slip,
                    effective_lateral_slip, normal_force, camber, fy, smooth
                );
            }
            DirectionalScalar aligning_moment =
                pac2002_aligning_moment_directional(
                    t, effective_longitudinal_slip,
                    effective_lateral_slip, normal_force, camber, fx, fy,
                    smooth
                );
            if (t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
                aligning_moment += pac2002_gyroscopic_moment_directional(
                    t, normal_force, camber, effective_lateral_slip,
                    lateral_state_rate, rolling_radius, spin_rate, smooth
                );
            }
            const DirectionalScalar limit_x =
                pac2002_force_limit_directional(
                    t, normal_force, false, camber, smooth
                );
            const DirectionalScalar limit_y =
                pac2002_force_limit_directional(
                    t, normal_force, true, camber, smooth
                );
            const DirectionalScalar utilization = d_sqrt(
                (fx/limit_x)*(fx/limit_x)
                +(fy/limit_y)*(fy/limit_y)
            );
            constexpr double kJacobianStep = 1e-7;
            const double trial_utilization = utilization.value
                + kJacobianStep*utilization.derivative;
            if (!combined_slip && (
                std::abs(utilization.value-1.0) <= 1e-12
                || (utilization.value-1.0)
                    *(trial_utilization-1.0) <= 0.0
            )) {
                smooth = false;
            }
            if (!combined_slip && utilization.value > 1.0) {
                fx = fx/utilization;
                fy = fy/utilization;
            }
            const DirectionalScalar overturning_moment =
                pac2002_overturning_moment_directional(
                    t, fy, normal_force, camber, smooth
                );
            DirectionalScalar rolling_resistance_moment =
                pac2002_rolling_resistance_moment_directional(
                    t, fx, normal_force, camber, rolling_speed, smooth
                );
            if (t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
                rolling_resistance_moment +=
                    -fx*(rolling_radius-loaded_radius);
            }
            if (!brush_only) {
                const DVec3 contact_force =
                    forward*fx+lateral*fy+normal*normal_force;
                const DVec3 body_origin{
                    {state.r[t.body].x, direction.dr[t.body].x},
                    {state.r[t.body].y, direction.dr[t.body].y},
                    {state.r[t.body].z, direction.dr[t.body].z}
                };
                const DVec3 contact_arm = center-body_origin+patch_arm;
                add_directional_force_at_arm(
                    force, torque, model, t.body, contact_arm,
                    contact_force
                );
                add_directional_torque(
                    torque, model, t.body,
                    forward*overturning_moment
                        +lateral*rolling_resistance_moment
                        +normal*aligning_moment
                );
            }
            continue;
        }
        if (t.model_kind == VEHICLE_TIRE_FIALA) {
            const double low_speed_threshold = std::max(
                fiala_parameter(t, FIALA_LOW_SPEED_THRESHOLD, 1e-3), 1e-3
            );
            DirectionalScalar slip_speed = rolling_speed;
            if (slip_speed.value < low_speed_threshold) {
                smooth = false;
                slip_speed = {low_speed_threshold, 0.0};
            }
            const DirectionalScalar longitudinal_slip = d_clamp(
                -vx/slip_speed, -1.0, 1.0, smooth
            );
            const DirectionalScalar lateral_slip = d_clamp(
                d_atan2(vy, slip_speed), -0.5*kPi+0.01, 0.5*kPi-0.01,
                smooth
            );
            const double relax_x = std::max(
                fiala_parameter(t, FIALA_RELAX_LENGTH_X, t.relaxation_length_longitudinal), 1e-6
            );
            const double relax_y = std::max(
                fiala_parameter(t, FIALA_RELAX_LENGTH_Y, t.relaxation_length_lateral), 1e-6
            );
            const DirectionalScalar state_rate_x =
                rolling_speed/relax_x*(longitudinal_slip-sx);
            const DirectionalScalar state_rate_y =
                rolling_speed/relax_y*(lateral_slip-sy);
            tire_brush_derivatives[2*i] = state_rate_x.derivative;
            tire_brush_derivatives[2*i+1] = state_rate_y.derivative;
            if (!brush_only) {
                DirectionalScalar fx, fy, mz;
                fiala_forces_directional(
                    t, sx, sy, normal_force, fx, fy, mz, smooth
                );
                const DVec3 contact_force = forward*fx+lateral*fy
                    +normal*normal_force;
                const DVec3 body_origin{
                    {state.r[t.body].x, direction.dr[t.body].x},
                    {state.r[t.body].y, direction.dr[t.body].y},
                    {state.r[t.body].z, direction.dr[t.body].z}
                };
                add_directional_force_at_arm(
                    force, torque, model, t.body,
                    center-body_origin+patch_arm, contact_force
                );
                add_directional_torque(
                    torque, model, t.body,
                    normal*mz
                    +lateral*(-fiala_parameter(t, FIALA_ROLLING_RESISTANCE, 0.0)
                        *normal_force)
                );
            }
            continue;
        }
        const DirectionalScalar brush_x =
            vx-rolling_speed/t.relaxation_length_longitudinal*sx;
        const DirectionalScalar brush_y =
            vy-rolling_speed/t.relaxation_length_lateral*sy;
        tire_brush_derivatives[2*i] = brush_x.derivative;
        tire_brush_derivatives[2*i+1] = brush_y.derivative;
        const DirectionalScalar normalized_x =
            t.brush_k_longitudinal*sx/(t.mu_longitudinal*normal_force);
        const DirectionalScalar normalized_y =
            t.brush_k_lateral*sy/(t.mu_lateral*normal_force);
        const DirectionalScalar utilization = d_sqrt(
            normalized_x*normalized_x+normalized_y*normalized_y
        );
        DVec3 projected{sx, sy, 0.0};
        // 刷胎投影是分段光滑的。方向扰动跨过活动集边界时使用差分列；
        // 在边界上选择饱和侧表达式，避免返回映射与 Newton 导数分支不一致。
        constexpr double kJacobianStep = 1e-7;
        const double trial_utilization =
            utilization.value + kJacobianStep*utilization.derivative;
        if (
            std::abs(utilization.value-1.0) <= 1e-12 ||
            (utilization.value-1.0)*(trial_utilization-1.0) <= 0.0
        ) {
            smooth = false;
        }
        if (utilization.value >= 1.0) {
            projected.x = sx/utilization;
            projected.y = sy/utilization;
        }
        const DirectionalScalar fx = -t.brush_k_longitudinal*projected.x;
        const DirectionalScalar fy = -t.brush_k_lateral*projected.y;
        if (!brush_only) {
            const DVec3 contact_force = forward*fx+lateral*fy+normal*normal_force;
            // The carrier locates the contact center while the spinning body
            // receives the wrench; form the arm in world coordinates.
            const DVec3 body_origin{
                {state.r[t.body].x, direction.dr[t.body].x},
                {state.r[t.body].y, direction.dr[t.body].y},
                {state.r[t.body].z, direction.dr[t.body].z}
            };
            const DVec3 contact_arm = center-body_origin+patch_arm;
            add_directional_force_at_arm(
                force, torque, model, t.body, contact_arm, contact_force
            );
        }
    }

    if (brush_only) return smooth;

    if (!brush_only) {
        for (std::size_t i = 0; i < model.tires.size(); ++i) {
            const Tire& t = model.tires[i];
            const int frame_body = tire_frame_body(t);
            const bool body_active = directional_body_active(direction, t.body);
            const bool frame_active = directional_body_active(direction, frame_body);
            const bool mapped_drive = t.drive_torque_body >= 0;
            const int drive_body = mapped_drive ? t.drive_torque_body : t.body;
            const int drive_axis_body = mapped_drive ? drive_body : frame_body;
            const Vec3 drive_axis_local = mapped_drive
                ? t.drive_torque_axis : t.spin_axis;
            const DVec3 drive_axis = d_normalized(
                d_rotate(
                    state.q[drive_axis_body], drive_axis_local,
                    direction.dtheta[drive_axis_body]
                ),
                smooth
            );
            const double drive_torque = i < input.torque.size()
                ? input.torque[i] : 0.0;
            add_directional_torque(
                torque, model, drive_body, drive_axis*drive_torque
            );
            add_directional_torque(
                torque, model, t.drive_torque_reaction_body,
                -(drive_axis*drive_torque)
            );

            const double brake_magnitude = i < input.brake_torque.size()
                ? input.brake_torque[i] : 0.0;
            double brake_torque = 0.0;
            if (brake_magnitude > kEps && (body_active || frame_active)) {
                const DVec3 tire_axis = d_normalized(
                    d_rotate(
                        state.q[frame_body], t.spin_axis,
                        direction.dtheta[frame_body]
                    ),
                    smooth
                );
                DVec3 omega{
                    {state.omega[t.body].x, direction.domega[t.body].x},
                    {state.omega[t.body].y, direction.domega[t.body].y},
                    {state.omega[t.body].z, direction.domega[t.body].z}
                };
                if (frame_body != t.body) {
                    omega = omega-DVec3{
                        {state.omega[frame_body].x,
                         direction.domega[frame_body].x},
                        {state.omega[frame_body].y,
                         direction.domega[frame_body].y},
                        {state.omega[frame_body].z,
                        direction.domega[frame_body].z}
                    };
                }
                const DirectionalScalar axial_rate = d_dot(tire_axis, omega);
                constexpr double kJacobianStep = 1e-7;
                const double trial_rate = axial_rate.value
                    + kJacobianStep*axial_rate.derivative;
                if (
                    std::abs(axial_rate.value) <= kEps ||
                    axial_rate.value*trial_rate <= 0.0
                ) {
                    smooth = false;
                } else if (axial_rate.value > 0.0) {
                    brake_torque = -brake_magnitude;
                } else {
                    brake_torque = brake_magnitude;
                }
                add_directional_torque(
                    torque, model, t.body, tire_axis*brake_torque
                );
            }
        }
    }

    generalized_force.assign(static_cast<std::size_t>(n), 0.0);
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int body_index = model.free_body[fi];
        const bool inertial_active =
            directional_inertial_active(direction, body_index);
        const Body& body = model.bodies[body_index];
        generalized_force[6*fi] = force[body_index].x;
        generalized_force[6*fi+1] = force[body_index].y;
        generalized_force[6*fi+2] = force[body_index].z;
        if (!inertial_active) {
            generalized_force[6*fi+3] = torque[body_index].x;
            generalized_force[6*fi+4] = torque[body_index].y;
            generalized_force[6*fi+5] = torque[body_index].z;
            continue;
        }
        const DMat3 rotation = d_qmat(d_body_quaternion(
            state.q[body_index], direction.dtheta[body_index]
        ));
        DMat3 inertia_body{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                inertia_body.a[row][col] = body.inertia_body.a[row][col];
            }
        }
        const DMat3 inertia =
            rotation*inertia_body*d_transpose(rotation);
        const DVec3 omega{
            {state.omega[body_index].x, direction.domega[body_index].x},
            {state.omega[body_index].y, direction.domega[body_index].y},
            {state.omega[body_index].z, direction.domega[body_index].z}
        };
        const DVec3 gyro = d_cross(omega, inertia*omega);
        generalized_force[6*fi+3] = torque[body_index].x-gyro.x.derivative;
        generalized_force[6*fi+4] = torque[body_index].y-gyro.y.derivative;
        generalized_force[6*fi+5] = torque[body_index].z-gyro.z.derivative;
    }
    return smooth;
}

bool constraint_jacobian_directional(
    const Model& model, const State& state, const DirectionalState& direction,
    std::vector<double>& derivative,
    std::vector<std::size_t>* touched_indices = nullptr
) {
    const int n = model.ndof;
    bool smooth = true;
    const std::size_t derivative_size =
        static_cast<std::size_t>(model.rows*n);
    if (touched_indices == nullptr) {
        derivative.assign(derivative_size, 0.0);
    } else {
        if (derivative.size() != derivative_size) {
            derivative.assign(derivative_size, 0.0);
        } else {
            for (const std::size_t index : *touched_indices) {
                derivative[index] = 0.0;
            }
        }
        touched_indices->clear();
    }
    const DMat3 eye = d_identity3();
    const DMat3 zero{};

    auto add_row = [&](int row, int body, const DVec3& translation,
                       const DVec3& rotation) {
        const int fi = model.body_to_free[static_cast<std::size_t>(body)];
        if (fi < 0) return;
        double* out = &derivative[
            static_cast<std::size_t>(row*n+6*fi)
        ];
        if (touched_indices != nullptr) {
            for (int k = 0; k < 6; ++k) {
                touched_indices->push_back(
                    static_cast<std::size_t>(row*n+6*fi+k)
                );
            }
        }
        out[0] += translation.x.derivative;
        out[1] += translation.y.derivative;
        out[2] += translation.z.derivative;
        out[3] += rotation.x.derivative;
        out[4] += rotation.y.derivative;
        out[5] += rotation.z.derivative;
    };
    auto add_block = [&](int row0, int body, const DMat3& translation,
                         const DMat3& rotation) {
        for (int i = 0; i < 3; ++i) {
            add_row(
                row0+i, body,
                {translation.a[i][0], translation.a[i][1], translation.a[i][2]},
                {rotation.a[i][0], rotation.a[i][1], rotation.a[i][2]}
            );
        }
    };

    for (const auto& c : model.constraints) {
        if (
            !directional_body_active(direction, c.a) &&
            !directional_body_active(direction, c.b)
        ) {
            continue;
        }
        const DQuat qa = d_body_quaternion(
            state.q[c.a], direction.dtheta[c.a]
        );
        const DQuat qb = d_body_quaternion(
            state.q[c.b], direction.dtheta[c.b]
        );
        const DMat3 ra = d_qmat(qa);
        const DVec3 arm_a = d_rotate(
            state.q[c.a], c.pa, direction.dtheta[c.a]
        );
        const DVec3 arm_b = d_rotate(
            state.q[c.b], c.pb, direction.dtheta[c.b]
        );
        const DVec3 pa = d_state_point(
            state, direction.dr, direction.dtheta, c.a, c.pa
        );
        const DVec3 pb = d_state_point(
            state, direction.dr, direction.dtheta, c.b, c.pb
        );
        const DVec3 dp = pa-pb;
        const DMat3 zero_rotation = zero;
        int k = c.row;

        if (c.type == AXLE_SPHERICAL || c.type == AXLE_REVOLUTE ||
            c.type == AXLE_FIXED || c.type == AXLE_UNIVERSAL ||
            c.type == AXLE_CONVEL) {
            add_block(
                k, c.a, eye,
                d_skew(arm_a)*(-1.0)
            );
            add_block(
                k, c.b, eye*(-1.0),
                d_skew(arm_b)
            );
            k += 3;
        }

        auto add_relative_rotation = [&](int row0) {
            const DVec3 phi = d_qlog(
                d_qmul(d_qconj(qa), qb), smooth
            );
            const DMat3 map =
                d_log_left_jacobian_inverse(phi, smooth)*d_transpose(ra);
            add_block(row0, c.a, zero_rotation, map*(-1.0));
            add_block(row0, c.b, zero_rotation, map);
        };
        auto add_relative_rotation_row = [&](int row0, const Vec3& axis_local) {
            const DVec3 phi = d_qlog(
                d_qmul(d_qconj(qa), qb), smooth
            );
            const DMat3 map =
                d_log_left_jacobian_inverse(phi, smooth)*d_transpose(ra);
            const DVec3 axis(axis_local.x, axis_local.y, axis_local.z);
            const DVec3 row = d_row_times(d_normalized(axis, smooth), map);
            add_row(row0, c.a, {}, -row);
            add_row(row0, c.b, {}, row);
        };

        if (c.type == AXLE_FIXED) {
            add_relative_rotation(k);
        } else if (c.type == AXLE_UNIVERSAL) {
            const DVec3 aa = d_normalized(
                d_rotate(state.q[c.a], c.axis_a, direction.dtheta[c.a]),
                smooth
            );
            const DVec3 ab = d_normalized(
                d_rotate(state.q[c.b], c.axis_b, direction.dtheta[c.b]),
                smooth
            );
            add_row(
                k, c.a, {},
                d_row_times(ab, d_skew(aa)*(-1.0))
            );
            add_row(
                k, c.b, {},
                d_row_times(aa, d_skew(ab)*(-1.0))
            );
        } else if (c.type == AXLE_CONVEL) {
            const DVec3 xa = d_normalized(
                d_rotate(state.q[c.a], c.axis_a, direction.dtheta[c.a]),
                smooth
            );
            const DVec3 ya = d_normalized(
                d_rotate(
                    state.q[c.a], c.axis_a_secondary,
                    direction.dtheta[c.a]
                ),
                smooth
            );
            const DVec3 yb = d_normalized(
                d_rotate(state.q[c.b], c.axis_b, direction.dtheta[c.b]),
                smooth
            );
            const DVec3 xb = d_normalized(
                d_rotate(
                    state.q[c.b], c.axis_b_secondary,
                    direction.dtheta[c.b]
                ),
                smooth
            );
            add_row(
                k, c.a, {},
                d_row_times(yb, d_skew(xa)*(-1.0))
                    + d_row_times(xb, d_skew(ya)*(-1.0))
            );
            add_row(
                k, c.b, {},
                d_row_times(xa, d_skew(yb)*(-1.0))
                    + d_row_times(ya, d_skew(xb)*(-1.0))
            );
        } else if (c.type == AXLE_INPLANE) {
            const DVec3 aa = d_normalized(
                d_rotate(state.q[c.a], c.axis_a, direction.dtheta[c.a]),
                smooth
            );
            add_row(
                k, c.a, aa,
                d_row_times(aa, d_skew(arm_a)*(-1.0))
                    +d_row_times(dp, d_skew(aa)*(-1.0))
            );
            add_row(k, c.b, aa*(-1.0), d_row_times(aa, d_skew(arm_b)));
        } else if (
            c.type == AXLE_REVOLUTE || c.type == AXLE_PRISMATIC ||
            c.type == AXLE_CYLINDRICAL
        ) {
            const DVec3 aa = d_normalized(
                d_rotate(state.q[c.a], c.axis_a, direction.dtheta[c.a]),
                smooth
            );
            const Vec3 reference = perpendicular_reference(aa.value());
            if (std::abs(std::abs(aa.value().x)-0.8) <= 1e-12) {
                smooth = false;
            }
            const DVec3 g(reference.x, reference.y, reference.z);
            const DirectionalScalar g_dot_aa = d_dot(g, aa);
            const DVec3 u = g-aa*g_dot_aa;
            const DirectionalScalar u_norm = d_norm(u);
            if (u_norm.value <= kEps) smooth = false;
            const DVec3 e1 = u/u_norm;
            const DVec3 e2 = d_cross(aa, e1);
            const DMat3 d_aa = d_skew(aa)*(-1.0);
            const DMat3 d_u =
                (eye*g_dot_aa+d_outer(aa, g))*(-1.0);
            const DMat3 d_e1 =
                ((eye-d_outer(e1, e1))*(1.0/u_norm))*d_u*d_aa;
            const DMat3 d_e2 =
                d_skew(e1)*(-1.0)*d_aa+d_skew(aa)*d_e1;

            if (c.type == AXLE_REVOLUTE) {
                const DVec3 ab = d_normalized(
                    d_rotate(state.q[c.b], c.axis_b, direction.dtheta[c.b]),
                    smooth
                );
                const DMat3 d_ab = d_skew(ab)*(-1.0);
                add_row(k, c.b, {}, d_row_times(e1, d_ab));
                add_row(k, c.a, {}, d_row_times(ab, d_e1));
                add_row(k+1, c.b, {}, d_row_times(e2, d_ab));
                add_row(k+1, c.a, {}, d_row_times(ab, d_e2));
            } else if (c.type == AXLE_CYLINDRICAL) {
                add_row(
                    k, c.a, e1,
                    d_row_times(e1, d_skew(arm_a)*(-1.0))
                        +d_row_times(dp, d_e1)
                );
                add_row(k, c.b, e1*(-1.0), d_row_times(e1, d_skew(arm_b)));
                add_row(
                    k+1, c.a, e2,
                    d_row_times(e2, d_skew(arm_a)*(-1.0))
                        +d_row_times(dp, d_e2)
                );
                add_row(k+1, c.b, e2*(-1.0), d_row_times(e2, d_skew(arm_b)));
                const DVec3 ab = d_normalized(
                    d_rotate(state.q[c.b], c.axis_b, direction.dtheta[c.b]),
                    smooth
                );
                const DMat3 d_ab = d_skew(ab)*(-1.0);
                add_row(k+2, c.b, {}, d_row_times(e1, d_ab));
                add_row(k+2, c.a, {}, d_row_times(ab, d_e1));
                add_row(k+3, c.b, {}, d_row_times(e2, d_ab));
                add_row(k+3, c.a, {}, d_row_times(ab, d_e2));
            } else {
                add_row(
                    k, c.a, e1,
                    d_row_times(e1, d_skew(arm_a)*(-1.0))
                        +d_row_times(dp, d_e1)
                );
                add_row(k, c.b, e1*(-1.0), d_row_times(e1, d_skew(arm_b)));
                add_row(
                    k+1, c.a, e2,
                    d_row_times(e2, d_skew(arm_a)*(-1.0))
                        +d_row_times(dp, d_e2)
                );
                add_row(k+1, c.b, e2*(-1.0), d_row_times(e2, d_skew(arm_b)));
                const DVec3 ab = d_normalized(
                    d_rotate(state.q[c.b], c.axis_b, direction.dtheta[c.b]),
                    smooth
                );
                const DMat3 d_ab = d_skew(ab)*(-1.0);
                add_row(k+2, c.b, {}, d_row_times(e1, d_ab));
                add_row(k+2, c.a, {}, d_row_times(ab, d_e1));
                add_row(k+3, c.b, {}, d_row_times(e2, d_ab));
                add_row(k+3, c.a, {}, d_row_times(ab, d_e2));
                add_relative_rotation_row(k+4, c.axis_a);
            }
        }
    }
    for (const auto& coupler : model.coordinate_couplers) {
        auto add_joint_coordinate = [&](int row, int joint_index, int coordinate,
                                        const Quat& reference_rotation,
                                        double scale) {
            const Constraint& joint = model.constraints[
                static_cast<std::size_t>(joint_index)
            ];
            const DQuat qa = d_body_quaternion(
                state.q[joint.a], direction.dtheta[joint.a]
            );
            const DQuat qb = d_body_quaternion(
                state.q[joint.b], direction.dtheta[joint.b]
            );
            const DMat3 ra = d_qmat(qa);
            if (coordinate == 0) {
                const DQuat reference_conjugate{
                    {reference_rotation.w, 0.0},
                    {-reference_rotation.x, 0.0},
                    {-reference_rotation.y, 0.0},
                    {-reference_rotation.z, 0.0}
                };
                const DQuat qrel = d_qmul(d_qconj(qa), qb);
                const DVec3 phi = d_qlog(
                    d_qmul(reference_conjugate, qrel), smooth
                );
                const DMat3 reference_rotation_matrix = d_qmat(DQuat{
                    {reference_rotation.w, 0.0},
                    {reference_rotation.x, 0.0},
                    {reference_rotation.y, 0.0},
                    {reference_rotation.z, 0.0}
                });
                const DMat3 map = d_log_left_jacobian_inverse(phi, smooth)
                    * d_transpose(reference_rotation_matrix)
                    * d_transpose(ra);
                const DVec3 axis(
                    joint.axis_a.x, joint.axis_a.y, joint.axis_a.z
                );
                const DVec3 row_value = d_row_times(
                    d_normalized(axis, smooth), map
                ) * scale;
                add_row(row, joint.a, {}, -row_value);
                add_row(row, joint.b, {}, row_value);
                return;
            }
            const DVec3 arm_a = d_rotate(
                state.q[joint.a], joint.pa, direction.dtheta[joint.a]
            );
            const DVec3 arm_b = d_rotate(
                state.q[joint.b], joint.pb, direction.dtheta[joint.b]
            );
            const DVec3 separation = d_state_point(
                state, direction.dr, direction.dtheta,
                joint.a, joint.pa
            ) - d_state_point(
                state, direction.dr, direction.dtheta,
                joint.b, joint.pb
            );
            const DVec3 axis = d_normalized(
                d_rotate(state.q[joint.a], joint.axis_a,
                         direction.dtheta[joint.a]),
                smooth
            );
            const DMat3 d_axis = d_skew(axis)*(-1.0);
            add_row(
                row, joint.a, axis*scale,
                (d_row_times(axis, d_skew(arm_a)*(-1.0))
                    + d_row_times(separation, d_axis))*scale
            );
            add_row(
                row, joint.b, axis*(-scale),
                d_row_times(axis, d_skew(arm_b))*scale
            );
        };
        add_joint_coordinate(
            coupler.row, coupler.joint_a, coupler.coordinate_a,
            coupler.reference_rotation_a, coupler.scale_a
        );
        add_joint_coordinate(
            coupler.row, coupler.joint_b, coupler.coordinate_b,
            coupler.reference_rotation_b, coupler.scale_b
        );
    }
    for (const auto& actuator : model.steering_actuators) {
        if (!prescribed_steering(actuator)) continue;
        if (actuator.type == VEHICLE_STEERING_PRESCRIBED_TRANSLATION) {
            const bool body_active = directional_body_active(
                direction, actuator.body
            );
            const bool reaction_active = actuator.reaction_body >= 0 &&
                directional_body_active(direction, actuator.reaction_body);
            if (!body_active && !reaction_active) continue;
            const DVec3 body_arm = d_rotate(
                state.q[actuator.body], actuator.point_local,
                direction.dtheta[actuator.body]
            );
            const DVec3 body_point = d_state_point(
                state, direction.dr, direction.dtheta,
                actuator.body, actuator.point_local
            );
            DVec3 reaction_arm{};
            DVec3 reaction_point{};
            DVec3 axis(
                actuator.axis_local.x,
                actuator.axis_local.y,
                actuator.axis_local.z
            );
            if (actuator.reaction_body >= 0) {
                reaction_arm = d_rotate(
                    state.q[actuator.reaction_body],
                    actuator.reaction_point_local,
                    direction.dtheta[actuator.reaction_body]
                );
                reaction_point = d_state_point(
                    state, direction.dr, direction.dtheta,
                    actuator.reaction_body,
                    actuator.reaction_point_local
                );
                axis = d_normalized(d_rotate(
                    state.q[actuator.reaction_body], actuator.axis_local,
                    direction.dtheta[actuator.reaction_body]
                ), smooth);
            } else {
                axis = d_normalized(axis, smooth);
            }
            const DVec3 separation = body_point-reaction_point;
            add_row(
                actuator.constraint_row, actuator.body, axis,
                d_row_times(axis, d_skew(body_arm)*(-1.0))
            );
            if (actuator.reaction_body >= 0) {
                add_row(
                    actuator.constraint_row, actuator.reaction_body,
                    axis*(-1.0),
                    d_row_times(separation, d_skew(axis)*(-1.0))
                        +d_row_times(axis, d_skew(reaction_arm))
                );
            }
            continue;
        }
        const bool body_active = directional_orientation_active(
            direction, actuator.body
        );
        const bool reaction_active = actuator.reaction_body >= 0 &&
            directional_orientation_active(direction, actuator.reaction_body);
        if (!body_active && !reaction_active) continue;
        const DQuat body_q = d_body_quaternion(
            state.q[actuator.body], direction.dtheta[actuator.body]
        );
        const DQuat reaction_q = actuator.reaction_body >= 0
            ? d_body_quaternion(
                state.q[actuator.reaction_body],
                direction.dtheta[actuator.reaction_body]
            )
            : DQuat{};
        const DQuat relative = d_qmul(d_qconj(reaction_q), body_q);
        const DQuat reference_conjugate{
            actuator.reference.w,
            -actuator.reference.x,
            -actuator.reference.y,
            -actuator.reference.z
        };
        const DVec3 error_rotation = d_qlog(
            d_qmul(reference_conjugate, relative), smooth
        );
        const DMat3 map = d_log_left_jacobian_inverse(
            error_rotation, smooth
        ) * d_transpose(d_qmat(DQuat{
            actuator.reference.w,
            actuator.reference.x,
            actuator.reference.y,
            actuator.reference.z
        })) * d_transpose(d_qmat(reaction_q));
        const Vec3 axis_reference_value = rotate(
            actuator.reference, actuator.axis_local
        );
        const DVec3 axis_reference(
            axis_reference_value.x,
            axis_reference_value.y,
            axis_reference_value.z
        );
        const DVec3 row_value = d_row_times(
            d_normalized(axis_reference, smooth), map
        );
        add_row(actuator.constraint_row, actuator.body, {}, row_value);
        if (actuator.reaction_body >= 0) {
            add_row(
                actuator.constraint_row, actuator.reaction_body, {},
                -row_value
            );
        }
    }
    return smooth;
}

void apply_acceleration(State& state, const Model& model, const std::vector<double>& a) {
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        state.a[bi] = {a[6*fi], a[6*fi+1], a[6*fi+2]};
        state.alpha[bi] = {a[6*fi+3], a[6*fi+4], a[6*fi+5]};
    }
}

bool mass_inverse_into(
    const Model& model, const State& state, const std::vector<double>& rhs,
    std::vector<double>& out
) {
    out.assign(rhs.size(), 0.0);
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        const Body& b = model.bodies[bi];
        out[6*fi] = rhs[6*fi] / b.mass;
        out[6*fi+1] = rhs[6*fi+1] / b.mass;
        out[6*fi+2] = rhs[6*fi+2] / b.mass;
        const Mat3 rotation = qmat(state.q[bi]);
        const Mat3 world_inertia = rotation * b.inertia_body * transpose(rotation);
        Mat3 inverse{};
        if (!inverse3(world_inertia, inverse)) {
            return false;
        }
        const Vec3 angular = inverse * Vec3{rhs[6*fi+3], rhs[6*fi+4], rhs[6*fi+5]};
        out[6*fi+3] = angular.x;
        out[6*fi+4] = angular.y;
        out[6*fi+5] = angular.z;
    }
    return true;
}

// Fused form of `mass_inverse(model, state, J^T * mu)`.  The intermediate
// J^T*mu vector is consumed immediately, so it is formed one body at a time
// rather than allocated in full on every residual evaluation.
bool mass_inverse_of_jt_mu(
    const Model& model, const State& state,
    const std::vector<double>& J, const std::vector<double>& mu,
    int n, int m, std::vector<double>& out
) {
    out.assign(static_cast<std::size_t>(n), 0.0);
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        const Body& b = model.bodies[bi];
        double jt[6];
        for (int k = 0; k < 6; ++k) {
            double s = 0.0;
            const int col = 6*fi + k;
            for (int row = 0; row < m; ++row) {
                s += J[static_cast<std::size_t>(row*n + col)]
                   * mu[static_cast<std::size_t>(row)];
            }
            jt[k] = s;
        }
        out[6*fi] = jt[0] / b.mass;
        out[6*fi+1] = jt[1] / b.mass;
        out[6*fi+2] = jt[2] / b.mass;
        const Mat3 rotation = qmat(state.q[bi]);
        const Mat3 world_inertia =
            rotation * b.inertia_body * transpose(rotation);
        Mat3 inverse{};
        if (!inverse3(world_inertia, inverse)) return false;
        const Vec3 angular = inverse * Vec3{jt[3], jt[4], jt[5]};
        out[6*fi+3] = angular.x;
        out[6*fi+4] = angular.y;
        out[6*fi+5] = angular.z;
    }
    return true;
}

bool mass_inverse_jt_mu_directional(
    const Model& model, const State& state, const DirectionalState& direction,
    const std::vector<double>& dJ, const std::vector<double>& mu,
    const std::vector<double>& base_mass_inverse, int n,
    std::vector<double>& derivative
) {
    derivative.assign(static_cast<std::size_t>(n), 0.0);
    std::vector<double> djt(static_cast<std::size_t>(n), 0.0);
    for (const auto& c : model.constraints) {
        if (
            !directional_body_active(direction, c.a) &&
            !directional_body_active(direction, c.b)
        ) {
            continue;
        }
        const int rows = constraint_rows(c.type);
        const int endpoint_bodies[2] = {c.a, c.b};
        for (int row_offset = 0; row_offset < rows; ++row_offset) {
            const int row = c.row+row_offset;
            const double multiplier = mu[static_cast<std::size_t>(row)];
            for (int endpoint = 0; endpoint < 2; ++endpoint) {
                const int body_index = endpoint_bodies[endpoint];
                const int fi = model.body_to_free[
                    static_cast<std::size_t>(body_index)
                ];
                if (fi < 0) continue;
                for (int k = 0; k < 6; ++k) {
                    const int col = 6*fi+k;
                    djt[static_cast<std::size_t>(col)] +=
                        dJ[static_cast<std::size_t>(row*n+col)]
                        * multiplier;
                }
            }
        }
    }
    for (const auto& coupler : model.coordinate_couplers) {
        const Constraint& joint_a = model.constraints[
            static_cast<std::size_t>(coupler.joint_a)
        ];
        const Constraint& joint_b = model.constraints[
            static_cast<std::size_t>(coupler.joint_b)
        ];
        const int endpoints[4] = {
            joint_a.a, joint_a.b, joint_b.a, joint_b.b
        };
        const bool active = std::any_of(
            std::begin(endpoints), std::end(endpoints), [&](int body) {
                return directional_body_active(direction, body);
            }
        );
        if (!active) continue;
        const int row = coupler.row;
        for (int endpoint = 0; endpoint < 4; ++endpoint) {
            bool duplicate = false;
            for (int previous = 0; previous < endpoint; ++previous) {
                duplicate = duplicate || endpoints[previous] == endpoints[endpoint];
            }
            if (duplicate) continue;
            const int fi = model.body_to_free[
                static_cast<std::size_t>(endpoints[endpoint])
            ];
            if (fi < 0) continue;
            for (int k = 0; k < 6; ++k) {
                const int col = 6*fi+k;
                djt[static_cast<std::size_t>(col)] +=
                    dJ[static_cast<std::size_t>(row*n+col)]
                    * mu[static_cast<std::size_t>(row)];
            }
        }
    }
    for (const SteeringActuator& actuator : model.steering_actuators) {
        if (!prescribed_steering(actuator)) continue;
        const int endpoints[2] = {actuator.body, actuator.reaction_body};
        const bool active =
            actuator.type == VEHICLE_STEERING_PRESCRIBED_TRANSLATION
                ? directional_body_active(direction, actuator.body) ||
                    (actuator.reaction_body >= 0 && directional_body_active(
                        direction, actuator.reaction_body
                    ))
                : directional_orientation_active(direction, actuator.body) ||
                    (actuator.reaction_body >= 0 &&
                        directional_orientation_active(
                            direction, actuator.reaction_body
                        ));
        if (!active) continue;
        for (int endpoint = 0; endpoint < 2; ++endpoint) {
            const int body = endpoints[endpoint];
            if (body < 0) continue;
            const int fi = model.body_to_free[static_cast<std::size_t>(body)];
            if (fi < 0) continue;
            for (int k = 0; k < 6; ++k) {
                const int col = 6*fi+k;
                djt[static_cast<std::size_t>(col)] +=
                    dJ[static_cast<std::size_t>(
                        actuator.constraint_row*n+col
                    )] * mu[static_cast<std::size_t>(
                        actuator.constraint_row
                    )];
            }
        }
    }
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[static_cast<std::size_t>(fi)];
        const Body& body = model.bodies[bi];
        const double* djt_body = djt.data()+6*fi;
        derivative[6*fi] = djt_body[0]/body.mass;
        derivative[6*fi+1] = djt_body[1]/body.mass;
        derivative[6*fi+2] = djt_body[2]/body.mass;

        if (!directional_orientation_active(direction, bi)) {
            const Mat3 rotation = qmat(state.q[bi]);
            const Mat3 world_inertia =
                rotation * body.inertia_body * transpose(rotation);
            Mat3 inverse{};
            if (!inverse3(world_inertia, inverse)) return false;
            const Vec3 dangular = inverse*Vec3{
                djt_body[3], djt_body[4], djt_body[5]
            };
            derivative[6*fi+3] = dangular.x;
            derivative[6*fi+4] = dangular.y;
            derivative[6*fi+5] = dangular.z;
            continue;
        }

        const DMat3 rotation = d_qmat(d_body_quaternion(
            state.q[bi], direction.dtheta[bi]
        ));
        DMat3 inertia_body{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                inertia_body.a[row][col] = body.inertia_body.a[row][col];
            }
        }
        const DMat3 world_inertia =
            rotation*inertia_body*d_transpose(rotation);
        Mat3 world_inertia_value{};
        Mat3 inverse{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                world_inertia_value.a[row][col] =
                    world_inertia.a[row][col].value;
            }
        }
        if (!inverse3(world_inertia_value, inverse)) return false;
        const double angular_values[3] = {
            base_mass_inverse[6*fi+3],
            base_mass_inverse[6*fi+4],
            base_mass_inverse[6*fi+5]
        };
        double d_inertia_values[3]{};
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                d_inertia_values[row] +=
                    world_inertia.a[row][col].derivative*angular_values[col];
            }
        }
        const Vec3 rhs{
            djt_body[3]-d_inertia_values[0],
            djt_body[4]-d_inertia_values[1],
            djt_body[5]-d_inertia_values[2]
        };
        const Vec3 dangular = inverse*rhs;
        derivative[6*fi+3] = dangular.x;
        derivative[6*fi+4] = dangular.y;
        derivative[6*fi+5] = dangular.z;
    }
    return true;
}

double max_abs(const std::vector<double>& x) {
    double r = 0.0;
    for (double v : x) r = std::max(r, std::abs(v));
    return r;
}

// A reusable LU factorization with partial pivoting.  Rebuilding the Newton
// Jacobian costs `dim` residual evaluations, which dominates the whole solve,
// so a modified-Newton iteration reuses one factorization for several
// iterations and only refactors when the update stops reducing the residual.
struct LuFactorization {
    std::vector<double> lu;
    std::vector<int> pivot;
    std::vector<int> column_pivot;
    std::vector<double> row_scale;
    std::vector<double> column_scale;
    int dim{0};
    bool identity{false};

    bool factor(std::vector<double> a, int n) {
        lu = std::move(a);
        dim = n;
        identity = false;
        pivot.resize(static_cast<std::size_t>(n));
        column_pivot.resize(static_cast<std::size_t>(n));
        std::iota(column_pivot.begin(), column_pivot.end(), 0);
        row_scale.assign(static_cast<std::size_t>(n), 0.0);
        column_scale.assign(static_cast<std::size_t>(n), 0.0);
        // Newton rows mix metres, radians, velocities, forces and moments.
        // Equilibrate rows and columns before LU so a small SI coordinate row
        // cannot be discarded by a pivot test against a large tire stiffness.
        for (int row = 0; row < n; ++row) {
            double maximum = 0.0;
            for (int column = 0; column < n; ++column) {
                const double value = lu[static_cast<std::size_t>(row*n+column)];
                if (!std::isfinite(value)) return false;
                maximum = std::max(maximum, std::abs(value));
            }
            if (!(maximum > 0.0) || !std::isfinite(maximum)) return false;
            row_scale[static_cast<std::size_t>(row)] = 1.0/maximum;
            for (int column = 0; column < n; ++column) {
                lu[static_cast<std::size_t>(row*n+column)] *=
                    row_scale[static_cast<std::size_t>(row)];
            }
        }
        for (int column = 0; column < n; ++column) {
            double maximum = 0.0;
            for (int row = 0; row < n; ++row) {
                maximum = std::max(
                    maximum,
                    std::abs(lu[static_cast<std::size_t>(row*n+column)])
                );
            }
            if (!(maximum > 0.0) || !std::isfinite(maximum)) return false;
            column_scale[static_cast<std::size_t>(column)] = 1.0/maximum;
            for (int row = 0; row < n; ++row) {
                lu[static_cast<std::size_t>(row*n+column)] *=
                    column_scale[static_cast<std::size_t>(column)];
            }
        }
        for (int k = 0; k < n; ++k) {
            int best_row = k;
            double best = std::abs(lu[static_cast<std::size_t>(k*n+k)]);
            for (int i = k+1; i < n; ++i) {
                const double value =
                    std::abs(lu[static_cast<std::size_t>(i*n+k)]);
                if (value > best) { best = value; best_row = i; }
            }
            if (best < 1e-14) return false;
            pivot[static_cast<std::size_t>(k)] = best_row;
            if (best_row != k) {
                double* pivot_row = lu.data()+static_cast<std::size_t>(k*n);
                double* swap_row =
                    lu.data()+static_cast<std::size_t>(best_row*n);
                for (int j = 0; j < n; ++j) {
                    std::swap(pivot_row[j], swap_row[j]);
                }
            }
            const double diagonal = lu[static_cast<std::size_t>(k*n+k)];
            const double* __restrict pivot_row =
                lu.data()+static_cast<std::size_t>(k*n);
            for (int i = k+1; i < n; ++i) {
                double* __restrict row =
                    lu.data()+static_cast<std::size_t>(i*n);
                const double factor = row[k] / diagonal;
                row[k] = factor;
                if (std::abs(factor) < 1e-30) continue;
                int j = k+1;
                for (; j+3 < n; j += 4) {
                    row[j] -= factor*pivot_row[j];
                    row[j+1] -= factor*pivot_row[j+1];
                    row[j+2] -= factor*pivot_row[j+2];
                    row[j+3] -= factor*pivot_row[j+3];
                }
                for (; j < n; ++j) {
                    row[j] -= factor*pivot_row[j];
                }
            }
        }
        return true;
    }

    void set_identity(int n) {
        dim = n;
        identity = true;
        lu.clear();
        pivot.clear();
        column_pivot.clear();
        row_scale.clear();
        column_scale.clear();
    }

    void solve(const std::vector<double>& b, std::vector<double>& x) const {
        const int n = dim;
        if (identity) {
            x = b;
            return;
        }
        x.resize(static_cast<std::size_t>(n));
        for (int i = 0; i < n; ++i) {
            x[static_cast<std::size_t>(i)] =
                b[static_cast<std::size_t>(i)]
                *row_scale[static_cast<std::size_t>(i)];
        }
        // Apply the whole row permutation before eliminating.  Interleaving the
        // swaps with forward substitution is wrong: a swap at step k can move a
        // right-hand-side entry that earlier steps already eliminated into, so
        // the elimination and the permutation must not be interleaved.
        for (int k = 0; k < n; ++k) {
            const int row = pivot[static_cast<std::size_t>(k)];
            if (row != k) std::swap(x[static_cast<std::size_t>(k)],
                                    x[static_cast<std::size_t>(row)]);
        }
        for (int i = 0; i < n; ++i) {
            const double* __restrict row =
                lu.data()+static_cast<std::size_t>(i*n);
            double value = x[static_cast<std::size_t>(i)];
            int j = 0;
            for (; j+3 < i; j += 4) {
                value -= row[j]*x[static_cast<std::size_t>(j)]
                    + row[j+1]*x[static_cast<std::size_t>(j+1)]
                    + row[j+2]*x[static_cast<std::size_t>(j+2)]
                    + row[j+3]*x[static_cast<std::size_t>(j+3)];
            }
            for (; j < i; ++j) {
                value -= row[j]*x[static_cast<std::size_t>(j)];
            }
            x[static_cast<std::size_t>(i)] = value;
        }
        for (int i = n-1; i >= 0; --i) {
            const double* __restrict row =
                lu.data()+static_cast<std::size_t>(i*n);
            double value = x[static_cast<std::size_t>(i)];
            int j = i+1;
            for (; j+3 < n; j += 4) {
                value -= row[j]*x[static_cast<std::size_t>(j)]
                    + row[j+1]*x[static_cast<std::size_t>(j+1)]
                    + row[j+2]*x[static_cast<std::size_t>(j+2)]
                    + row[j+3]*x[static_cast<std::size_t>(j+3)];
            }
            for (; j < n; ++j) {
                value -= row[j]*x[static_cast<std::size_t>(j)];
            }
            x[static_cast<std::size_t>(i)] = value/row[i];
        }
        for (int k = n-1; k >= 0; --k) {
            const int column = column_pivot[static_cast<std::size_t>(k)];
            if (column != k) {
                std::swap(
                    x[static_cast<std::size_t>(k)],
                    x[static_cast<std::size_t>(column)]
                );
            }
        }
        for (int i = 0; i < n; ++i) {
            x[static_cast<std::size_t>(i)] *=
                column_scale[static_cast<std::size_t>(i)];
        }
    }

};

bool finite_vec(const std::vector<double>& v);

// 整车隐式 Newton 方程具有固定的块结构：位置方程只通过位置、加速度
// 和位置乘子修正耦合，速度方程只通过速度和加速度修正耦合。先消去这
// 两个运动学块，再分解剩余的动力学、约束和轮胎内部状态块，可以避免
// 对包含大量单位阵块的完整系统重复做三次方消元。块结构不满足时由
// 调用方回退到完整稠密 LU，不改变方程含义。
struct ReducedNewtonFactorization {
    LuFactorization pose_factorization;
    LuFactorization reduced_factorization;
    std::vector<double> q_from_a;
    std::vector<double> q_from_mu;
    std::vector<double> v_from_a;
    std::vector<double> remaining_q;
    std::vector<double> remaining_v;
    int total_dim{0};
    int pose_dim{0};
    int constraint_dim{0};
    int brush_dim{0};
    int reduced_dim{0};

    // These buffers are reused across factor/solve calls.  The factorization
    // is owned by one Newton solve, so mutable solve scratch is thread-safe at
    // the same level as the factorization itself and avoids per-step vectors.
    mutable std::vector<double> solve_pose_rhs;
    mutable std::vector<double> solve_q0;
    mutable std::vector<double> solve_v0;
    mutable std::vector<double> solve_reduced_rhs;
    mutable std::vector<double> solve_reduced_solution;

    static int remaining_full_row(
        int row, int pose, int constraints, int brush
    ) {
        (void)brush;
        if (row < pose) return 2*pose+row;
        if (row < pose+constraints) {
            return 3*pose+(row-pose);
        }
        if (row < pose+2*constraints) {
            return 3*pose+constraints+(row-pose-constraints);
        }
        return 3*pose+2*constraints+(row-pose-2*constraints);
    }

    static int reduced_full_column(
        int column, int pose, int constraints, int brush
    ) {
        (void)brush;
        if (column < pose) return 2*pose+column;
        if (column < pose+constraints) {
            return 3*pose+(column-pose);
        }
        if (column < pose+2*constraints) {
            return 3*pose+constraints+(column-pose-constraints);
        }
        return 3*pose+2*constraints+(column-pose-2*constraints);
    }

    static bool row_has_only_blocks(
        const std::vector<double>& matrix, int dimension, int row,
        const std::vector<unsigned char>& allowed
    ) {
        double row_scale = 1.0;
        for (int column = 0; column < dimension; ++column) {
            row_scale = std::max(
                row_scale,
                std::abs(matrix[static_cast<std::size_t>(row*dimension+column)])
            );
        }
        const double tolerance = 1.0e-12*row_scale;
        for (int column = 0; column < dimension; ++column) {
            if (allowed[static_cast<std::size_t>(column)] != 0) continue;
            if (std::abs(matrix[
                    static_cast<std::size_t>(row*dimension+column)
                ]) > tolerance) {
                return false;
            }
        }
        return true;
    }

    bool factor(
        const std::vector<double>& matrix, int dimension,
        int pose, int constraints, int brush
    ) {
        total_dim = dimension;
        pose_dim = pose;
        constraint_dim = constraints;
        brush_dim = brush;
        reduced_dim = pose+2*constraints+brush;
        if (dimension != 3*pose+2*constraints+brush || pose <= 0) {
            return false;
        }
        const auto at = [dimension](int row, int column) {
            return static_cast<std::size_t>(row*dimension+column);
        };

        bool pose_identity = true;
        for (int row = 0; row < pose && pose_identity; ++row) {
            for (int column = 0; column < pose; ++column) {
                const double expected = row == column ? 1.0 : 0.0;
                if (matrix[at(row, column)] != expected) {
                    pose_identity = false;
                    break;
                }
            }
        }
        if (pose_identity) {
            pose_factorization.set_identity(pose);
        } else {
            std::vector<double> pose_matrix(
                static_cast<std::size_t>(pose*pose), 0.0
            );
            for (int row = 0; row < pose; ++row) {
                for (int column = 0; column < pose; ++column) {
                    pose_matrix[static_cast<std::size_t>(row*pose+column)] =
                        matrix[at(row, column)];
                }
            }
            if (!pose_factorization.factor(std::move(pose_matrix), pose)) {
                return false;
            }
        }

        std::vector<unsigned char> pose_allowed(
            static_cast<std::size_t>(dimension), 0
        );
        for (int column = 0; column < pose; ++column) {
            pose_allowed[static_cast<std::size_t>(column)] = 1;
            pose_allowed[static_cast<std::size_t>(2*pose+column)] = 1;
        }
        for (int column = 0; column < constraints; ++column) {
            pose_allowed[static_cast<std::size_t>(3*pose+constraints+column)] = 1;
        }
        for (int row = 0; row < pose; ++row) {
            if (!row_has_only_blocks(matrix, dimension, row, pose_allowed)) {
                return false;
            }
        }

        q_from_a.assign(static_cast<std::size_t>(pose*pose), 0.0);
        q_from_mu.assign(static_cast<std::size_t>(pose*constraints), 0.0);
        std::vector<double> right_hand_side(static_cast<std::size_t>(pose), 0.0);
        std::vector<double> solution;
        for (int column = 0; column < pose; ++column) {
            std::fill(right_hand_side.begin(), right_hand_side.end(), 0.0);
            for (int row = 0; row < pose; ++row) {
                right_hand_side[static_cast<std::size_t>(row)] =
                    matrix[at(row, 2*pose+column)];
            }
            pose_factorization.solve(right_hand_side, solution);
            if (static_cast<int>(solution.size()) != pose ||
                !finite_vec(solution)) {
                return false;
            }
            for (int row = 0; row < pose; ++row) {
                q_from_a[static_cast<std::size_t>(column*pose+row)] =
                    -solution[static_cast<std::size_t>(row)];
            }
        }
        for (int column = 0; column < constraints; ++column) {
            std::fill(right_hand_side.begin(), right_hand_side.end(), 0.0);
            for (int row = 0; row < pose; ++row) {
                right_hand_side[static_cast<std::size_t>(row)] =
                    matrix[at(row, 3*pose+constraints+column)];
            }
            pose_factorization.solve(right_hand_side, solution);
            if (static_cast<int>(solution.size()) != pose ||
                !finite_vec(solution)) {
                return false;
            }
            for (int row = 0; row < pose; ++row) {
                q_from_mu[static_cast<std::size_t>(column*pose+row)] =
                    -solution[static_cast<std::size_t>(row)];
            }
        }

        std::vector<unsigned char> velocity_allowed(
            static_cast<std::size_t>(dimension), 0
        );
        for (int column = 0; column < pose; ++column) {
            velocity_allowed[static_cast<std::size_t>(pose+column)] = 1;
            velocity_allowed[static_cast<std::size_t>(2*pose+column)] = 1;
        }
        for (int row = pose; row < 2*pose; ++row) {
            if (!row_has_only_blocks(matrix, dimension, row, velocity_allowed)) {
                return false;
            }
            const int local_row = row-pose;
            const double diagonal = matrix[at(row, pose+local_row)];
            if (std::abs(diagonal-1.0) > 1.0e-12) return false;
            for (int column = 0; column < pose; ++column) {
                if (column == local_row) continue;
                if (std::abs(matrix[at(row, pose+column)]) > 1.0e-12) {
                    return false;
                }
            }
        }
        v_from_a.assign(static_cast<std::size_t>(pose), 0.0);
        for (int row = 0; row < pose; ++row) {
            v_from_a[static_cast<std::size_t>(row)] =
                -matrix[at(pose+row, 2*pose+row)];
            for (int column = 0; column < pose; ++column) {
                if (column == row) continue;
                if (std::abs(matrix[at(pose+row, 2*pose+column)]) > 1.0e-12) {
                    return false;
                }
            }
        }

        std::vector<double> reduced_matrix(
            static_cast<std::size_t>(reduced_dim*reduced_dim), 0.0
        );
        remaining_q.assign(
            static_cast<std::size_t>(reduced_dim*pose), 0.0
        );
        remaining_v.assign(
            static_cast<std::size_t>(reduced_dim*pose), 0.0
        );
        std::vector<std::pair<int, double>> q_terms;
        std::vector<std::pair<int, double>> v_terms;
        q_terms.reserve(static_cast<std::size_t>(pose));
        v_terms.reserve(static_cast<std::size_t>(pose));
        for (int row = 0; row < reduced_dim; ++row) {
            const int full_row = remaining_full_row(
                row, pose, constraints, brush
            );
            for (int column = 0; column < pose; ++column) {
                remaining_q[static_cast<std::size_t>(row*pose+column)] =
                    matrix[at(full_row, column)];
                remaining_v[static_cast<std::size_t>(row*pose+column)] =
                    matrix[at(full_row, pose+column)];
            }
            q_terms.clear();
            v_terms.clear();
            for (int source = 0; source < pose; ++source) {
                const double q_value = remaining_q[
                    static_cast<std::size_t>(row*pose+source)
                ];
                if (q_value != 0.0) q_terms.emplace_back(source, q_value);
                const double v_value = remaining_v[
                    static_cast<std::size_t>(row*pose+source)
                ];
                if (v_value != 0.0) v_terms.emplace_back(source, v_value);
            }
            for (int column = 0; column < reduced_dim; ++column) {
                const int full_column = reduced_full_column(
                    column, pose, constraints, brush
                );
                double value = matrix[at(full_row, full_column)];
                if (column < pose) {
                    for (const auto& term : q_terms) {
                        value += term.second * q_from_a[
                            static_cast<std::size_t>(column*pose+term.first)
                        ];
                    }
                    for (const auto& term : v_terms) {
                        if (term.first == column) {
                            value += term.second * v_from_a[
                                static_cast<std::size_t>(column)
                            ];
                        }
                    }
                } else if (column >= pose+constraints &&
                           column < pose+2*constraints) {
                    const int mu_column = column-pose-constraints;
                    for (const auto& term : q_terms) {
                        value += term.second * q_from_mu[
                            static_cast<std::size_t>(mu_column*pose+term.first)
                        ];
                    }
                }
                reduced_matrix[static_cast<std::size_t>(
                    row*reduced_dim+column
                )] = value;
            }
        }
        return reduced_factorization.factor(
            std::move(reduced_matrix), reduced_dim
        );
    }

    bool solve(
        const std::vector<double>& right_hand_side,
        std::vector<double>& solution
    ) const {
        if (static_cast<int>(right_hand_side.size()) != total_dim) {
            return false;
        }
        solve_pose_rhs.assign(
            right_hand_side.begin(), right_hand_side.begin()+pose_dim
        );
        pose_factorization.solve(
            solve_pose_rhs, solve_q0
        );
        if (static_cast<int>(solve_q0.size()) != pose_dim ||
            !finite_vec(solve_q0)) {
            return false;
        }
        solve_v0.assign(
            right_hand_side.begin()+pose_dim,
            right_hand_side.begin()+2*pose_dim
        );
        solve_reduced_rhs.assign(
            static_cast<std::size_t>(reduced_dim), 0.0
        );
        for (int row = 0; row < reduced_dim; ++row) {
            const int full_row = remaining_full_row(
                row, pose_dim, constraint_dim, brush_dim
            );
            double value = right_hand_side[static_cast<std::size_t>(full_row)];
            for (int column = 0; column < pose_dim; ++column) {
                value -= remaining_q[
                    static_cast<std::size_t>(row*pose_dim+column)
                ] * solve_q0[static_cast<std::size_t>(column)];
                value -= remaining_v[
                    static_cast<std::size_t>(row*pose_dim+column)
                ] * solve_v0[static_cast<std::size_t>(column)];
            }
            solve_reduced_rhs[static_cast<std::size_t>(row)] = value;
        }
        reduced_factorization.solve(
            solve_reduced_rhs, solve_reduced_solution
        );
        if (static_cast<int>(solve_reduced_solution.size()) != reduced_dim ||
            !finite_vec(solve_reduced_solution)) {
            return false;
        }
        solution.resize(static_cast<std::size_t>(total_dim));
        std::fill(solution.begin(), solution.end(), 0.0);
        for (int row = 0; row < pose_dim; ++row) {
            double value = solve_q0[static_cast<std::size_t>(row)];
            for (int column = 0; column < pose_dim; ++column) {
                value += q_from_a[static_cast<std::size_t>(column*pose_dim+row)]
                    * solve_reduced_solution[static_cast<std::size_t>(column)];
            }
            for (int column = 0; column < constraint_dim; ++column) {
                value += q_from_mu[
                    static_cast<std::size_t>(column*pose_dim+row)
                ] * solve_reduced_solution[
                    static_cast<std::size_t>(pose_dim+constraint_dim+column)
                ];
            }
            solution[static_cast<std::size_t>(row)] = value;
        }
        for (int row = 0; row < pose_dim; ++row) {
            double value = solve_v0[static_cast<std::size_t>(row)];
            value += v_from_a[static_cast<std::size_t>(row)]
                * solve_reduced_solution[static_cast<std::size_t>(row)];
            solution[static_cast<std::size_t>(pose_dim+row)] = value;
        }
        for (int column = 0; column < reduced_dim; ++column) {
            solution[static_cast<std::size_t>(reduced_full_column(
                column, pose_dim, constraint_dim, brush_dim
            ))] = solve_reduced_solution[static_cast<std::size_t>(column)];
        }
        return finite_vec(solution);
    }
};

struct NewtonSystemFactorization {
    ReducedNewtonFactorization reduced;
    LuFactorization full;
    bool use_reduced{false};

    bool factor(
        std::vector<double> matrix, int dimension,
        int pose, int constraints, int brush
    ) {
        const char* disabled = std::getenv(
            "SUSPENSION_AXLE_DISABLE_REDUCED_KKT"
        );
        if (disabled == nullptr || disabled[0] == '\0' || disabled[0] == '0') {
            if (reduced.factor(matrix, dimension, pose, constraints, brush)) {
                use_reduced = true;
                return true;
            }
        }
        use_reduced = false;
        return full.factor(std::move(matrix), dimension);
    }

    bool solve(
        const std::vector<double>& right_hand_side,
        std::vector<double>& solution
    ) const {
        if (use_reduced) {
            return reduced.solve(right_hand_side, solution);
        }
        full.solve(right_hand_side, solution);
        return finite_vec(solution);
    }
};

// 隐式积分器可以在多个 Newton 求解之间复用线性化矩阵，并在当前线性化
// 不再降低残差时刷新。整轴求解器只把时间步系数作为兼容性键；过期线性化
// 导致试探步失败时，会在重新分解前使缓存失效。
struct ResidualContext;

struct NewtonLinearizationCache {
    NewtonSystemFactorization factorization;
    int dim{0};
    double h{0.0};
    double alpha_m{0.0};
    double alpha_f{0.0};
    double beta{0.0};
    double gamma{0.0};
    double alpha_m_z{0.0};
    double alpha_f_z{0.0};
    double gamma_z{0.0};
    int reuse_steps{0};
    bool valid{false};

    bool matches(const ResidualContext& context, int dimension) const;

    void update_key(const ResidualContext& context, int dimension);

    void invalidate() {
        valid = false;
        reuse_steps = 0;
    }
};

bool solve_linear(std::vector<double> A, std::vector<double> b, std::vector<double>& x) {
    const int n = static_cast<int>(b.size());
    // The static KKT system combines metres, radians, newtons, and newton
    // metres. Equilibrating rows and columns is an algebraic change of
    // variables; it does not add stiffness or alter the physical residual.
    std::vector<double> row_scale(static_cast<std::size_t>(n), 1.0);
    for (int i = 0; i < n; ++i) {
        double largest = 0.0;
        for (int j = 0; j < n; ++j) {
            largest = std::max(largest, std::abs(A[i*n+j]));
        }
        if (largest > 0.0) row_scale[static_cast<std::size_t>(i)] = 1.0/largest;
        for (int j = 0; j < n; ++j) A[i*n+j] *= row_scale[static_cast<std::size_t>(i)];
        b[i] *= row_scale[static_cast<std::size_t>(i)];
    }
    std::vector<double> column_scale(static_cast<std::size_t>(n), 1.0);
    for (int j = 0; j < n; ++j) {
        double largest = 0.0;
        for (int i = 0; i < n; ++i) {
            largest = std::max(largest, std::abs(A[i*n+j]));
        }
        if (largest > 0.0) column_scale[static_cast<std::size_t>(j)] = 1.0/largest;
        for (int i = 0; i < n; ++i) A[i*n+j] *= column_scale[static_cast<std::size_t>(j)];
    }
    std::vector<int> column_permutation(static_cast<std::size_t>(n));
    std::iota(column_permutation.begin(), column_permutation.end(), 0);
    const double matrix_scale = std::max(1.0, max_abs(A));
    const double pivot_tolerance =
        1.0e-12*matrix_scale
        *static_cast<double>(std::max(1, n));
    const double consistency_tolerance =
        1.0e-6*std::max(1.0, max_abs(b));
    int rank = 0;
    for (; rank < n; ++rank) {
        int pivot_row = rank;
        int pivot_column = rank;
        double best = 0.0;
        for (int i = rank; i < n; ++i) {
            for (int j = rank; j < n; ++j) {
                const double value = std::abs(A[i*n+j]);
                if (value > best) {
                    best = value;
                    pivot_row = i;
                    pivot_column = j;
                }
            }
        }
        if (best <= pivot_tolerance) break;
        if (pivot_row != rank) {
            for (int j = 0; j < n; ++j) {
                std::swap(A[rank*n+j], A[pivot_row*n+j]);
            }
            std::swap(b[rank], b[pivot_row]);
        }
        if (pivot_column != rank) {
            for (int i = 0; i < n; ++i) {
                std::swap(A[i*n+rank], A[i*n+pivot_column]);
            }
            std::swap(column_permutation[static_cast<std::size_t>(rank)],
                      column_permutation[static_cast<std::size_t>(pivot_column)]);
        }
        const double diagonal = A[rank*n+rank];
        for (int i = rank+1; i < n; ++i) {
            const double factor = A[i*n+rank]/diagonal;
            A[i*n+rank] = 0.0;
            if (std::abs(factor) <= 1.0e-30) continue;
            for (int j = rank+1; j < n; ++j) {
                A[i*n+j] -= factor*A[rank*n+j];
            }
            b[i] -= factor*b[rank];
        }
    }
    for (int i = rank; i < n; ++i) {
        double row_scale_value = 0.0;
        for (int j = rank; j < n; ++j) {
            row_scale_value = std::max(row_scale_value, std::abs(A[i*n+j]));
        }
        if (std::abs(b[i]) > consistency_tolerance *
            std::max(1.0, row_scale_value)) {
            return false;
        }
    }
    std::vector<double> scaled_solution(static_cast<std::size_t>(n), 0.0);
    for (int i = rank-1; i >= 0; --i) {
        double value = b[i];
        for (int j = i+1; j < rank; ++j) {
            value -= A[i*n+j]*scaled_solution[static_cast<std::size_t>(j)];
        }
        scaled_solution[static_cast<std::size_t>(i)] = value/A[i*n+i];
    }
    x.assign(static_cast<std::size_t>(n), 0.0);
    for (int i = 0; i < n; ++i) {
        const int original_column = column_permutation[static_cast<std::size_t>(i)];
        x[static_cast<std::size_t>(original_column)] =
            scaled_solution[static_cast<std::size_t>(i)]
            * column_scale[static_cast<std::size_t>(original_column)];
    }
    return true;
}

int matrix_rank(
    std::vector<double> matrix, int rows, int columns
) {
    if (rows == 0 || columns == 0) return 0;
    double scale = 0.0;
    for (double value : matrix) scale = std::max(scale, std::abs(value));
    const double tolerance =
        static_cast<double>(std::max(rows, columns))
        * std::numeric_limits<double>::epsilon()*std::max(1.0, scale);
    int rank = 0;
    for (int column = 0; column < columns && rank < rows; ++column) {
        int pivot = rank;
        double pivot_value = std::abs(matrix[rank*columns+column]);
        for (int row = rank+1; row < rows; ++row) {
            const double candidate = std::abs(matrix[row*columns+column]);
            if (candidate > pivot_value) {
                pivot = row;
                pivot_value = candidate;
            }
        }
        if (pivot_value <= tolerance) continue;
        if (pivot != rank) {
            for (int col = column; col < columns; ++col) {
                std::swap(
                    matrix[rank*columns+col],
                    matrix[pivot*columns+col]
                );
            }
        }
        const double diagonal = matrix[rank*columns+column];
        for (int row = rank+1; row < rows; ++row) {
            const double factor = matrix[row*columns+column]/diagonal;
            for (int col = column; col < columns; ++col) {
                matrix[row*columns+col] -=
                    factor*matrix[rank*columns+col];
            }
        }
        ++rank;
    }
    return rank;
}

void interpolate_input(const AxleInput& in, double t, SampleInput& out);
void interpolate_input(
    const Model& model, const AxleInput& in, double t, SampleInput& out
);

std::vector<double> generalized_velocity(const Model& model, const State& state) {
    std::vector<double> velocity(model.ndof, 0.0);
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        velocity[6*fi] = state.v[bi].x;
        velocity[6*fi+1] = state.v[bi].y;
        velocity[6*fi+2] = state.v[bi].z;
        velocity[6*fi+3] = state.omega[bi].x;
        velocity[6*fi+4] = state.omega[bi].y;
        velocity[6*fi+5] = state.omega[bi].z;
    }
    return velocity;
}

bool validate_initial_velocity(
    const Model& model, const AxleInput& input, const State& state,
    double tolerance, double& residual
) {
    if (model.rows == 0) {
        residual = 0.0;
        return true;
    }
    const auto jacobian = constraint_jacobian(model, state);
    const std::vector<double> velocity = generalized_velocity(model, state);
    SampleInput sample;
    interpolate_input(model, input, input.sample_times[0], sample);
    residual = 0.0;
    for (int row = 0; row < model.rows; ++row) {
        double violation = 0.0;
        for (int col = 0; col < model.ndof; ++col) {
            violation += jacobian[row*model.ndof+col] * velocity[col];
        }
        for (std::size_t index = 0;
             index < model.steering_actuators.size(); ++index) {
            const auto& actuator = model.steering_actuators[index];
            if (prescribed_steering(actuator) &&
                actuator.constraint_row == row &&
                index < sample.steering_target_rate.size()) {
                violation -= sample.steering_target_rate[index];
            }
        }
        residual = std::max(residual, std::abs(violation));
    }
    return residual <= tolerance;
}

bool initialize_acceleration(
    const Model& model, const AxleInput& input, State& state, double& residual,
    std::vector<double>& constraint_multiplier
) {
    const int n = model.ndof;
    const int m = model.rows;
    SampleInput sample;
    interpolate_input(model, input, input.sample_times[0], sample);
    std::vector<double> tire_forces;
    std::vector<double> tire_derivatives;
    std::vector<double> tire_output;
    double potential = 0.0;
    double power = 0.0;
    double dissipation = 0.0;
    std::vector<double> force;
    external_force_vector(
        model, state, sample, input.gravity_x, input.gravity_y, input.gravity_z,
        tire_forces, tire_derivatives, tire_output, potential, power, dissipation,
        force
    );
    const auto jacobian = constraint_jacobian(model, state);
    const auto velocity = generalized_velocity(model, state);
    std::vector<double> jdot_v(m, 0.0);
    if (m > 0) {
        const double epsilon = 1e-7;
        State advanced = state;
        perturb_pose(advanced, model, velocity, epsilon);
        const auto advanced_jacobian = constraint_jacobian(model, advanced);
        for (int row = 0; row < m; ++row) {
            for (int col = 0; col < n; ++col) {
                jdot_v[row] += (
                    advanced_jacobian[row*n+col] - jacobian[row*n+col]
                ) / epsilon * velocity[col];
            }
        }
    }
    for (const auto& actuator : model.steering_actuators) {
        if (!prescribed_steering(actuator)) continue;
        const std::size_t index = static_cast<std::size_t>(
            &actuator - model.steering_actuators.data()
        );
        if (index < sample.steering_target_acceleration.size()) {
            jdot_v[actuator.constraint_row] -=
                sample.steering_target_acceleration[index];
        }
    }
    const int dimension = n + m;
    std::vector<double> matrix(dimension * dimension, 0.0);
    std::vector<double> rhs(dimension, 0.0);
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        const int offset = 6*fi;
        for (int axis = 0; axis < 3; ++axis) {
            matrix[(offset+axis)*dimension+offset+axis] = model.bodies[bi].mass;
        }
        const Mat3 rotation = qmat(state.q[bi]);
        const Mat3 inertia =
            rotation * model.bodies[bi].inertia_body * transpose(rotation);
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                matrix[(offset+3+row)*dimension+offset+3+col] = inertia.a[row][col];
            }
        }
    }
    for (int row = 0; row < m; ++row) {
        for (int col = 0; col < n; ++col) {
            matrix[col*dimension+n+row] = -jacobian[row*n+col];
            matrix[(n+row)*dimension+col] = jacobian[row*n+col];
        }
        rhs[n+row] = -jdot_v[row];
    }
    std::copy(force.begin(), force.end(), rhs.begin());
    std::vector<double> solution;
    if (!solve_linear(matrix, rhs, solution)) return false;
    apply_acceleration(
        state, model, std::vector<double>(solution.begin(), solution.begin()+n)
    );
    constraint_multiplier.assign(solution.begin()+n, solution.end());
    residual = 0.0;
    for (int row = 0; row < dimension; ++row) {
        double value = -rhs[row];
        for (int col = 0; col < dimension; ++col) {
            value += matrix[row*dimension+col] * solution[col];
        }
        residual = std::max(residual, std::abs(value));
    }
    return residual <= input.dynamics_tolerance;
}

void interpolate_input(const AxleInput& in, double t, SampleInput& out) {
    const std::size_t n = in.sample_count;
    const std::size_t nb = in.body_count;
    const std::size_t nt = in.tire_count;
    out.body_wrench.assign(nb * 6, 0.0);
    out.road_z.assign(nt, 0.0);
    out.road_v.assign(nt, 0.0);
    out.torque.assign(nt, 0.0);
    out.brake_torque.assign(nt, 0.0);
    if (n == 0) return;
    std::size_t k = 0;
    while (k+1 < n && in.sample_times[k+1] < t) ++k;
    const std::size_t k1 = std::min(k+1, n-1);
    const double t0 = in.sample_times[k], t1 = in.sample_times[k1];
    const double raw_u = std::abs(t1-t0) > kEps ? (t-t0)/(t1-t0) : 0.0;
    const double u = std::max(0.0, std::min(1.0, raw_u));
    auto interp_tire = [&](const double* data, std::size_t j) {
        if (!data) return 0.0;
        const double a = data[k*nt+j], b = data[k1*nt+j];
        return (1.0-u)*a+u*b;
    };
    auto interp_body = [&](const double* data, std::size_t j) {
        if (!data) return 0.0;
        const std::size_t stride = nb * 6;
        const double a = data[k*stride+j], b = data[k1*stride+j];
        return (1.0-u)*a+u*b;
    };
    for (std::size_t j = 0; j < nb * 6; ++j) {
        out.body_wrench[j] = interp_body(in.body_wrench, j);
    }
    for (std::size_t j = 0; j < nt; ++j) {
        out.road_z[j] = interp_tire(in.road_z, j);
        out.road_v[j] = interp_tire(in.road_z_velocity, j);
        out.torque[j] = interp_tire(in.wheel_torque, j);
    }
}

void interpolate_input(
    const Model& model, const AxleInput& in, double t, SampleInput& out
) {
    interpolate_input(in, t, out);
    const std::size_t n = in.sample_count;
    const std::size_t count = model.steering_actuators.size();
    if (n > 0 && model.vehicle_brake_torque != nullptr) {
        const std::size_t k = [&]() {
            std::size_t index = 0;
            while (index + 1 < n && in.sample_times[index + 1] < t) {
                ++index;
            }
            return index;
        }();
        const std::size_t k1 = std::min(k + 1, n - 1);
        const double t0 = in.sample_times[k];
        const double t1 = in.sample_times[k1];
        const double raw_u = std::abs(t1 - t0) > kEps
            ? (t - t0) / (t1 - t0) : 0.0;
        const double u = std::max(0.0, std::min(1.0, raw_u));
        const std::size_t tire_count = in.tire_count;
        for (std::size_t j = 0; j < tire_count; ++j) {
            const double a = model.vehicle_brake_torque[k*tire_count+j];
            const double b = model.vehicle_brake_torque[k1*tire_count+j];
            out.brake_torque[j] = (1.0 - u)*a + u*b;
        }
    }
    out.steering_target.assign(count, 0.0);
    out.steering_target_rate.assign(count, 0.0);
    out.steering_target_acceleration.assign(count, 0.0);
    if (n == 0 || count == 0) return;
    std::size_t k = 0;
    while (k + 1 < n && in.sample_times[k + 1] < t) ++k;
    const std::size_t k1 = std::min(k + 1, n - 1);
    const double t0 = in.sample_times[k];
    const double t1 = in.sample_times[k1];
    const double raw_u = std::abs(t1 - t0) > kEps ? (t - t0) / (t1 - t0) : 0.0;
    const double u = std::max(0.0, std::min(1.0, raw_u));
    for (std::size_t j = 0; j < count; ++j) {
        const SteeringActuator& actuator = model.steering_actuators[j];
        if (actuator.target_angle != nullptr) {
            const double a = actuator.target_angle[k * count + j];
            const double b = actuator.target_angle[k1 * count + j];
            out.steering_target[j] = (1.0 - u) * a + u * b;
        }
        if (actuator.target_rate != nullptr) {
            const double a = actuator.target_rate[k * count + j];
            const double b = actuator.target_rate[k1 * count + j];
            out.steering_target_rate[j] = (1.0 - u) * a + u * b;
            if (k1 != k && std::abs(t1 - t0) > kEps) {
                out.steering_target_acceleration[j] = (b-a)/(t1-t0);
            }
        }
    }
}

// The public schema currently uses one time grid for both outputs and
// prescribed inputs. Keep this boundary query in the native loop anyway: it
// prevents a future caller with a coarser output target from stepping across a
// piecewise-linear input knot in one adaptive trial.
double next_prescribed_input_breakpoint(
    const AxleInput& input, double time, double target
) {
    if (input.sample_times == nullptr || input.sample_count < 2) {
        return target;
    }
    const double tolerance =
        1e-14 * std::max({1.0, std::abs(time), std::abs(target)});
    for (std::size_t index = 0; index < input.sample_count; ++index) {
        const double knot = input.sample_times[index];
        if (knot > time + tolerance && knot < target - tolerance) {
            return knot;
        }
    }
    return target;
}

struct ResidualContext {
    const Model* model{};
    const AxleInput* input{};
    const SampleInput* previous_sample{};
    const SampleInput* evaluation_sample{};
    const SampleInput* next_sample{};
    const SampleInput* internal_evaluation_sample{};
    State previous{};
    double h{0.0};
    double alpha_m{0.0}, alpha_f{0.0}, beta{0.0}, gamma{0.0};
    double alpha_m_z{0.0}, alpha_f_z{0.0}, gamma_z{0.0};
    PerformanceCounters* performance{nullptr};
};

bool NewtonLinearizationCache::matches(
    const ResidualContext& context, int dimension
) const {
    return valid && dim == dimension &&
        h == context.h && alpha_m == context.alpha_m &&
        alpha_f == context.alpha_f && beta == context.beta &&
        gamma == context.gamma && alpha_m_z == context.alpha_m_z &&
        alpha_f_z == context.alpha_f_z && gamma_z == context.gamma_z;
}

void NewtonLinearizationCache::update_key(
    const ResidualContext& context, int dimension
) {
    dim = dimension;
    h = context.h;
    alpha_m = context.alpha_m;
    alpha_f = context.alpha_f;
    beta = context.beta;
    gamma = context.gamma;
    alpha_m_z = context.alpha_m_z;
    alpha_f_z = context.alpha_f_z;
    gamma_z = context.gamma_z;
    reuse_steps = 0;
    valid = true;
}

// All mutable storage used by one residual evaluation lives here. A Newton
// step owns one primary instance; OpenMP finite-difference columns use one
// instance per worker slot. No instance is shared across axle_run calls.
struct ResidualWorkspace {
    State next{};
    State evaluation{};
    State internal_evaluation{};
    std::vector<double> dy;
    std::vector<double> pose_increment;
    std::vector<double> a_next;
    std::vector<double> v_next;
    std::vector<double> mu;
    std::vector<double> lambda;
    std::vector<double> J_storage;
    std::vector<double> J_eval_storage;
    std::vector<double> pose_cache;
    bool pose_cache_valid{false};
    std::vector<double> tire_forces;
    std::vector<double> tire_brush_derivatives;
    std::vector<double> tire_output;
    std::vector<double> generalized_force;
    std::vector<Vec3> body_force;
    std::vector<Vec3> body_torque;
    std::vector<double> q_old_v;
    std::vector<double> perturbed_x;
    std::vector<double> output;
    std::vector<double> dy_ga;
    std::vector<double> mass_mu;
    std::vector<double> analytic_unit;
    std::vector<double> analytic_scaled;
    std::vector<double> phi_storage;
    std::vector<double> internal_dy;
    std::vector<double> internal_tire_forces;
    std::vector<double> internal_tire_derivatives;
    std::vector<double> internal_tire_output;

    void ensure(const Model& model, int dimension) {
        const std::size_t body_count = model.bodies.size();
        const std::size_t tire_count = model.tires.size();
        const std::size_t n = static_cast<std::size_t>(model.ndof);
        const std::size_t m = static_cast<std::size_t>(model.rows);
        const std::size_t nz = 2 * tire_count;
        const auto ensure_state = [body_count, tire_count](State& state) {
            state.r.resize(body_count);
            state.v.resize(body_count);
            state.a.resize(body_count);
            state.omega.resize(body_count);
            state.alpha.resize(body_count);
            state.q.resize(body_count);
            state.tire_sx.resize(tire_count);
            state.tire_sy.resize(tire_count);
            state.tire_sx_dot.resize(tire_count);
            state.tire_sy_dot.resize(tire_count);
        };
        ensure_state(next);
        ensure_state(evaluation);
        ensure_state(internal_evaluation);
        dy.resize(n);
        pose_increment.resize(n);
        if (pose_cache.size() != n) {
            pose_cache.resize(n);
            pose_cache_valid = false;
        }
        a_next.resize(n);
        v_next.resize(n);
        mu.resize(m);
        lambda.resize(m);
        J_storage.resize(m*n);
        J_eval_storage.resize(m*n);
        tire_forces.resize(tire_count);
        tire_brush_derivatives.resize(nz);
        tire_output.resize(tire_count*kTireOutputWidth);
        generalized_force.resize(n);
        body_force.resize(body_count);
        body_torque.resize(body_count);
        q_old_v.resize(n);
        perturbed_x.resize(static_cast<std::size_t>(dimension));
        output.resize(static_cast<std::size_t>(dimension));
        dy_ga.resize(n);
        mass_mu.resize(n);
        analytic_unit.resize(n);
        analytic_scaled.resize(n);
        phi_storage.resize(m);
        internal_dy.resize(n);
        internal_tire_forces.resize(tire_count);
        internal_tire_derivatives.resize(nz);
        internal_tire_output.resize(tire_count*kTireOutputWidth);
    }
};

void pose_candidate(
    const State& base, const Model& model, const std::vector<double>& dy,
    State& candidate
) {
    candidate = base;
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        candidate.r[bi] += Vec3{dy[6*fi],dy[6*fi+1],dy[6*fi+2]};
        candidate.q[bi] = normalized_continuous(
            qmul(
                qexp({dy[6*fi+3],dy[6*fi+4],dy[6*fi+5]}),
                candidate.q[bi]
            ),
            candidate.q[bi]
        );
    }
}

State pose_candidate(const State& base, const Model& model, const std::vector<double>& dy) {
    State candidate = base;
    pose_candidate(base, model, dy, candidate);
    return candidate;
}

State state_from_unknown(
    const ResidualContext& ctx, const std::vector<double>& x, std::vector<double>& a_next,
    std::vector<double>& v_next, std::vector<double>& mu,
    std::vector<double>& lambda) {
    const Model& model = *ctx.model;
    const int n = model.ndof;
    const int m = model.rows;
    const int nz = 2 * static_cast<int>(model.tires.size());
    std::vector<double> dy(n, 0.0);
    std::copy(x.begin(), x.begin()+n, dy.begin());
    State next = pose_candidate(ctx.previous, model, dy);
    v_next.assign(x.begin()+n, x.begin()+2*n);
    a_next.assign(x.begin()+2*n, x.begin()+3*n);
    lambda.assign(x.begin()+3*n, x.begin()+3*n+m);
    mu.assign(x.begin()+3*n+m, x.begin()+3*n+2*m);
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        next.v[bi] = {v_next[6*fi],v_next[6*fi+1],v_next[6*fi+2]};
        next.omega[bi] = {v_next[6*fi+3],v_next[6*fi+4],v_next[6*fi+5]};
    }
    next.tire_sx.resize(model.tires.size(), 0.0);
    next.tire_sy.resize(model.tires.size(), 0.0);
    const int z_offset = 3*n + 2*m;
    for (int i = 0; i < nz / 2; ++i) {
        next.tire_sx[static_cast<std::size_t>(i)] = x[z_offset + 2*i];
        next.tire_sy[static_cast<std::size_t>(i)] = x[z_offset + 2*i + 1];
    }
    return next;
}

void state_from_unknown(
    const ResidualContext& ctx, const std::vector<double>& x,
    ResidualWorkspace& workspace
) {
    const Model& model = *ctx.model;
    const int n = model.ndof;
    const int m = model.rows;
    const int nz = 2 * static_cast<int>(model.tires.size());
    std::copy(x.begin(), x.begin()+n, workspace.dy.begin());
    workspace.pose_increment = workspace.dy;
    pose_candidate(ctx.previous, model, workspace.dy, workspace.next);
    std::copy(x.begin()+n, x.begin()+2*n, workspace.v_next.begin());
    std::copy(x.begin()+2*n, x.begin()+3*n, workspace.a_next.begin());
    std::copy(x.begin()+3*n, x.begin()+3*n+m, workspace.lambda.begin());
    std::copy(
        x.begin()+3*n+m, x.begin()+3*n+2*m, workspace.mu.begin()
    );
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        workspace.next.v[bi] = {
            workspace.v_next[6*fi], workspace.v_next[6*fi+1],
            workspace.v_next[6*fi+2]
        };
        workspace.next.omega[bi] = {
            workspace.v_next[6*fi+3], workspace.v_next[6*fi+4],
            workspace.v_next[6*fi+5]
        };
    }
    workspace.next.tire_sx.resize(model.tires.size(), 0.0);
    workspace.next.tire_sy.resize(model.tires.size(), 0.0);
    const int z_offset = 3*n + 2*m;
    for (int i = 0; i < nz / 2; ++i) {
        workspace.next.tire_sx[static_cast<std::size_t>(i)] =
            x[z_offset + 2*i];
        workspace.next.tire_sy[static_cast<std::size_t>(i)] =
            x[z_offset + 2*i + 1];
    }
}

// Constraint Jacobians evaluated at the two poses a residual needs.  Both
// depend only on the pose increment block of the unknown vector, so a finite
// difference on a velocity, multiplier or brush column may reuse them.
struct PoseJacobians {
    std::vector<double> at_next;
    std::vector<double> at_evaluation;
    std::vector<double> position_residual;
    bool valid{false};
};

void residual(
    const ResidualContext& ctx, const std::vector<double>& x,
    std::vector<double>& out, double& pos_res, double& vel_res,
    double& dyn_res, int& active, ResidualWorkspace& workspace,
    const PoseJacobians* pose_jacobians = nullptr
) {
    if (ctx.performance != nullptr) {
        ctx.performance->residual_calls.fetch_add(1, std::memory_order_relaxed);
    }
    ScopedPerformanceTimer residual_timer(
        ctx.performance == nullptr
            ? nullptr
            : &ctx.performance->residual_nanoseconds
    );
    const Model& model = *ctx.model;
    const int n = model.ndof, m = model.rows;
    const int nz = 2 * static_cast<int>(model.tires.size());
    workspace.ensure(model, 3*n+2*m+nz);
    std::vector<double>& a_next = workspace.a_next;
    std::vector<double>& v_next = workspace.v_next;
    std::vector<double>& mu = workspace.mu;
    std::vector<double>& lambda_from_state = workspace.lambda;
    state_from_unknown(ctx, x, workspace);
    State& next = workspace.next;
    std::vector<double>& dy = workspace.dy;
    for (double& value : dy) value *= (1.0-ctx.alpha_f);
    // Built directly; a `State evaluation = ctx.previous;` first would copy ten
    // vectors that the very next line overwrites.
    State& evaluation = workspace.evaluation;
    pose_candidate(ctx.previous, model, dy, evaluation);
    for (int fi=0; fi<static_cast<int>(model.free_body.size()); ++fi) {
        const int bi=model.free_body[fi];
        evaluation.v[bi] = ctx.previous.v[bi]*ctx.alpha_f + next.v[bi]*(1.0-ctx.alpha_f);
        evaluation.omega[bi] = ctx.previous.omega[bi]*ctx.alpha_f + next.omega[bi]*(1.0-ctx.alpha_f);
    }
    evaluation.tire_sx.resize(model.tires.size(), 0.0);
    evaluation.tire_sy.resize(model.tires.size(), 0.0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        evaluation.tire_sx[i] =
            ctx.previous.tire_sx[i] * ctx.alpha_f + next.tire_sx[i] * (1.0-ctx.alpha_f);
        evaluation.tire_sy[i] =
            ctx.previous.tire_sy[i] * ctx.alpha_f + next.tire_sy[i] * (1.0-ctx.alpha_f);
    }
    // Bind by reference: `const auto` copies an m*n matrix on every one of the
    // many residual evaluations a Jacobian assembly performs.  The storage is a
    // plain local, so no state is shared between calls.
    std::vector<double>& J_storage = workspace.J_storage;
    std::vector<double>& J_eval_storage = workspace.J_eval_storage;
    const bool external_cached = pose_jacobians && pose_jacobians->valid;
    bool cached = external_cached;
    if (!cached && workspace.pose_cache_valid) {
        cached = std::equal(
            x.begin(), x.begin()+n, workspace.pose_cache.begin()
        );
    }
    if (!cached) {
        if (ctx.performance != nullptr) {
            ctx.performance->constraint_jacobian_calls.fetch_add(
                2, std::memory_order_relaxed
            );
        }
        ScopedPerformanceTimer jacobian_timer(
            ctx.performance == nullptr
                ? nullptr
                : &ctx.performance->constraint_jacobian_nanoseconds
        );
        J_storage = constraint_jacobian(model, next);
        J_eval_storage = constraint_jacobian(model, evaluation);
        std::copy(x.begin(), x.begin()+n, workspace.pose_cache.begin());
        workspace.pose_cache_valid = true;
    }
    const std::vector<double>& J =
        external_cached ? pose_jacobians->at_next : J_storage;
    const std::vector<double>& J_eval =
        external_cached ? pose_jacobians->at_evaluation : J_eval_storage;
    const SampleInput& sample = *ctx.evaluation_sample;
    std::vector<double>& tire_forces = workspace.tire_forces;
    std::vector<double>& tire_brush_derivatives = workspace.tire_brush_derivatives;
    std::vector<double>& tire_output = workspace.tire_output;
    double potential = 0.0;
    double external_power = 0.0;
    double dissipation = 0.0;
    if (ctx.performance != nullptr) {
        ctx.performance->force_evaluations.fetch_add(
            1, std::memory_order_relaxed
        );
    }
    std::vector<double>& force = workspace.generalized_force;
    {
        ScopedPerformanceTimer force_timer(
            ctx.performance == nullptr
                ? nullptr
                : &ctx.performance->force_nanoseconds
        );
        external_force_vector(
            model, evaluation, sample, ctx.input->gravity_x, ctx.input->gravity_y,
            ctx.input->gravity_z, tire_forces, tire_brush_derivatives, tire_output,
            potential, external_power, dissipation, force,
            nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, false,
            &workspace.body_force, &workspace.body_torque, true
        );
    }
    const std::vector<double>& lambda = lambda_from_state;
    std::vector<double>& q_old_v = workspace.q_old_v;
    std::fill(q_old_v.begin(), q_old_v.end(), 0.0);
    for (int fi=0; fi<static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        q_old_v[6*fi]=ctx.previous.v[bi].x;
        q_old_v[6*fi+1]=ctx.previous.v[bi].y;
        q_old_v[6*fi+2]=ctx.previous.v[bi].z;
        q_old_v[6*fi+3]=ctx.previous.omega[bi].x;
        q_old_v[6*fi+4]=ctx.previous.omega[bi].y;
        q_old_v[6*fi+5]=ctx.previous.omega[bi].z;
    }
    out.resize(static_cast<std::size_t>(3*n+2*m+nz));
    const double rho = ctx.input->rho_inf;
    (void)rho;
    std::vector<double>& dy_ga = workspace.dy_ga;
    std::fill(dy_ga.begin(), dy_ga.end(), 0.0);
    for (int fi=0; fi<static_cast<int>(model.free_body.size()); ++fi) {
        const int bi=model.free_body[fi];
        dy_ga[6*fi] = ctx.h*q_old_v[6*fi] + ctx.h*ctx.h*((0.5-ctx.beta)*ctx.previous.a[bi].x + ctx.beta*a_next[6*fi]);
        dy_ga[6*fi+1] = ctx.h*q_old_v[6*fi+1] + ctx.h*ctx.h*((0.5-ctx.beta)*ctx.previous.a[bi].y + ctx.beta*a_next[6*fi+1]);
        dy_ga[6*fi+2] = ctx.h*q_old_v[6*fi+2] + ctx.h*ctx.h*((0.5-ctx.beta)*ctx.previous.a[bi].z + ctx.beta*a_next[6*fi+2]);
        dy_ga[6*fi+3] = ctx.h*q_old_v[6*fi+3] + ctx.h*ctx.h*((0.5-ctx.beta)*ctx.previous.alpha[bi].x + ctx.beta*a_next[6*fi+3]);
        dy_ga[6*fi+4] = ctx.h*q_old_v[6*fi+4] + ctx.h*ctx.h*((0.5-ctx.beta)*ctx.previous.alpha[bi].y + ctx.beta*a_next[6*fi+4]);
        dy_ga[6*fi+5] = ctx.h*q_old_v[6*fi+5] + ctx.h*ctx.h*((0.5-ctx.beta)*ctx.previous.alpha[bi].z + ctx.beta*a_next[6*fi+5]);
    }
    std::vector<double>& mass_mu = workspace.mass_mu;
    if (ctx.performance != nullptr) {
        ctx.performance->mass_inverse_calls.fetch_add(
            1, std::memory_order_relaxed
        );
    }
    bool mass_inverse_ok = false;
    {
        ScopedPerformanceTimer mass_inverse_timer(
            ctx.performance == nullptr
                ? nullptr
                : &ctx.performance->mass_inverse_nanoseconds
        );
        mass_inverse_ok = mass_inverse_of_jt_mu(
            model, next, J, mu, n, m, mass_mu
        );
    }
    if (!mass_inverse_ok) {
        // Unreachable for a validated model: free-body inertia is checked to be
        // positive definite at load, so the world inertia is always invertible.
        // Return a non-finite residual rather than a silently wrong one.
        std::fill(
            out.begin(), out.end(), std::numeric_limits<double>::quiet_NaN()
        );
        pos_res = vel_res = dyn_res =
            std::numeric_limits<double>::quiet_NaN();
        active = 0;
        return;
    }
    for (int i=0; i<n; ++i) out[i] = x[i]-dy_ga[i]-mass_mu[i];
    for (int fi=0; fi<static_cast<int>(model.free_body.size()); ++fi) {
        const int bi=model.free_body[fi];
        const int off=n+6*fi;
        out[off+0]=v_next[6*fi]-q_old_v[6*fi]-ctx.h*((1-ctx.gamma)*ctx.previous.a[bi].x+ctx.gamma*a_next[6*fi]);
        out[off+1]=v_next[6*fi+1]-q_old_v[6*fi+1]-ctx.h*((1-ctx.gamma)*ctx.previous.a[bi].y+ctx.gamma*a_next[6*fi+1]);
        out[off+2]=v_next[6*fi+2]-q_old_v[6*fi+2]-ctx.h*((1-ctx.gamma)*ctx.previous.a[bi].z+ctx.gamma*a_next[6*fi+2]);
        out[off+3]=v_next[6*fi+3]-q_old_v[6*fi+3]-ctx.h*((1-ctx.gamma)*ctx.previous.alpha[bi].x+ctx.gamma*a_next[6*fi+3]);
        out[off+4]=v_next[6*fi+4]-q_old_v[6*fi+4]-ctx.h*((1-ctx.gamma)*ctx.previous.alpha[bi].y+ctx.gamma*a_next[6*fi+4]);
        out[off+5]=v_next[6*fi+5]-q_old_v[6*fi+5]-ctx.h*((1-ctx.gamma)*ctx.previous.alpha[bi].z+ctx.gamma*a_next[6*fi+5]);
    }
    double force_scale = 1.0;
    ScopedPerformanceTimer reaction_timer(
        ctx.performance == nullptr
            ? nullptr
            : &ctx.performance->reaction_nanoseconds
    );
    for (int fi=0; fi<static_cast<int>(model.free_body.size()); ++fi) {
        const int bi=model.free_body[fi];
        const Body& b=model.bodies[bi];
        const Mat3 R=qmat(evaluation.q[bi]);
        const Mat3 Iw=R*b.inertia_body*transpose(R);
        Vec3 av{
            (1-ctx.alpha_m)*a_next[6*fi]+ctx.alpha_m*ctx.previous.a[bi].x,
            (1-ctx.alpha_m)*a_next[6*fi+1]+ctx.alpha_m*ctx.previous.a[bi].y,
            (1-ctx.alpha_m)*a_next[6*fi+2]+ctx.alpha_m*ctx.previous.a[bi].z};
        Vec3 aa{
            (1-ctx.alpha_m)*a_next[6*fi+3]+ctx.alpha_m*ctx.previous.alpha[bi].x,
            (1-ctx.alpha_m)*a_next[6*fi+4]+ctx.alpha_m*ctx.previous.alpha[bi].y,
            (1-ctx.alpha_m)*a_next[6*fi+5]+ctx.alpha_m*ctx.previous.alpha[bi].z};
        const int off=2*n+6*fi;
        const double* Fx=force.data();
        // external_force_vector already moved the gyroscopic term onto the
        // right-hand side, so the rotational rows must not add it again.
        const Vec3 inertial_force = av*b.mass;
        const Vec3 inertial_torque = Iw*aa;
        out[off+0]=inertial_force.x-Fx[6*fi+0];
        out[off+1]=inertial_force.y-Fx[6*fi+1];
        out[off+2]=inertial_force.z-Fx[6*fi+2];
        out[off+3]=inertial_torque.x-Fx[6*fi+3];
        out[off+4]=inertial_torque.y-Fx[6*fi+4];
        out[off+5]=inertial_torque.z-Fx[6*fi+5];
        // The dynamics rows are compared against a normalized tolerance, so the
        // largest term entering them sets the scale of "small".
        const double terms[6] = {
            inertial_force.x, inertial_force.y, inertial_force.z,
            inertial_torque.x, inertial_torque.y, inertial_torque.z
        };
        for (int k=0; k<6; ++k) {
            force_scale = std::max(force_scale, std::abs(terms[k]));
            force_scale = std::max(force_scale, std::abs(Fx[6*fi+k]));
        }
    }
    // Constraint Jacobians are local to their two endpoints.  Traverse the
    // descriptor in constraint order instead of scanning every row for every
    // body; each free-body accumulation sees the same row order as before.
    for (const Constraint& constraint : model.constraints) {
        const int rows = constraint_rows(constraint.type);
        const int endpoints[2] = {constraint.a, constraint.b};
        const int endpoint_count = constraint.a == constraint.b ? 1 : 2;
        for (int row_offset = 0; row_offset < rows; ++row_offset) {
            const int row = constraint.row + row_offset;
            for (int endpoint = 0; endpoint < endpoint_count; ++endpoint) {
                const int fi = model.body_to_free[
                    static_cast<std::size_t>(endpoints[endpoint])
                ];
                if (fi < 0) continue;
                const int off = 2*n + 6*fi;
                for (int k = 0; k < 6; ++k) {
                    const double reaction =
                        J_eval[row*n+6*fi+k]*lambda[row];
                    out[off+k] -= reaction;
                    force_scale = std::max(force_scale, std::abs(reaction));
                }
            }
        }
    }
    for (const CoordinateCoupler& coupler : model.coordinate_couplers) {
        const Constraint& joint_a = model.constraints[
            static_cast<std::size_t>(coupler.joint_a)
        ];
        const Constraint& joint_b = model.constraints[
            static_cast<std::size_t>(coupler.joint_b)
        ];
        const int endpoints[4] = {
            joint_a.a, joint_a.b, joint_b.a, joint_b.b
        };
        for (int endpoint = 0; endpoint < 4; ++endpoint) {
            bool duplicate = false;
            for (int previous = 0; previous < endpoint; ++previous) {
                duplicate = duplicate || endpoints[previous] == endpoints[endpoint];
            }
            if (duplicate) continue;
            const int fi = model.body_to_free[
                static_cast<std::size_t>(endpoints[endpoint])
            ];
            if (fi < 0) continue;
            const int row = coupler.row;
            const int off = 2*n + 6*fi;
            for (int k = 0; k < 6; ++k) {
                const double reaction =
                    J_eval[row*n+6*fi+k]*lambda[row];
                out[off+k] -= reaction;
                force_scale = std::max(force_scale, std::abs(reaction));
            }
        }
    }
    for (const SteeringActuator& actuator : model.steering_actuators) {
        if (!prescribed_steering(actuator)) continue;
        const int endpoints[2] = {actuator.body, actuator.reaction_body};
        for (int endpoint = 0; endpoint < 2; ++endpoint) {
            const int body = endpoints[endpoint];
            if (body < 0) continue;
            const int fi = model.body_to_free[static_cast<std::size_t>(body)];
            if (fi < 0) continue;
            const int off = 2*n+6*fi;
            for (int k = 0; k < 6; ++k) {
                const double reaction =
                    J_eval[actuator.constraint_row*n+6*fi+k]
                    * lambda[actuator.constraint_row];
                out[off+k] -= reaction;
                force_scale = std::max(force_scale, std::abs(reaction));
            }
        }
    }
    std::vector<double>& phi_storage = workspace.phi_storage;
    if (!cached) {
        phi_storage = constraint_residual(model, next, ctx.next_sample);
    }
    const std::vector<double>& phi =
        external_cached ? pose_jacobians->position_residual : phi_storage;
    for (int i=0;i<m;++i) out[3*n+i]=phi[i];
    for (int row=0;row<m;++row) {
        double s=0.0;
        for (int col=0;col<n;++col) s += J[row*n+col]*v_next[col];
        out[3*n+m+row]=s;
    }
    for (const auto& actuator : model.steering_actuators) {
        if (!prescribed_steering(actuator)) continue;
        const std::size_t index = static_cast<std::size_t>(
            &actuator - model.steering_actuators.data()
        );
        if (index < ctx.next_sample->steering_target_rate.size()) {
            out[3*n+m+actuator.constraint_row] -=
                ctx.next_sample->steering_target_rate[index];
        }
    }
    std::vector<double>& internal_dy = workspace.internal_dy;
    std::copy(x.begin(), x.begin()+n, internal_dy.begin());
    for (double& value : internal_dy) value *= ctx.alpha_f_z;
    State& internal_evaluation = workspace.internal_evaluation;
    pose_candidate(ctx.previous, model, internal_dy, internal_evaluation);
    for (int fi=0; fi<static_cast<int>(model.free_body.size()); ++fi) {
        const int bi=model.free_body[fi];
        internal_evaluation.v[bi] =
            ctx.previous.v[bi]*(1.0-ctx.alpha_f_z)
            + next.v[bi]*ctx.alpha_f_z;
        internal_evaluation.omega[bi] =
            ctx.previous.omega[bi]*(1.0-ctx.alpha_f_z)
            + next.omega[bi]*ctx.alpha_f_z;
    }
    internal_evaluation.tire_sx.resize(model.tires.size(), 0.0);
    internal_evaluation.tire_sy.resize(model.tires.size(), 0.0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        internal_evaluation.tire_sx[i] =
            ctx.previous.tire_sx[i] * (1.0-ctx.alpha_f_z)
            + next.tire_sx[i] * ctx.alpha_f_z;
        internal_evaluation.tire_sy[i] =
            ctx.previous.tire_sy[i] * (1.0-ctx.alpha_f_z)
            + next.tire_sy[i] * ctx.alpha_f_z;
    }
    std::vector<double>& internal_tire_forces = workspace.internal_tire_forces;
    std::vector<double>& internal_tire_derivatives =
        workspace.internal_tire_derivatives;
    std::vector<double>& internal_tire_output = workspace.internal_tire_output;
    double internal_potential = 0.0;
    double internal_external_power = 0.0;
    double internal_dissipation = 0.0;
    if (ctx.performance != nullptr) {
        ctx.performance->force_evaluations.fetch_add(
            1, std::memory_order_relaxed
        );
    }
    {
        ScopedPerformanceTimer force_timer(
            ctx.performance == nullptr
                ? nullptr
                : &ctx.performance->force_nanoseconds
        );
        external_force_vector(
            model, internal_evaluation, *ctx.internal_evaluation_sample,
            ctx.input->gravity_x, ctx.input->gravity_y,
            ctx.input->gravity_z, internal_tire_forces, internal_tire_derivatives,
            internal_tire_output, internal_potential, internal_external_power,
            internal_dissipation, workspace.generalized_force,
            nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
            /*brush_only=*/true,
            &workspace.body_force, &workspace.body_torque
        );
    }
    const int z_offset = 3*n + 2*m;
    for (int i = 0; i < nz; ++i) {
        const std::size_t tire = static_cast<std::size_t>(i / 2);
        const double old_value = i % 2 == 0
            ? ctx.previous.tire_sx[tire]
            : ctx.previous.tire_sy[tire];
        const double old_derivative = i % 2 == 0
            ? ctx.previous.tire_sx_dot[tire]
            : ctx.previous.tire_sy_dot[tire];
        const double next_derivative =
            (x[z_offset+i]-old_value)/(ctx.gamma_z*ctx.h)
            - (1.0-ctx.gamma_z)/ctx.gamma_z*old_derivative;
        out[z_offset+i] =
            ctx.alpha_m_z*next_derivative
            + (1.0-ctx.alpha_m_z)*old_derivative
            - internal_tire_derivatives[static_cast<std::size_t>(i)];
    }
    pos_res=max_abs(phi);
    vel_res=0.0;
    for (int row=0;row<m;++row) {
        vel_res=std::max(vel_res,std::abs(out[3*n+m+row]));
    }
    dyn_res=0.0;
    for (int i=2*n;i<3*n;++i) dyn_res=std::max(dyn_res,std::abs(out[i]));
    dyn_res/=force_scale;
    active=0;
    for (double f:tire_forces) if (f>0.0) ++active;
    (void)potential;
}

bool finite_vec(const std::vector<double>& v) {
    for (double x:v) if (!std::isfinite(x)) return false;
    return true;
}

// Fill Newton Jacobian columns that are either exactly linear or have a
// directional force derivative. `analytic_columns` is also the authoritative
// list used by the finite-difference fallback and the runtime audit.
//
//   a_next : GA pose/velocity updates and the inertial terms
//   lambda : the reaction term -J_eval^T lambda in the dynamics rows
//   mu     : the GGL position augmentation -W J^T mu in the pose rows
void fill_analytic_jacobian_columns(
    const ResidualContext& ctx, int dim, std::vector<double>& J,
    ResidualWorkspace& workspace, const PoseJacobians& pose_jacobians,
    std::vector<unsigned char>& analytic_columns
) {
    const Model& model = *ctx.model;
    const int n = model.ndof;
    const int m = model.rows;
    const int nz = 2 * static_cast<int>(model.tires.size());
    const auto at = [dim](int row, int col) {
        return static_cast<std::size_t>(row)*static_cast<std::size_t>(dim)
             + static_cast<std::size_t>(col);
    };

    // The base residual already built these states and constraint Jacobians for
    // this x. Reuse them instead of reconstructing the same nonlinear data.
    const State& next = workspace.next;
    const State& evaluation = workspace.evaluation;
    const std::vector<double>& J_next = pose_jacobians.at_next;
    const std::vector<double>& J_evaluation = pose_jacobians.at_evaluation;

    analytic_columns.assign(static_cast<std::size_t>(dim), 0);

    // d(pose rows)/d(a_next)     = -h^2*beta
    // d(velocity rows)/d(a_next) = -h*gamma
    // d(dynamics rows)/d(a_next) = (1-alpha_m) * M(evaluation)
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        const Body& body = model.bodies[bi];
        const Mat3 rotation = qmat(evaluation.q[bi]);
        const Mat3 world_inertia =
            rotation * body.inertia_body * transpose(rotation);
        for (int k = 0; k < 6; ++k) {
            const int column = 2*n + 6*fi + k;
            analytic_columns[static_cast<std::size_t>(column)] = 1;
            J[at(6*fi+k, column)] = -ctx.h*ctx.h*ctx.beta;
            J[at(n + 6*fi + k, column)] = -ctx.h*ctx.gamma;
        }
        for (int k = 0; k < 3; ++k) {
            J[at(2*n + 6*fi + k, 2*n + 6*fi + k)] =
                (1.0-ctx.alpha_m)*body.mass;
        }
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                J[at(2*n + 6*fi + 3 + row, 2*n + 6*fi + 3 + col)] =
                    (1.0-ctx.alpha_m)*world_inertia.a[row][col];
            }
        }
    }

    // d(dynamics rows)/d(lambda) = -J_eval^T
    for (int row = 0; row < m; ++row) {
        analytic_columns[static_cast<std::size_t>(3*n + row)] = 1;
        for (int col = 0; col < n; ++col) {
            J[at(2*n + col, 3*n + row)] =
                -J_evaluation[static_cast<std::size_t>(row*n + col)];
        }
    }

    // d(pose rows)/d(mu) = -M(next)^-1 J(next)^T, one column per multiplier.
    std::vector<double>& unit = workspace.analytic_unit;
    std::vector<double>& scaled = workspace.analytic_scaled;
    for (int row = 0; row < m; ++row) {
        analytic_columns[static_cast<std::size_t>(3*n + m + row)] = 1;
        std::fill(unit.begin(), unit.end(), 0.0);
        for (int col = 0; col < n; ++col) {
            unit[static_cast<std::size_t>(col)] =
                J_next[static_cast<std::size_t>(row*n + col)];
        }
        if (!mass_inverse_into(model, next, unit, scaled)) continue;
        for (int col = 0; col < n; ++col) {
            J[at(col, 3*n + m + row)] =
                -scaled[static_cast<std::size_t>(col)];
        }
    }

    const int z_offset = 3*n + 2*m;
    struct AnalyticColumnWorkspace {
        DirectionalState next_direction;
        DirectionalState evaluation_direction;
        DirectionalState internal_direction;
        DirectionalForceScratch directional_force_scratch;
        std::vector<double> dJ_next;
        std::vector<double> dJ_evaluation;
        std::vector<std::size_t> dJ_next_touched;
        std::vector<std::size_t> dJ_evaluation_touched;
        std::vector<double> force_derivative;
        std::vector<double> brush_derivative;
        std::vector<double> unused_force;
        std::vector<double> internal_brush_derivative;
        std::vector<double> mass_mu_derivative;
        std::vector<double> velocity_constraint_derivative;
        std::vector<double> reaction_derivative;
    };
    int analytic_threads = 1;
#ifdef _OPENMP
    analytic_threads = std::max(1, std::min(4, dim/4));
    if (const char* value = std::getenv("SUSPENSION_AXLE_ANALYTIC_THREADS")) {
        const int requested = std::atoi(value);
        if (requested > 0) analytic_threads = requested;
    }
#endif
    std::vector<AnalyticColumnWorkspace> analytic_workspaces(
        static_cast<std::size_t>(analytic_threads)
    );
    for (AnalyticColumnWorkspace& column_workspace : analytic_workspaces) {
        ensure_directional_state(model, column_workspace.next_direction);
        ensure_directional_state(model, column_workspace.evaluation_direction);
        ensure_directional_state(model, column_workspace.internal_direction);
        column_workspace.directional_force_scratch.force.resize(
            model.bodies.size()
        );
        column_workspace.directional_force_scratch.torque.resize(
            model.bodies.size()
        );
        column_workspace.velocity_constraint_derivative.resize(
            static_cast<std::size_t>(m)
        );
        column_workspace.reaction_derivative.resize(
            static_cast<std::size_t>(n)
        );
    }
    auto make_pose_direction = [&](
        DirectionalState& direction, int column, double scale
    ) {
        reset_directional_state(model, direction);
        const int free_body = column/6;
        const int component = column%6;
        const int body_index = model.free_body[
            static_cast<std::size_t>(free_body)
        ];
        if (component < 3) {
            direction.dr[static_cast<std::size_t>(body_index)] = Vec3{
                component == 0 ? scale : 0.0,
                component == 1 ? scale : 0.0,
                component == 2 ? scale : 0.0
            };
        } else {
            const Vec3 increment{
                workspace.pose_increment[static_cast<std::size_t>(6*free_body+3)],
                workspace.pose_increment[static_cast<std::size_t>(6*free_body+4)],
                workspace.pose_increment[static_cast<std::size_t>(6*free_body+5)]
            };
            const Vec3 parameter_direction{
                component == 3 ? scale : 0.0,
                component == 4 ? scale : 0.0,
                component == 5 ? scale : 0.0
            };
            direction.dtheta[static_cast<std::size_t>(body_index)] =
                so3_left_jacobian(increment*scale)*parameter_direction;
        }
    };

#ifdef _OPENMP
#pragma omp parallel num_threads(analytic_threads)
#endif
    {
        int worker_index = 0;
#ifdef _OPENMP
        worker_index = omp_get_thread_num();
#endif
        AnalyticColumnWorkspace& column_workspace = analytic_workspaces[
            static_cast<std::size_t>(worker_index)
        ];
        DirectionalState& next_direction = column_workspace.next_direction;
        DirectionalState& evaluation_direction =
            column_workspace.evaluation_direction;
        DirectionalState& internal_direction =
            column_workspace.internal_direction;
        DirectionalForceScratch& directional_force_scratch =
            column_workspace.directional_force_scratch;
        std::vector<double>& dJ_next = column_workspace.dJ_next;
        std::vector<double>& dJ_evaluation = column_workspace.dJ_evaluation;
        std::vector<std::size_t>& dJ_next_touched =
            column_workspace.dJ_next_touched;
        std::vector<std::size_t>& dJ_evaluation_touched =
            column_workspace.dJ_evaluation_touched;
        std::vector<double>& force_derivative =
            column_workspace.force_derivative;
        std::vector<double>& brush_derivative =
            column_workspace.brush_derivative;
        std::vector<double>& unused_force = column_workspace.unused_force;
        std::vector<double>& internal_brush_derivative =
            column_workspace.internal_brush_derivative;
        std::vector<double>& mass_mu_derivative =
            column_workspace.mass_mu_derivative;
        std::vector<double>& velocity_constraint_derivative =
            column_workspace.velocity_constraint_derivative;
        std::vector<double>& reaction_derivative =
            column_workspace.reaction_derivative;

#ifdef _OPENMP
#pragma omp for schedule(static)
#endif
    for (int column = 0; column < n; ++column) {
        make_pose_direction(next_direction, column, 1.0);
        const bool next_smooth = constraint_jacobian_directional(
            model, next, next_direction, dJ_next, &dJ_next_touched
        );
        make_pose_direction(evaluation_direction, column, 1.0-ctx.alpha_f);
        const bool evaluation_smooth = constraint_jacobian_directional(
            model, evaluation, evaluation_direction, dJ_evaluation,
            &dJ_evaluation_touched
        );
        const bool force_smooth = external_force_directional(
            model, evaluation, *ctx.evaluation_sample, evaluation_direction,
            force_derivative, brush_derivative, false, nullptr,
            &directional_force_scratch
        );
        make_pose_direction(internal_direction, column, ctx.alpha_f_z);
        const bool internal_smooth = external_force_directional(
            model, workspace.internal_evaluation,
            *ctx.internal_evaluation_sample, internal_direction,
            unused_force, internal_brush_derivative, true, nullptr,
            &directional_force_scratch
        );
        const bool mass_mu_smooth = mass_inverse_jt_mu_directional(
            model, next, next_direction, dJ_next, workspace.mu,
            workspace.mass_mu, n, mass_mu_derivative
        );
        if (
            !next_smooth || !evaluation_smooth || !force_smooth ||
            !internal_smooth || !mass_mu_smooth
        ) {
            continue;
        }
        analytic_columns[static_cast<std::size_t>(column)] = 1;

        // A directional constraint derivative is nonzero only in the six
        // columns belonging to each endpoint of an incident constraint.  Use
        // that block structure for both J_dot*v and J_dot^T*lambda instead of
        // scanning the full dense matrix for every pose column.
        std::fill(
            velocity_constraint_derivative.begin(),
            velocity_constraint_derivative.end(), 0.0
        );
        std::fill(reaction_derivative.begin(), reaction_derivative.end(), 0.0);
        for (const auto& c : model.constraints) {
            const bool next_active =
                directional_body_active(next_direction, c.a) ||
                directional_body_active(next_direction, c.b);
            const bool evaluation_active =
                directional_body_active(evaluation_direction, c.a) ||
                directional_body_active(evaluation_direction, c.b);
            if (!next_active && !evaluation_active) continue;
            const int rows = constraint_rows(c.type);
            const int endpoint_bodies[2] = {c.a, c.b};
            for (int row_offset = 0; row_offset < rows; ++row_offset) {
                const int row = c.row+row_offset;
                for (int endpoint = 0; endpoint < 2; ++endpoint) {
                    const int body_index = endpoint_bodies[endpoint];
                    const int fi = model.body_to_free[
                        static_cast<std::size_t>(body_index)
                    ];
                    if (fi < 0) continue;
                    for (int k = 0; k < 6; ++k) {
                        const int col = 6*fi+k;
                        if (next_active) {
                            velocity_constraint_derivative[
                                static_cast<std::size_t>(row)
                            ] += dJ_next[static_cast<std::size_t>(row*n+col)]
                                * workspace.v_next[static_cast<std::size_t>(col)];
                        }
                        if (evaluation_active) {
                            reaction_derivative[static_cast<std::size_t>(col)] +=
                                dJ_evaluation[
                                    static_cast<std::size_t>(row*n+col)
                                ]
                                * workspace.lambda[
                                    static_cast<std::size_t>(row)
                                ];
                        }
                    }
                }
            }
        }
        for (const auto& coupler : model.coordinate_couplers) {
            const Constraint& joint_a = model.constraints[
                static_cast<std::size_t>(coupler.joint_a)
            ];
            const Constraint& joint_b = model.constraints[
                static_cast<std::size_t>(coupler.joint_b)
            ];
            const int endpoints[4] = {
                joint_a.a, joint_a.b, joint_b.a, joint_b.b
            };
            const bool next_active = std::any_of(
                std::begin(endpoints), std::end(endpoints), [&](int body) {
                    return directional_body_active(next_direction, body);
                }
            );
            const bool evaluation_active = std::any_of(
                std::begin(endpoints), std::end(endpoints), [&](int body) {
                    return directional_body_active(evaluation_direction, body);
                }
            );
            if (!next_active && !evaluation_active) continue;
            const int row = coupler.row;
            for (int endpoint = 0; endpoint < 4; ++endpoint) {
                bool duplicate = false;
                for (int previous = 0; previous < endpoint; ++previous) {
                    duplicate = duplicate || endpoints[previous] == endpoints[endpoint];
                }
                if (duplicate) continue;
                const int fi = model.body_to_free[
                    static_cast<std::size_t>(endpoints[endpoint])
                ];
                if (fi < 0) continue;
                for (int k = 0; k < 6; ++k) {
                    const int col = 6*fi+k;
                    if (next_active) {
                        velocity_constraint_derivative[
                            static_cast<std::size_t>(row)
                        ] += dJ_next[static_cast<std::size_t>(row*n+col)]
                            * workspace.v_next[static_cast<std::size_t>(col)];
                    }
                    if (evaluation_active) {
                        reaction_derivative[static_cast<std::size_t>(col)] +=
                            dJ_evaluation[
                                static_cast<std::size_t>(row*n+col)
                            ] * workspace.lambda[static_cast<std::size_t>(row)];
                    }
                }
            }
        }
        for (const SteeringActuator& actuator : model.steering_actuators) {
            if (!prescribed_steering(actuator)) continue;
            const int endpoints[2] = {actuator.body, actuator.reaction_body};
            const bool translation = actuator.type ==
                VEHICLE_STEERING_PRESCRIBED_TRANSLATION;
            const bool next_active = translation
                ? directional_body_active(next_direction, actuator.body) ||
                    (actuator.reaction_body >= 0 && directional_body_active(
                        next_direction, actuator.reaction_body
                    ))
                : directional_orientation_active(
                    next_direction, actuator.body
                ) || (actuator.reaction_body >= 0 &&
                    directional_orientation_active(
                        next_direction, actuator.reaction_body
                    ));
            const bool evaluation_active = translation
                ? directional_body_active(
                    evaluation_direction, actuator.body
                ) || (actuator.reaction_body >= 0 && directional_body_active(
                    evaluation_direction, actuator.reaction_body
                ))
                : directional_orientation_active(
                    evaluation_direction, actuator.body
                ) || (actuator.reaction_body >= 0 &&
                    directional_orientation_active(
                        evaluation_direction, actuator.reaction_body
                    ));
            if (!next_active && !evaluation_active) continue;
            const int row = actuator.constraint_row;
            for (int endpoint = 0; endpoint < 2; ++endpoint) {
                const int body = endpoints[endpoint];
                if (body < 0) continue;
                const int fi = model.body_to_free[static_cast<std::size_t>(body)];
                if (fi < 0) continue;
                for (int k = 0; k < 6; ++k) {
                    const int col = 6*fi+k;
                    if (next_active) {
                        velocity_constraint_derivative[
                            static_cast<std::size_t>(row)
                        ] += dJ_next[static_cast<std::size_t>(row*n+col)]
                            * workspace.v_next[static_cast<std::size_t>(col)];
                    }
                    if (evaluation_active) {
                        reaction_derivative[static_cast<std::size_t>(col)] +=
                            dJ_evaluation[static_cast<std::size_t>(row*n+col)]
                            * workspace.lambda[static_cast<std::size_t>(row)];
                    }
                }
            }
        }

        const int local_body = column/6;
        const int body_index = model.free_body[
            static_cast<std::size_t>(local_body)
        ];
        const Vec3 dtranslation =
            next_direction.dr[static_cast<std::size_t>(body_index)];
        const Vec3 drotation =
            next_direction.dtheta[static_cast<std::size_t>(body_index)];
        const double pose_tangent[6] = {
            dtranslation.x, dtranslation.y, dtranslation.z,
            drotation.x, drotation.y, drotation.z
        };

        for (int row = 0; row < n; ++row) {
            J[at(row, column)] =
                (row == column ? 1.0 : 0.0)
                -mass_mu_derivative[static_cast<std::size_t>(row)];
        }
        for (int row = 0; row < m; ++row) {
            double position_derivative = 0.0;
            for (int k = 0; k < 6; ++k) {
                position_derivative +=
                    J_next[static_cast<std::size_t>(row*n+6*local_body+k)]
                    *pose_tangent[k];
            }
            J[at(3*n+row, column)] = position_derivative;
            J[at(3*n+m+row, column)] =
                velocity_constraint_derivative[static_cast<std::size_t>(row)];
        }

        for (int row = 0; row < n; ++row) {
            double inertial_derivative = 0.0;
            const int local_body = row/6;
            const int local_component = row%6;
            if (local_component >= 3) {
                const int bi = model.free_body[
                    static_cast<std::size_t>(local_body)
                ];
                const Body& body = model.bodies[bi];
                const DMat3 rotation = d_qmat(d_body_quaternion(
                    evaluation.q[bi],
                    evaluation_direction.dtheta[static_cast<std::size_t>(bi)]
                ));
                DMat3 inertia_body{};
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        inertia_body.a[i][j] = body.inertia_body.a[i][j];
                    }
                }
                const DMat3 world_inertia =
                    rotation*inertia_body*d_transpose(rotation);
                const Vec3 angular_acceleration{
                    (1.0-ctx.alpha_m)
                        *workspace.a_next[static_cast<std::size_t>(6*local_body+3)]
                        +ctx.alpha_m*ctx.previous.alpha[bi].x,
                    (1.0-ctx.alpha_m)
                        *workspace.a_next[static_cast<std::size_t>(6*local_body+4)]
                        +ctx.alpha_m*ctx.previous.alpha[bi].y,
                    (1.0-ctx.alpha_m)
                        *workspace.a_next[static_cast<std::size_t>(6*local_body+5)]
                        +ctx.alpha_m*ctx.previous.alpha[bi].z
                };
                const int angular_row = local_component-3;
                for (int j = 0; j < 3; ++j) {
                    const double acceleration_value =
                        j == 0 ? angular_acceleration.x
                        : j == 1 ? angular_acceleration.y
                        : angular_acceleration.z;
                    inertial_derivative +=
                        world_inertia.a[angular_row][j].derivative
                        *acceleration_value;
                }
            }
            J[at(2*n+row, column)] =
                inertial_derivative
                -force_derivative[static_cast<std::size_t>(row)]
                -reaction_derivative[static_cast<std::size_t>(row)];
        }
        for (int row = 0; row < nz; ++row) {
            J[at(z_offset+row, column)] =
                -internal_brush_derivative[static_cast<std::size_t>(row)];
        }
    }

#ifdef _OPENMP
#pragma omp for schedule(static)
#endif
    for (int column = n; column < 2*n; ++column) {
        reset_directional_state(model, evaluation_direction);
        const int local = column-n;
        const int free_body = local/6;
        const int component = local%6;
        if (component < 3) {
            evaluation_direction.dv[
                model.free_body[static_cast<std::size_t>(free_body)]
            ] =
                Vec3{
                    component == 0 ? 1.0-ctx.alpha_f : 0.0,
                    component == 1 ? 1.0-ctx.alpha_f : 0.0,
                    component == 2 ? 1.0-ctx.alpha_f : 0.0
                };
        } else {
            evaluation_direction.domega[
                model.free_body[static_cast<std::size_t>(free_body)]
            ] = Vec3{
                component == 3 ? 1.0-ctx.alpha_f : 0.0,
                component == 4 ? 1.0-ctx.alpha_f : 0.0,
                component == 5 ? 1.0-ctx.alpha_f : 0.0
            };
        }
        const bool main_smooth = external_force_directional(
            model, evaluation, *ctx.evaluation_sample, evaluation_direction,
            force_derivative, brush_derivative, false, nullptr,
            &directional_force_scratch
        );
        reset_directional_state(model, internal_direction);
        if (component < 3) {
            internal_direction.dv[
                model.free_body[static_cast<std::size_t>(free_body)]
            ] = Vec3{
                component == 0 ? ctx.alpha_f_z : 0.0,
                component == 1 ? ctx.alpha_f_z : 0.0,
                component == 2 ? ctx.alpha_f_z : 0.0
            };
        } else {
            internal_direction.domega[
                model.free_body[static_cast<std::size_t>(free_body)]
            ] = Vec3{
                component == 3 ? ctx.alpha_f_z : 0.0,
                component == 4 ? ctx.alpha_f_z : 0.0,
                component == 5 ? ctx.alpha_f_z : 0.0
            };
        }
        const bool internal_smooth = external_force_directional(
            model, workspace.internal_evaluation,
            *ctx.internal_evaluation_sample, internal_direction,
            unused_force, internal_brush_derivative, true, nullptr,
            &directional_force_scratch
        );
        if (!main_smooth || !internal_smooth) continue;
        analytic_columns[static_cast<std::size_t>(column)] = 1;
        for (int row = 0; row < n; ++row) {
            J[at(2*n+row, column)] =
                -force_derivative[static_cast<std::size_t>(row)];
        }
        for (int row = 0; row < m; ++row) {
            J[at(3*n+m+row, column)] =
                J_next[static_cast<std::size_t>(row*n+local)];
        }
        J[at(n+local, column)] = 1.0;
        for (int row = 0; row < nz; ++row) {
            J[at(z_offset+row, column)] =
                -internal_brush_derivative[static_cast<std::size_t>(row)];
        }
    }

#ifdef _OPENMP
#pragma omp for schedule(static)
#endif
    for (int column = z_offset; column < dim; ++column) {
        reset_directional_state(model, evaluation_direction);
        const int brush_index = column-z_offset;
        const int tire = brush_index/2;
        if (brush_index%2 == 0) {
            evaluation_direction.dsx[static_cast<std::size_t>(tire)] =
                1.0-ctx.alpha_f;
        } else {
            evaluation_direction.dsy[static_cast<std::size_t>(tire)] =
                1.0-ctx.alpha_f;
        }
        const bool main_smooth = external_force_directional(
            model, evaluation, *ctx.evaluation_sample, evaluation_direction,
            force_derivative, brush_derivative, false, nullptr,
            &directional_force_scratch
        );
        reset_directional_state(model, internal_direction);
        if (brush_index%2 == 0) {
            internal_direction.dsx[static_cast<std::size_t>(tire)] =
                ctx.alpha_f_z;
        } else {
            internal_direction.dsy[static_cast<std::size_t>(tire)] =
                ctx.alpha_f_z;
        }
        const bool internal_smooth = external_force_directional(
            model, workspace.internal_evaluation,
            *ctx.internal_evaluation_sample, internal_direction,
            unused_force, internal_brush_derivative, true, nullptr,
            &directional_force_scratch
        );
        if (!main_smooth || !internal_smooth) continue;
        analytic_columns[static_cast<std::size_t>(column)] = 1;
        for (int row = 0; row < n; ++row) {
            J[at(2*n+row, column)] =
                -force_derivative[static_cast<std::size_t>(row)];
        }
        for (int row = 0; row < nz; ++row) {
            J[at(z_offset+row, column)] =
                -internal_brush_derivative[static_cast<std::size_t>(row)];
        }
        J[at(z_offset+brush_index, column)] +=
            ctx.alpha_m_z/(ctx.gamma_z*ctx.h);
    }
    }
}

bool analytic_newton_columns_match_runtime_difference(
    const ResidualContext& ctx, const std::vector<double>& x,
    const std::vector<double>& base_residual, const std::vector<double>& J,
    int dim, const std::vector<unsigned char>& analytic_columns,
    const PoseJacobians& pose_jacobians
) {
    constexpr double eps = 1e-7;
    ResidualWorkspace workspace;
    workspace.ensure(*ctx.model, dim);
    double scale = 1.0;
    double worst = 0.0;
    for (int column = 0; column < dim; ++column) {
        if (analytic_columns[static_cast<std::size_t>(column)] == 0) continue;
        std::copy(x.begin(), x.end(), workspace.perturbed_x.begin());
        workspace.perturbed_x[static_cast<std::size_t>(column)] += eps;
        double position = 0.0;
        double velocity = 0.0;
        double dynamics = 0.0;
        int active = 0;
        residual(
            ctx, workspace.perturbed_x, workspace.output,
            position, velocity, dynamics, active, workspace,
            column < ctx.model->ndof ? nullptr : &pose_jacobians
        );
        if (!finite_vec(workspace.output)) return false;
        for (int row = 0; row < dim; ++row) {
            const double numerical = (
                workspace.output[static_cast<std::size_t>(row)]
                - base_residual[static_cast<std::size_t>(row)]
            ) / eps;
            const double analytic = J[
                static_cast<std::size_t>(row*dim + column)
            ];
            scale = std::max(scale, std::abs(numerical));
            scale = std::max(scale, std::abs(analytic));
            const double error = std::abs(numerical-analytic);
            worst = std::max(worst, error);
        }
    }
    return worst <= 1e-6*scale;
}

bool newton_step(const ResidualContext& ctx, std::vector<double>& x,
                 double& pos, double& vel, double& dyn, int& active, int& iterations,
                 const AxleInput& in,
                 NewtonLinearizationCache* linearization_cache = nullptr
) {
    const int dim=static_cast<int>(x.size());
    const int n=ctx.model->ndof;
    const bool validate_jacobian = runtime_jacobian_validation_enabled();
    // Modified Newton: the finite-difference Jacobian costs `dim` residual
    // evaluations, so it is reused until a step fails to reduce the residual.
    // Convergence is still decided by the exact residual, so reuse changes the
    // iteration path but never the converged solution.
    NewtonSystemFactorization local_factorization;
    NewtonSystemFactorization* factorization = &local_factorization;
    ResidualWorkspace primary_workspace;
    std::vector<double> r;
    std::vector<double> rhs(static_cast<std::size_t>(dim), 0.0);
    std::vector<double> dx;
    std::vector<double> trial;
    bool residual_ready = false;
    bool factored=false;
    const auto increment_residual = [n](const std::vector<double>& values) {
        double maximum = 0.0;
        for (int index = 0; index < static_cast<int>(values.size()); ++index) {
            if (index >= 2*n && index < 3*n) continue;
            maximum = std::max(
                maximum, std::abs(values[static_cast<std::size_t>(index)])
            );
        }
        return maximum;
    };
    const auto converged = [&](double position, double velocity,
                               double dynamics,
                               const std::vector<double>& values) {
        return position <= in.position_tolerance &&
            velocity <= in.velocity_tolerance &&
            dynamics <= in.dynamics_tolerance &&
            increment_residual(values) <= in.increment_tolerance;
    };
    if (linearization_cache != nullptr) {
        factorization = &linearization_cache->factorization;
        if (linearization_cache->matches(ctx, dim)) {
            factored = true;
        } else {
            linearization_cache->invalidate();
        }
    }
    for (iterations=0; iterations<in.max_newton_iterations; ++iterations) {
        if (ctx.performance != nullptr) {
            ctx.performance->newton_iterations.fetch_add(
                1, std::memory_order_relaxed
            );
        }
        if (residual_ready) {
            residual_ready = false;
        } else {
            residual(ctx, x, r, pos, vel, dyn, active, primary_workspace);
        }
        if (!finite_vec(r)) return false;
        double rmax=max_abs(r);
        if (converged(pos, vel, dyn, r)) {
            return true;
        }
        for (int i=0;i<dim;++i) rhs[i]=-r[i];
        bool accepted=false;
        // Try the retained factorization first; on failure rebuild it once and
        // retry the same iteration with a fresh Jacobian.
        for (int attempt=0; attempt<2 && !accepted; ++attempt) {
            bool fresh=false;
            if (!factored) {
                std::vector<double> J(
                    static_cast<std::size_t>(dim)*static_cast<std::size_t>(dim),
                    0.0
                );
                const double eps=1e-7;
                // Columns at or past the velocity block leave every body pose
                // untouched, so both constraint Jacobians are identical to the
                // ones at the base point and are computed once here.
                PoseJacobians pose_jacobians;
                {
                    // The base residual immediately above evaluated the same
                    // x without cached Jacobians; retain those exact buffers.
                    pose_jacobians.at_next = primary_workspace.J_storage;
                    pose_jacobians.at_evaluation = primary_workspace.J_eval_storage;
                    pose_jacobians.position_residual = primary_workspace.phi_storage;
                    pose_jacobians.valid = true;
                }
                std::vector<unsigned char> analytic_columns;
                {
                    ScopedPerformanceTimer analytic_timer(
                        ctx.performance == nullptr
                            ? nullptr
                            : &ctx.performance->analytic_jacobian_nanoseconds
                    );
                    fill_analytic_jacobian_columns(
                        ctx, dim, J, primary_workspace, pose_jacobians,
                        analytic_columns
                    );
                }
                // Each differenced column is an independent residual evaluation
                // writing a disjoint column of J, so the loop carries no
                // reduction and the assembled matrix is bitwise identical for
                // any thread count.  `residual` reads only const inputs and
                // per-call locals.
                std::vector<int> columns;
                columns.reserve(static_cast<std::size_t>(dim));
                for (int j=0;j<dim;++j) {
                    if (analytic_columns[static_cast<std::size_t>(j)] != 0) continue;
                    columns.push_back(j);
                }
                const int column_count = static_cast<int>(columns.size());
                if (ctx.performance != nullptr) {
                    std::uint64_t analytic_count = 0;
                    for (unsigned char value : analytic_columns) {
                        analytic_count += value != 0 ? 1U : 0U;
                    }
                    ctx.performance->analytic_jacobian_columns.fetch_add(
                        analytic_count, std::memory_order_relaxed
                    );
                    ctx.performance->finite_difference_jacobian_columns.fetch_add(
                        static_cast<std::uint64_t>(column_count),
                        std::memory_order_relaxed
                    );
                    const int z_offset = 3*n + 2*ctx.model->rows;
                    std::uint64_t nonsmooth_count = 0;
                    for (int j = 0; j < n; ++j) {
                        if (analytic_columns[static_cast<std::size_t>(j)] == 0) {
                            ++nonsmooth_count;
                        }
                    }
                    for (int j = n; j < 2*n; ++j) {
                        if (analytic_columns[static_cast<std::size_t>(j)] == 0) {
                            ++nonsmooth_count;
                        }
                    }
                    for (int j = z_offset; j < dim; ++j) {
                        if (analytic_columns[static_cast<std::size_t>(j)] == 0) {
                            ++nonsmooth_count;
                        }
                    }
                    ctx.performance->nonsmooth_fallback_columns.fetch_add(
                        nonsmooth_count, std::memory_order_relaxed
                    );
                }
                int threads = 1;
#ifdef _OPENMP
                // One Jacobian assembly is sub-millisecond, so thread start-up
                // dominates beyond a modest team; measured scaling saturates at
                // eight threads and regresses past sixteen.  Capping keeps the
                // run insensitive to the machine's core count, and the loop has
                // no reduction so the result is identical for any team size.
                threads =
                    std::max(1, std::min(8, column_count/4));
#endif
                if (column_count > 0) {
                    ScopedPerformanceTimer finite_difference_timer(
                        ctx.performance == nullptr
                            ? nullptr
                            : &ctx.performance->finite_difference_jacobian_nanoseconds
                    );
                    std::vector<ResidualWorkspace> column_workspaces(
                        static_cast<std::size_t>(threads)
                    );
                    for (ResidualWorkspace& worker : column_workspaces) {
                        worker.ensure(*ctx.model, dim);
                    }
#ifdef _OPENMP
#pragma omp parallel for schedule(static) num_threads(threads)
#endif
                    for (int index=0; index<column_count; ++index) {
                        const int j = columns[static_cast<std::size_t>(index)];
                        int workspace_index = 0;
#ifdef _OPENMP
                        workspace_index = omp_get_thread_num();
#endif
                        ResidualWorkspace& worker =
                            column_workspaces[static_cast<std::size_t>(workspace_index)];
                        std::copy(x.begin(), x.end(), worker.perturbed_x.begin());
                        worker.perturbed_x[static_cast<std::size_t>(j)] += eps;
                        double p2,v2,d2; int a2;
                        residual(
                            ctx, worker.perturbed_x, worker.output,
                            p2, v2, d2, a2, worker,
                            j >= n ? &pose_jacobians : nullptr
                        );
                        for (int i=0;i<dim;++i) {
                            J[static_cast<std::size_t>(i*dim+j)] =
                                (worker.output[static_cast<std::size_t>(i)]
                                 - r[static_cast<std::size_t>(i)]) / eps;
                        }
                    }
                }
                if (validate_jacobian &&
                    !analytic_newton_columns_match_runtime_difference(
                        ctx, x, r, J, dim, analytic_columns,
                        pose_jacobians
                    )) {
                    return false;
                }
                if (ctx.performance != nullptr) {
                    ctx.performance->linear_factorizations.fetch_add(
                        1, std::memory_order_relaxed
                    );
                }
                ScopedPerformanceTimer factorization_timer(
                    ctx.performance == nullptr
                        ? nullptr
                        : &ctx.performance->linear_factorization_nanoseconds
                );
                const bool factorized = factorization->factor(
                    std::move(J), dim, n, ctx.model->rows,
                    2*static_cast<int>(ctx.model->tires.size())
                );
                if (!factorized) {
                    if (linearization_cache != nullptr) {
                        linearization_cache->invalidate();
                    }
                    return false;
                }
                if (linearization_cache != nullptr) {
                    linearization_cache->update_key(ctx, dim);
                }
                factored=true;
                fresh=true;
            }
            if (ctx.performance != nullptr) {
                ctx.performance->linear_solves.fetch_add(
                    1, std::memory_order_relaxed
                );
            }
            bool solved = false;
            {
                ScopedPerformanceTimer solve_timer(
                    ctx.performance == nullptr
                        ? nullptr
                        : &ctx.performance->linear_solve_nanoseconds
                );
                solved = factorization->solve(rhs, dx);
            }
            if (solved && finite_vec(dx)) {
                double scale=1.0;
                for (int ls=0;ls<in.max_line_search_iterations;++ls) {
                    if (ctx.performance != nullptr) {
                        ctx.performance->line_search_trials.fetch_add(
                            1, std::memory_order_relaxed
                        );
                    }
                    trial=x;
                    for (int i=0;i<dim;++i) trial[i]+=scale*dx[i];
                    double p2,v2,d2; int a2;
                    residual(
                        ctx, trial, primary_workspace.output,
                        p2, v2, d2, a2, primary_workspace
                    );
                    if (finite_vec(primary_workspace.output) &&
                        max_abs(primary_workspace.output) < rmax) {
                        x=std::move(trial);
                        pos = p2;
                        vel = v2;
                        dyn = d2;
                        active = a2;
                        r.swap(primary_workspace.output);
                        residual_ready = true;
                        accepted = true;
                        break;
                    }
                    scale*=0.5;
                }
            }
            // A stale Jacobian is the likely cause, so rebuild and retry once.
            // A freshly built one failing is a genuine failure, not staleness.
            if (!accepted) {
                factored=false;
                if (linearization_cache != nullptr && !fresh) {
                    linearization_cache->invalidate();
                }
                if (fresh) {
                    if (linearization_cache != nullptr) {
                        linearization_cache->invalidate();
                    }
                    return false;
                }
            }
        }
        if (!accepted) return false;
        // 第一次达到容差可能发生在本轮接受更新之后；不能等到下一轮
        // 才检查，否则最后一次允许的 Newton 更新会被误报为失败。
        if (converged(pos, vel, dyn, r)) return true;
    }
    return false;
}

struct StepResult {
    State state{};
    std::vector<double> constraint_multiplier;
    double position_residual{0.0};
    double velocity_residual{0.0};
    double dynamics_residual{0.0};
    int active_contacts{0};
    int iterations{0};
};

bool finite_state(const Model& model, const State& state) {
    for (std::size_t i = 0; i < model.bodies.size(); ++i) {
        const Vec3 vectors[] = {
            state.r[i], state.v[i], state.a[i], state.omega[i], state.alpha[i]
        };
        for (const Vec3& vector : vectors) {
            if (!std::isfinite(vector.x) || !std::isfinite(vector.y) ||
                !std::isfinite(vector.z)) {
                return false;
            }
        }
        if (!unit_quaternion(state.q[i], 1e-9)) return false;
    }
    for (double value : state.tire_sx) {
        if (!std::isfinite(value)) return false;
    }
    for (double value : state.tire_sy) {
        if (!std::isfinite(value)) return false;
    }
    for (double value : state.tire_sx_dot) {
        if (!std::isfinite(value)) return false;
    }
    for (double value : state.tire_sy_dot) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

std::vector<double> initial_step_unknown(
    const Model& model, const State& state, double h,
    const std::vector<Vec3>* previous_acceleration = nullptr,
    const std::vector<Vec3>* previous_alpha = nullptr,
    const std::vector<double>* previous_constraint_multiplier = nullptr
) {
    const int n = model.ndof;
    const int m = model.rows;
    const int dimension =
        3*n + 2*m + 2*static_cast<int>(model.tires.size());
    std::vector<double> x(static_cast<std::size_t>(dimension), 0.0);
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int bi = model.free_body[fi];
        Vec3 acceleration = state.a[bi];
        Vec3 angular_acceleration = state.alpha[bi];
        if (
            previous_acceleration != nullptr &&
            previous_alpha != nullptr &&
            previous_acceleration->size() == model.bodies.size() &&
            previous_alpha->size() == model.bodies.size()
        ) {
            acceleration = state.a[bi]
                + (state.a[bi]-(*previous_acceleration)[bi]);
            angular_acceleration = state.alpha[bi]
                + (state.alpha[bi]-(*previous_alpha)[bi]);
        }
        const Vec3 displacement =
            state.v[bi]*h + acceleration*(0.5*h*h);
        const Vec3 rotation =
            state.omega[bi]*h + angular_acceleration*(0.5*h*h);
        x[6*fi] = displacement.x;
        x[6*fi+1] = displacement.y;
        x[6*fi+2] = displacement.z;
        x[6*fi+3] = rotation.x;
        x[6*fi+4] = rotation.y;
        x[6*fi+5] = rotation.z;
        x[n+6*fi] = state.v[bi].x + h*acceleration.x;
        x[n+6*fi+1] = state.v[bi].y + h*acceleration.y;
        x[n+6*fi+2] = state.v[bi].z + h*acceleration.z;
        x[n+6*fi+3] = state.omega[bi].x + h*angular_acceleration.x;
        x[n+6*fi+4] = state.omega[bi].y + h*angular_acceleration.y;
        x[n+6*fi+5] = state.omega[bi].z + h*angular_acceleration.z;
        x[2*n+6*fi] = acceleration.x;
        x[2*n+6*fi+1] = acceleration.y;
        x[2*n+6*fi+2] = acceleration.z;
        x[2*n+6*fi+3] = angular_acceleration.x;
        x[2*n+6*fi+4] = angular_acceleration.y;
        x[2*n+6*fi+5] = angular_acceleration.z;
    }
    const int z_offset = 3*n + 2*m;
    if (
        previous_constraint_multiplier != nullptr &&
        previous_constraint_multiplier->size() == static_cast<std::size_t>(m)
    ) {
        std::copy(
            previous_constraint_multiplier->begin(),
            previous_constraint_multiplier->end(),
            x.begin()+3*n
        );
    }
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        x[static_cast<std::size_t>(z_offset)+2*i] =
            state.tire_sx[i] + h*state.tire_sx_dot[i];
        x[static_cast<std::size_t>(z_offset)+2*i+1] =
            state.tire_sy[i] + h*state.tire_sy_dot[i];
    }
    return x;
}

bool initialize_internal_derivatives(
    const Model& model, const AxleInput& input, State& state
) {
    SampleInput sample;
    interpolate_input(model, input, input.sample_times[0], sample);
    std::vector<double> tire_forces;
    std::vector<double> tire_derivatives;
    std::vector<double> tire_output;
    double potential = 0.0;
    double power = 0.0;
    double dissipation = 0.0;
    std::vector<double> force;
    const auto evaluate_derivatives = [&]() {
        potential = 0.0;
        power = 0.0;
        dissipation = 0.0;
        external_force_vector(
            model, state, sample, input.gravity_x, input.gravity_y,
            input.gravity_z, tire_forces, tire_derivatives, tire_output,
            potential, power, dissipation, force,
            nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, true
        );
        return finite_vec(tire_derivatives);
    };
    if (!evaluate_derivatives()) return false;

    // A supplied moving state represents an already-running source model.
    // Initialize PAC relaxation states at the current kinematic fixed point
    // instead of fabricating a zero-slip startup transient. The PAC ODE is
    // affine in each private state, so two evaluations recover its exact
    // relaxation rate and fixed point without duplicating slip kinematics.
    const std::vector<double> zero_state_derivatives = tire_derivatives;
    bool has_pac2002 = false;
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& tire = model.tires[i];
        if (
            tire.model_kind != VEHICLE_TIRE_PAC2002_PURE_SLIP &&
            tire.model_kind != VEHICLE_TIRE_PAC2002_ADAMS_SOURCE &&
            tire.model_kind != VEHICLE_TIRE_FIALA
        ) {
            continue;
        }
        has_pac2002 = true;
        state.tire_sx[i] = 1.0;
        state.tire_sy[i] = 1.0;
    }
    if (has_pac2002) {
        if (!evaluate_derivatives()) return false;
        for (std::size_t i = 0; i < model.tires.size(); ++i) {
            const Tire& tire = model.tires[i];
            if (
                tire.model_kind != VEHICLE_TIRE_PAC2002_PURE_SLIP &&
                tire.model_kind != VEHICLE_TIRE_PAC2002_ADAMS_SOURCE &&
                tire.model_kind != VEHICLE_TIRE_FIALA
            ) {
                continue;
            }
            const double longitudinal_rate =
                zero_state_derivatives[2*i] - tire_derivatives[2*i];
            const double lateral_rate =
                zero_state_derivatives[2*i+1] - tire_derivatives[2*i+1];
            state.tire_sx[i] = std::abs(longitudinal_rate) > kEps
                ? zero_state_derivatives[2*i] / longitudinal_rate
                : 0.0;
            state.tire_sy[i] = std::abs(lateral_rate) > kEps
                ? zero_state_derivatives[2*i+1] / lateral_rate
                : 0.0;
        }
        if (!finite_vec(state.tire_sx) || !finite_vec(state.tire_sy)) {
            return false;
        }
        if (!evaluate_derivatives()) return false;
    }
    state.tire_sx_dot.resize(model.tires.size(), 0.0);
    state.tire_sy_dot.resize(model.tires.size(), 0.0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        state.tire_sx_dot[i] = tire_derivatives[2*i];
        state.tire_sy_dot[i] = tire_derivatives[2*i+1];
    }
    return true;
}

bool apply_brush_return_mapping(
    const Model& model, const AxleInput& input, const State& previous,
    double time, double h, double gamma_z, State& state
) {
    SampleInput sample;
    interpolate_input(model, input, time+h, sample);
    std::vector<double> tire_forces;
    std::vector<double> tire_derivatives;
    std::vector<double> tire_output;
    double potential = 0.0;
    double power = 0.0;
    double dissipation = 0.0;
    std::vector<double> force;
    external_force_vector(
        model, state, sample, input.gravity_x, input.gravity_y, input.gravity_z,
        tire_forces, tire_derivatives, tire_output, potential, power, dissipation,
        force, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, true
    );
    state.tire_sx_dot.resize(model.tires.size(), 0.0);
    state.tire_sy_dot.resize(model.tires.size(), 0.0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        if (!std::isfinite(tire_forces[i])) return false;
        if (
            tire_forces[i] > 0.0 &&
            model.tires[i].model_kind != VEHICLE_TIRE_PAC2002_PURE_SLIP
            && model.tires[i].model_kind != VEHICLE_TIRE_PAC2002_ADAMS_SOURCE
            && model.tires[i].model_kind != VEHICLE_TIRE_FIALA
        ) {
            double projected_sx = state.tire_sx[i];
            double projected_sy = state.tire_sy[i];
            double trial_utilization = 0.0;
            project_brush_state(
                model.tires[i], tire_forces[i],
                state.tire_sx[i], state.tire_sy[i],
                projected_sx, projected_sy, trial_utilization
            );
            state.tire_sx[i] = projected_sx;
            state.tire_sy[i] = projected_sy;
        }
        state.tire_sx_dot[i] =
            (state.tire_sx[i]-previous.tire_sx[i])/(gamma_z*h)
            - (1.0-gamma_z)/gamma_z*previous.tire_sx_dot[i];
        state.tire_sy_dot[i] =
            (state.tire_sy[i]-previous.tire_sy[i])/(gamma_z*h)
            - (1.0-gamma_z)/gamma_z*previous.tire_sy_dot[i];
    }
    return finite_vec(state.tire_sx_dot) && finite_vec(state.tire_sy_dot);
}

bool solve_one_step(
    const Model& model, const AxleInput& input, const State& start,
    double time, double h, double alpha_m, double alpha_f,
    double beta, double gamma, double alpha_m_z, double alpha_f_z,
    double gamma_z, StepResult& result,
    PerformanceCounters* performance = nullptr,
    const std::vector<Vec3>* previous_acceleration = nullptr,
    const std::vector<Vec3>* previous_alpha = nullptr,
    const std::vector<double>* previous_constraint_multiplier = nullptr,
    NewtonLinearizationCache* linearization_cache = nullptr
) {
    if (linearization_cache != nullptr) {
        // 跨步状态变化会使完整 Jacobian 逐渐过期；限制复用步数可以保留
        // 主要性能收益，同时避免强转向工况在旧线性化附近停滞。
        bool has_pac_tire = false;
        for (const Tire& tire : model.tires) {
            has_pac_tire = has_pac_tire ||
                tire.model_kind == VEHICLE_TIRE_PAC2002_PURE_SLIP ||
                tire.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE ||
                tire.model_kind == VEHICLE_TIRE_FIALA;
        }
        // PAC 纯滑移力随法向载荷和滑移率变化，但轮胎状态在相邻隐式
        // 步内通常仍处于同一局部支路。延长修正 Newton 的分解复用窗口
        // 可以显著减少整车 KKT 分解；线搜索失败时仍会立即使缓存失效，
        // 因而不会用旧切线跨过当前非线性支路。
        const int maximum_reuse_steps = has_pac_tire ? 24 : 32;
        if (linearization_cache->valid &&
            linearization_cache->reuse_steps >= maximum_reuse_steps) {
            linearization_cache->invalidate();
        }
        if (linearization_cache->valid) ++linearization_cache->reuse_steps;
    }
    SampleInput previous_sample;
    SampleInput evaluation_sample;
    SampleInput next_sample;
    SampleInput internal_evaluation_sample;
    interpolate_input(model, input, time, previous_sample);
    interpolate_input(
        model, input, time + (1.0-alpha_f)*h, evaluation_sample
    );
    interpolate_input(model, input, time+h, next_sample);
    interpolate_input(
        model, input, time + alpha_f_z*h, internal_evaluation_sample
    );
    ResidualContext context{
        &model, &input, &previous_sample, &evaluation_sample,
        &next_sample, &internal_evaluation_sample, start, h,
        alpha_m, alpha_f, beta, gamma, alpha_m_z, alpha_f_z, gamma_z,
        performance
    };
    std::vector<double> x = initial_step_unknown(
        model, start, h, previous_acceleration, previous_alpha,
        previous_constraint_multiplier
    );
    if (!newton_step(
            context, x, result.position_residual, result.velocity_residual,
            result.dynamics_residual, result.active_contacts,
            result.iterations, input, linearization_cache
        )) {
        return false;
    }
    std::vector<double> acceleration;
    std::vector<double> velocity;
    std::vector<double> position_multiplier;
    result.state =
        state_from_unknown(
            context, x, acceleration, velocity, position_multiplier,
            result.constraint_multiplier
        );
    apply_acceleration(result.state, model, acceleration);
    if (!apply_brush_return_mapping(
            model, input, start, time, h, gamma_z, result.state
        )) {
        return false;
    }
    return finite_state(model, result.state);
}

double scaled_scalar_error(
    double a, double b, double absolute_tolerance, double relative_tolerance
) {
    const double scale =
        absolute_tolerance + relative_tolerance*std::max(std::abs(a), std::abs(b));
    return std::abs(a-b) / scale;
}

double normalized_state_error(
    const Model& model, const AxleInput& input, const State& start,
    const State& full_step, const State& two_half_steps
) {
    double error = 0.0;
    for (int bi : model.free_body) {
        const double full_position[] = {
            full_step.r[bi].x, full_step.r[bi].y, full_step.r[bi].z
        };
        const double half_position[] = {
            two_half_steps.r[bi].x,
            two_half_steps.r[bi].y,
            two_half_steps.r[bi].z
        };
        const double full_velocity[] = {
            full_step.v[bi].x, full_step.v[bi].y, full_step.v[bi].z
        };
        const double half_velocity[] = {
            two_half_steps.v[bi].x,
            two_half_steps.v[bi].y,
            two_half_steps.v[bi].z
        };
        const double full_omega[] = {
            full_step.omega[bi].x,
            full_step.omega[bi].y,
            full_step.omega[bi].z
        };
        const double half_omega[] = {
            two_half_steps.omega[bi].x,
            two_half_steps.omega[bi].y,
            two_half_steps.omega[bi].z
        };
        for (int axis = 0; axis < 3; ++axis) {
            error = std::max(error, scaled_scalar_error(
                full_position[axis], half_position[axis],
                input.local_position_tolerance, input.local_relative_tolerance
            ));
            error = std::max(error, scaled_scalar_error(
                full_velocity[axis], half_velocity[axis],
                input.local_velocity_tolerance, input.local_relative_tolerance
            ));
            error = std::max(error, scaled_scalar_error(
                full_omega[axis], half_omega[axis],
                input.local_angular_velocity_tolerance,
                input.local_relative_tolerance
            ));
        }
        const Vec3 orientation_difference = qlog(qmul(
            qconj(full_step.q[bi]), two_half_steps.q[bi]
        ));
        const double full_rotation = norm(qlog(qmul(
            qconj(start.q[bi]), full_step.q[bi]
        )));
        const double half_rotation = norm(qlog(qmul(
            qconj(start.q[bi]), two_half_steps.q[bi]
        )));
        const double orientation_scale =
            input.local_angle_tolerance
            + input.local_relative_tolerance*std::max(full_rotation, half_rotation);
        error = std::max(error, norm(orientation_difference)/orientation_scale);
    }
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        error = std::max(error, scaled_scalar_error(
            full_step.tire_sx[i], two_half_steps.tire_sx[i],
            input.local_brush_tolerance, input.local_relative_tolerance
        ));
        error = std::max(error, scaled_scalar_error(
            full_step.tire_sy[i], two_half_steps.tire_sy[i],
            input.local_brush_tolerance, input.local_relative_tolerance
        ));
    }
    // Generalized-alpha is second order; the two-half-step local error is
    // one third of the full-versus-half difference.
    return error / 3.0;
}

std::vector<double> contact_penetrations(
    const Model& model, const AxleInput& input, const State& state, double time
) {
    SampleInput sample;
    interpolate_input(model, input, time, sample);
    std::vector<double> penetrations(model.tires.size(), 0.0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& tire = model.tires[i];
        const Vec3 center = state_point(
            state, tire_center_body(tire), tire_center_local(tire)
        );
        const double road =
            (i < sample.road_z.size() ? sample.road_z[i] : 0.0)
            + road_profile_height(model, state, i);
        penetrations[i] = tire.radius + road - center.z;
    }
    return penetrations;
}

std::vector<int> contact_modes(
    const Model& model, const AxleInput& input,
    const State& state, double time
) {
    SampleInput sample;
    interpolate_input(model, input, time, sample);
    std::vector<int> modes(model.tires.size(), 0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& tire = model.tires[i];
        const int center_body = tire_center_body(tire);
        const Vec3 center_local = tire_center_local(tire);
        const Vec3 center = state_point(state, center_body, center_local);
        const Vec3 center_velocity =
            state_point_velocity(state, center_body, center_local);
        const double road =
            (i < sample.road_z.size() ? sample.road_z[i] : 0.0)
            + road_profile_height(model, state, i);
        const double road_velocity =
            (i < sample.road_v.size() ? sample.road_v[i] : 0.0)
            + road_profile_slope(model, state, i) * center_velocity.x;
        const double delta = tire.radius + road - center.z;
        const double delta_dot = road_velocity - center_velocity.z;
        const double raw_normal_force = tire.k*delta + tire.c*delta_dot;
        // A geometrically touching tire with zero normal force is not a
        // loaded unilateral contact.  Treating that neutral boundary as
        // detached avoids fabricating a contact event for an unloaded wheel.
        modes[i] = delta > 0.0 && raw_normal_force > 0.0 ? 1 : 0;
    }
    return modes;
}

bool contact_transition(
    const Model& model, const AxleInput& input,
    const State& before, double before_time,
    const State& after, double after_time
) {
    return contact_modes(model, input, before, before_time) !=
        contact_modes(model, input, after, after_time);
}

bool solve_event_localization_step(
    const Model& model, const AxleInput& input,
    const State& start, double time, double h,
    StepResult& result, int depth = 0
) {
    const double alpha_m =
        (2.0*input.rho_inf-1.0)/(input.rho_inf+1.0);
    const double alpha_f = input.rho_inf/(input.rho_inf+1.0);
    const double gamma = 0.5-alpha_m+alpha_f;
    const double beta = 0.25*(1.0-alpha_m+alpha_f)
        *(1.0-alpha_m+alpha_f);
    const double alpha_m_z =
        (3.0-input.rho_inf)/(2.0*(1.0+input.rho_inf));
    const double alpha_f_z = 1.0/(1.0+input.rho_inf);
    const double gamma_z = 0.5+alpha_m_z-alpha_f_z;
    if (solve_one_step(
            model, input, start, time, h,
            alpha_m, alpha_f, beta, gamma,
            alpha_m_z, alpha_f_z, gamma_z, result
        )) {
        return true;
    }

    // A contact-event probe is an auxiliary solve.  A failed large probe does
    // not mean the event cannot be localized: the same stiff interval may be
    // solvable when advanced as two smaller implicit steps.  Bound the
    // subdivision so a genuinely unsolvable probe remains a hard failure.
    const double minimum_step = std::max(
        input.contact_event_tolerance*0.25,
        std::numeric_limits<double>::epsilon()
            * std::max(1.0, std::abs(time))
    );
    if (depth >= 8 || h <= minimum_step) return false;
    const double half = 0.5*h;
    StepResult first;
    if (!solve_event_localization_step(
            model, input, start, time, half, first, depth+1
        )) {
        return false;
    }
    StepResult second;
    if (!solve_event_localization_step(
            model, input, first.state, time+half, half, second, depth+1
        )) {
        return false;
    }
    result = std::move(second);
    return true;
}

void append_contact_events(
    const Model& model, const AxleInput& input,
    const State& before, double before_time,
    const State& after, double after_time,
    std::vector<ContactEventRecord>& events
) {
    const auto before_modes =
        contact_modes(model, input, before, before_time);
    const auto after_modes =
        contact_modes(model, input, after, after_time);
    for (std::size_t tire = 0; tire < before_modes.size(); ++tire) {
        if (before_modes[tire] == after_modes[tire]) continue;
        events.push_back(
            ContactEventRecord{
                after_time,
                static_cast<int>(tire),
                after_modes[tire] != 0 ? 1 : -1,
            }
        );
    }
}

bool write_contact_events(
    const std::vector<ContactEventRecord>& events,
    AxleOutput& output
) {
    *output.contact_event_count = events.size();
    const std::size_t required =
        events.size()*kContactEventOutputWidth;
    if (required > output.contact_event_output_capacity) return false;
    for (std::size_t i = 0; i < events.size(); ++i) {
        const std::size_t offset = i*kContactEventOutputWidth;
        output.contact_event_output[offset] = events[i].time;
        output.contact_event_output[offset+1] =
            static_cast<double>(events[i].tire_index);
        output.contact_event_output[offset+2] =
            static_cast<double>(events[i].transition);
    }
    return true;
}

bool localize_contact_event(
    const Model& model, const AxleInput& input,
    const State& start, double time, double h,
    const StepResult& end_step, StepResult& event_step, double& event_h
) {
    const auto start_modes = contact_modes(model, input, start, time);
    const auto end_modes =
        contact_modes(model, input, end_step.state, time+h);
    if (start_modes == end_modes) return false;
    double left = 0.0;
    double right = h;
    StepResult right_step = end_step;
    const int max_iterations = std::max(
        8,
        static_cast<int>(
            std::ceil(std::log2(
                std::max(1.0, h/input.contact_event_tolerance)
            ))
        )+4
    );
    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        if (right-left <= input.contact_event_tolerance) break;
        const double mid = 0.5*(left+right);
        StepResult mid_step;
        if (!solve_event_localization_step(
                model, input, start, time, mid, mid_step
            )) {
            return false;
        }
        if (contact_modes(model, input, mid_step.state, time+mid) ==
            start_modes) {
            left = mid;
        } else {
            right = mid;
            right_step = std::move(mid_step);
        }
    }
    event_h = right;
    event_step = std::move(right_step);
    return event_h > std::numeric_limits<double>::epsilon()
        * std::max(1.0, std::abs(time));
}

bool exceeds_tire_compression_limit(
    const Model& model, const AxleInput& input, const State& state, double time
) {
    const auto penetrations = contact_penetrations(model, input, state, time);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        if (penetrations[i] > model.tires[i].maximum_compression) return true;
    }
    return false;
}

const StaticRotationGauge* static_rotation_gauge_for_pivot(
    const Model& model, int coordinate
);

double static_rotation_gauge_value(
    const Model& model, const StaticRotationGauge& gauge,
    const std::vector<double>& pose_increment
);

std::vector<double> static_residual(
    const Model& model, const State& base, const SampleInput& sample,
    double gravity_x, double gravity_y, double gravity_z,
    const std::vector<int>& active_tires, const std::vector<double>& x,
    double& force_residual, double& position_residual,
    int* worst_force_coordinate = nullptr, double* worst_force_value = nullptr,
    double load_scale = 1.0
) {
    const int n=model.ndof, m=model.rows;
    std::vector<double> dy(x.begin(),x.begin()+n);
    const State candidate=pose_candidate(base,model,dy);
    const auto J=constraint_jacobian(model,candidate);
    std::vector<double> lambda(x.begin()+n,x.begin()+n+m);
    std::vector<int> active_mask(model.tires.size(), 0);
    std::vector<double> static_compression(model.tires.size(), 0.0);
    for (std::size_t i = 0; i < active_tires.size(); ++i) {
        const int tire_index = active_tires[i];
        active_mask[static_cast<std::size_t>(tire_index)] = 1;
        static_compression[static_cast<std::size_t>(tire_index)] =
            x[static_cast<std::size_t>(n+m)+i];
    }
    const StaticContactOverride static_contact{
        &active_mask, &static_compression, nullptr, load_scale, load_scale
    };
    std::vector<double> tire_forces;
    std::vector<double> tire_brush_derivatives;
    std::vector<double> tire_output;
    double potential=0.0;
    double external_power=0.0;
    double dissipation=0.0;
    std::vector<double> force;
    external_force_vector(
        model, candidate, sample, gravity_x, gravity_y, gravity_z, tire_forces,
        tire_brush_derivatives, tire_output, potential, external_power,
        dissipation, force, nullptr, nullptr, nullptr, &static_contact
    );
    std::vector<double> out(
        static_cast<std::size_t>(n+m)+active_tires.size(), 0.0
    );
    std::vector<double> equilibrium(static_cast<std::size_t>(n), 0.0);
    for(int col=0;col<n;++col){
        double reaction=0.0;
        for(int row=0;row<m;++row) reaction+=J[row*n+col]*lambda[row];
        equilibrium[static_cast<std::size_t>(col)] = -force[col]-reaction;
        out[col] = equilibrium[static_cast<std::size_t>(col)];
    }
    for (const auto& gauge : model.static_rotation_gauges) {
        if (gauge.pivot < 0 || gauge.pivot >= n) continue;
        // 该方程只固定静态任意转角；轴向真实力矩在下方单独计入残差。
        out[static_cast<std::size_t>(gauge.pivot)] =
            static_rotation_gauge_value(model, gauge, dy);
    }
    const auto phi=constraint_residual(model,candidate,&sample);
    for(int row=0;row<m;++row) out[n+row]=phi[row];
    for (std::size_t i = 0; i < active_tires.size(); ++i) {
        const int tire_index = active_tires[i];
        const Tire& tire = model.tires[static_cast<std::size_t>(tire_index)];
        const Vec3 center =
            state_point(
                candidate, tire_center_body(tire), tire_center_local(tire)
            );
        const double road =
            (static_cast<std::size_t>(tire_index) < sample.road_z.size()
                ? sample.road_z[static_cast<std::size_t>(tire_index)]
                : 0.0)
            + road_profile_height(
                model, candidate, static_cast<std::size_t>(tire_index)
            );
        const double geometric_compression = tire.radius + road - center.z;
        out[static_cast<std::size_t>(n+m)+i] =
            static_compression[static_cast<std::size_t>(tire_index)]
            - geometric_compression;
    }
    int gauge_free_index = -1;
    if (model.static_gauge_body >= 0 && model.static_gauge_dof_mask != 0) {
        gauge_free_index = model.body_to_free[
            static_cast<std::size_t>(model.static_gauge_body)
        ];
        if (gauge_free_index >= 0) {
            for (int local = 0; local < 6; ++local) {
                if ((model.static_gauge_dof_mask &
                     (static_cast<std::uint32_t>(1) << local)) == 0) {
                    continue;
                }
                const int coordinate = 6*gauge_free_index + local;
                // A global gauge replaces a redundant force-balance row with
                // the coordinate condition dy=0. It is not a reaction.
                out[static_cast<std::size_t>(coordinate)] =
                    dy[static_cast<std::size_t>(coordinate)];
            }
        }
    }
    double translational_residual = 0.0;
    double rotational_residual = 0.0;
    double worst_normalized_residual = 0.0;
    int worst_coordinate = -1;
    double worst_value = 0.0;
    // Translational and rotational equations have different physical units.
    // Normalize each block by the largest term in the same block so a large
    // reaction force cannot hide an unresolved moment balance.
    double force_scale = 1.0;
    double moment_scale = 1.0;
    for (int i = 0; i < n; ++i) {
        const bool gauge = gauge_free_index >= 0 &&
            i >= 6*gauge_free_index && i < 6*gauge_free_index+6 &&
            (model.static_gauge_dof_mask &
             (static_cast<std::uint32_t>(1) << (i-6*gauge_free_index))) != 0;
        if (gauge || static_rotation_gauge_for_pivot(model, i) != nullptr) {
            continue;
        }
        double& scale = i % 6 < 3 ? force_scale : moment_scale;
        scale = std::max(scale, std::abs(force[i]));
        double reaction = 0.0;
        for (int row = 0; row < m; ++row) {
            reaction = std::max(
                reaction, std::abs(J[row*n+i]*lambda[static_cast<std::size_t>(row)])
            );
        }
        scale = std::max(scale, reaction);
    }
    for (int i = 0; i < n; ++i) {
        const bool gauge = gauge_free_index >= 0 &&
            i >= 6*gauge_free_index && i < 6*gauge_free_index+6 &&
            (model.static_gauge_dof_mask &
             (static_cast<std::uint32_t>(1) << (i-6*gauge_free_index))) != 0;
        if (gauge || static_rotation_gauge_for_pivot(model, i) != nullptr) {
            continue;
        }
        const double normalized = std::abs(out[i]) / (
            i % 6 < 3 ? force_scale : moment_scale
        );
        if (normalized > worst_normalized_residual) {
            worst_normalized_residual = normalized;
            worst_coordinate = i;
            worst_value = out[i];
        }
        if (i % 6 < 3) {
            translational_residual = std::max(
                translational_residual, std::abs(out[i]) / force_scale
            );
        } else {
            rotational_residual = std::max(
                rotational_residual, std::abs(out[i]) / moment_scale
            );
        }
    }
    // 静态转轴 gauge 已经用坐标条件替换对应的广义力平衡方程；该方程
    // 的单体转矩是人为选定参考坐标后的约束反力，不属于物理平衡残差。
    // 其余未被替换的广义力方程仍全部参与上面的 force_residual 计算。
    force_residual = std::max(translational_residual, rotational_residual);
    position_residual=max_abs(phi);
    for (const auto& gauge : model.static_rotation_gauges) {
        position_residual = std::max(
            position_residual,
            std::abs(static_rotation_gauge_value(model, gauge, dy))
        );
    }
    for (std::size_t i = 0; i < active_tires.size(); ++i) {
        position_residual = std::max(
            position_residual,
            std::abs(out[static_cast<std::size_t>(n+m)+i])
        );
    }
    if (worst_force_coordinate != nullptr) {
        *worst_force_coordinate = worst_coordinate;
    }
    if (worst_force_value != nullptr) {
        *worst_force_value = worst_value;
    }
    return out;
}

struct ConstraintResidualMaxima {
    double position{0.0};
    double angle{0.0};
};

ConstraintResidualMaxima constraint_residual_maxima(
    const Model& model, const State& state, const SampleInput* input = nullptr
) {
    const auto values = constraint_residual(model, state, input);
    ConstraintResidualMaxima result;
    auto consume = [&](int& index, int count, double& target) {
        for (int item = 0; item < count; ++item) {
            target = std::max(target, std::abs(values[index++]));
        }
    };
    for (const auto& c : model.constraints) {
        int index = c.row;
        if (c.type == AXLE_SPHERICAL || c.type == AXLE_REVOLUTE ||
            c.type == AXLE_FIXED || c.type == AXLE_UNIVERSAL ||
            c.type == AXLE_CONVEL) {
            consume(index, 3, result.position);
        }
        if (c.type == AXLE_FIXED) {
            consume(index, 3, result.angle);
        } else if (c.type == AXLE_REVOLUTE) {
            consume(index, 2, result.angle);
        } else if (c.type == AXLE_UNIVERSAL) {
            consume(index, 1, result.angle);
        } else if (c.type == AXLE_CONVEL) {
            consume(index, 1, result.angle);
        } else if (c.type == AXLE_CYLINDRICAL) {
            consume(index, 2, result.position);
            consume(index, 2, result.angle);
        } else if (c.type == AXLE_PRISMATIC) {
            consume(index, 2, result.position);
            consume(index, 3, result.angle);
        } else if (c.type == AXLE_INPLANE) {
            consume(index, 1, result.position);
        }
    }
    for (const auto& coupler : model.coordinate_couplers) {
        const double value = std::abs(
            coupler.scale_a * joint_coordinate_value(
                model, state, coupler.joint_a, coupler.coordinate_a,
                coupler.reference_translation_a, coupler.reference_rotation_a
            )
            + coupler.scale_b * joint_coordinate_value(
                model, state, coupler.joint_b, coupler.coordinate_b,
                coupler.reference_translation_b, coupler.reference_rotation_b
            )
        );
        const bool angular = coupler.coordinate_a == 0
            && coupler.coordinate_b == 0;
        if (angular) result.angle = std::max(result.angle, value);
        else result.position = std::max(result.position, value);
    }
    for (const auto& actuator : model.steering_actuators) {
        if (!prescribed_steering(actuator)) continue;
        const double value = std::abs(values[static_cast<std::size_t>(
            actuator.constraint_row
        )]);
        if (actuator.type == VEHICLE_STEERING_PRESCRIBED_TRANSLATION) {
            result.position = std::max(result.position, value);
        } else {
            result.angle = std::max(result.angle, value);
        }
    }
    return result;
}

double static_contact_tolerance(const Model& model) {
    double length_scale = 1.0;
    for (const Tire& tire : model.tires) {
        length_scale = std::max(length_scale, std::abs(tire.radius));
    }
    return std::max(
        1.0e-12,
        100.0*std::numeric_limits<double>::epsilon()*length_scale
    );
}

// A pose coordinate can be statically indeterminate: a revolute wheel spin
// changes no force, no constraint, and no residual, so the static Jacobian has
// an identically zero row AND column there. Such a coordinate is arbitrary, and
// its residual is already satisfied, so it is pinned at its initial value. This
// is not a regularization of an identifiable direction: pinning is applied only
// where the coordinate provably has no influence on any residual, and the count
// is reported so a genuinely rank-deficient model is never silently trimmed.
int pin_null_pose_directions(
    std::vector<double>& jacobian, const std::vector<double>& residual,
    int dimension, int pose_dimension, double tolerance
) {
    // The columns carry mixed units, so "influential" is judged relative to the
    // largest entry in the matrix rather than against an absolute tolerance in
    // newtons.  A pure absolute test would call every column of a kilonewton
    // model influential and never pin a genuine null direction.
    double largest = 0.0;
    for (double value : jacobian) largest = std::max(largest, std::abs(value));
    const double influence_floor = std::max(tolerance, 1e-9 * largest);
    int pinned = 0;
    for (int index = 0; index < pose_dimension; ++index) {
        if (std::abs(residual[static_cast<std::size_t>(index)]) > tolerance) {
            continue;
        }
        bool influential = false;
        for (int other = 0; other < dimension && !influential; ++other) {
            influential =
                std::abs(
                    jacobian[static_cast<std::size_t>(index*dimension+other)]
                ) > influence_floor ||
                std::abs(
                    jacobian[static_cast<std::size_t>(other*dimension+index)]
                ) > influence_floor;
        }
        if (influential) continue;
        jacobian[static_cast<std::size_t>(index*dimension+index)] = 1.0;
        ++pinned;
    }
    return pinned;
}

std::vector<int> static_gauge_coordinates(const Model& model) {
    std::vector<int> coordinates;
    if (model.static_gauge_body < 0 || model.static_gauge_dof_mask == 0) {
        return coordinates;
    }
    const int free_body = model.body_to_free[
        static_cast<std::size_t>(model.static_gauge_body)
    ];
    if (free_body < 0) return coordinates;
    for (int local = 0; local < 6; ++local) {
        if ((model.static_gauge_dof_mask &
             (static_cast<std::uint32_t>(1) << local)) != 0) {
            coordinates.push_back(6*free_body+local);
        }
    }
    return coordinates;
}

const StaticRotationGauge* static_rotation_gauge_for_pivot(
    const Model& model, int coordinate
) {
    for (const auto& gauge : model.static_rotation_gauges) {
        if (gauge.pivot == coordinate) return &gauge;
    }
    return nullptr;
}

double static_rotation_gauge_value(
    const Model& model, const StaticRotationGauge& gauge,
    const std::vector<double>& pose_increment
) {
    const int free_body = model.body_to_free[
        static_cast<std::size_t>(gauge.body)
    ];
    if (free_body < 0) return 0.0;
    return dot(
        gauge.axis_world,
        Vec3{
            pose_increment[static_cast<std::size_t>(6*free_body+3)],
            pose_increment[static_cast<std::size_t>(6*free_body+4)],
            pose_increment[static_cast<std::size_t>(6*free_body+5)]
        }
    );
}

std::vector<unsigned char> static_position_constraint_mask(
    const Model& model
) {
    std::vector<unsigned char> mask(
        static_cast<std::size_t>(model.rows), 1
    );
    for (const auto& coupler : model.coordinate_couplers) {
        if (coupler.row >= 0 && coupler.row < model.rows) {
            mask[static_cast<std::size_t>(coupler.row)] = 1;
        }
    }
    for (const auto& actuator : model.steering_actuators) {
        if (!prescribed_steering(actuator)) continue;
        if (actuator.constraint_row >= 0 && actuator.constraint_row < model.rows) {
            mask[static_cast<std::size_t>(actuator.constraint_row)] = 1;
        }
    }
    return mask;
}

std::vector<int> static_position_constraint_rows(const Model& model) {
    const auto mask = static_position_constraint_mask(model);
    std::vector<int> rows;
    rows.reserve(static_cast<std::size_t>(model.rows));
    for (int row = 0; row < model.rows; ++row) {
        if (mask[static_cast<std::size_t>(row)] != 0) rows.push_back(row);
    }
    return rows;
}

std::vector<double> static_position_constraint_jacobian(
    const Model& model, const State& state
) {
    auto jacobian = constraint_jacobian(model, state);
    const auto mask = static_position_constraint_mask(model);
    for (int row = 0; row < model.rows; ++row) {
        if (mask[static_cast<std::size_t>(row)] != 0) continue;
        std::fill(
            jacobian.begin()+static_cast<std::size_t>(row*model.ndof),
            jacobian.begin()+static_cast<std::size_t>((row+1)*model.ndof),
            0.0
        );
    }
    return jacobian;
}

bool static_jacobian(
    const Model& model, const State& base, const SampleInput& sample,
    double gravity_x, double gravity_y, double gravity_z,
    const std::vector<int>& active_tires, const std::vector<double>& x,
    std::vector<double>& jacobian, double load_scale = 1.0
) {
    const int n = model.ndof;
    const int m = model.rows;
    (void)gravity_x;
    (void)gravity_y;
    (void)gravity_z;
    const int active_count = static_cast<int>(active_tires.size());
    const int dimension = n+m+active_count;
    if (static_cast<int>(x.size()) != dimension) return false;

    const State candidate = pose_candidate(
        base, model, std::vector<double>(x.begin(), x.begin()+n)
    );
    const auto constraint = constraint_jacobian(model, candidate);
    const auto position_constraint = static_position_constraint_jacobian(
        model, candidate
    );
    std::vector<int> active_mask(model.tires.size(), 0);
    std::vector<double> compression(model.tires.size(), 0.0);
    std::vector<double> compression_derivative(model.tires.size(), 0.0);
    for (std::size_t index = 0; index < active_tires.size(); ++index) {
        const int tire_index = active_tires[index];
        active_mask[static_cast<std::size_t>(tire_index)] = 1;
        compression[static_cast<std::size_t>(tire_index)] =
            x[static_cast<std::size_t>(n+m)+index];
    }
    StaticContactOverride static_contact{
        &active_mask, &compression, &compression_derivative,
        load_scale, load_scale
    };
    const std::vector<double> lambda(x.begin()+n, x.begin()+n+m);
    const auto gauges = static_gauge_coordinates(model);
    std::vector<unsigned char> gauge(static_cast<std::size_t>(n), 0);
    for (const int coordinate : gauges) {
        if (coordinate >= 0 && coordinate < n) {
            gauge[static_cast<std::size_t>(coordinate)] = 1;
        }
    }
    jacobian.assign(
        static_cast<std::size_t>(dimension)*static_cast<std::size_t>(dimension),
        0.0
    );
    const auto at = [dimension](int row, int column) {
        return static_cast<std::size_t>(row)*static_cast<std::size_t>(dimension)
            + static_cast<std::size_t>(column);
    };
    DirectionalState direction;
    ensure_directional_state(model, direction);
    DirectionalForceScratch force_scratch;
    force_scratch.force.resize(model.bodies.size());
    force_scratch.torque.resize(model.bodies.size());
    std::vector<double> force_derivative;
    std::vector<double> brush_derivative;
    std::vector<double> jacobian_derivative;
    bool smooth = true;

    for (int column = 0; column < n; ++column) {
        reset_directional_state(model, direction);
        const int free_body = column/6;
        const int component = column%6;
        const int body = model.free_body[static_cast<std::size_t>(free_body)];
        if (component < 3) {
            direction.dr[static_cast<std::size_t>(body)] = {
                component == 0 ? 1.0 : 0.0,
                component == 1 ? 1.0 : 0.0,
                component == 2 ? 1.0 : 0.0
            };
        } else {
            const Vec3 increment{
                x[6*free_body+3], x[6*free_body+4], x[6*free_body+5]
            };
            const Vec3 parameter_direction{
                component == 3 ? 1.0 : 0.0,
                component == 4 ? 1.0 : 0.0,
                component == 5 ? 1.0 : 0.0
            };
            direction.dtheta[static_cast<std::size_t>(body)] =
                so3_left_jacobian(increment)*parameter_direction;
        }
        const bool force_smooth = external_force_directional(
            model, candidate, sample, direction, force_derivative,
            brush_derivative, false, &static_contact, &force_scratch
        );
        const bool constraint_smooth = constraint_jacobian_directional(
            model, candidate, direction, jacobian_derivative
        );
        smooth = smooth && force_smooth && constraint_smooth;
        for (int row = 0; row < n; ++row) {
            if (gauge[static_cast<std::size_t>(row)] != 0) {
                jacobian[at(row, column)] = row == column ? 1.0 : 0.0;
                continue;
            }
            double value = -force_derivative[static_cast<std::size_t>(row)];
            for (int constraint_row = 0; constraint_row < m; ++constraint_row) {
                value -= jacobian_derivative[
                    static_cast<std::size_t>(constraint_row*n+row)
                ] * lambda[static_cast<std::size_t>(constraint_row)];
            }
            jacobian[at(row, column)] = value;
        }
        for (int row = 0; row < m; ++row) {
            if (column % 6 < 3) {
                jacobian[at(n+row, column)] = position_constraint[
                    static_cast<std::size_t>(row*n+column)
                ];
            } else {
                const int free_body = column/6;
                const int first_rotation = 6*free_body+3;
                const Vec3 increment{
                    x[6*free_body+3], x[6*free_body+4], x[6*free_body+5]
                };
                const Mat3 rotation_map = so3_left_jacobian(increment);
                double value = 0.0;
                for (int tangent = 0; tangent < 3; ++tangent) {
                    value += position_constraint[
                        static_cast<std::size_t>(row*n+first_rotation+tangent)
                    ] * rotation_map.a[tangent][column % 6 - 3];
                }
                jacobian[at(n+row, column)] = value;
            }
        }
        for (std::size_t index = 0; index < active_tires.size(); ++index) {
            const int tire_index = active_tires[index];
            bool road_smooth = true;
            const auto road = road_profile_directional(
                model, candidate, direction,
                static_cast<std::size_t>(tire_index), road_smooth
            );
            smooth = smooth && road_smooth;
            const Tire& tire = model.tires[static_cast<std::size_t>(tire_index)];
            const DVec3 center = d_state_point(
                candidate, direction.dr, direction.dtheta,
                tire_center_body(tire), tire_center_local(tire)
            );
            jacobian[at(n+m+static_cast<int>(index), column)] =
                center.z.derivative-road.height.derivative;
        }
    }

    for (int row = 0; row < m; ++row) {
        for (int coordinate = 0; coordinate < n; ++coordinate) {
            if (gauge[static_cast<std::size_t>(coordinate)] == 0 &&
                static_rotation_gauge_for_pivot(model, coordinate) == nullptr) {
                jacobian[at(coordinate, n+row)] = -constraint[
                    static_cast<std::size_t>(row*n+coordinate)
                ];
            }
        }
    }

    for (std::size_t active_index = 0;
         active_index < active_tires.size(); ++active_index) {
        std::fill(compression_derivative.begin(), compression_derivative.end(), 0.0);
        const int tire_index = active_tires[active_index];
        compression_derivative[static_cast<std::size_t>(tire_index)] = 1.0;
        reset_directional_state(model, direction);
        const bool force_smooth = external_force_directional(
            model, candidate, sample, direction, force_derivative,
            brush_derivative, false, &static_contact, &force_scratch
        );
        smooth = smooth && force_smooth;
        const int column = n+m+static_cast<int>(active_index);
        for (int row = 0; row < n; ++row) {
            if (gauge[static_cast<std::size_t>(row)] == 0 &&
                static_rotation_gauge_for_pivot(model, row) == nullptr) {
                jacobian[at(row, column)] =
                    -force_derivative[static_cast<std::size_t>(row)];
            }
        }
        jacobian[at(n+m+static_cast<int>(active_index), column)] = 1.0;
    }
    for (const auto& rotation_gauge : model.static_rotation_gauges) {
        const int free_body = model.body_to_free[
            static_cast<std::size_t>(rotation_gauge.body)
        ];
        if (free_body < 0 || rotation_gauge.pivot < 0 ||
            rotation_gauge.pivot >= n) {
            continue;
        }
        const int first = 6*free_body+3;
        for (int column = 0; column < dimension; ++column) {
            jacobian[at(rotation_gauge.pivot, column)] = 0.0;
        }
        jacobian[at(rotation_gauge.pivot, first)] = rotation_gauge.axis_world.x;
        jacobian[at(rotation_gauge.pivot, first+1)] = rotation_gauge.axis_world.y;
        jacobian[at(rotation_gauge.pivot, first+2)] = rotation_gauge.axis_world.z;
    }
    return smooth && finite_vec(jacobian);
}

bool normalized_static_constraint_matrix(
    const Model& model, const State& state,
    std::vector<double>& matrix, std::vector<double>& row_scale,
    int& row_count
) {
    const int n = model.ndof;
    const auto constraint = static_position_constraint_jacobian(model, state);
    const auto constraint_rows = static_position_constraint_rows(model);
    const auto gauges = static_gauge_coordinates(model);
    row_count = static_cast<int>(constraint_rows.size())+
        static_cast<int>(gauges.size())+
        static_cast<int>(model.static_rotation_gauges.size());
    matrix.assign(static_cast<std::size_t>(row_count*n), 0.0);
    row_scale.assign(static_cast<std::size_t>(row_count), 1.0);
    for (std::size_t index = 0; index < constraint_rows.size(); ++index) {
        const int row = constraint_rows[index];
        double largest = 0.0;
        for (int column = 0; column < n; ++column) {
            largest = std::max(
                largest,
                std::abs(constraint[static_cast<std::size_t>(row*n+column)])
            );
        }
        if (largest <= kEps) return false;
        row_scale[index] = largest;
        for (int column = 0; column < n; ++column) {
            matrix[index*static_cast<std::size_t>(n)+
                   static_cast<std::size_t>(column)] =
                constraint[static_cast<std::size_t>(row*n+column)]/largest;
        }
    }
    for (std::size_t index = 0; index < gauges.size(); ++index) {
        const int row = static_cast<int>(constraint_rows.size())+
            static_cast<int>(index);
        matrix[static_cast<std::size_t>(row*n+gauges[index])] = 1.0;
    }
    const int rotation_offset = static_cast<int>(constraint_rows.size())+
        static_cast<int>(gauges.size());
    for (std::size_t index = 0;
         index < model.static_rotation_gauges.size();
         ++index) {
        const auto& gauge = model.static_rotation_gauges[index];
        const int free_body = model.body_to_free[
            static_cast<std::size_t>(gauge.body)
        ];
        if (free_body < 0) return false;
        const int row = rotation_offset+static_cast<int>(index);
        const int first = 6*free_body+3;
        matrix[static_cast<std::size_t>(row*n+first)] = gauge.axis_world.x;
        matrix[static_cast<std::size_t>(row*n+first+1)] = gauge.axis_world.y;
        matrix[static_cast<std::size_t>(row*n+first+2)] = gauge.axis_world.z;
    }
    return true;
}

bool static_tangent_projection(
    const Model& model, const State& state,
    const std::vector<double>& pose_residual,
    std::vector<double>& projected, double& projected_norm
) {
    const int n = model.ndof;
    if (static_cast<int>(pose_residual.size()) != n) return false;
    std::vector<double> matrix;
    std::vector<double> row_scale;
    int row_count = 0;
    if (!normalized_static_constraint_matrix(
            model, state, matrix, row_scale, row_count
        )) {
        return false;
    }
    std::vector<double> gram(static_cast<std::size_t>(row_count*row_count), 0.0);
    std::vector<double> rhs(static_cast<std::size_t>(row_count), 0.0);
    for (int row = 0; row < row_count; ++row) {
        for (int column = 0; column < n; ++column) {
            rhs[static_cast<std::size_t>(row)] +=
                matrix[static_cast<std::size_t>(row*n+column)]
                * pose_residual[static_cast<std::size_t>(column)];
        }
        for (int other = 0; other < row_count; ++other) {
            double value = 0.0;
            for (int column = 0; column < n; ++column) {
                value +=
                    matrix[static_cast<std::size_t>(row*n+column)]
                    * matrix[static_cast<std::size_t>(other*n+column)];
            }
            gram[static_cast<std::size_t>(row*row_count+other)] = value;
        }
    }
    std::vector<double> multipliers;
    if (!solve_linear(gram, rhs, multipliers)) return false;
    projected = pose_residual;
    for (int column = 0; column < n; ++column) {
        double correction = 0.0;
        for (int row = 0; row < row_count; ++row) {
            correction +=
                matrix[static_cast<std::size_t>(row*n+column)]
                * multipliers[static_cast<std::size_t>(row)];
        }
        projected[static_cast<std::size_t>(column)] -= correction;
    }
    projected_norm = max_abs(projected);
    return finite_vec(projected);
}

bool project_static_pose(
    const Model& model, const State& base, std::vector<double>& pose_increment,
    double tolerance, const SampleInput* sample
) {
    const int n = model.ndof;
    const auto gauges = static_gauge_coordinates(model);
    for (int iteration = 0; iteration < 12; ++iteration) {
        const State candidate = pose_candidate(base, model, pose_increment);
        const auto constraints = constraint_residual(model, candidate, sample);
        double residual_norm = max_abs(constraints);
        for (int coordinate : gauges) {
            residual_norm = std::max(
                residual_norm,
                std::abs(pose_increment[static_cast<std::size_t>(coordinate)])
            );
        }
        for (const auto& gauge : model.static_rotation_gauges) {
            residual_norm = std::max(
                residual_norm,
                std::abs(static_rotation_gauge_value(model, gauge, pose_increment))
            );
        }
        if (residual_norm <= tolerance) return true;
        std::vector<double> matrix;
        std::vector<double> row_scale;
        int row_count = 0;
        if (!normalized_static_constraint_matrix(
                model, candidate, matrix, row_scale, row_count
            )) {
            return false;
        }
        std::vector<double> rhs(static_cast<std::size_t>(row_count), 0.0);
        const auto constraint_rows = static_position_constraint_rows(model);
        for (std::size_t index = 0; index < constraint_rows.size(); ++index) {
            const int row = constraint_rows[index];
            rhs[index] =
                -constraints[static_cast<std::size_t>(row)] / row_scale[index];
        }
        for (std::size_t index = 0; index < gauges.size(); ++index) {
            rhs[constraint_rows.size()+index] =
                -pose_increment[static_cast<std::size_t>(gauges[index])];
        }
        const int rotation_offset = static_cast<int>(constraint_rows.size())+
            static_cast<int>(gauges.size());
        for (std::size_t index = 0;
             index < model.static_rotation_gauges.size();
             ++index) {
            rhs[static_cast<std::size_t>(rotation_offset)+index] =
                -static_rotation_gauge_value(
                    model, model.static_rotation_gauges[index], pose_increment
                );
        }
        std::vector<double> gram(static_cast<std::size_t>(row_count*row_count), 0.0);
        for (int row = 0; row < row_count; ++row) {
            for (int other = 0; other < row_count; ++other) {
                double value = 0.0;
                for (int column = 0; column < n; ++column) {
                    value +=
                        matrix[static_cast<std::size_t>(row*n+column)]
                        * matrix[static_cast<std::size_t>(other*n+column)];
                }
                gram[static_cast<std::size_t>(row*row_count+other)] = value;
            }
        }
        std::vector<double> multipliers;
        if (!solve_linear(gram, rhs, multipliers)) return false;
        std::vector<double> correction(static_cast<std::size_t>(n), 0.0);
        for (int column = 0; column < n; ++column) {
            for (int row = 0; row < row_count; ++row) {
                correction[static_cast<std::size_t>(column)] +=
                    matrix[static_cast<std::size_t>(row*n+column)]
                    * multipliers[static_cast<std::size_t>(row)];
            }
        }
        for (int column = 0; column < n; ++column) {
            pose_increment[static_cast<std::size_t>(column)] +=
                correction[static_cast<std::size_t>(column)];
        }
        if (!finite_vec(pose_increment)) return false;
    }
    return false;
}

bool solve_static_least_squares(
    const std::vector<double>& matrix, const std::vector<double>& rhs,
    int dimension, std::vector<double>& solution
) {
    if (dimension <= 0 || static_cast<int>(rhs.size()) != dimension ||
        static_cast<int>(matrix.size()) != dimension*dimension) {
        return false;
    }
    std::vector<double> scaled_matrix = matrix;
    std::vector<double> scaled_rhs = rhs;
    for (int row = 0; row < dimension; ++row) {
        double largest = 0.0;
        for (int column = 0; column < dimension; ++column) {
            largest = std::max(
                largest,
                std::abs(scaled_matrix[static_cast<std::size_t>(
                    row*dimension+column
                )])
            );
        }
        if (largest <= kEps) continue;
        const double inverse = 1.0/largest;
        for (int column = 0; column < dimension; ++column) {
            scaled_matrix[static_cast<std::size_t>(row*dimension+column)] *=
                inverse;
        }
        scaled_rhs[static_cast<std::size_t>(row)] *= inverse;
    }
    std::vector<double> column_scale(
        static_cast<std::size_t>(dimension), 1.0
    );
    for (int column = 0; column < dimension; ++column) {
        double largest = 0.0;
        for (int row = 0; row < dimension; ++row) {
            largest = std::max(
                largest,
                std::abs(scaled_matrix[static_cast<std::size_t>(
                    row*dimension+column
                )])
            );
        }
        if (largest > kEps) {
            column_scale[static_cast<std::size_t>(column)] = 1.0/largest;
            for (int row = 0; row < dimension; ++row) {
                scaled_matrix[static_cast<std::size_t>(row*dimension+column)] *=
                    column_scale[static_cast<std::size_t>(column)];
            }
        }
    }
    std::vector<double> normal(
        static_cast<std::size_t>(dimension*dimension), 0.0
    );
    std::vector<double> normal_rhs(static_cast<std::size_t>(dimension), 0.0);
    for (int row = 0; row < dimension; ++row) {
        for (int column = 0; column < dimension; ++column) {
            double value = 0.0;
            for (int equation = 0; equation < dimension; ++equation) {
                value += scaled_matrix[static_cast<std::size_t>(
                    equation*dimension+row
                )] * scaled_matrix[static_cast<std::size_t>(
                    equation*dimension+column
                )];
            }
            normal[static_cast<std::size_t>(row*dimension+column)] = value;
        }
        double value = 0.0;
        for (int equation = 0; equation < dimension; ++equation) {
            value += scaled_matrix[static_cast<std::size_t>(
                equation*dimension+row
            )] * scaled_rhs[static_cast<std::size_t>(equation)];
        }
        normal_rhs[static_cast<std::size_t>(row)] = value;
    }
    const double normal_scale = std::max(1.0, max_abs(normal));
    for (const double damping : {1.0e-6, 1.0e-8, 1.0e-10, 1.0e-12}) {
        std::vector<double> regularized = normal;
        for (int index = 0; index < dimension; ++index) {
            regularized[static_cast<std::size_t>(index*dimension+index)] +=
                damping*normal_scale;
        }
        std::vector<double> scaled_solution;
        if (!solve_linear(regularized, normal_rhs, scaled_solution) ||
            !finite_vec(scaled_solution)) {
            continue;
        }
        solution.resize(static_cast<std::size_t>(dimension));
        for (int index = 0; index < dimension; ++index) {
            solution[static_cast<std::size_t>(index)] =
                scaled_solution[static_cast<std::size_t>(index)]
                * column_scale[static_cast<std::size_t>(index)];
        }
        if (finite_vec(solution)) return true;
    }
    return false;
}

bool static_manifold_relaxation_step(
    const Model& model, const State& base, const SampleInput& sample,
    double gravity_x, double gravity_y, double gravity_z,
    const std::vector<int>& active_tires, const std::vector<double>& residual,
    std::vector<double>& x, double tolerance, double load_scale
) {
    const int n = model.ndof;
    const int m = model.rows;
    if (static_cast<int>(residual.size()) < n) return false;
    const std::vector<double> pose_increment(x.begin(), x.begin()+n);
    const State candidate = pose_candidate(base, model, pose_increment);
    std::vector<double> projected;
    double projected_norm = 0.0;
    if (!static_tangent_projection(
            model, candidate,
            std::vector<double>(residual.begin(), residual.begin()+n),
            projected, projected_norm
        ) || projected_norm <= tolerance) {
        return false;
    }
    double largest = max_abs(projected);
    if (largest <= kEps) return false;
    for (double& value : projected) value /= largest;
    const double initial_norm = projected_norm;
    for (int line_search = 0; line_search < 18; ++line_search) {
        const double step = 0.02*std::pow(0.5, line_search);
        std::vector<double> trial = x;
        for (int index = 0; index < n; ++index) {
            trial[static_cast<std::size_t>(index)] -=
                step*projected[static_cast<std::size_t>(index)];
        }
        std::vector<double> trial_pose(trial.begin(), trial.begin()+n);
        if (!project_static_pose(
                model, base, trial_pose, tolerance, &sample
            )) continue;
        std::copy(trial_pose.begin(), trial_pose.end(), trial.begin());
        const State trial_state = pose_candidate(base, model, trial_pose);
        bool valid_contact = true;
        for (std::size_t index = 0; index < active_tires.size(); ++index) {
            const int tire_index = active_tires[index];
            const Tire& tire = model.tires[static_cast<std::size_t>(tire_index)];
            const Vec3 center = state_point(
                trial_state, tire_center_body(tire), tire_center_local(tire)
            );
            const double road =
                (static_cast<std::size_t>(tire_index) < sample.road_z.size()
                    ? sample.road_z[static_cast<std::size_t>(tire_index)] : 0.0)
                + road_profile_height(
                    model, trial_state, static_cast<std::size_t>(tire_index)
                );
            const double compression = tire.radius+road-center.z;
            if (compression > tire.maximum_compression+tolerance) {
                valid_contact = false;
                break;
            }
            trial[static_cast<std::size_t>(n+m)+index] =
                std::max(0.0, compression);
        }
        if (!valid_contact) continue;
        double trial_force = 0.0;
        double trial_position = 0.0;
        const auto trial_residual = static_residual(
            model, base, sample, gravity_x, gravity_y, gravity_z,
            active_tires, trial, trial_force, trial_position,
            nullptr, nullptr, load_scale
        );
        if (trial_position > tolerance || !finite_vec(trial_residual)) continue;
        std::vector<double> trial_projected;
        double trial_norm = 0.0;
        if (!static_tangent_projection(
                model, trial_state,
                std::vector<double>(
                    trial_residual.begin(), trial_residual.begin()+n
                ),
                trial_projected, trial_norm
            )) {
            continue;
        }
        if (trial_position <= tolerance && trial_norm < initial_norm) {
            x = std::move(trial);
            return true;
        }
    }
    return false;
}

bool static_global_contact_pretrim(
    const Model& model, const SampleInput& sample,
    double gravity_x, double gravity_y, double gravity_z,
    double external_load_scale, State& base
) {
    if (model.free_body.empty() || model.tires.size() < 2) return false;
    for (const Body& body : model.bodies) {
        if (body.fixed) return false;
    }

    int pivot_body = model.free_body.front();
    if (model.static_gauge_body >= 0 &&
        model.static_gauge_body < static_cast<int>(model.bodies.size()) &&
        !model.bodies[static_cast<std::size_t>(model.static_gauge_body)].fixed) {
        pivot_body = model.static_gauge_body;
    }
    const Vec3 pivot = base.r[static_cast<std::size_t>(pivot_body)];
    const Vec3 gravity{
        external_load_scale*gravity_x,
        external_load_scale*gravity_y,
        external_load_scale*gravity_z
    };
    double total_mass = 0.0;
    double wheelbase = 1.0;
    for (const Body& body : model.bodies) total_mass += body.mass;
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& tire = model.tires[i];
        const Vec3 center = state_point(
            base, tire_center_body(tire), tire_center_local(tire)
        );
        if (i == 0) {
            wheelbase = 1.0;
        } else {
            const Vec3 first = state_point(
                base, tire_center_body(model.tires[0]),
                tire_center_local(model.tires[0])
            );
            wheelbase = std::max(wheelbase, std::abs(center.x-first.x));
        }
    }
    const double force_scale = std::max(
        1.0, std::abs(total_mass*gravity_z)
    );
    const double moment_scale = std::max(1.0, force_scale*wheelbase);

    const auto candidate = [&](double translation, double pitch) {
        State result = base;
        const Quat rotation = qexp({0.0, pitch, 0.0});
        for (const int body : model.free_body) {
            const std::size_t index = static_cast<std::size_t>(body);
            result.r[index] = pivot + rotate(
                rotation, base.r[index]-pivot
            ) + Vec3{0.0, 0.0, translation};
            result.q[index] = normalized_continuous(
                qmul(rotation, base.q[index]), base.q[index]
            );
        }
        return result;
    };

    const auto balance = [&](const State& state) {
        std::array<double, 2> result{0.0, 0.0};
        for (std::size_t body_index = 0;
             body_index < model.bodies.size(); ++body_index) {
            const Body& body = model.bodies[body_index];
            const Vec3 force = gravity*body.mass;
            result[0] += force.z;
            result[1] += cross(
                state.r[body_index], force
            ).y;
        }
        for (std::size_t i = 0; i < model.tires.size(); ++i) {
            const Tire& tire = model.tires[i];
            const Vec3 center = state_point(
                state, tire_center_body(tire), tire_center_local(tire)
            );
            const double road =
                (i < sample.road_z.size() ? sample.road_z[i] : 0.0)
                + road_profile_height(model, state, i);
            const double compression = tire.radius+road-center.z;
            const double normal_force = external_load_scale * std::max(
                0.0, tire.k*compression
            );
            const Vec3 force{0.0, 0.0, normal_force};
            result[0] += normal_force;
            result[1] += cross(center, force).y;
        }
        return result;
    };

    // Start slightly inside the contact branch so a wheel exactly at zero
    // compression has a usable tangent in the two-variable warm start.
    double translation = -0.005*std::abs(model.tires.front().radius);
    double pitch = 0.0;
    const auto scaled_norm = [&](const std::array<double, 2>& value) {
        return std::max(
            std::abs(value[0])/force_scale,
            std::abs(value[1])/moment_scale
        );
    };
    for (int iteration = 0; iteration < 16; ++iteration) {
        const auto current = balance(candidate(translation, pitch));
        if (scaled_norm(current) <= 1.0e-8) {
            base = candidate(translation, pitch);
            return true;
        }
        const double translation_step = 1.0e-4;
        const double pitch_step = 1.0e-5;
        const auto translated = balance(
            candidate(translation+translation_step, pitch)
        );
        const auto pitched = balance(
            candidate(translation, pitch+pitch_step)
        );
        const double jacobian00 =
            (translated[0]-current[0])/translation_step;
        const double jacobian10 =
            (translated[1]-current[1])/translation_step;
        const double jacobian01 = (pitched[0]-current[0])/pitch_step;
        const double jacobian11 = (pitched[1]-current[1])/pitch_step;
        const double determinant = jacobian00*jacobian11-jacobian01*jacobian10;
        if (std::abs(determinant) <= 1.0e-12) return false;
        const double delta_translation = (
            -current[0]*jacobian11 + jacobian01*current[1]
        )/determinant;
        const double delta_pitch = (
            -jacobian00*current[1] + jacobian10*current[0]
        )/determinant;
        if (!std::isfinite(delta_translation) || !std::isfinite(delta_pitch)) {
            return false;
        }
        bool accepted = false;
        const double current_norm = scaled_norm(current);
        for (int line_search = 0; line_search < 14; ++line_search) {
            const double scale = std::ldexp(1.0, -line_search);
            const auto trial = balance(candidate(
                translation+scale*delta_translation,
                pitch+scale*delta_pitch
            ));
            if (scaled_norm(trial) < current_norm) {
                translation += scale*delta_translation;
                pitch += scale*delta_pitch;
                accepted = true;
                break;
            }
        }
        if (!accepted) return false;
    }
    return false;
}

bool static_trim(const Model& model, const AxleInput& input, State& state,
                 double& force_residual, double& position_residual, int& iterations,
                 std::vector<double>& constraint_multiplier, int& pinned_directions,
                 int& worst_force_coordinate, double& worst_force_value) {
    const bool debug = static_debug_enabled();
    SampleInput sample;
    interpolate_input(model, input, input.sample_times[0], sample);
    State base = state;
    const double contact_tolerance = static_contact_tolerance(model);
    // 源整车的编译姿态可能让前后轮分别处于压缩和离地状态。若把离地
    // 轮胎直接加入活动集，互补方程在初始点不一致；若只保留压缩轮，
    // 单轴接触又无法平衡整车重力的俯仰力矩。对没有固定刚体的车辆，
    // 先沿水平路面的法向整体下移最大间隙，保持所有相对约束和姿态不变，
    // 将静态接触初始化带到接触支撑支路。该操作只作用于静态初猜。
    double maximum_gap = 0.0;
    bool all_contact_bodies_free = true;
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& tire = model.tires[i];
        if (model.bodies[tire.body].fixed) {
            all_contact_bodies_free = false;
            continue;
        }
        const Vec3 center = state_point(
            base, tire_center_body(tire), tire_center_local(tire)
        );
        const double road =
            (i < sample.road_z.size() ? sample.road_z[i] : 0.0)
            + road_profile_height(model, base, i);
        maximum_gap = std::max(
            maximum_gap, center.z - (tire.radius + road)
        );
    }
    if (all_contact_bodies_free && maximum_gap > contact_tolerance) {
        for (const int body : model.free_body) {
            base.r[static_cast<std::size_t>(body)].z -= maximum_gap;
        }
        static_global_contact_pretrim(
            model, sample, input.gravity_x, input.gravity_y, input.gravity_z,
            1.0/8.0, base
        );
    }
    std::vector<int> active_tires;
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& tire = model.tires[i];
        if (model.bodies[tire.body].fixed) continue;
        const Vec3 center = state_point(
            base, tire_center_body(tire), tire_center_local(tire)
        );
        const double road =
            (i < sample.road_z.size() ? sample.road_z[i] : 0.0)
            + road_profile_height(model, base, i);
        const double geometric_compression =
            tire.radius + road - center.z;
        // 只把已压缩或处于数值接触边界的轮胎放入初始活动集。离地轮胎
        // 由后续活动集迭代在几何压缩变为正时加入，避免从初始点同时施加
        // 非负压缩和负几何压缩这两个互相矛盾的方程。
        const double activation_tolerance = std::max(
            10.0*input.position_tolerance,
            100.0*std::numeric_limits<double>::epsilon()
                * std::max(1.0, std::abs(tire.radius))
        );
        if (geometric_compression >= -activation_tolerance) {
            active_tires.push_back(static_cast<int>(i));
        }
    }
    if (debug) {
        std::fprintf(stderr, "静态调试: 初始活动轮胎数=%zu", active_tires.size());
        for (const int tire_index : active_tires) {
            std::fprintf(stderr, " %d", tire_index);
        }
        std::fprintf(stderr, "\n");
    }
    const int active_set_limit =
        std::max(2, static_cast<int>(model.tires.size())+2);
    // 参考 Chrono 的增量静力分析：逐级恢复载荷，让源模型从编译姿态
    // 过渡到完整静态平衡；最终级仍使用 100% 的原始载荷。
    constexpr int static_load_steps = 8;
    pinned_directions = 0;
    std::vector<double> x(
        static_cast<std::size_t>(model.ndof+model.rows)
            + active_tires.size(),
        0.0
    );
    for (std::size_t index = 0; index < active_tires.size(); ++index) {
        const std::size_t tire_index = static_cast<std::size_t>(
            active_tires[index]
        );
        const Tire& tire = model.tires[tire_index];
        const Vec3 center = state_point(
            base, tire_center_body(tire), tire_center_local(tire)
        );
        const double road =
            (tire_index < sample.road_z.size() ? sample.road_z[tire_index] : 0.0)
            + road_profile_height(model, base, tire_index);
        x[static_cast<std::size_t>(model.ndof+model.rows)+index] = std::max(
            0.0, tire.radius + road - center.z
        );
    }
    iterations = 0;
    worst_force_coordinate = -1;
    worst_force_value = 0.0;
    for (int load_step = 0; load_step < static_load_steps; ++load_step) {
        const double load_scale = static_cast<double>(load_step+1)
            / static_cast<double>(static_load_steps);
        bool load_converged = false;
        for (int active_iteration = 0;
             active_iteration < active_set_limit; ++active_iteration) {
            const int dim = static_cast<int>(x.size());
            bool converged = false;
            for (int newton_iteration = 0;
                 newton_iteration < input.max_newton_iterations;
                 ++newton_iteration) {
            ++iterations;
            const auto r=static_residual(
                model, base, sample, input.gravity_x, input.gravity_y,
                input.gravity_z, active_tires, x, force_residual,
                position_residual, &worst_force_coordinate,
                &worst_force_value, load_scale
            );
            if (debug) {
                std::fprintf(
                    stderr,
                    "静态调试: 载荷级=%d/ %d 因子=%.6g 活动集=%d 牛顿=%d 力残差=%.17g "
                    "位形残差=%.17g 最差坐标=%d 最差值=%.17g\n",
                    load_step+1, static_load_steps, load_scale,
                    active_iteration, newton_iteration, force_residual,
                    position_residual, worst_force_coordinate,
                    worst_force_value
                );
            }
            // `position_residual` already covers the constraint and tire
            // compression rows in metres, and `force_residual` covers the
            // force rows normalized by the largest force in them.  Testing
            // max_abs(r) as well would compare newtons against a metre
            // tolerance, which no kilonewton-scale model can ever satisfy.
            if(position_residual<=input.position_tolerance &&
               force_residual<=input.dynamics_tolerance){
                converged = true;
                break;
            }
            std::vector<double> jac(
                static_cast<std::size_t>(dim*dim), 0.0
            );
            // The static KKT Jacobian uses the same directional chain-rule
            // primitives as the transient Newton solver.  A trim at a
            // declared piecewise-force boundary is rejected instead of using
            // a finite-difference derivative across two different branches.
            if (!static_jacobian(
                    model, base, sample, input.gravity_x, input.gravity_y,
                    input.gravity_z, active_tires, x, jac, load_scale
                )) {
                return false;
            }
            pinned_directions = pin_null_pose_directions(
                jac, r, dim, model.ndof,
                std::max(input.dynamics_tolerance, input.position_tolerance)
            );
            std::vector<double> rhs(static_cast<std::size_t>(dim));
            for(int i=0;i<dim;++i) rhs[i]=-r[i];
            std::vector<double> dx;
            const bool force_least_squares =
                std::getenv("SUSPENSION_AXLE_DEBUG_STATIC_LEAST_SQUARES") != nullptr;
            if (force_least_squares || !solve_linear(jac,rhs,dx) || !finite_vec(dx)) {
                if (!solve_static_least_squares(jac, rhs, dim, dx) ||
                    !finite_vec(dx)) {
                    return false;
                }
            }
            if (debug) {
                double linearized_residual = 0.0;
                int linearized_coordinate = -1;
                for (int row = 0; row < dim; ++row) {
                    double value = r[static_cast<std::size_t>(row)];
                    for (int column = 0; column < dim; ++column) {
                        value += jac[
                            static_cast<std::size_t>(row*dim+column)
                        ]*dx[static_cast<std::size_t>(column)];
                    }
                    if (std::abs(value) > linearized_residual) {
                        linearized_residual = std::abs(value);
                        linearized_coordinate = row;
                    }
                }
                double max_pose_translation = 0.0;
                double max_pose_rotation = 0.0;
                double max_multiplier = 0.0;
                double max_compression = 0.0;
                for (int coordinate = 0; coordinate < model.ndof; ++coordinate) {
                    if (coordinate % 6 < 3) {
                        max_pose_translation = std::max(
                            max_pose_translation,
                            std::abs(dx[static_cast<std::size_t>(coordinate)])
                        );
                    } else {
                        max_pose_rotation = std::max(
                            max_pose_rotation,
                            std::abs(dx[static_cast<std::size_t>(coordinate)])
                        );
                    }
                }
                for (int coordinate = model.ndof;
                     coordinate < model.ndof+model.rows; ++coordinate) {
                    max_multiplier = std::max(
                        max_multiplier,
                        std::abs(dx[static_cast<std::size_t>(coordinate)])
                    );
                }
                for (int coordinate = model.ndof+model.rows;
                     coordinate < dim; ++coordinate) {
                    max_compression = std::max(
                        max_compression,
                        std::abs(dx[static_cast<std::size_t>(coordinate)])
                    );
                }
                std::fprintf(
                    stderr,
                    "静态调试: 线性化最大残差=%.17g 坐标=%d "
                    "位移步=%.17g 转角步=%.17g 乘子步=%.17g "
                    "压缩步=%.17g\n",
                    linearized_residual, linearized_coordinate,
                    max_pose_translation, max_pose_rotation,
                    max_multiplier, max_compression
                );
                if (std::getenv("SUSPENSION_AXLE_DEBUG_STATIC_DETAIL") != nullptr) {
                    std::vector<int> coordinates(model.ndof);
                    std::iota(coordinates.begin(), coordinates.end(), 0);
                    std::sort(
                        coordinates.begin(), coordinates.end(),
                        [&dx](int left, int right) {
                            return std::abs(dx[static_cast<std::size_t>(left)]) >
                                std::abs(dx[static_cast<std::size_t>(right)]);
                        }
                    );
                    const int count = std::min(20, model.ndof);
                    for (int index = 0; index < count; ++index) {
                        const int coordinate = coordinates[static_cast<std::size_t>(index)];
                        const int free_body = coordinate/6;
                        const int body = model.free_body[static_cast<std::size_t>(free_body)];
                        std::fprintf(
                            stderr,
                            "静态调试: 步长坐标=%d 自由体=%d 原始部件=%d 局部坐标=%d 步长=%.17g\n",
                            coordinate, free_body, body, coordinate%6,
                            dx[static_cast<std::size_t>(coordinate)]
                        );
                    }
                }
            }
            // 静态 KKT 的线性解可能沿机构近奇异方向给出很大的刚体
            // 位形增量。信赖域按模型几何尺度设置，再由约束投影和线
            // 搜索确定实际步长；这只限制 Newton 试探步，不改变平衡方程。
            double characteristic_length = 0.0;
            for (const Spring& spring : model.springs) {
                characteristic_length = std::max(
                    characteristic_length, std::abs(spring.free_length)
                );
            }
            double maximum_translation_step = std::max(
                1.0e-3, 0.1*characteristic_length
            );
            for (const Tire& tire : model.tires) {
                maximum_translation_step = std::max(
                    maximum_translation_step, 0.02*std::abs(tire.radius)
                );
            }
            double maximum_pose_translation = 0.0;
            double maximum_pose_rotation = 0.0;
            for (int coordinate = 0; coordinate < model.ndof; ++coordinate) {
                if (coordinate % 6 < 3) {
                    maximum_pose_translation = std::max(
                        maximum_pose_translation,
                        std::abs(dx[static_cast<std::size_t>(coordinate)])
                    );
                } else {
                    maximum_pose_rotation = std::max(
                        maximum_pose_rotation,
                        std::abs(dx[static_cast<std::size_t>(coordinate)])
                    );
                }
            }
            double scale = 1.0;
            if (maximum_pose_translation > maximum_translation_step) {
                scale = std::min(
                    scale, maximum_translation_step/maximum_pose_translation
                );
            }
            if (maximum_pose_rotation > 0.02) {
                scale = std::min(scale, 0.02/maximum_pose_rotation);
            }
            bool accepted=false;
            // Measure progress the same way convergence is measured, so the
            // search cannot reject a step that improves both residuals just
            // because a raw newton-valued row grew.
            const double base_merit =
                position_residual/input.position_tolerance
                + force_residual/input.dynamics_tolerance;
            for(int ls=0;ls<input.max_line_search_iterations;++ls){
                std::vector<double> trial=x;
                for(int i=0;i<dim;++i) trial[i]+=scale*dx[i];
                std::vector<double> trial_pose(trial.begin(), trial.begin()+model.ndof);
                if (!project_static_pose(
                        model, base, trial_pose, input.position_tolerance,
                        &sample
                    )) {
                    scale*=0.5;
                    continue;
                }
                std::copy(trial_pose.begin(), trial_pose.end(), trial.begin());
                const State trial_state = pose_candidate(base, model, trial_pose);
                bool valid_contact = true;
                for (std::size_t active_index = 0;
                     active_index < active_tires.size(); ++active_index) {
                    const int tire_index = active_tires[active_index];
                    const Tire& tire = model.tires[
                        static_cast<std::size_t>(tire_index)
                    ];
                    const Vec3 center = state_point(
                        trial_state, tire_center_body(tire),
                        tire_center_local(tire)
                    );
                    const double road =
                        (static_cast<std::size_t>(tire_index) < sample.road_z.size()
                            ? sample.road_z[
                                static_cast<std::size_t>(tire_index)
                            ] : 0.0)
                        + road_profile_height(
                            model, trial_state,
                            static_cast<std::size_t>(tire_index)
                        );
                    const double compression = tire.radius+road-center.z;
                    if (compression > tire.maximum_compression+
                        input.position_tolerance) {
                        valid_contact = false;
                        break;
                    }
                    trial[static_cast<std::size_t>(model.ndof+model.rows)
                        +active_index] = std::max(0.0, compression);
                }
                if (!valid_contact) {
                    scale*=0.5;
                    continue;
                }
                double f2,p2;
                const auto rr=static_residual(
                    model, base, sample, input.gravity_x, input.gravity_y,
                    input.gravity_z, active_tires, trial, f2, p2,
                    nullptr, nullptr, load_scale
                );
                const double merit =
                    p2/input.position_tolerance + f2/input.dynamics_tolerance;
                if (debug) {
                    std::fprintf(
                        stderr,
                        "静态调试: 线搜索=%d 步长=%.17g 试探力残差=%.17g "
                        "试探位形残差=%.17g 评价值=%.17g 基准=%.17g\n",
                        ls, scale, f2, p2, merit, base_merit
                    );
                }
                // Chrono 的增量静力分析允许载荷级开始时出现有限的残差
                // 增长，再由后续 Newton 步恢复；否则初始姿态离平衡点较远
                // 时，单纯要求每一步严格下降会把有效路径提前截断。
                const double growth_limit = newton_iteration == 0 ? 2.0 : 1.0;
                if(finite_vec(rr)&&merit<=growth_limit*base_merit){
                    x=std::move(trial);
                    accepted=true;
                    break;
                }
                scale*=0.5;
            }
            if(!accepted) {
                if (static_manifold_relaxation_step(
                        model, base, sample, input.gravity_x, input.gravity_y,
                        input.gravity_z, active_tires, r, x,
                        input.position_tolerance, load_scale
                    )) {
                    continue;
                }
                return false;
            }
            }
            if (!converged) {
                return false;
            }

        std::vector<double> dy(x.begin(),x.begin()+model.ndof);
        const State candidate=pose_candidate(base,model,dy);
        std::vector<int> next_active;
        std::vector<double> geometric_compression(model.tires.size(), 0.0);
        for (std::size_t i = 0; i < model.tires.size(); ++i) {
            const Tire& tire = model.tires[i];
            const Vec3 center =
                state_point(
                    candidate, tire_center_body(tire),
                    tire_center_local(tire)
                );
            const double road =
                (i < sample.road_z.size() ? sample.road_z[i] : 0.0)
                + road_profile_height(model, candidate, i);
            geometric_compression[i] = tire.radius + road - center.z;
            if (model.bodies[tire.body].fixed) continue;
            const auto existing =
                std::find(active_tires.begin(), active_tires.end(),
                          static_cast<int>(i));
            if (existing != active_tires.end()) {
                const std::size_t active_index = static_cast<std::size_t>(
                    std::distance(active_tires.begin(), existing)
                );
                const double compression =
                    x[static_cast<std::size_t>(model.ndof+model.rows)
                      +active_index];
                if (compression > contact_tolerance) {
                    if (compression >
                        tire.maximum_compression+input.position_tolerance) {
                        return false;
                    }
                    next_active.push_back(static_cast<int>(i));
                }
            } else if (geometric_compression[i] > contact_tolerance) {
                next_active.push_back(static_cast<int>(i));
            }
        }
            if (next_active == active_tires) {
            for (std::size_t i = 0; i < model.tires.size(); ++i) {
                if (std::find(
                        active_tires.begin(), active_tires.end(),
                        static_cast<int>(i)
                    ) == active_tires.end() &&
                    geometric_compression[i] > contact_tolerance) {
                    return false;
                }
            }
                load_converged = true;
                if (debug) {
                    std::fprintf(
                        stderr,
                        "静态调试: 载荷级=%d 活动集收敛\n",
                        load_step+1
                    );
                }
                break;
            }

        std::vector<double> next_x(
            static_cast<std::size_t>(model.ndof+model.rows)
                +next_active.size(),
            0.0
        );
        std::copy(
            x.begin(), x.begin()+model.ndof+model.rows, next_x.begin()
        );
        for (std::size_t i = 0; i < next_active.size(); ++i) {
            const int tire_index = next_active[i];
            const auto existing =
                std::find(active_tires.begin(), active_tires.end(), tire_index);
            next_x[static_cast<std::size_t>(model.ndof+model.rows)+i] =
                existing == active_tires.end()
                    ? std::max(
                        0.0,
                        geometric_compression[static_cast<std::size_t>(
                            tire_index
                        )]
                    )
                    : x[static_cast<std::size_t>(model.ndof+model.rows)
                        +static_cast<std::size_t>(
                            std::distance(active_tires.begin(), existing)
                        )];
        }
            active_tires=std::move(next_active);
            if (debug) {
                std::fprintf(
                    stderr,
                    "静态调试: 载荷级=%d 更新活动集数=%zu\n",
                    load_step+1, active_tires.size()
                );
            }
            x=std::move(next_x);
        }
        if (!load_converged) {
            return false;
        }
    }
    state=pose_candidate(
        base, model, std::vector<double>(x.begin(), x.begin()+model.ndof)
    );
    constraint_multiplier.assign(
        x.begin()+model.ndof,
        x.begin()+model.ndof+model.rows
    );
    return true;
}

Model build_model(
    const AxleInput& in, std::string& error,
    const double* axis_a_secondary = nullptr,
    const double* axis_b_secondary = nullptr,
    const double* convel_angle_target = nullptr,
    std::size_t coordinate_coupler_count = 0,
    const int* coordinate_coupler_joint_a = nullptr,
    const int* coordinate_coupler_coordinate_a = nullptr,
    const double* coordinate_coupler_scale_a = nullptr,
    const int* coordinate_coupler_joint_b = nullptr,
    const int* coordinate_coupler_coordinate_b = nullptr,
    const double* coordinate_coupler_scale_b = nullptr
) {
    Model m;
    if (!in.body_mass || !in.body_inertia_body_3x3 || !in.body_pose_position_quaternion ||
        in.body_count==0) { error="body arrays are missing"; return m; }
    m.bodies.resize(in.body_count);
    m.body_to_free.assign(in.body_count,-1);
    for (std::size_t i=0;i<in.body_count;++i) {
        Body& b=m.bodies[i];
        b.mass=in.body_mass[i];
        b.fixed=in.body_fixed && in.body_fixed[i]!=0;
        if (!(b.mass>0.0) && !b.fixed) { error="free body mass must be positive"; return m; }
        for(int r=0;r<3;++r)for(int c=0;c<3;++c)b.inertia_body.a[r][c]=in.body_inertia_body_3x3[(i*9)+r*3+c];
        if (!finite_symmetric(b.inertia_body) ||
            (!b.fixed && !symmetric_positive_definite(b.inertia_body))) {
            error="body inertia must be finite and symmetric; free-body inertia must be positive definite";
            return m;
        }
        const double* p=&in.body_pose_position_quaternion[i*7];
        const Quat body_quaternion{p[3],p[4],p[5],p[6]};
        if (!unit_quaternion(body_quaternion)) {
            error="body quaternion must be finite and unit length";
            return m;
        }
        b.r={p[0],p[1],p[2]}; b.q=qnormalize(body_quaternion);
        const double* vw=&in.body_velocity_omega[i*6];
        b.v={vw[0],vw[1],vw[2]}; b.omega={vw[3],vw[4],vw[5]};
        if (!std::isfinite(b.r.x) || !std::isfinite(b.r.y) || !std::isfinite(b.r.z) ||
            !std::isfinite(b.v.x) || !std::isfinite(b.v.y) || !std::isfinite(b.v.z) ||
            !std::isfinite(b.omega.x) || !std::isfinite(b.omega.y) || !std::isfinite(b.omega.z)) {
            error="body pose and velocity arrays must be finite";
            return m;
        }
    }
    for(std::size_t i=0;i<in.body_count;++i) if(!m.bodies[i].fixed){m.body_to_free[i]=static_cast<int>(m.free_body.size());m.free_body.push_back(static_cast<int>(i));}
    m.ndof=6*static_cast<int>(m.free_body.size());
    for(std::size_t i=0;i<in.constraint_count;++i){
        Constraint c;
        c.type=in.constraint_type[i]; c.a=in.constraint_body_a[i]; c.b=in.constraint_body_b[i];
        if(c.a<0||c.b<0||c.a>=static_cast<int>(in.body_count)||c.b>=static_cast<int>(in.body_count)){error="constraint body index out of range";return m;}
        c.pa={in.constraint_point_a[i*3],in.constraint_point_a[i*3+1],in.constraint_point_a[i*3+2]};
        c.pb={in.constraint_point_b[i*3],in.constraint_point_b[i*3+1],in.constraint_point_b[i*3+2]};
        c.axis_a={in.constraint_axis_a[i*3],in.constraint_axis_a[i*3+1],in.constraint_axis_a[i*3+2]};
        c.axis_b={in.constraint_axis_b[i*3],in.constraint_axis_b[i*3+1],in.constraint_axis_b[i*3+2]};
        if (axis_a_secondary != nullptr) {
            c.axis_a_secondary={
                axis_a_secondary[i*3], axis_a_secondary[i*3+1],
                axis_a_secondary[i*3+2]
            };
        }
        if (axis_b_secondary != nullptr) {
            c.axis_b_secondary={
                axis_b_secondary[i*3], axis_b_secondary[i*3+1],
                axis_b_secondary[i*3+2]
            };
        }
        if (convel_angle_target != nullptr) {
            c.convel_angle_target = convel_angle_target[i];
            if (!std::isfinite(c.convel_angle_target) ||
                std::abs(c.convel_angle_target) > 2.0) {
                error="constant-velocity angle target must be finite and in [-2,2]";
                return m;
            }
        }
        if(norm(c.axis_a)<kEps||norm(c.axis_b)<kEps){error="constraint axis must be nonzero";return m;}
        if (c.type == AXLE_CONVEL &&
            (norm(c.axis_a_secondary) < kEps || norm(c.axis_b_secondary) < kEps)) {
            error="constant-velocity secondary axis must be nonzero";
            return m;
        }
        c.row=m.rows; const int rows=constraint_rows(c.type); if(rows<0){error="unsupported constraint type";return m;}
        m.rows+=rows; m.constraints.push_back(c);
    }
    if (coordinate_coupler_count > 0 && (
        coordinate_coupler_joint_a == nullptr ||
        coordinate_coupler_coordinate_a == nullptr ||
        coordinate_coupler_scale_a == nullptr ||
        coordinate_coupler_joint_b == nullptr ||
        coordinate_coupler_coordinate_b == nullptr ||
        coordinate_coupler_scale_b == nullptr
    )) {
        error = "vehicle coordinate coupler arrays are missing";
        return m;
    }
    for (std::size_t i = 0; i < coordinate_coupler_count; ++i) {
        CoordinateCoupler coupler;
        coupler.joint_a = coordinate_coupler_joint_a[i];
        coupler.coordinate_a = coordinate_coupler_coordinate_a[i];
        coupler.scale_a = coordinate_coupler_scale_a[i];
        coupler.joint_b = coordinate_coupler_joint_b[i];
        coupler.coordinate_b = coordinate_coupler_coordinate_b[i];
        coupler.scale_b = coordinate_coupler_scale_b[i];
        if (coupler.joint_a < 0 ||
            coupler.joint_b < 0 ||
            coupler.joint_a >= static_cast<int>(m.constraints.size()) ||
            coupler.joint_b >= static_cast<int>(m.constraints.size()) ||
            coupler.joint_a == coupler.joint_b ||
            (coupler.coordinate_a != 0 && coupler.coordinate_a != 1) ||
            (coupler.coordinate_b != 0 && coupler.coordinate_b != 1) ||
            !std::isfinite(coupler.scale_a) ||
            !std::isfinite(coupler.scale_b) ||
            std::abs(coupler.scale_a) <= kEps ||
            std::abs(coupler.scale_b) <= kEps) {
            error = "invalid vehicle coordinate coupler";
            return m;
        }
        const Constraint& first = m.constraints[
            static_cast<std::size_t>(coupler.joint_a)
        ];
        const Constraint& second = m.constraints[
            static_cast<std::size_t>(coupler.joint_b)
        ];
        const auto coordinate_supported = [](const Constraint& joint,
                                             int coordinate) {
            if (coordinate == 0) {
                return joint.type == AXLE_REVOLUTE ||
                    joint.type == AXLE_CYLINDRICAL;
            }
            return joint.type == AXLE_PRISMATIC ||
                joint.type == AXLE_CYLINDRICAL;
        };
        if (!coordinate_supported(first, coupler.coordinate_a) ||
            !coordinate_supported(second, coupler.coordinate_b)) {
            error = "vehicle coordinate coupler uses an incompatible joint coordinate";
            return m;
        }
        const auto coordinate_reference = [&](const Constraint& joint,
                                              int coordinate,
                                              Quat& rotation) {
            if (coordinate == 0) {
                rotation = qmul(
                    qconj(m.bodies[joint.a].q), m.bodies[joint.b].q
                );
                return 0.0;
            }
            const Vec3 axis = normalized(
                rotate(m.bodies[joint.a].q, joint.axis_a)
            );
            const Vec3 point_a = m.bodies[joint.a].r
                + rotate(m.bodies[joint.a].q, joint.pa);
            const Vec3 point_b = m.bodies[joint.b].r
                + rotate(m.bodies[joint.b].q, joint.pb);
            return dot(axis, point_a-point_b);
        };
        coupler.reference_translation_a = coordinate_reference(
            first, coupler.coordinate_a, coupler.reference_rotation_a
        );
        coupler.reference_translation_b = coordinate_reference(
            second, coupler.coordinate_b, coupler.reference_rotation_b
        );
        m.rows += 1;
        coupler.row = m.rows - 1;
        m.coordinate_couplers.push_back(coupler);
    }
    for(std::size_t i=0;i<in.spring_count;++i){
        Spring s; s.a=in.spring_body_a[i]; s.b=in.spring_body_b[i];
        s.pa={in.spring_point_a[i*3],in.spring_point_a[i*3+1],in.spring_point_a[i*3+2]};
        s.pb={in.spring_point_b[i*3],in.spring_point_b[i*3+1],in.spring_point_b[i*3+2]};
        s.k=in.spring_stiffness[i];
        s.c_compression=in.spring_compression_damping[i];
        s.c_rebound=in.spring_rebound_damping[i];
        s.free_length=in.spring_free_length[i];
        s.minimum_length=in.spring_minimum_length[i];
        s.maximum_length=in.spring_maximum_length[i];
        s.compression_stop_k=in.spring_compression_stop_stiffness[i];
        s.compression_stop_c=in.spring_compression_stop_damping[i];
        s.rebound_stop_k=in.spring_rebound_stop_stiffness[i];
        s.rebound_stop_c=in.spring_rebound_stop_damping[i];
        if (in.spring_damper_curve_count && in.spring_damper_curve_offset &&
            in.spring_damper_curve_velocity && in.spring_damper_curve_force) {
            const int count = in.spring_damper_curve_count[i];
            const int offset = in.spring_damper_curve_offset[i];
            if (count < 0 || offset < 0) {
                error="invalid damper curve range";
                return m;
            }
            if (count == 1) {
                error="a damper curve needs at least two points";
                return m;
            }
            for (int p = 0; p < count; ++p) {
                const double velocity =
                    in.spring_damper_curve_velocity[offset+p];
                const double force = in.spring_damper_curve_force[offset+p];
                if (!std::isfinite(velocity) || !std::isfinite(force)) {
                    error="damper curve must be finite";
                    return m;
                }
                // Strictly increasing in velocity keeps the interpolation
                // single-valued; a non-monotonic force is allowed because real
                // shocks are not monotonic near the blow-off point.
                if (p > 0 && velocity <= s.damper_velocity.back()) {
                    error="damper curve velocity must strictly increase";
                    return m;
                }
                s.damper_velocity.push_back(velocity);
                s.damper_force.push_back(force);
            }
        }
        if(s.a<0||s.b<0||s.a>=static_cast<int>(in.body_count)||s.b>=static_cast<int>(in.body_count)||
           s.k<0||s.c_compression<0||s.c_rebound<0||s.free_length<0||
           s.compression_stop_k<0||s.compression_stop_c<0||s.rebound_stop_k<0||s.rebound_stop_c<0||
           (std::isfinite(s.minimum_length) && s.minimum_length<0)||
           (std::isfinite(s.maximum_length) && s.maximum_length<0)||
           (std::isfinite(s.minimum_length) && std::isfinite(s.maximum_length) &&
            s.minimum_length>=s.maximum_length)){
            error="invalid spring or stop parameters";
            return m;
        }
        m.springs.push_back(s);
    }
    for(std::size_t i=0;i<in.bushing_count;++i){
        Bushing b;
        b.a=in.bushing_body_a[i]; b.b=in.bushing_body_b[i];
        b.pa={in.bushing_point_a[i*3],in.bushing_point_a[i*3+1],in.bushing_point_a[i*3+2]};
        b.pb={in.bushing_point_b[i*3],in.bushing_point_b[i*3+1],in.bushing_point_b[i*3+2]};
        const double* qa=&in.bushing_frame_a_quaternion[i*4];
        const double* qb=&in.bushing_frame_b_quaternion[i*4];
        const double* qr=&in.bushing_reference_quaternion[i*4];
        const Quat frame_a{qa[0],qa[1],qa[2],qa[3]};
        const Quat frame_b{qb[0],qb[1],qb[2],qb[3]};
        const Quat reference{qr[0],qr[1],qr[2],qr[3]};
        if (!unit_quaternion(frame_a) || !unit_quaternion(frame_b) ||
            !unit_quaternion(reference)) {
            error="bushing quaternions must be finite and unit length";
            return m;
        }
        b.frame_a=qnormalize(frame_a);
        b.frame_b=qnormalize(frame_b);
        b.reference=qnormalize(reference);
        b.reference_translation={
            in.bushing_reference_translation[i*3],
            in.bushing_reference_translation[i*3+1],
            in.bushing_reference_translation[i*3+2]
        };
        for(int j=0;j<36;++j){
            b.stiffness[static_cast<std::size_t>(j)]=in.bushing_stiffness_6x6[i*36+j];
            b.damping[static_cast<std::size_t>(j)]=in.bushing_damping_6x6[i*36+j];
        }
        for(int j=0;j<6;++j) b.preload[static_cast<std::size_t>(j)]=in.bushing_preload_6[i*6+j];
        if(b.a<0||b.b<0||b.a>=static_cast<int>(in.body_count)||b.b>=static_cast<int>(in.body_count)){
            error="invalid bushing body index"; return m;
        }
        for(int r=0;r<6;++r) for(int c=0;c<6;++c){
            const double ks=b.stiffness[static_cast<std::size_t>(r*6+c)];
            const double cs=b.damping[static_cast<std::size_t>(r*6+c)];
            if(!std::isfinite(ks)||!std::isfinite(cs)){error="bushing matrices must be finite";return m;}
            if(std::abs(ks-b.stiffness[static_cast<std::size_t>(c*6+r)])>1e-10||
               std::abs(cs-b.damping[static_cast<std::size_t>(c*6+r)])>1e-10){
                error="bushing matrices must be symmetric"; return m;
            }
        }
        m.bushings.push_back(b);
    }
    for(std::size_t i=0;i<in.anti_roll_bar_count;++i){
        AntiRollBar bar;
        bar.a=in.anti_roll_body_a[i]; bar.b=in.anti_roll_body_b[i];
        bar.axis_a={
            in.anti_roll_axis_a[i*3],
            in.anti_roll_axis_a[i*3+1],
            in.anti_roll_axis_a[i*3+2]
        };
        const double* qr=&in.anti_roll_reference_quaternion[i*4];
        const Quat reference{qr[0],qr[1],qr[2],qr[3]};
        if (!unit_quaternion(reference)) {
            error="anti-roll reference quaternion must be finite and unit length";
            return m;
        }
        bar.reference=qnormalize(reference);
        bar.stiffness=in.anti_roll_stiffness[i];
        bar.damping=in.anti_roll_damping[i];
        if(bar.a<0||bar.b<0||bar.a>=static_cast<int>(in.body_count)||bar.b>=static_cast<int>(in.body_count)||
           norm(bar.axis_a)<kEps||bar.stiffness<0||bar.damping<0){
            error="invalid anti-roll bar"; return m;
        }
        bar.axis_a=normalized(bar.axis_a);
        m.anti_roll_bars.push_back(bar);
    }
    for(std::size_t i=0;i<in.tire_count;++i){
        Tire t;
        t.body=in.tire_body[i];
        t.center={in.tire_center_local[i*3],in.tire_center_local[i*3+1],in.tire_center_local[i*3+2]};
        t.frame_body=t.body;
        t.frame_center=t.center;
        t.spin_axis={
            in.tire_spin_axis_local[i*3],
            in.tire_spin_axis_local[i*3+1],
            in.tire_spin_axis_local[i*3+2]
        };
        t.forward_axis={
            in.tire_forward_axis_local[i*3],
            in.tire_forward_axis_local[i*3+1],
            in.tire_forward_axis_local[i*3+2]
        };
        t.radius=in.tire_radius[i];
        t.maximum_compression=in.tire_maximum_compression[i];
        t.k=in.tire_stiffness[i];
        t.c=in.tire_damping[i];
        t.mu_longitudinal=in.tire_mu_longitudinal[i];
        t.mu_lateral=in.tire_mu_lateral[i];
        t.brush_k_longitudinal=in.tire_brush_stiffness_longitudinal[i];
        t.brush_k_lateral=in.tire_brush_stiffness_lateral[i];
        t.relaxation_length_longitudinal=
            in.tire_relaxation_length_longitudinal[i];
        t.relaxation_length_lateral=
            in.tire_relaxation_length_lateral[i];
        t.detached_relaxation=in.tire_detached_relaxation[i];
        if(t.body<0||t.body>=static_cast<int>(in.body_count)||t.radius<=0||
           t.maximum_compression<=0||t.maximum_compression>=t.radius||
           t.k<0||t.c<0||t.mu_longitudinal<=0||t.mu_lateral<=0||
           norm(t.spin_axis)<kEps||norm(t.forward_axis)<kEps||
           norm(cross(t.spin_axis,t.forward_axis))<kEps||
           t.brush_k_longitudinal<=0||t.brush_k_lateral<=0||
           t.relaxation_length_longitudinal<=0||
           t.relaxation_length_lateral<=0||t.detached_relaxation<=0){
            error="invalid tire parameters";return m;
        }
        t.spin_axis=normalized(t.spin_axis);
        t.forward_axis=normalized(t.forward_axis);
        m.tires.push_back(t);
    }
    if (m.rows > 0) {
        State audit_state;
        audit_state.r.resize(m.bodies.size());
        audit_state.q.resize(m.bodies.size());
        audit_state.v.resize(m.bodies.size());
        audit_state.omega.resize(m.bodies.size());
        audit_state.a.resize(m.bodies.size());
        audit_state.alpha.resize(m.bodies.size());
        audit_state.tire_sx.assign(m.tires.size(), 0.0);
        audit_state.tire_sy.assign(m.tires.size(), 0.0);
        audit_state.tire_sx_dot.assign(m.tires.size(), 0.0);
        audit_state.tire_sy_dot.assign(m.tires.size(), 0.0);
        for (std::size_t i = 0; i < m.bodies.size(); ++i) {
            audit_state.r[i] = m.bodies[i].r;
            audit_state.q[i] = m.bodies[i].q;
            audit_state.v[i] = m.bodies[i].v;
            audit_state.omega[i] = m.bodies[i].omega;
        }
        const auto jacobian = constraint_jacobian(m, audit_state);
        if (matrix_rank(jacobian, m.rows, m.ndof) < m.rows) {
            error="constraint Jacobian is rank deficient at the initial pose";
            return m;
        }
        double jacobian_error = 0.0;
        if (!analytic_constraint_jacobian_matches_reference(
                m, audit_state, jacobian_error
            )) {
            error =
                "analytic constraint Jacobian disagrees with central "
                "differences by " + std::to_string(jacobian_error);
            return m;
        }
    }
    return m;
}

bool add_vehicle_steering_actuators(
    const VehicleInput& input, Model& model, std::string& error
) {
    const std::size_t count = input.steering_count;
    if (count == 0) return true;
    if (!input.steering_type || !input.steering_body ||
        !input.steering_reaction_body || !input.steering_point_local ||
        !input.steering_reaction_point_local || !input.steering_axis_local ||
        !input.steering_reference_quaternion ||
        !input.steering_target_angle || !input.steering_target_rate ||
        !input.steering_stiffness || !input.steering_damping) {
        error = "vehicle steering arrays are missing";
        return false;
    }
    if (input.axle.sample_count == 0 ||
        count > std::numeric_limits<std::size_t>::max()
            / input.axle.sample_count) {
        error = "vehicle steering sample count overflows";
        return false;
    }
    const std::size_t target_size = input.axle.sample_count * count;
    for (std::size_t i = 0; i < target_size; ++i) {
        if (!std::isfinite(input.steering_target_angle[i]) ||
            !std::isfinite(input.steering_target_rate[i])) {
            error = "vehicle steering targets must be finite";
            return false;
        }
    }
    model.steering_actuators.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        SteeringActuator actuator;
        actuator.type = input.steering_type[i];
        actuator.body = input.steering_body[i];
        actuator.reaction_body = input.steering_reaction_body[i];
        actuator.point_local = {
            input.steering_point_local[i*3],
            input.steering_point_local[i*3+1],
            input.steering_point_local[i*3+2]
        };
        actuator.reaction_point_local = {
            input.steering_reaction_point_local[i*3],
            input.steering_reaction_point_local[i*3+1],
            input.steering_reaction_point_local[i*3+2]
        };
        if (actuator.body < 0 ||
            actuator.body >= static_cast<int>(model.bodies.size()) ||
            model.bodies[actuator.body].fixed ||
            actuator.reaction_body < -1 ||
            actuator.reaction_body >= static_cast<int>(model.bodies.size()) ||
            actuator.reaction_body == actuator.body) {
            error = "invalid vehicle steering body index";
            return false;
        }
        actuator.axis_local = {
            input.steering_axis_local[i*3],
            input.steering_axis_local[i*3+1],
            input.steering_axis_local[i*3+2]
        };
        const double* reference =
            &input.steering_reference_quaternion[i*4];
        const Quat reference_quaternion{
            reference[0], reference[1], reference[2], reference[3]
        };
        actuator.reference = reference_quaternion;
        actuator.stiffness = input.steering_stiffness[i];
        actuator.damping = input.steering_damping[i];
        if ((actuator.type != VEHICLE_STEERING_TRANSLATION &&
             actuator.type != VEHICLE_STEERING_ROTATION &&
             actuator.type != VEHICLE_STEERING_PRESCRIBED_ROTATION &&
             actuator.type != VEHICLE_STEERING_PRESCRIBED_TRANSLATION) ||
            !std::isfinite(actuator.point_local.x) ||
            !std::isfinite(actuator.point_local.y) ||
            !std::isfinite(actuator.point_local.z) ||
            !std::isfinite(actuator.reaction_point_local.x) ||
            !std::isfinite(actuator.reaction_point_local.y) ||
            !std::isfinite(actuator.reaction_point_local.z) ||
            norm(actuator.axis_local) <= kEps ||
            !unit_quaternion(reference_quaternion) ||
            !std::isfinite(actuator.stiffness) ||
            !std::isfinite(actuator.damping) ||
            actuator.stiffness <= 0.0 || actuator.damping < 0.0) {
            error = "invalid vehicle steering actuator parameters";
            return false;
        }
        actuator.axis_local = normalized(actuator.axis_local);
        actuator.reference = qnormalize(actuator.reference);
        actuator.target_angle = input.steering_target_angle;
        actuator.target_rate = input.steering_target_rate;
        if (prescribed_steering(actuator)) {
            actuator.constraint_row = model.rows;
            model.rows += 1;
        }
        model.steering_actuators.push_back(actuator);
    }
    return true;
}

bool add_vehicle_aerodynamic_drags(
    const VehicleInput& input, Model& model, std::string& error
) {
    const std::size_t count = input.aerodynamic_drag_count;
    if (count == 0) return true;
    if (!input.aerodynamic_drag_body ||
        !input.aerodynamic_drag_application_point ||
        !input.aerodynamic_drag_forward_axis ||
        !input.aerodynamic_drag_coefficient) {
        error = "vehicle aerodynamic drag arrays are missing";
        return false;
    }
    model.aerodynamic_drags.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        AerodynamicDrag drag;
        drag.body = input.aerodynamic_drag_body[i];
        drag.application_point = {
            input.aerodynamic_drag_application_point[i*3],
            input.aerodynamic_drag_application_point[i*3+1],
            input.aerodynamic_drag_application_point[i*3+2]
        };
        drag.forward_axis = {
            input.aerodynamic_drag_forward_axis[i*3],
            input.aerodynamic_drag_forward_axis[i*3+1],
            input.aerodynamic_drag_forward_axis[i*3+2]
        };
        drag.coefficient = input.aerodynamic_drag_coefficient[i];
        const double axis_norm = norm(drag.forward_axis);
        if (drag.body < 0 ||
            drag.body >= static_cast<int>(model.bodies.size()) ||
            model.bodies[drag.body].fixed ||
            !std::isfinite(drag.application_point.x) ||
            !std::isfinite(drag.application_point.y) ||
            !std::isfinite(drag.application_point.z) ||
            !std::isfinite(axis_norm) || axis_norm <= 1e-12 ||
            !std::isfinite(drag.coefficient) || drag.coefficient < 0.0) {
            error = "vehicle aerodynamic drag input is invalid";
            return false;
        }
        drag.forward_axis = drag.forward_axis / axis_norm;
        model.aerodynamic_drags.push_back(drag);
    }
    return true;
}

bool add_vehicle_road_profile(
    const VehicleInput& input, Model& model, std::string& error
) {
    if (input.road_kind < 0 || input.road_kind > 5) {
        error = "invalid vehicle road profile kind";
        return false;
    }
    if (input.road_kind == 0) {
        model.road_profile = RoadProfile{};
        return true;
    }
    if (!std::isfinite(input.road_origin_x) ||
        !std::isfinite(input.road_origin_z) ||
        !std::isfinite(input.road_amplitude) ||
        !std::isfinite(input.road_wavelength) ||
        !std::isfinite(input.road_phase) ||
        !std::isfinite(input.road_bump_start) ||
        !std::isfinite(input.road_bump_length) ||
        input.road_amplitude < 0.0 || input.road_wavelength <= 0.0 ||
        input.road_bump_length <= 0.0) {
        error = "vehicle road profile parameters are invalid";
        return false;
    }
    if (!input.road_corner_scale) {
        error = "vehicle road corner scales are missing";
        return false;
    }
    RoadProfile profile;
    profile.kind = input.road_kind;
    profile.origin_x = input.road_origin_x;
    profile.origin_z = input.road_origin_z;
    profile.amplitude = input.road_amplitude;
    profile.wavelength = input.road_wavelength;
    profile.phase = input.road_phase;
    profile.bump_start = input.road_bump_start;
    profile.bump_length = input.road_bump_length;
    for (std::size_t i = 0; i < profile.corner_scale.size(); ++i) {
        const double value = input.road_corner_scale[i];
        if (!std::isfinite(value) || value < 0.0) {
            error = "vehicle road corner scales must be finite and non-negative";
            return false;
        }
        profile.corner_scale[i] = value;
    }
    model.road_profile = profile;
    return true;
}

bool add_vehicle_static_rotation_gauges(
    const VehicleInput& input, Model& model, std::string& error
) {
    const std::size_t count_end =
        offsetof(VehicleInput, static_rotation_gauge_count)
        + sizeof(input.static_rotation_gauge_count);
    const std::size_t body_end =
        offsetof(VehicleInput, static_rotation_gauge_body)
        + sizeof(input.static_rotation_gauge_body);
    const std::size_t axis_end =
        offsetof(VehicleInput, static_rotation_gauge_axis_local)
        + sizeof(input.static_rotation_gauge_axis_local);
    if (input.struct_size < count_end) return true;
    if (input.struct_size < axis_end ||
        input.static_rotation_gauge_count > model.bodies.size() ||
        (input.static_rotation_gauge_count > 0 &&
         (input.static_rotation_gauge_body == nullptr ||
          input.static_rotation_gauge_axis_local == nullptr))) {
        error = "vehicle static rotation gauge arrays are incomplete";
        return false;
    }
    if (input.struct_size < body_end) {
        error = "vehicle static rotation gauge body array is incomplete";
        return false;
    }
    for (std::size_t i = 0; i < input.static_rotation_gauge_count; ++i) {
        const int body = input.static_rotation_gauge_body[i];
        if (body < 0 || body >= static_cast<int>(model.bodies.size()) ||
            model.bodies[static_cast<std::size_t>(body)].fixed ||
            model.body_to_free[static_cast<std::size_t>(body)] < 0) {
            error = "invalid vehicle static rotation gauge body";
            return false;
        }
        for (const auto& existing : model.static_rotation_gauges) {
            if (existing.body == body) {
                error = "duplicate vehicle static rotation gauge body";
                return false;
            }
        }
        const Vec3 axis_local{
            input.static_rotation_gauge_axis_local[3*i],
            input.static_rotation_gauge_axis_local[3*i+1],
            input.static_rotation_gauge_axis_local[3*i+2]
        };
        if (!std::isfinite(axis_local.x) || !std::isfinite(axis_local.y) ||
            !std::isfinite(axis_local.z) || norm(axis_local) <= kEps) {
            error = "vehicle static rotation gauge axis must be finite and nonzero";
            return false;
        }
        const Vec3 axis_world = normalized(
            rotate(model.bodies[static_cast<std::size_t>(body)].q, axis_local)
        );
        const int free_body = model.body_to_free[static_cast<std::size_t>(body)];
        const int axis_component =
            std::abs(axis_world.x) >= std::abs(axis_world.y) &&
            std::abs(axis_world.x) >= std::abs(axis_world.z) ? 0 :
            (std::abs(axis_world.y) >= std::abs(axis_world.z) ? 1 : 2);
        const int pivot = 6*free_body+3+axis_component;
        const bool collides_with_global =
            model.static_gauge_body == body &&
            (model.static_gauge_dof_mask &
             (static_cast<std::uint32_t>(1) << (3+axis_component))) != 0;
        if (collides_with_global || static_rotation_gauge_for_pivot(model, pivot)) {
            error = "vehicle static rotation gauge conflicts with another static gauge";
            return false;
        }
        model.static_rotation_gauges.push_back({body, axis_world, pivot});
    }
    return true;
}

bool add_vehicle_tire_frames(
    const VehicleInput& input, Model& model, std::string& error
) {
    const std::size_t frame_body_end =
        offsetof(VehicleInput, tire_frame_body)
        + sizeof(input.tire_frame_body);
    const std::size_t frame_center_end =
        offsetof(VehicleInput, tire_frame_center_local)
        + sizeof(input.tire_frame_center_local);
    if (input.struct_size < frame_body_end) return true;
    if (input.struct_size < frame_center_end ||
        input.tire_frame_body == nullptr ||
        input.tire_frame_center_local == nullptr) {
        error = "vehicle tire frame arrays are incomplete";
        return false;
    }
    if (model.tires.size() != input.axle.tire_count) {
        error = "vehicle tire frame count does not match tire count";
        return false;
    }
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const int frame_body = input.tire_frame_body[i];
        const Vec3 frame_center{
            input.tire_frame_center_local[i*3],
            input.tire_frame_center_local[i*3+1],
            input.tire_frame_center_local[i*3+2]
        };
        if (frame_body < 0 ||
            frame_body >= static_cast<int>(model.bodies.size()) ||
            !std::isfinite(frame_center.x) ||
            !std::isfinite(frame_center.y) ||
            !std::isfinite(frame_center.z)) {
            error = "invalid vehicle tire frame mapping";
            return false;
        }
        model.tires[i].frame_body = frame_body;
        model.tires[i].frame_center = frame_center;
    }
    return true;
}

bool add_vehicle_tire_models(
    const VehicleInput& input, Model& model, std::string& error
) {
    const std::size_t kind_end =
        offsetof(VehicleInput, tire_model_kind)
        + sizeof(input.tire_model_kind);
    const std::size_t parameter_end =
        offsetof(VehicleInput, tire_pac2002_parameters)
        + sizeof(input.tire_pac2002_parameters);
    const std::size_t mirror_end =
        offsetof(VehicleInput, tire_pac2002_mirror)
        + sizeof(input.tire_pac2002_mirror);
    if (input.struct_size < kind_end) return true;
    if (input.struct_size < parameter_end ||
        input.struct_size < mirror_end ||
        input.tire_model_kind == nullptr ||
        input.tire_pac2002_parameters == nullptr ||
        input.tire_pac2002_mirror == nullptr) {
        error = "vehicle tire model arrays are incomplete";
        return false;
    }
    if (model.tires.size() != input.axle.tire_count) {
        error = "vehicle tire model count does not match tire count";
        return false;
    }
    const std::size_t parameter_count =
        static_cast<std::size_t>(VEHICLE_PAC2002_PARAMETER_COUNT);
    if (model.tires.size() > std::numeric_limits<std::size_t>::max()
        / parameter_count) {
        error = "vehicle tire parameter count overflows";
        return false;
    }
    const std::size_t value_count = model.tires.size()*parameter_count;
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const int kind = input.tire_model_kind[i];
        if (kind != VEHICLE_TIRE_NATIVE_BRUSH &&
            kind != VEHICLE_TIRE_PAC2002_PURE_SLIP &&
            kind != VEHICLE_TIRE_PAC2002_ADAMS_SOURCE &&
            kind != VEHICLE_TIRE_FIALA) {
            error = "unsupported vehicle tire model kind";
            return false;
        }
        model.tires[i].model_kind = kind;
        for (std::size_t j = 0; j < parameter_count; ++j) {
            const double value = input.tire_pac2002_parameters[
                i*parameter_count+j
            ];
            if (!std::isfinite(value)) {
                error = "vehicle PAC2002 parameters must be finite";
                return false;
            }
            model.tires[i].pac2002_parameters[j] = value;
        }
        if (kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE &&
            input.tire_pac2002_mirror[i] != 0) {
            for (const int index : {
                     PAC_RHX1, PAC_QSX1, PAC_PEY3, PAC_PHY1, PAC_PHY2,
                     PAC_PVY1, PAC_PVY2, PAC_RBY3, PAC_RVY1, PAC_RVY2,
                     PAC_QBZ4, PAC_QDZ3, PAC_QDZ6, PAC_QDZ7, PAC_QEZ4,
                     PAC_QHZ1, PAC_QHZ2, PAC_SSZ1
                 }) {
                model.tires[i].pac2002_parameters[
                    static_cast<std::size_t>(index)
                ] *= -1.0;
            }
        }
    }
    (void)value_count;
    return true;
}

bool add_vehicle_drive_torque_mappings(
    const VehicleInput& input, Model& model, std::string& error
) {
    const std::size_t body_end =
        offsetof(VehicleInput, tire_drive_torque_body)
        + sizeof(input.tire_drive_torque_body);
    const std::size_t reaction_end =
        offsetof(VehicleInput, tire_drive_torque_reaction_body)
        + sizeof(input.tire_drive_torque_reaction_body);
    const std::size_t axis_end =
        offsetof(VehicleInput, tire_drive_torque_axis_local)
        + sizeof(input.tire_drive_torque_axis_local);
    if (input.struct_size < body_end) return true;
    if (input.struct_size < reaction_end || input.struct_size < axis_end ||
        input.tire_drive_torque_body == nullptr ||
        input.tire_drive_torque_reaction_body == nullptr ||
        input.tire_drive_torque_axis_local == nullptr) {
        error = "vehicle drive torque mapping arrays are incomplete";
        return false;
    }
    if (model.tires.size() != input.axle.tire_count) {
        error = "vehicle drive torque mapping count does not match tire count";
        return false;
    }
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const int body = input.tire_drive_torque_body[i];
        const int reaction = input.tire_drive_torque_reaction_body[i];
        Vec3 axis{
            input.tire_drive_torque_axis_local[3*i],
            input.tire_drive_torque_axis_local[3*i+1],
            input.tire_drive_torque_axis_local[3*i+2]
        };
        const double axis_norm = norm(axis);
        if (body == -1) {
            if (reaction != -1 || !std::isfinite(axis_norm) ||
                axis_norm > kEps) {
                error = "invalid default vehicle drive torque mapping";
                return false;
            }
            continue;
        }
        if (body < 0 || body >= static_cast<int>(model.bodies.size()) ||
            model.bodies[body].fixed || reaction < -1 ||
            reaction >= static_cast<int>(model.bodies.size()) ||
            reaction == body || !std::isfinite(axis_norm) ||
            axis_norm <= kEps) {
            error = "invalid vehicle drive torque mapping";
            return false;
        }
        model.tires[i].drive_torque_body = body;
        model.tires[i].drive_torque_reaction_body = reaction;
        model.tires[i].drive_torque_axis = axis/axis_norm;
    }
    return true;
}

bool add_vehicle_spring_curves(
    const VehicleInput& input, Model& model, std::string& error
) {
    const std::size_t first_end =
        offsetof(VehicleInput, vehicle_spring_elastic_curve_offset)
        + sizeof(input.vehicle_spring_elastic_curve_offset);
    const std::size_t last_end =
        offsetof(VehicleInput, vehicle_spring_rebound_stop_curve_force)
        + sizeof(input.vehicle_spring_rebound_stop_curve_force);
    const std::size_t interpolation_end =
        offsetof(VehicleInput, bushing_force_curve_interpolation)
        + sizeof(input.bushing_force_curve_interpolation);
    if (input.struct_size < first_end) return true;
    if (input.struct_size < last_end || input.struct_size < interpolation_end ||
        input.vehicle_spring_elastic_curve_offset == nullptr ||
        input.vehicle_spring_elastic_curve_count == nullptr ||
        input.vehicle_spring_elastic_curve_deflection == nullptr ||
        input.vehicle_spring_elastic_curve_force == nullptr ||
        input.vehicle_spring_compression_stop_curve_offset == nullptr ||
        input.vehicle_spring_compression_stop_curve_count == nullptr ||
        input.vehicle_spring_compression_stop_curve_penetration == nullptr ||
        input.vehicle_spring_compression_stop_curve_force == nullptr ||
        input.vehicle_spring_rebound_stop_curve_offset == nullptr ||
        input.vehicle_spring_rebound_stop_curve_count == nullptr ||
        input.vehicle_spring_rebound_stop_curve_penetration == nullptr ||
        input.vehicle_spring_rebound_stop_curve_force == nullptr) {
        error = "vehicle spring curve arrays are incomplete";
        return false;
    }
    if (model.springs.size() != input.axle.spring_count) {
        error = "vehicle spring curve count does not match spring count";
        return false;
    }

    const auto copy_curve = [&error](
        int offset, int count, const double* abscissa, const double* force,
        std::vector<double>& target_abscissa,
        std::vector<double>& target_force,
        const char* label
    ) -> bool {
        if (count < 0 || offset < 0 ||
            count > std::numeric_limits<int>::max()-offset) {
            error = std::string("invalid ") + label + " curve range";
            return false;
        }
        if (count == 1) {
            error = std::string(label) + " curve needs at least two points";
            return false;
        }
        target_abscissa.clear();
        target_force.clear();
        target_abscissa.reserve(static_cast<std::size_t>(count));
        target_force.reserve(static_cast<std::size_t>(count));
        double previous = 0.0;
        for (int point = 0; point < count; ++point) {
            const double x = abscissa[offset+point];
            const double y = force[offset+point];
            if (!std::isfinite(x) || !std::isfinite(y) ||
                (point > 0 && x <= previous)) {
                error = std::string(label) +
                    " curve must be finite and strictly increasing";
                return false;
            }
            target_abscissa.push_back(x);
            target_force.push_back(y);
            previous = x;
        }
        return true;
    };

    for (std::size_t i = 0; i < model.springs.size(); ++i) {
        Spring& spring = model.springs[i];
        if (!copy_curve(
                input.vehicle_spring_elastic_curve_offset[i],
                input.vehicle_spring_elastic_curve_count[i],
                input.vehicle_spring_elastic_curve_deflection,
                input.vehicle_spring_elastic_curve_force,
                spring.elastic_deflection, spring.elastic_force, "elastic")) {
            return false;
        }
        if (!copy_curve(
                input.vehicle_spring_compression_stop_curve_offset[i],
                input.vehicle_spring_compression_stop_curve_count[i],
                input.vehicle_spring_compression_stop_curve_penetration,
                input.vehicle_spring_compression_stop_curve_force,
                spring.compression_stop_penetration,
                spring.compression_stop_force,
                "compression stop")) {
            return false;
        }
        if (!copy_curve(
                input.vehicle_spring_rebound_stop_curve_offset[i],
                input.vehicle_spring_rebound_stop_curve_count[i],
                input.vehicle_spring_rebound_stop_curve_penetration,
                input.vehicle_spring_rebound_stop_curve_force,
                spring.rebound_stop_penetration,
                spring.rebound_stop_force,
                "rebound stop")) {
            return false;
        }
    }
    return true;
}

bool add_vehicle_bushing_curves(
    const VehicleInput& input, Model& model, std::string& error
) {
    const std::size_t first_end =
        offsetof(VehicleInput, vehicle_bushing_force_curve_offset)
        + sizeof(input.vehicle_bushing_force_curve_offset);
    const std::size_t last_end =
        offsetof(VehicleInput, vehicle_bushing_force_curve_force)
        + sizeof(input.vehicle_bushing_force_curve_force);
    if (input.struct_size < first_end) return true;
    if (input.struct_size < last_end ||
        input.vehicle_bushing_force_curve_offset == nullptr ||
        input.vehicle_bushing_force_curve_count == nullptr ||
        input.vehicle_bushing_force_curve_coordinate == nullptr ||
        input.vehicle_bushing_force_curve_force == nullptr ||
        input.bushing_force_curve_interpolation == nullptr) {
        error = "vehicle bushing curve arrays are incomplete";
        return false;
    }
    if (model.bushings.size() != input.axle.bushing_count) {
        error = "vehicle bushing curve count does not match bushing count";
        return false;
    }

    const auto copy_curve = [&error](
        int offset, int count, const double* coordinate, const double* force,
        std::vector<double>& target_coordinate,
        std::vector<double>& target_force
    ) -> bool {
        if (count < 0 || offset < 0 ||
            count > std::numeric_limits<int>::max()-offset) {
            error = "invalid bushing force curve range";
            return false;
        }
        if (count == 1) {
            error = "bushing force curve needs at least two points";
            return false;
        }
        target_coordinate.clear();
        target_force.clear();
        target_coordinate.reserve(static_cast<std::size_t>(count));
        target_force.reserve(static_cast<std::size_t>(count));
        double previous = 0.0;
        for (int point = 0; point < count; ++point) {
            const double x = coordinate[offset+point];
            const double y = force[offset+point];
            if (!std::isfinite(x) || !std::isfinite(y) ||
                (point > 0 && x <= previous)) {
                error = "bushing force curve must be finite and strictly increasing";
                return false;
            }
            target_coordinate.push_back(x);
            target_force.push_back(y);
            previous = x;
        }
        return true;
    };

    for (std::size_t i = 0; i < model.bushings.size(); ++i) {
        Bushing& bushing = model.bushings[i];
        if (input.bushing_force_curve_interpolation == nullptr) {
            error = "vehicle bushing curve interpolation array is missing";
            return false;
        }
        const int interpolation = input.bushing_force_curve_interpolation[i];
        if (interpolation != 0 && interpolation != 1) {
            error = "unsupported vehicle bushing curve interpolation";
            return false;
        }
        bushing.force_curve_interpolation = interpolation;
        for (int axis = 0; axis < 6; ++axis) {
            const std::size_t slot = i*6 + static_cast<std::size_t>(axis);
            if (!copy_curve(
                    input.vehicle_bushing_force_curve_offset[slot],
                    input.vehicle_bushing_force_curve_count[slot],
                    input.vehicle_bushing_force_curve_coordinate,
                    input.vehicle_bushing_force_curve_force,
                    bushing.elastic_coordinate[static_cast<std::size_t>(axis)],
                    bushing.elastic_force[static_cast<std::size_t>(axis)])) {
                return false;
            }
        }
    }
    return true;
}

bool add_vehicle_bushing_rotation_coordinates(
    const VehicleInput& input, Model& model, std::string& error
) {
    const std::size_t field_end =
        offsetof(VehicleInput, bushing_rotation_coordinates)
        + sizeof(input.bushing_rotation_coordinates);
    if (input.struct_size < field_end) return true;
    if (model.bushings.size() != input.axle.bushing_count) {
        error = "vehicle bushing rotation coordinate count does not match bushing count";
        return false;
    }
    if (model.bushings.empty()) return true;
    if (input.bushing_rotation_coordinates == nullptr) {
        error = "vehicle bushing rotation coordinate array is missing";
        return false;
    }
    for (std::size_t i = 0; i < model.bushings.size(); ++i) {
        const int coordinates = input.bushing_rotation_coordinates[i];
        if (coordinates != VEHICLE_BUSHING_ROTATION_VECTOR &&
            coordinates != VEHICLE_BUSHING_CARDAN_XYZ) {
            error = "unsupported vehicle bushing rotation coordinates";
            return false;
        }
        model.bushings[i].rotation_coordinates = coordinates;
    }
    return true;
}

void copy_state(const State& s, const Model& m, double* out, std::size_t sample) {
    const std::size_t stride=static_cast<std::size_t>(m.bodies.size()*kStatePerBody);
    for(std::size_t i=0;i<m.bodies.size();++i){
        const std::size_t k=sample*stride+i*kStatePerBody;
        out[k+0]=s.r[i].x;out[k+1]=s.r[i].y;out[k+2]=s.r[i].z;
        out[k+3]=s.q[i].w;out[k+4]=s.q[i].x;out[k+5]=s.q[i].y;out[k+6]=s.q[i].z;
        out[k+7]=s.v[i].x;out[k+8]=s.v[i].y;out[k+9]=s.v[i].z;
        out[k+10]=s.omega[i].x;out[k+11]=s.omega[i].y;out[k+12]=s.omega[i].z;
        out[k+13]=s.a[i].x;out[k+14]=s.a[i].y;out[k+15]=s.a[i].z;
        out[k+16]=s.alpha[i].x;out[k+17]=s.alpha[i].y;out[k+18]=s.alpha[i].z;
    }
}

struct SteeringMeasurement {
    double angle{0.0};
    double rate{0.0};
    double torque{0.0};
};

SteeringMeasurement measure_steering(
    const SteeringActuator& actuator, const State& state,
    const SampleInput& input, std::size_t index
) {
    const int reaction = actuator.reaction_body;
    const double target = index < input.steering_target.size()
        ? input.steering_target[index] : 0.0;
    const double target_rate = index < input.steering_target_rate.size()
        ? input.steering_target_rate[index] : 0.0;
    if (actuator.type == VEHICLE_STEERING_PRESCRIBED_ROTATION) {
        const Quat reaction_q = reaction >= 0
            ? state.q[reaction] : Quat{};
        const Quat relative = qmul(
            qconj(reaction_q), state.q[actuator.body]
        );
        const Vec3 error_rotation = qlog(
            qmul(qconj(actuator.reference), relative)
        );
        const Vec3 axis_reference = steering_axis_reference(actuator);
        const Vec3 axis_world = normalized(
            rotate(reaction_q, axis_reference)
        );
        const Vec3 relative_omega = state.omega[actuator.body]
            - (reaction >= 0 ? state.omega[reaction] : Vec3{});
        return {
            dot(error_rotation, actuator.axis_local),
            dot(axis_world, relative_omega),
            std::numeric_limits<double>::quiet_NaN()
        };
    }
    if (actuator.type == VEHICLE_STEERING_TRANSLATION ||
        actuator.type == VEHICLE_STEERING_PRESCRIBED_TRANSLATION) {
        const Vec3 body_point = state_point(
            state, actuator.body, actuator.point_local
        );
        const Vec3 reaction_point = reaction >= 0
            ? state_point(state, reaction, actuator.reaction_point_local)
            : Vec3{};
        const Vec3 body_velocity = state_point_velocity(
            state, actuator.body, actuator.point_local
        );
        const Vec3 reaction_velocity = reaction >= 0
            ? state_point_velocity(
                state, reaction, actuator.reaction_point_local
            )
            : Vec3{};
        const Vec3 axis_world = normalized(
            reaction >= 0
                ? rotate(state.q[reaction], actuator.axis_local)
                : actuator.axis_local
        );
        const double coordinate = dot(
            axis_world, body_point-reaction_point
        );
        const double rate = dot(
            axis_world, body_velocity-reaction_velocity
        );
        return {
            coordinate,
            rate,
            actuator.type == VEHICLE_STEERING_PRESCRIBED_TRANSLATION
                ? std::numeric_limits<double>::quiet_NaN()
                : actuator.stiffness * (target-coordinate)
                    + actuator.damping * (target_rate-rate)
        };
    }
    const Quat reaction_q = reaction >= 0
        ? state.q[reaction] : Quat{};
    const Vec3 axis_reference =
        rotate(actuator.reference, actuator.axis_local);
    const Quat relative = qmul(
        qconj(reaction_q), state.q[actuator.body]
    );
    const Vec3 error_rotation = qlog(
        qmul(qconj(actuator.reference), relative)
    );
    const Vec3 axis_world = normalized(
        rotate(reaction_q, axis_reference)
    );
    const Vec3 relative_omega = state.omega[actuator.body]
        - (reaction >= 0 ? state.omega[reaction] : Vec3{});
    const double angle = dot(error_rotation, actuator.axis_local);
    const double rate = dot(axis_world, relative_omega);
    return {
        angle,
        rate,
        actuator.stiffness * (target-angle)
            + actuator.damping * (target_rate-rate)
    };
}

void write_vehicle_steering_output(
    const Model& model, const AxleInput& input,
    const AxleOutput& axle_output, VehicleOutput& output
) {
    if (model.steering_actuators.empty() ||
        output.steering_output == nullptr) {
        return;
    }
    State state;
    state.r.resize(model.bodies.size());
    state.q.resize(model.bodies.size());
    state.v.resize(model.bodies.size());
    state.omega.resize(model.bodies.size());
    const std::size_t body_stride =
        model.bodies.size() * kStatePerBody;
    for (std::size_t sample = 0; sample < input.sample_count; ++sample) {
        const double* row = axle_output.body_state + sample * body_stride;
        for (std::size_t body = 0; body < model.bodies.size(); ++body) {
            const double* values = row + body * kStatePerBody;
            state.r[body] = {values[0], values[1], values[2]};
            state.q[body] = {values[3], values[4], values[5], values[6]};
            state.v[body] = {values[7], values[8], values[9]};
            state.omega[body] = {values[10], values[11], values[12]};
        }
        SampleInput sample_input;
        interpolate_input(
            model, input, input.sample_times[sample], sample_input
        );
        for (std::size_t actuator = 0;
             actuator < model.steering_actuators.size(); ++actuator) {
            const SteeringMeasurement measurement = measure_steering(
                model.steering_actuators[actuator], state,
                sample_input, actuator
            );
            const std::size_t offset =
                (sample * model.steering_actuators.size() + actuator)
                * kSteeringOutputWidth;
            output.steering_output[offset] = measurement.angle;
            output.steering_output[offset+1] = measurement.rate;
            output.steering_output[offset+2] =
                sample_input.steering_target[actuator];
            output.steering_output[offset+3] = measurement.torque;
        }
    }
}

void write_constraint_wrenches(
    const Model& model, const State& state,
    const std::vector<double>& multiplier,
    double* output, std::size_t sample
) {
    if (model.constraints.empty()) return;
    std::fill(
        output+sample*model.constraints.size()*kConstraintOutputWidth,
        output+(sample+1)*model.constraints.size()*kConstraintOutputWidth,
        0.0
    );
    const auto jacobian = constraint_jacobian(model, state);
    const std::size_t sample_offset =
        sample*model.constraints.size()*kConstraintOutputWidth;
    for (std::size_t constraint_index = 0;
         constraint_index < model.constraints.size(); ++constraint_index) {
        const Constraint& constraint = model.constraints[constraint_index];
        Vec3 force{};
        Vec3 moment_about_com{};
        Vec3 reference{};
        int source_body = constraint.b;
        double sign = 1.0;
        int free_index = model.body_to_free[constraint.b];
        if (free_index < 0) {
            source_body = constraint.a;
            free_index = model.body_to_free[constraint.a];
            sign = -1.0;
        }
        if (free_index >= 0) {
            const int rows = constraint_rows(constraint.type);
            for (int local_row = 0; local_row < rows; ++local_row) {
                const int row = constraint.row+local_row;
                const double lambda =
                    row < static_cast<int>(multiplier.size())
                    ? multiplier[static_cast<std::size_t>(row)]
                    : 0.0;
                force.x +=
                    jacobian[row*model.ndof+6*free_index]*lambda;
                force.y +=
                    jacobian[row*model.ndof+6*free_index+1]*lambda;
                force.z +=
                    jacobian[row*model.ndof+6*free_index+2]*lambda;
                moment_about_com.x +=
                    jacobian[row*model.ndof+6*free_index+3]*lambda;
                moment_about_com.y +=
                    jacobian[row*model.ndof+6*free_index+4]*lambda;
                moment_about_com.z +=
                    jacobian[row*model.ndof+6*free_index+5]*lambda;
            }
            reference = source_body == constraint.b
                ? state_point(state, constraint.b, constraint.pb)
                : state_point(state, constraint.a, constraint.pa);
            const Vec3 arm = reference-state.r[source_body];
            force = force*sign;
            const Vec3 moment_at_reference =
                (moment_about_com-cross(arm, force/sign))*sign;
            const std::size_t offset =
                sample_offset+constraint_index*kConstraintOutputWidth;
            output[offset] = force.x;
            output[offset+1] = force.y;
            output[offset+2] = force.z;
            output[offset+3] = moment_at_reference.x;
            output[offset+4] = moment_at_reference.y;
            output[offset+5] = moment_at_reference.z;
        }
    }
}

double kinetic_energy(const Model& model, const State& state) {
    double energy = 0.0;
    for (std::size_t i = 0; i < model.bodies.size(); ++i) {
        if (model.bodies[i].fixed) continue;
        const Mat3 rotation = qmat(state.q[i]);
        const Mat3 inertia = rotation * model.bodies[i].inertia_body * transpose(rotation);
        energy += 0.5 * model.bodies[i].mass * dot(state.v[i], state.v[i]);
        energy += 0.5 * dot(state.omega[i], inertia * state.omega[i]);
    }
    return energy;
}

struct EnergyInterval {
    double external_work{0.0};
    double road_work{0.0};
    double drive_work{0.0};
    double damper_dissipation{0.0};
    double friction_dissipation{0.0};
    double contact_dissipation{0.0};

    double total_work() const {
        return external_work+road_work+drive_work;
    }

    double total_physical_dissipation() const {
        return damper_dissipation+friction_dissipation+
            contact_dissipation;
    }
};

bool accumulate_energy_step(
    const Model& model, const AxleInput& input,
    const State& start, const State& end,
    double time, double h, double alpha_f,
    EnergyInterval& interval
) {
    State evaluation = start;
    const double weight = 1.0-alpha_f;
    for (int bi : model.free_body) {
        evaluation.r[bi] =
            start.r[bi]*(1.0-weight)+end.r[bi]*weight;
        const Vec3 rotation_increment = qlog(
            qmul(end.q[bi], qconj(start.q[bi]))
        );
        evaluation.q[bi] = normalized_continuous(
            qmul(qexp(rotation_increment*weight), start.q[bi]),
            start.q[bi]
        );
        evaluation.v[bi] =
            start.v[bi]*(1.0-weight)+end.v[bi]*weight;
        evaluation.omega[bi] =
            start.omega[bi]*(1.0-weight)+end.omega[bi]*weight;
    }
    evaluation.tire_sx.resize(model.tires.size(), 0.0);
    evaluation.tire_sy.resize(model.tires.size(), 0.0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        evaluation.tire_sx[i] =
            start.tire_sx[i]*(1.0-weight)+end.tire_sx[i]*weight;
        evaluation.tire_sy[i] =
            start.tire_sy[i]*(1.0-weight)+end.tire_sy[i]*weight;
    }
    SampleInput sample;
    interpolate_input(model, input, time+weight*h, sample);
    std::vector<double> tire_forces;
    std::vector<double> tire_derivatives;
    std::vector<double> tire_output;
    double potential = 0.0;
    double power = 0.0;
    double dissipation = 0.0;
    EnergyRates rates;
    std::vector<double> force;
    external_force_vector(
        model, evaluation, sample,
        input.gravity_x, input.gravity_y, input.gravity_z,
        tire_forces, tire_derivatives, tire_output,
        potential, power, dissipation,
        force, nullptr, nullptr, nullptr, nullptr, &rates
    );
    if (!std::isfinite(power) || !std::isfinite(dissipation) ||
        dissipation < -1e-12) {
        return false;
    }
    interval.external_work += h*rates.external_power;
    interval.road_work += h*rates.road_power;
    interval.drive_work += h*rates.drive_power;
    interval.damper_dissipation += h*rates.damper_dissipation;
    interval.friction_dissipation += h*rates.friction_dissipation;
    interval.contact_dissipation += h*rates.contact_dissipation;
    return std::isfinite(interval.total_work()) &&
        std::isfinite(interval.total_physical_dissipation());
}

void write_physics_output(
    const Model& model, const AxleInput& input, const State& state, double time,
    AxleOutput& output, std::size_t sample, double previous_energy,
    const EnergyInterval& interval, bool first
) {
    SampleInput sample_input;
    interpolate_input(model, input, time, sample_input);
    std::vector<double> tire_forces;
    std::vector<double> tire_derivatives;
    std::vector<double> tire_output;
    std::vector<double> spring_output;
    std::vector<double> bushing_output;
    std::vector<double> anti_roll_output;
    double potential = 0.0;
    double power = 0.0;
    double dissipation = 0.0;
    EnergyStorage storage;
    std::vector<double> force;
    external_force_vector(
        model, state, sample_input, input.gravity_x, input.gravity_y, input.gravity_z,
        tire_forces, tire_derivatives, tire_output, potential, power, dissipation,
        force, &spring_output, &bushing_output, &anti_roll_output,
        nullptr, nullptr, &storage
    );
    const double kinetic = kinetic_energy(model, state);
    const double total = kinetic + potential;
    const double residual = first
        ? 0.0
        : total - previous_energy
            - interval.total_work()
            + interval.total_physical_dissipation();
    const std::size_t energy_offset = sample * kEnergyOutputWidth;
    output.energy_output[energy_offset+0] = kinetic;
    output.energy_output[energy_offset+1] = potential;
    output.energy_output[energy_offset+2] = total;
    output.energy_output[energy_offset+3] = residual;
    output.energy_output[energy_offset+4] = interval.external_work;
    output.energy_output[energy_offset+5] = interval.road_work;
    output.energy_output[energy_offset+6] = interval.drive_work;
    output.energy_output[energy_offset+7] = interval.damper_dissipation;
    output.energy_output[energy_offset+8] = interval.friction_dissipation;
    output.energy_output[energy_offset+9] = interval.contact_dissipation;
    // No derived generalized-alpha dissipation formula is used here. Keep
    // algorithmic dissipation separate from the directly measured residual.
    output.energy_output[energy_offset+10] = 0.0;
    output.energy_output[energy_offset+11] = interval.total_work();
    output.energy_output[energy_offset+12] =
        interval.total_physical_dissipation();
    output.energy_output[energy_offset+13] =
        std::isfinite(residual) ? 1.0 : 0.0;
    output.energy_output[energy_offset+14] = storage.gravity;
    output.energy_output[energy_offset+15] = storage.spring;
    output.energy_output[energy_offset+16] = storage.stop;
    output.energy_output[energy_offset+17] = storage.bushing;
    output.energy_output[energy_offset+18] = storage.anti_roll;
    output.energy_output[energy_offset+19] = storage.tire_normal;
    output.energy_output[energy_offset+20] = storage.tire_brush;
    for (std::size_t i = 0; i < tire_output.size(); ++i) {
        output.tire_output[
            sample * model.tires.size() * kTireOutputWidth + i
        ] = tire_output[i];
    }
    for (std::size_t i = 0; i < spring_output.size(); ++i) {
        output.spring_output[
            sample*model.springs.size()*kSpringOutputWidth+i
        ] = spring_output[i];
    }
    for (std::size_t i = 0; i < bushing_output.size(); ++i) {
        output.bushing_output[
            sample*model.bushings.size()*kBushingOutputWidth+i
        ] = bushing_output[i];
    }
    for (std::size_t i = 0; i < anti_roll_output.size(); ++i) {
        output.anti_roll_output[
            sample*model.anti_roll_bars.size()*kAntiRollOutputWidth+i
        ] = anti_roll_output[i];
    }
}

double performance_seconds(const std::atomic<std::uint64_t>& nanoseconds) {
    return static_cast<double>(nanoseconds.load(std::memory_order_relaxed))
        * 1e-9;
}

void write_performance_metrics(
    const PerformanceCounters& counters, bool enabled,
    std::size_t sample_count, AxleOutput& output
) {
    const std::size_t offset = sample_count * kDiagnosticsWidth;
    if (output.diagnostics_capacity < offset + kPerformanceWidth) return;
    double* row = output.diagnostics + offset;
    std::fill(row, row + kPerformanceWidth, 0.0);
    if (!enabled) {
        row[0] = 0.0;
        return;
    }
    row[0] = 1.0;
    row[1] = static_cast<double>(
        counters.residual_calls.load(std::memory_order_relaxed)
    );
    row[2] = performance_seconds(counters.residual_nanoseconds);
    row[3] = static_cast<double>(
        counters.constraint_jacobian_calls.load(std::memory_order_relaxed)
    );
    row[4] = performance_seconds(counters.constraint_jacobian_nanoseconds);
    row[5] = static_cast<double>(
        counters.force_evaluations.load(std::memory_order_relaxed)
    );
    row[6] = performance_seconds(counters.force_nanoseconds);
    row[7] = static_cast<double>(
        counters.mass_inverse_calls.load(std::memory_order_relaxed)
    );
    row[8] = performance_seconds(counters.mass_inverse_nanoseconds);
    row[9] = performance_seconds(counters.reaction_nanoseconds);
    row[10] = static_cast<double>(
        counters.linear_factorizations.load(std::memory_order_relaxed)
    );
    row[11] = performance_seconds(
        counters.linear_factorization_nanoseconds
    );
    row[12] = static_cast<double>(
        counters.linear_solves.load(std::memory_order_relaxed)
    );
    row[13] = performance_seconds(counters.linear_solve_nanoseconds);
    row[14] = static_cast<double>(
        counters.line_search_trials.load(std::memory_order_relaxed)
    );
    row[15] = static_cast<double>(
        counters.newton_iterations.load(std::memory_order_relaxed)
    );
    row[16] = static_cast<double>(
        counters.accepted_steps.load(std::memory_order_relaxed)
    );
    row[17] = static_cast<double>(
        counters.rejected_attempts.load(std::memory_order_relaxed)
    );
    row[18] = static_cast<double>(
        counters.analytic_jacobian_columns.load(std::memory_order_relaxed)
    );
    row[19] = static_cast<double>(
        counters.finite_difference_jacobian_columns.load(
            std::memory_order_relaxed
        )
    );
    row[20] = static_cast<double>(
        counters.nonsmooth_fallback_columns.load(std::memory_order_relaxed)
    );
    row[21] = performance_seconds(
        counters.analytic_jacobian_nanoseconds
    );
    row[22] = performance_seconds(
        counters.finite_difference_jacobian_nanoseconds
    );
}

} // namespace

extern "C" int axle_kernel_abi_version() { return 14; }

namespace {

constexpr std::uint32_t kVehicleKernelAbiVersion = 21;

int run_model(
    const AxleInput* input, AxleOutput* output,
    char* error_buffer, std::size_t error_capacity,
    const Model* model_override
) {
    if (!input || !output) { set_error(error_buffer,error_capacity,"input/output is null"); return 1; }
    if (input->sample_count<2 || !input->sample_times || !output->body_state || !output->diagnostics) {
        set_error(error_buffer,error_capacity,"sample arrays are missing or too short"); return 1;
    }
    std::string error;
    Model owned_model;
    const Model* model_pointer = model_override;
    if (model_pointer == nullptr) {
        owned_model = build_model(*input, error);
        if (!error.empty()) {
            set_error(error_buffer, error_capacity, error);
            return 2;
        }
        model_pointer = &owned_model;
    }
    const Model& model = *model_pointer;
    const bool profile = profiling_enabled();
    PerformanceCounters performance;
    const std::size_t state_need=input->sample_count*model.bodies.size()*kStatePerBody;
    const std::size_t constraint_need=
        input->sample_count*model.constraints.size()*kConstraintOutputWidth;
    const std::size_t spring_need=
        input->sample_count*model.springs.size()*kSpringOutputWidth;
    const std::size_t bushing_need=
        input->sample_count*model.bushings.size()*kBushingOutputWidth;
    const std::size_t anti_roll_need=
        input->sample_count*model.anti_roll_bars.size()*kAntiRollOutputWidth;
    const std::size_t diag_need=input->sample_count*kDiagnosticsWidth;
    const std::size_t tire_need=
        input->sample_count*model.tires.size()*kTireOutputWidth;
    const std::size_t energy_need=input->sample_count*kEnergyOutputWidth;
    if(output->body_state_capacity<state_need||
       output->constraint_wrench_capacity<constraint_need||
       output->spring_output_capacity<spring_need||
       output->bushing_output_capacity<bushing_need||
       output->anti_roll_output_capacity<anti_roll_need||
       output->diagnostics_capacity<diag_need||
       output->tire_output_capacity<tire_need||
       output->energy_output_capacity<energy_need||
       !output->contact_event_count||
       (output->contact_event_output_capacity>0 &&
        !output->contact_event_output)){
        set_error(error_buffer,error_capacity,"output buffers are too small");return 3;
    }
    *output->contact_event_count = 0;
    std::fill(
        output->diagnostics,
        output->diagnostics+diag_need,
        std::numeric_limits<double>::quiet_NaN()
    );
    if(!std::isfinite(input->rho_inf)||input->rho_inf<=0||input->rho_inf>1||
       (input->integrator_type!=0 && input->integrator_type!=1)||
       (input->integrator_type==1 &&
        (!std::isfinite(input->hht_alpha) ||
         input->hht_alpha < -1.0/3.0 || input->hht_alpha > 0.0))||
       (input->initialization_mode!=0 && input->initialization_mode!=1)||
       (input->adaptive_step!=0 && input->adaptive_step!=1)||
       input->internal_step<=0||input->min_step<=0||input->max_step<=0||
       input->min_step>input->internal_step||input->internal_step>input->max_step||
       input->local_relative_tolerance<=0||input->local_position_tolerance<=0||
       input->local_angle_tolerance<=0||input->local_velocity_tolerance<=0||
       input->local_angular_velocity_tolerance<=0||input->local_brush_tolerance<=0||
       input->contact_event_tolerance<=0||
       input->max_newton_iterations<1||input->max_line_search_iterations<1||
       input->position_tolerance<=0||input->velocity_tolerance<=0||
       input->dynamics_tolerance<=0||input->increment_tolerance<=0){
        set_error(error_buffer,error_capacity,"invalid solver settings");return 4;
    }
    for (std::size_t i = 0; i < input->sample_count; ++i) {
        if (!std::isfinite(input->sample_times[i]) ||
            (i > 0 && input->sample_times[i] <= input->sample_times[i-1])) {
            set_error(
                error_buffer, error_capacity,
                "sample times must be finite and strictly increasing"
            );
            return 4;
        }
    }
    double am=0.0, af=0.0, gamma=0.0, beta=0.0;
    double amz=0.0, afz=0.0, gammaz=0.0;
    if (input->integrator_type == 1) {
        // Adams HHT uses the standard second-order HHT coefficients.  Its
        // DIFF states are converted internally to second-order equations by
        // integrating the DIFF state once; the eliminated auxiliary position
        // gives the first-order coefficients below.
        const double alpha = input->hht_alpha;
        am = 0.0;
        af = -alpha;
        gamma = 0.5 - alpha;
        beta = 0.25*(1.0-alpha)*(1.0-alpha);
        amz = 1.0;
        afz = 1.0-af;
        gammaz = gamma;
    } else {
        am=(2*input->rho_inf-1)/(input->rho_inf+1);
        af=input->rho_inf/(input->rho_inf+1);
        gamma=0.5-am+af;
        beta=0.25*(1-am+af)*(1-am+af);
        amz=(3.0-input->rho_inf)/(2.0*(1.0+input->rho_inf));
        afz=1.0/(1.0+input->rho_inf);
        gammaz=0.5+amz-afz;
    }
    State state;
    state.r.resize(model.bodies.size());state.q.resize(model.bodies.size());state.v.resize(model.bodies.size());
    state.omega.resize(model.bodies.size());state.a.resize(model.bodies.size());state.alpha.resize(model.bodies.size());
    state.tire_sx.assign(model.tires.size(), 0.0);
    state.tire_sy.assign(model.tires.size(), 0.0);
    state.tire_sx_dot.assign(model.tires.size(), 0.0);
    state.tire_sy_dot.assign(model.tires.size(), 0.0);
    for(std::size_t i=0;i<model.bodies.size();++i){
        state.r[i]=model.bodies[i].r;
        state.q[i]=model.bodies[i].q;
        state.v[i]=model.bodies[i].v;
        state.omega[i]=model.bodies[i].omega;
    }
    double trim_force=0.0,trim_position=0.0;int trim_iterations=0;
    int trim_pinned_directions=0;
    int trim_worst_force_coordinate=-1;
    double trim_worst_force_value=0.0;
    std::vector<double> current_constraint_multiplier;
    if(input->initialization_mode==0){
        for (std::size_t i = 0; i < model.bodies.size(); ++i) {
            if (!model.bodies[i].fixed &&
                (norm(state.v[i]) > input->velocity_tolerance ||
                 norm(state.omega[i]) > input->velocity_tolerance)) {
                set_error(
                    error_buffer, error_capacity,
                    "static equilibrium initialization requires zero initial velocity"
                );
                return 6;
            }
        }
        std::fill(state.v.begin(), state.v.end(), Vec3{});
        std::fill(state.omega.begin(), state.omega.end(), Vec3{});
        if(!static_trim(
                model,*input,state,trim_force,trim_position,trim_iterations,
                current_constraint_multiplier,trim_pinned_directions,
                trim_worst_force_coordinate,trim_worst_force_value
            )){
            set_error(
                error_buffer,error_capacity,
                "static equilibrium initialization failed; iterations="+
                std::to_string(trim_iterations)+
                ", force_residual="+std::to_string(trim_force)+
                ", position_residual="+std::to_string(trim_position)+
                ", pinned_null_directions="+
                std::to_string(trim_pinned_directions)+
                ", worst_force_coordinate="+
                std::to_string(trim_worst_force_coordinate)+
                ", worst_force_value="+
                std::to_string(trim_worst_force_value)
            );
            return 6;
        }
        if (model.static_trim_then_release) {
            if (model.release_velocity.size() != model.bodies.size() ||
                model.release_omega.size() != model.bodies.size()) {
                set_error(
                    error_buffer, error_capacity,
                    "static-trim release velocities are missing"
                );
                return 6;
            }
            state.v = model.release_velocity;
            for (std::size_t i = 0; i < model.bodies.size(); ++i) {
                // 静态配平会改变悬架姿态；将初始世界角速度随刚体姿态旋转，
                // 保持轮胎自转轴仍与配平后的轮毂转轴一致。
                const Mat3 initial_rotation = qmat(model.bodies[i].q);
                const Mat3 trimmed_rotation = qmat(state.q[i]);
                state.omega[i] = trimmed_rotation * transpose(initial_rotation)
                    * model.release_omega[i];
            }
        }
    }else{
        SampleInput initial_sample;
        interpolate_input(
            model, *input, input->sample_times[0], initial_sample
        );
        const auto residual_maxima = constraint_residual_maxima(
            model, state, &initial_sample
        );
        const double initial_angle_tolerance =
            model.initial_state_angle_tolerance > 0.0
                ? model.initial_state_angle_tolerance
                : input->local_angle_tolerance;
        trim_position=std::max(
            residual_maxima.position, residual_maxima.angle
        );
        if(residual_maxima.position>input->position_tolerance ||
           residual_maxima.angle>initial_angle_tolerance){
            set_error(
                error_buffer,error_capacity,
                "provided initial state violates position constraints"
            );
            return 6;
        }
    }
    if (exceeds_tire_compression_limit(
            model, *input, state, input->sample_times[0]
        )) {
        set_error(
            error_buffer, error_capacity,
            "initial state exceeds a tire maximum compression"
        );
        return 9;
    }
    double initial_velocity_residual = 0.0;
    if(!validate_initial_velocity(
         model, *input, state, input->velocity_tolerance, initial_velocity_residual
     )){
        set_error(
            error_buffer,error_capacity,
            "initial velocity violates velocity constraints"
        );
        return 7;
    }
    if(!initialize_internal_derivatives(model,*input,state)){
        set_error(
            error_buffer,error_capacity,
            "initial tire internal-state derivative is invalid"
        );
        return 10;
    }
    double initial_dynamics_residual = 0.0;
    if(!initialize_acceleration(
            model,*input,state,initial_dynamics_residual,
            current_constraint_multiplier
        )){
        set_error(error_buffer,error_capacity,"initial acceleration KKT solve failed");
        return 8;
    }
    copy_state(state,model,output->body_state,0);
    write_constraint_wrenches(
        model,state,current_constraint_multiplier,output->constraint_wrench,0
    );
    write_physics_output(
        model, *input, state, input->sample_times[0], *output, 0,
        0.0, EnergyInterval{}, true
    );
    std::fill(
        output->diagnostics,
        output->diagnostics+kDiagnosticsWidth,
        0.0
    );
    output->diagnostics[0]=1;
    output->diagnostics[3]=trim_iterations;
    output->diagnostics[7]=trim_position;
    output->diagnostics[8]=initial_velocity_residual;
    output->diagnostics[9]=std::max(trim_force,initial_dynamics_residual);
    int initial_active_contacts = 0;
    for (std::size_t tire = 0; tire < model.tires.size(); ++tire) {
        if (output->tire_output[tire*kTireOutputWidth] > 0.5) {
            ++initial_active_contacts;
        }
    }
    output->diagnostics[10]=initial_active_contacts;
    output->diagnostics[13]=output->energy_output[3];
    output->diagnostics[15]=trim_pinned_directions;
    double previous_energy = output->energy_output[2];
    double suggested_step = std::max(
        input->min_step, std::min(input->max_step, input->internal_step)
    );
    std::vector<ContactEventRecord> contact_events;
    std::vector<Vec3> previous_acceleration(model.bodies.size());
    std::vector<Vec3> previous_alpha(model.bodies.size());
    bool have_acceleration_history = false;
    NewtonLinearizationCache single_step_cache;
    NewtonLinearizationCache full_step_cache;
    NewtonLinearizationCache half_step_cache;
    bool has_fiala_tire = false;
    for (const Tire& tire : model.tires) {
        has_fiala_tire = has_fiala_tire || tire.model_kind == VEHICLE_TIRE_FIALA;
    }
    for(std::size_t sample_index=1;sample_index<input->sample_count;++sample_index){
        const double target=input->sample_times[sample_index];
        double t=input->sample_times[sample_index-1];
        int accepted_steps=0,rejected_attempts=0,max_iterations=0;
        int event_count=0;
        double pmax=0,vmax=0,dmax=0,local_error_max=0.0;
        double min_accepted_step=std::numeric_limits<double>::infinity();
        double max_accepted_step=0.0,last_accepted_step=0.0;
        EnergyInterval energy_interval;
        bool failed=false;
        std::string failure_reason;
        auto record_step = [&](const StepResult& step) {
            max_iterations=std::max(max_iterations,step.iterations);
            pmax=std::max(pmax,step.position_residual);
            vmax=std::max(vmax,step.velocity_residual);
            dmax=std::max(dmax,step.dynamics_residual);
        };
        auto capture_acceleration_history = [&]() {
            previous_acceleration = state.a;
            previous_alpha = state.alpha;
            have_acceleration_history = true;
        };
        auto record_accepted_steps = [&](std::uint64_t count) {
            if (profile) {
                performance.accepted_steps.fetch_add(
                    count, std::memory_order_relaxed
                );
            }
        };
        // 内部步长累计到输出时刻时会产生双精度尾差；尾差不能再被当作
        // 一个新的积分步，否则会以接近零的步长进入 Newton 和 LU 分解。
        const double time_tolerance = 1.0e-12 * std::max(
            1.0, std::abs(target)
        );
        // The first interval starts from a statically trimmed state.  Its
        // step-doubling error is dominated by the zero-to-motion scale rather
        // than by the local truncation error, so use the validated fixed step
        // path once and enable adaptive control after the first public sample.
        const bool use_adaptive_step =
            input->adaptive_step != 0 && (sample_index > 1 || has_fiala_tire);
        while(t<target-time_tolerance){
            const double remaining=target-t;
            if (remaining <= time_tolerance) {
                t = target;
                break;
            }
            const double fiala_start_step = has_fiala_tire && sample_index == 1
                ? std::min(input->internal_step, 5.0e-4)
                : input->internal_step;
            double h=std::min(
                use_adaptive_step ? suggested_step : fiala_start_step,
                remaining
            );
            h=std::min(h,input->max_step);
            const double input_breakpoint =
                next_prescribed_input_breakpoint(*input, t, target);
            h=std::min(h,input_breakpoint-t);
            const double reduction_floor=std::min(input->min_step,remaining);
            bool accepted=false;
            while(!accepted){
                double reduction=0.5;
                if(use_adaptive_step){
                    StepResult full;
                    StepResult first_half;
                    StepResult second_half;
                    bool full_ok = false;
                    bool first_ok = false;
#ifdef _OPENMP
#pragma omp parallel sections num_threads(2)
                    {
#pragma omp section
                        {
                            full_ok=solve_one_step(
                                model,*input,state,t,h,am,af,beta,gamma,
                                amz,afz,gammaz,full,
                                profile ? &performance : nullptr,
                                have_acceleration_history
                                    ? &previous_acceleration : nullptr,
                                have_acceleration_history
                                    ? &previous_alpha : nullptr,
                                &current_constraint_multiplier,
                                &full_step_cache
                            );
                        }
#pragma omp section
                        {
                            first_ok=solve_one_step(
                                model,*input,state,t,0.5*h,am,af,beta,gamma,
                                amz,afz,gammaz,first_half,
                                profile ? &performance : nullptr,
                                have_acceleration_history
                                    ? &previous_acceleration : nullptr,
                                have_acceleration_history
                                    ? &previous_alpha : nullptr,
                                &current_constraint_multiplier,
                                &half_step_cache
                            );
                        }
                    }
#else
                    full_ok=solve_one_step(
                        model,*input,state,t,h,am,af,beta,gamma,
                        amz,afz,gammaz,full,
                        profile ? &performance : nullptr,
                        have_acceleration_history
                            ? &previous_acceleration : nullptr,
                        have_acceleration_history
                            ? &previous_alpha : nullptr,
                        &current_constraint_multiplier,
                        &full_step_cache
                    );
                    if (full_ok) {
                        first_ok=solve_one_step(
                            model,*input,state,t,0.5*h,am,af,beta,gamma,
                            amz,afz,gammaz,first_half,
                            profile ? &performance : nullptr,
                            have_acceleration_history
                                ? &previous_acceleration : nullptr,
                            have_acceleration_history
                                ? &previous_alpha : nullptr,
                            &current_constraint_multiplier,
                            &half_step_cache
                        );
                    }
#endif
                    record_step(full);
                    record_step(first_half);
                    bool second_ok=false;
                    if(full_ok && first_ok){
                        second_ok=solve_one_step(
                            model,*input,first_half.state,t+0.5*h,0.5*h,
                            am,af,beta,gamma,amz,afz,gammaz,second_half,
                            profile ? &performance : nullptr,
                            &state.a, &state.alpha,
                            &first_half.constraint_multiplier,
                            &half_step_cache
                        );
                        record_step(second_half);
                    }
                    if(full_ok&&first_ok&&second_ok){
                        const bool event = contact_transition(
                            model,*input,state,t,second_half.state,t+h
                        );
                        const bool compression_limit=
                            exceeds_tire_compression_limit(
                                model,*input,first_half.state,t+0.5*h
                            )||
                            exceeds_tire_compression_limit(
                                model,*input,second_half.state,t+h
                            );
                        const double error_ratio=normalized_state_error(
                            model,*input,state,full.state,second_half.state
                        );
                        if (std::isfinite(error_ratio)) {
                            local_error_max =
                                std::max(local_error_max,error_ratio);
                        }
                        if(event&&!compression_limit){
                            StepResult event_step;
                            double event_h = 0.0;
                            if (!localize_contact_event(
                                    model, *input, state, t, h, second_half,
                                    event_step, event_h
                                )) {
                                failure_reason =
                                    "contact event localization failed";
                            } else {
                                if (!accumulate_energy_step(
                                        model, *input, state,
                                        event_step.state, t, event_h, af,
                                        energy_interval
                                    )) {
                                    failure_reason =
                                        "nonphysical energy rate detected";
                                } else {
                                    append_contact_events(
                                        model, *input, state, t,
                                        event_step.state, t+event_h,
                                        contact_events
                                    );
                                    capture_acceleration_history();
                                    state=std::move(event_step.state);
                                    current_constraint_multiplier =
                                        std::move(
                                            event_step.constraint_multiplier
                                        );
                                    full_step_cache.invalidate();
                                    half_step_cache.invalidate();
                                    t+=event_h;
                                    accepted=true;
                                    ++accepted_steps;
                                    record_accepted_steps(1);
                                    ++event_count;
                                    min_accepted_step=std::min(
                                        min_accepted_step,event_h
                                    );
                                    max_accepted_step=std::max(
                                        max_accepted_step,event_h
                                    );
                                    last_accepted_step=event_h;
                                    suggested_step=std::max(
                                        input->min_step,
                                        std::min(input->max_step,event_h)
                                    );
                                }
                            }
                        } else if(!compression_limit&&
                           !event&&std::isfinite(error_ratio)&&error_ratio<=1.0){
                            if (!accumulate_energy_step(
                                    model, *input, state, first_half.state,
                                    t, 0.5*h, af, energy_interval
                                ) ||
                                !accumulate_energy_step(
                                    model, *input, first_half.state,
                                    second_half.state, t+0.5*h, 0.5*h, af,
                                    energy_interval
                                )) {
                                failure_reason =
                                    "nonphysical energy rate detected";
                            } else {
                                capture_acceleration_history();
                                state=std::move(second_half.state);
                                current_constraint_multiplier=
                                    std::move(
                                        second_half.constraint_multiplier
                                    );
                                t+=h;
                                accepted=true;
                                accepted_steps+=2;
                                record_accepted_steps(2);
                                min_accepted_step=std::min(
                                    min_accepted_step,0.5*h
                                );
                                max_accepted_step=std::max(
                                    max_accepted_step,0.5*h
                                );
                                last_accepted_step=0.5*h;
                                const double factor=error_ratio<=1e-14
                                    ? 2.0
                                    : std::max(
                                        0.5,
                                        std::min(
                                            2.0,
                                            0.9*std::pow(
                                                error_ratio,-1.0/3.0
                                            )
                                        )
                                    );
                                suggested_step=std::max(
                                    input->min_step,
                                    std::min(input->max_step,h*factor)
                                );
                            }
                        }else if(compression_limit){
                            failure_reason="tire maximum compression exceeded";
                        }else{
                            failure_reason="adaptive local error tolerance exceeded";
                            reduction=std::max(
                                0.2,
                                std::min(
                                    0.8,
                                    0.9*std::pow(error_ratio,-1.0/3.0)
                                )
                            );
                        }
                    }else{
                        failure_reason="Newton solve did not converge";
                    }
                } else {
                    StepResult step;
                    const bool converged=solve_one_step(
                        model,*input,state,t,h,am,af,beta,gamma,
                        amz,afz,gammaz,step,
                        profile ? &performance : nullptr,
                        have_acceleration_history
                            ? &previous_acceleration : nullptr,
                        have_acceleration_history
                            ? &previous_alpha : nullptr,
                        &current_constraint_multiplier,
                        &single_step_cache
                    );
                    record_step(step);
                    if(converged){
                        const bool event=contact_transition(
                            model,*input,state,t,step.state,t+h
                        );
                        const bool compression_limit=
                            exceeds_tire_compression_limit(
                                model,*input,step.state,t+h
                            );
                        if(event&&!compression_limit){
                            StepResult event_step;
                            double event_h = 0.0;
                            if (!localize_contact_event(
                                    model, *input, state, t, h, step,
                                    event_step, event_h
                                )) {
                                failure_reason =
                                    "contact event localization failed";
                            } else {
                                if (!accumulate_energy_step(
                                        model, *input, state,
                                        event_step.state, t, event_h, af,
                                        energy_interval
                                    )) {
                                    failure_reason =
                                        "nonphysical energy rate detected";
                                } else {
                                    append_contact_events(
                                        model, *input, state, t,
                                        event_step.state, t+event_h,
                                        contact_events
                                    );
                                    capture_acceleration_history();
                                    state=std::move(event_step.state);
                                    current_constraint_multiplier =
                                        std::move(
                                            event_step.constraint_multiplier
                                        );
                                    single_step_cache.invalidate();
                                    t+=event_h;
                                    accepted=true;
                                    ++accepted_steps;
                                    record_accepted_steps(1);
                                    ++event_count;
                                    min_accepted_step=std::min(
                                        min_accepted_step,event_h
                                    );
                                    max_accepted_step=std::max(
                                        max_accepted_step,event_h
                                    );
                                    last_accepted_step=event_h;
                                }
                            }
                        } else if(!compression_limit&&!event){
                            if (!accumulate_energy_step(
                                    model, *input, state, step.state,
                                    t, h, af, energy_interval
                                )) {
                                failure_reason =
                                    "nonphysical energy rate detected";
                            } else {
                                capture_acceleration_history();
                                state=std::move(step.state);
                                current_constraint_multiplier=
                                    std::move(step.constraint_multiplier);
                                t+=h;
                                accepted=true;
                                ++accepted_steps;
                                record_accepted_steps(1);
                                min_accepted_step=std::min(
                                    min_accepted_step,h
                                );
                                max_accepted_step=std::max(
                                    max_accepted_step,h
                                );
                                last_accepted_step=h;
                            }
                        }else if(compression_limit){
                            failure_reason="tire maximum compression exceeded";
                        }else{
                            failure_reason="contact event localization failed";
                        }
                    }else{
                        failure_reason="Newton solve did not converge";
                    }
                }
                if(!accepted){
                    ++rejected_attempts;
                    if (profile) {
                        performance.rejected_attempts.fetch_add(
                            1, std::memory_order_relaxed
                        );
                    }
                    if(!use_adaptive_step){
                        failed=true;
                        break;
                    }
                    if(h<=reduction_floor*(1.0+1e-12)){
                        failed=true;
                        break;
                    }
                    h=std::max(reduction_floor,h*reduction);
                    if(use_adaptive_step) suggested_step=h;
                }
            }
            if(failed) break;
        }
        if(failed){
            const std::size_t d=sample_index*kDiagnosticsWidth;
            output->diagnostics[d+0]=0.0;
            output->diagnostics[d+1]=accepted_steps;
            output->diagnostics[d+2]=rejected_attempts;
            output->diagnostics[d+3]=max_iterations;
            output->diagnostics[d+4]=
                std::isfinite(min_accepted_step) ? min_accepted_step : 0.0;
            output->diagnostics[d+5]=max_accepted_step;
            output->diagnostics[d+6]=last_accepted_step;
            output->diagnostics[d+7]=pmax;
            output->diagnostics[d+8]=vmax;
            output->diagnostics[d+9]=dmax;
            output->diagnostics[d+10]=0.0;
            output->diagnostics[d+11]=event_count;
            output->diagnostics[d+12]=local_error_max;
            output->diagnostics[d+13]=
                std::numeric_limits<double>::quiet_NaN();
            int failure_code = 1;
            if (failure_reason.find("local error") != std::string::npos) {
                failure_code = 2;
            } else if (
                failure_reason.find("contact event") != std::string::npos
            ) {
                failure_code = 3;
            } else if (
                failure_reason.find("maximum compression") !=
                std::string::npos
            ) {
                failure_code = 4;
            } else if (
                failure_reason.find("energy") != std::string::npos
            ) {
                failure_code = 5;
            }
            output->diagnostics[d+14]=failure_code;
            output->diagnostics[d+15]=trim_pinned_directions;
            set_error(
                error_buffer,error_capacity,
                "time integration failed at t="+std::to_string(t)+
                " s: "+failure_reason
            );
            write_contact_events(contact_events, *output);
            write_performance_metrics(
                performance, profile, input->sample_count, *output
            );
            return 5;
        }
        copy_state(state,model,output->body_state,sample_index);
        write_constraint_wrenches(
            model,state,current_constraint_multiplier,
            output->constraint_wrench,sample_index
        );
        write_physics_output(
            model, *input, state, target, *output, sample_index,
            previous_energy, energy_interval, false
        );
        const std::size_t d=sample_index*kDiagnosticsWidth;
        output->diagnostics[d+0]=1.0;
        output->diagnostics[d+1]=accepted_steps;
        output->diagnostics[d+2]=rejected_attempts;
        output->diagnostics[d+3]=max_iterations;
        output->diagnostics[d+4]=
            std::isfinite(min_accepted_step) ? min_accepted_step : 0.0;
        output->diagnostics[d+5]=max_accepted_step;
        output->diagnostics[d+6]=last_accepted_step;
        output->diagnostics[d+7]=pmax;
        output->diagnostics[d+8]=vmax;
        output->diagnostics[d+9]=dmax;
        int active_contacts=0;
        for(std::size_t tire=0;tire<model.tires.size();++tire){
            const std::size_t offset=
                sample_index*model.tires.size()*kTireOutputWidth
                +tire*kTireOutputWidth;
            if(output->tire_output[offset]>0.5) ++active_contacts;
        }
        output->diagnostics[d+10]=active_contacts;
        output->diagnostics[d+11]=event_count;
        output->diagnostics[d+12]=local_error_max;
        output->diagnostics[d+13]=output->energy_output[
            sample_index*kEnergyOutputWidth+3
        ];
        output->diagnostics[d+14]=0.0;
        output->diagnostics[d+15]=trim_pinned_directions;
        previous_energy = output->energy_output[
            sample_index*kEnergyOutputWidth+2
        ];
    }
    if (!write_contact_events(contact_events, *output)) {
        set_error(
            error_buffer,error_capacity,
            "contact event output capacity is too small"
        );
        return 10;
    }
    write_performance_metrics(
        performance, profile, input->sample_count, *output
    );
    return 0;
}

} // namespace

extern "C" int axle_run(
    const AxleInput* input, AxleOutput* output,
    char* error_buffer, std::size_t error_capacity
) {
    return run_model(
        input, output, error_buffer, error_capacity, nullptr
    );
}

extern "C" int vehicle_kernel_abi_version() {
    return static_cast<int>(kVehicleKernelAbiVersion);
}

extern "C" int vehicle_run(
    const VehicleInput* input, VehicleOutput* output,
    char* error_buffer, std::size_t error_capacity
) {
    if (!input || !output) {
        set_error(error_buffer, error_capacity, "input/output is null");
        return 1;
    }
    const std::size_t vehicle_input_required_size =
        offsetof(VehicleInput, bushing_force_curve_interpolation)
        + sizeof(input->bushing_force_curve_interpolation);
    const std::size_t static_trim_field_end =
        offsetof(VehicleInput, static_trim_then_release)
        + sizeof(input->static_trim_then_release);
    if (input->struct_size < vehicle_input_required_size ||
        input->abi_version != kVehicleKernelAbiVersion ||
        output->struct_size < sizeof(VehicleOutput) ||
        output->abi_version != kVehicleKernelAbiVersion) {
        set_error(
            error_buffer, error_capacity,
            "vehicle ABI version or struct size is unsupported"
        );
        return 2;
    }
    if (!std::isfinite(input->initial_state_angle_tolerance) ||
        input->initial_state_angle_tolerance <= 0.0) {
        set_error(
            error_buffer, error_capacity,
            "initial state angle tolerance must be finite and positive"
        );
        return 4;
    }
    if (input->steering_count > 0) {
        const std::size_t sample_count = input->axle.sample_count;
        const bool count_overflow =
            sample_count == 0 ||
            input->steering_count >
                std::numeric_limits<std::size_t>::max() / sample_count;
        const std::size_t row_count = count_overflow
            ? 0
            : input->steering_count * sample_count;
        const bool width_overflow =
            count_overflow ||
            row_count > std::numeric_limits<std::size_t>::max()
                / kSteeringOutputWidth;
        if (width_overflow || output->steering_output == nullptr ||
            output->steering_output_capacity <
                row_count * kSteeringOutputWidth) {
            set_error(
                error_buffer, error_capacity,
                "vehicle steering output buffer is too small"
            );
            return 3;
        }
    }
    if (input->brake_torque != nullptr) {
        const std::size_t sample_count = input->axle.sample_count;
        const std::size_t tire_count = input->axle.tire_count;
        const bool count_overflow =
            tire_count != 0 && sample_count >
                std::numeric_limits<std::size_t>::max() / tire_count;
        if (count_overflow) {
            set_error(
                error_buffer, error_capacity,
                "vehicle brake torque sample count overflows"
            );
            return 3;
        }
        const std::size_t value_count = sample_count*tire_count;
        for (std::size_t i = 0; i < value_count; ++i) {
            if (!std::isfinite(input->brake_torque[i]) ||
                input->brake_torque[i] < 0.0) {
                set_error(
                    error_buffer, error_capacity,
                    "vehicle brake torque must be finite and non-negative"
                );
                return 3;
            }
        }
    }
    std::string error;
    const std::size_t convel_axes_required_size =
        offsetof(VehicleInput, constraint_axis_b_secondary)
        + sizeof(input->constraint_axis_b_secondary);
    const bool has_convel_axes =
        input->struct_size >= convel_axes_required_size;
    const std::size_t convel_target_required_size =
        offsetof(VehicleInput, constraint_convel_angle_target)
        + sizeof(input->constraint_convel_angle_target);
    const bool has_convel_angle_target =
        input->struct_size >= convel_target_required_size;
    Model model = build_model(
        input->axle, error,
        has_convel_axes ? input->constraint_axis_a_secondary : nullptr,
        has_convel_axes ? input->constraint_axis_b_secondary : nullptr,
        has_convel_angle_target ? input->constraint_convel_angle_target : nullptr,
        input->coordinate_coupler_count,
        input->coordinate_coupler_joint_a,
        input->coordinate_coupler_coordinate_a,
        input->coordinate_coupler_scale_a,
        input->coordinate_coupler_joint_b,
        input->coordinate_coupler_coordinate_b,
        input->coordinate_coupler_scale_b
    );
    if (!error.empty()) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (!add_vehicle_spring_curves(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (!add_vehicle_bushing_curves(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (!add_vehicle_bushing_rotation_coordinates(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (!add_vehicle_tire_frames(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (!add_vehicle_tire_models(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (!add_vehicle_drive_torque_mappings(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (!add_vehicle_aerodynamic_drags(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (!add_vehicle_steering_actuators(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (!add_vehicle_road_profile(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    if (input->static_gauge_dof_mask != 0) {
        if (input->static_gauge_dof_mask & ~static_cast<std::uint32_t>(0x3F) ||
            input->static_gauge_body >= model.bodies.size() ||
            model.bodies[input->static_gauge_body].fixed ||
            input->axle.initialization_mode != 0) {
            set_error(
                error_buffer, error_capacity,
                "invalid static-trim gauge configuration"
            );
            return 2;
        }
        model.static_gauge_body = static_cast<int>(input->static_gauge_body);
        model.static_gauge_dof_mask = input->static_gauge_dof_mask;
    }
    if (!add_vehicle_static_rotation_gauges(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }
    const bool static_trim_then_release =
        input->struct_size >=
            static_trim_field_end &&
        input->static_trim_then_release != 0;
    if (static_trim_then_release) {
        if (input->axle.initialization_mode != 0) {
            set_error(
                error_buffer, error_capacity,
                "static-trim release requires static equilibrium initialization"
            );
            return 2;
        }
        model.static_trim_then_release = true;
        model.release_velocity.resize(model.bodies.size());
        model.release_omega.resize(model.bodies.size());
        for (std::size_t i = 0; i < model.bodies.size(); ++i) {
            model.release_velocity[i] = model.bodies[i].v;
            model.release_omega[i] = model.bodies[i].omega;
            model.bodies[i].v = Vec3{};
            model.bodies[i].omega = Vec3{};
        }
    }
    model.initial_state_angle_tolerance =
        input->initial_state_angle_tolerance;
    model.vehicle_brake_torque = input->brake_torque;
    const int status = run_model(
        &input->axle, &output->axle,
        error_buffer, error_capacity, &model
    );
    if (status == 0) {
        write_vehicle_steering_output(
            model, input->axle, output->axle, *output
        );
    }
    return status;
}
