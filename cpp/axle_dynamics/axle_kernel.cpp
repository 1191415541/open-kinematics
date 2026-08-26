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
#include <string>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

constexpr double kEps = 1e-12;
constexpr int kStatePerBody = 19;
constexpr int kTireOutputWidth = 12;
constexpr int kConstraintOutputWidth = 6;
constexpr int kSpringOutputWidth = 7;
constexpr int kBushingOutputWidth = 12;
constexpr int kAntiRollOutputWidth = 3;
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
    const double d = determinant(m);
    if (!std::isfinite(d) || std::abs(d) <= 1e-14) return false;
    out.a[0][0] = (m.a[1][1]*m.a[2][2]-m.a[1][2]*m.a[2][1])/d;
    out.a[0][1] = (m.a[0][2]*m.a[2][1]-m.a[0][1]*m.a[2][2])/d;
    out.a[0][2] = (m.a[0][1]*m.a[1][2]-m.a[0][2]*m.a[1][1])/d;
    out.a[1][0] = (m.a[1][2]*m.a[2][0]-m.a[1][0]*m.a[2][2])/d;
    out.a[1][1] = (m.a[0][0]*m.a[2][2]-m.a[0][2]*m.a[2][0])/d;
    out.a[1][2] = (m.a[0][2]*m.a[1][0]-m.a[0][0]*m.a[1][2])/d;
    out.a[2][0] = (m.a[1][0]*m.a[2][1]-m.a[1][1]*m.a[2][0])/d;
    out.a[2][1] = (m.a[0][1]*m.a[2][0]-m.a[0][0]*m.a[2][1])/d;
    out.a[2][2] = (m.a[0][0]*m.a[1][1]-m.a[0][1]*m.a[1][0])/d;
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
    const double d1 = m.a[0][0];
    const double d2 = m.a[0][0]*m.a[1][1]-m.a[0][1]*m.a[1][0];
    const double d3 = determinant(m);
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

struct Bushing {
    int a{-1}, b{-1};
    Vec3 pa{}, pb{};
    Quat frame_a{}, frame_b{}, reference{};
    Vec3 reference_translation{};
    std::array<double, 36> stiffness{};
    std::array<double, 36> damping{};
    std::array<double, 6> preload{};
};

struct AntiRollBar {
    int a{-1}, b{-1};
    Vec3 axis_a{0, 0, 1};
    Quat reference{};
    double stiffness{0.0}, damping{0.0};
};

struct Tire {
    int body{-1};
    Vec3 center{};
    Vec3 spin_axis{0, 1, 0};
    Vec3 forward_axis{1, 0, 0};
    double radius{0.0}, maximum_compression{0.0}, k{0.0}, c{0.0};
    double mu_longitudinal{0.0}, mu_lateral{0.0};
    double brush_k_longitudinal{0.0}, brush_k_lateral{0.0};
    double relaxation_length_longitudinal{0.0};
    double relaxation_length_lateral{0.0};
    double detached_relaxation{0.0};
};

struct Model {
    std::vector<Body> bodies;
    std::vector<Constraint> constraints;
    std::vector<Spring> springs;
    std::vector<Bushing> bushings;
    std::vector<AntiRollBar> anti_roll_bars;
    std::vector<Tire> tires;
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
    std::vector<double> body_wrench, road_z, road_v, torque;
};

void set_error(char* buffer, std::size_t capacity, const std::string& text) {
    if (!buffer || capacity == 0) return;
    std::snprintf(buffer, capacity, "%s", text.c_str());
}

int constraint_rows(int type) {
    if (type == AXLE_SPHERICAL) return 3;
    if (type == AXLE_REVOLUTE || type == AXLE_PRISMATIC) return 5;
    if (type == AXLE_FIXED) return 6;
    if (type == AXLE_UNIVERSAL || type == AXLE_CYLINDRICAL) return 4;
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

// The reference vector used to complete a frame perpendicular to `axis`. Both
// the residual and the analytic Jacobian must call this, or the Jacobian would
// linearize a different frame than the residual defines.
Vec3 perpendicular_reference(const Vec3& axis) {
    return std::abs(axis.x) < 0.8 ? Vec3{1,0,0} : Vec3{0,1,0};
}

std::vector<double> constraint_residual(const Model& model, const State& state) {
    std::vector<double> out(model.rows, 0.0);
    for (const auto& c : model.constraints) {
        const Vec3 pa = state_point(state, c.a, c.pa);
        const Vec3 pb = state_point(state, c.b, c.pb);
        const Vec3 dp = pa - pb;
        int k = c.row;
        if (c.type == AXLE_SPHERICAL || c.type == AXLE_REVOLUTE ||
            c.type == AXLE_FIXED || c.type == AXLE_UNIVERSAL) {
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
            Vec3 e1 = perpendicular_reference(aa);
            e1 = normalized(e1 - aa*dot(e1, aa));
            const Vec3 e2 = cross(aa, e1);
            out[k++] = dot(dp, e1);
            out[k++] = dot(dp, e2);
            const Vec3 dr = qlog(qmul(qconj(state.q[c.a]), state.q[c.b]));
            out[k++] = dr.x; out[k++] = dr.y; out[k++] = dr.z;
        }
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
            c.type == AXLE_FIXED || c.type == AXLE_UNIVERSAL) {
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

        if (c.type == AXLE_FIXED) {
            add_relative_rotation(k);
        } else if (c.type == AXLE_UNIVERSAL) {
            // d(aa . ab) = ab . d(aa) + aa . d(ab)
            const Vec3 aa = normalized(rotate(state.q[c.a], c.axis_a));
            const Vec3 ab = normalized(rotate(state.q[c.b], c.axis_b));
            add_row(k, c.a, {}, row_times(ab, skew(aa)*(-1.0)));
            add_row(k, c.b, {}, row_times(aa, skew(ab)*(-1.0)));
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
                add_relative_rotation(k+2);
            }
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
    if (trial_utilization > 1.0) {
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
    const Vec3 rotation = qlog(qmul(qconj(bushing.reference), qrel));
    const Vec3 translation = rel - bushing.reference_translation;
    std::array<double, 6> deformation{
        translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z
    };
    rate = {
        rel_rate.x, rel_rate.y, rel_rate.z,
        rel_omega.x, rel_omega.y, rel_omega.z
    };
    (void)model;
    return deformation;
}

struct StaticContactOverride {
    const std::vector<int>* active{nullptr};
    const std::vector<double>* compression{nullptr};
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
    std::vector<Vec3>* torque_workspace = nullptr) {
    const int n = static_cast<int>(model.bodies.size());
    std::vector<Vec3> local_force;
    std::vector<Vec3> local_torque;
    std::vector<Vec3>& force = force_workspace != nullptr
        ? *force_workspace : local_force;
    std::vector<Vec3>& torque = torque_workspace != nullptr
        ? *torque_workspace : local_torque;
    force.assign(static_cast<std::size_t>(n), Vec3{});
    torque.assign(static_cast<std::size_t>(n), Vec3{});
    potential = 0.0;
    external_power = 0.0;
    dissipation = 0.0;
    if (energy_rates) *energy_rates = EnergyRates{};
    if (energy_storage) *energy_storage = EnergyStorage{};
    for (int i = 0; i < n; ++i) {
        if (input.body_wrench.size() >= static_cast<std::size_t>(6 * n)) {
            force[i] = {
                input.body_wrench[static_cast<std::size_t>(6 * i)],
                input.body_wrench[static_cast<std::size_t>(6 * i + 1)],
                input.body_wrench[static_cast<std::size_t>(6 * i + 2)]
            };
            torque[i] = {
                input.body_wrench[static_cast<std::size_t>(6 * i + 3)],
                input.body_wrench[static_cast<std::size_t>(6 * i + 4)],
                input.body_wrench[static_cast<std::size_t>(6 * i + 5)]
            };
            const double applied_power =
                dot(force[i], state.v[i]) +
                dot(torque[i], state.omega[i]);
            external_power += applied_power;
            if (energy_rates) {
                energy_rates->external_power += applied_power;
            }
        }
    }
    for (int i = 0; i < n; ++i) {
        if (!model.bodies[i].fixed) {
            force[i].x += model.bodies[i].mass * gravity_x;
            force[i].y += model.bodies[i].mass * gravity_y;
            force[i].z += model.bodies[i].mass * gravity_z;
            const double gravity_energy = -model.bodies[i].mass * (
                gravity_x * state.r[i].x + gravity_y * state.r[i].y + gravity_z * state.r[i].z
            );
            potential += gravity_energy;
            if (energy_storage) {
                energy_storage->gravity += gravity_energy;
            }
        }
    }
    tire_forces.assign(model.tires.size(), 0.0);
    tire_brush_derivatives.assign(model.tires.size() * 2, 0.0);
    tire_output.assign(model.tires.size() * kTireOutputWidth, 0.0);
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
        const double elastic_force = s.k*compression;
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
        const double damper_dissipation =
            has_curve ? -(damping_force - preload_force)*dL : damping*dL*dL;
        dissipation += damper_dissipation;
        if (energy_rates) {
            energy_rates->damper_dissipation += damper_dissipation;
        }
        if (std::isfinite(s.minimum_length) && L < s.minimum_length) {
            const double penetration = s.minimum_length - L;
            compression_stop_elastic_force =
                s.compression_stop_k * penetration;
            if (dL < 0.0) {
                compression_stop_damping_force =
                    -s.compression_stop_c * dL;
            }
            fscalar += compression_stop_elastic_force
                + compression_stop_damping_force;
            if (dL < 0.0) {
                const double stop_dissipation =
                    s.compression_stop_c*dL*dL;
                dissipation += stop_dissipation;
                if (energy_rates) {
                    energy_rates->damper_dissipation += stop_dissipation;
                }
            }
            const double stop_energy =
                0.5*s.compression_stop_k*penetration*penetration;
            potential += stop_energy;
            if (energy_storage) energy_storage->stop += stop_energy;
        }
        if (std::isfinite(s.maximum_length) && L > s.maximum_length) {
            const double penetration = L - s.maximum_length;
            rebound_stop_elastic_force =
                -s.rebound_stop_k * penetration;
            if (dL > 0.0) {
                rebound_stop_damping_force =
                    -s.rebound_stop_c * dL;
            }
            fscalar += rebound_stop_elastic_force
                + rebound_stop_damping_force;
            if (dL > 0.0) {
                const double stop_dissipation =
                    s.rebound_stop_c*dL*dL;
                dissipation += stop_dissipation;
                if (energy_rates) {
                    energy_rates->damper_dissipation += stop_dissipation;
                }
            }
            const double stop_energy =
                0.5*s.rebound_stop_k*penetration*penetration;
            potential += stop_energy;
            if (energy_storage) energy_storage->stop += stop_energy;
        }
        const Vec3 f = e*fscalar;
        add_force_on_body(force, torque, model, state, s.b, s.pb, f);
        add_force_on_body(force, torque, model, state, s.a, s.pa, f*(-1.0));
        const double spring_energy =
            0.5*s.k*compression*compression;
        potential += spring_energy;
        if (energy_storage) energy_storage->spring += spring_energy;
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
        std::array<double, 6> rate{};
        const auto deformation = bushing_deformation(b, model, state, rate);
        const auto elastic = mat6_mul(b.stiffness, deformation);
        const auto viscous = mat6_mul(b.damping, rate);
        std::array<double, 6> wrench_local{};
        for (int i = 0; i < 6; ++i) {
            wrench_local[i] = b.preload[static_cast<std::size_t>(i)]
                - elastic[static_cast<std::size_t>(i)]
                - viscous[static_cast<std::size_t>(i)];
        }
        const Quat qfa = qmul(state.q[b.a], b.frame_a);
        const Mat3 rfa = qmat(qfa);
        const Vec3 f_world = rfa * Vec3{wrench_local[0], wrench_local[1], wrench_local[2]};
        const Vec3 t_world = rfa * Vec3{wrench_local[3], wrench_local[4], wrench_local[5]};
        add_force_on_body(force, torque, model, state, b.b, b.pb, f_world);
        add_force_on_body(force, torque, model, state, b.a, b.pa, f_world * (-1.0));
        add_torque_on_body(torque, model, b.b, t_world);
        add_torque_on_body(torque, model, b.a, t_world * (-1.0));
        double bushing_energy = 0.0;
        for (int i = 0; i < 6; ++i) {
            bushing_energy +=
                0.5*deformation[static_cast<std::size_t>(i)]
                    *elastic[static_cast<std::size_t>(i)]
                -b.preload[static_cast<std::size_t>(i)]
                    *deformation[static_cast<std::size_t>(i)];
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
        const double tau = -bar.stiffness * angle - bar.damping * rate;
        add_torque_on_body(torque, model, bar.b, axis_world * tau);
        add_torque_on_body(torque, model, bar.a, axis_world * (-tau));
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
        if (anti_roll_component_output) {
            const std::size_t offset =
                bar_index*kAntiRollOutputWidth;
            (*anti_roll_component_output)[offset] = angle;
            (*anti_roll_component_output)[offset+1] = rate;
            (*anti_roll_component_output)[offset+2] = tau;
        }
    }
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& t = model.tires[i];
        const Vec3 center = state_point(state, t.body, t.center);
        const Vec3 vc = state_point_velocity(state, t.body, t.center);
        const double road = i < input.road_z.size() ? input.road_z[i] : 0.0;
        const double road_v = i < input.road_v.size() ? input.road_v[i] : 0.0;
        const Vec3 normal{0.0, 0.0, 1.0};
        const double delta = t.radius + road - center.z;
        const double sx = i < state.tire_sx.size() ? state.tire_sx[i] : 0.0;
        const double sy = i < state.tire_sy.size() ? state.tire_sy[i] : 0.0;
        const double delta_dot = road_v - vc.z;
        const Mat3 body_rotation = qmat(state.q[t.body]);
        Vec3 forward = rotate(state.q[t.body], t.forward_axis);
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
        const Vec3 patch_arm = normal * (-t.radius);
        const Vec3 patch_velocity = vc + cross(state.omega[t.body], patch_arm);
        const Vec3 road_velocity{0.0, 0.0, road_v};
        const Vec3 relative_patch_velocity = patch_velocity - road_velocity;
        const double vx = dot(relative_patch_velocity, forward);
        const double vy = dot(relative_patch_velocity, lateral);
        const std::size_t output_offset = i*kTireOutputWidth;
        tire_output[output_offset+0] = 0.0;
        tire_output[output_offset+1] = -delta;
        tire_output[output_offset+2] = std::max(0.0, delta);
        tire_output[output_offset+3] = -delta_dot;
        tire_output[output_offset+7] = vx;
        tire_output[output_offset+8] = vy;
        tire_output[output_offset+10] = sx;
        tire_output[output_offset+11] = sy;
        const bool static_active =
            static_contact != nullptr && static_contact->active != nullptr &&
            static_contact->compression != nullptr &&
            i < static_contact->active->size() &&
            (*static_contact->active)[i] != 0;
        const double static_delta =
            static_active && i < static_contact->compression->size()
                ? (*static_contact->compression)[i]
                : delta;
        if (!static_active && delta <= 0.0) {
            tire_brush_derivatives[2*i] = -sx / t.detached_relaxation;
            tire_brush_derivatives[2*i+1] = -sy / t.detached_relaxation;
            continue;
        }
        const double fn = static_active
            ? std::max(0.0, t.k*static_delta)
            : std::max(0.0, t.k*delta + t.c*delta_dot);
        tire_forces[i] = fn;
        if (fn <= 0.0) {
            tire_brush_derivatives[2*i] = -sx / t.detached_relaxation;
            tire_brush_derivatives[2*i+1] = -sy / t.detached_relaxation;
            const double normal_energy = 0.5*t.k*delta*delta;
            potential += normal_energy;
            if (energy_storage) {
                energy_storage->tire_normal += normal_energy;
            }
            continue;
        }
        if (static_active) {
            const Mat3 body_rotation = qmat(state.q[t.body]);
            const Vec3 contact_point = center + patch_arm;
            const Vec3 contact_local = transpose(body_rotation) * (
                contact_point - state.r[t.body]
            );
            add_force_on_body(
                force, torque, model, state, t.body, contact_local,
                normal * fn
            );
            tire_output[output_offset+0] = 1.0;
            tire_output[output_offset+4] = fn;
            tire_output[output_offset+9] = 0.0;
            const double normal_energy =
                0.5*t.k*static_delta*static_delta;
            potential += normal_energy;
            if (energy_storage) {
                energy_storage->tire_normal += normal_energy;
            }
            continue;
        }
        const double rolling_speed = std::abs(dot(vc, forward));
        const double rolling_relaxation_longitudinal =
            rolling_speed / t.relaxation_length_longitudinal;
        const double rolling_relaxation_lateral =
            rolling_speed / t.relaxation_length_lateral;
        tire_brush_derivatives[2*i] =
            vx - rolling_relaxation_longitudinal * sx;
        tire_brush_derivatives[2*i+1] =
            vy - rolling_relaxation_lateral * sy;
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
        const Vec3 contact_point = center + patch_arm;
        const Vec3 contact_local = transpose(body_rotation) * (
            contact_point - state.r[t.body]
        );
        const Vec3 contact_force = forward * fx + lateral * fy + normal * fn;
        add_force_on_body(force, torque, model, state, t.body, contact_local, contact_force);
        tire_output[output_offset+0] = 1.0;
        tire_output[output_offset+4] = fn;
        tire_output[output_offset+5] = fx;
        tire_output[output_offset+6] = fy;
        tire_output[output_offset+9] = utilization;
        const double normal_energy = 0.5*t.k*delta*delta;
        const double brush_energy =
            0.5*t.brush_k_longitudinal*projected_sx*projected_sx
            +0.5*t.brush_k_lateral*projected_sy*projected_sy;
        potential += normal_energy+brush_energy;
        if (energy_storage) {
            energy_storage->tire_normal += normal_energy;
            energy_storage->tire_brush += brush_energy;
        }
        if (fn > 0.0) {
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
        if (i < input.torque.size()) {
            const Tire& t = model.tires[i];
            const Vec3 axis = normalized(rotate(state.q[t.body], t.spin_axis));
            if (!model.bodies[t.body].fixed) {
                torque[t.body] += axis*input.torque[i];
                const double drive_power =
                    input.torque[i] * dot(axis, state.omega[t.body]);
                external_power += drive_power;
                if (energy_rates) {
                    energy_rates->drive_power += drive_power;
                }
            }
        }
    }
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
    const Spring& spring, const DirectionalScalar& value, bool& smooth
) {
    if (spring.damper_velocity.empty()) return {};
    const auto& x = spring.damper_velocity;
    const auto& y = spring.damper_force;
    double slope = 0.0;
    if (x.size() > 1 && value.value > x.front() && value.value < x.back()) {
        std::size_t high = 1;
        while (high < x.size() && x[high] < value.value) ++high;
        const double x0 = x[high-1], x1 = x[high];
        const double span = x1-x0;
        if (span > 0.0) slope = (y[high]-y[high-1])/span;
        if (
            std::abs(value.value-x[high-1]) <= 1e-12 ||
            std::abs(value.value-x[high]) <= 1e-12
        ) smooth = false;
    } else if (x.size() > 1) {
        for (double knot : x) {
            if (std::abs(value.value-knot) <= 1e-12) smooth = false;
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
    bool smooth = static_contact == nullptr;
    if (static_contact != nullptr) return false;
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

    if (!brush_only) {
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
                std::abs(dlength.value) <= 1e-12 ||
                dlength.value*trial_dlength <= 0.0
            ) smooth = false;
            const DirectionalScalar compression = s.free_length-length;
            const double damping =
                dlength.value < 0.0 ? s.c_compression : s.c_rebound;
            const DirectionalScalar elastic_force = s.k*compression;
            const bool has_curve = !s.damper_velocity.empty();
            const DirectionalScalar damping_force = has_curve
                ? -interpolated_curve_directional(s, dlength, smooth)
                : -damping*dlength;
            DirectionalScalar scalar_force = elastic_force+damping_force;
            if (
                std::isfinite(s.minimum_length) &&
                length.value < s.minimum_length
            ) {
                const DirectionalScalar penetration =
                    s.minimum_length-length;
                scalar_force += s.compression_stop_k*penetration;
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
                scalar_force += -s.rebound_stop_k*penetration;
                if (dlength.value > 0.0) {
                    scalar_force += -s.rebound_stop_c*dlength;
                }
            } else if (
                std::isfinite(s.maximum_length) &&
                std::abs(length.value-s.maximum_length) <= 1e-12
            ) {
                smooth = false;
            }
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
            const DVec3 rotation = d_qlog(
                d_qmul(
                    DQuat{{b.reference.w}, { -b.reference.x},
                           { -b.reference.y}, { -b.reference.z}},
                    qrel
                ), smooth
            );
            const std::array<DirectionalScalar, 6> deformation{
                rel.x-b.reference_translation.x,
                rel.y-b.reference_translation.y,
                rel.z-b.reference_translation.z,
                rotation.x, rotation.y, rotation.z
            };
            const std::array<DirectionalScalar, 6> rate{
                rel_rate.x, rel_rate.y, rel_rate.z,
                rel_omega.x, rel_omega.y, rel_omega.z
            };
            std::array<DirectionalScalar, 6> wrench{};
            for (int i = 0; i < 6; ++i) {
                DirectionalScalar elastic{}, viscous{};
                for (int j = 0; j < 6; ++j) {
                    elastic = elastic + b.stiffness[static_cast<std::size_t>(i*6+j)]
                        * deformation[static_cast<std::size_t>(j)];
                    viscous = viscous + b.damping[static_cast<std::size_t>(i*6+j)]
                        * rate[static_cast<std::size_t>(j)];
                }
                wrench[static_cast<std::size_t>(i)] =
                    b.preload[static_cast<std::size_t>(i)]-elastic-viscous;
            }
            const DVec3 f_local{
                wrench[0], wrench[1], wrench[2]
            };
            const DVec3 t_local{
                wrench[3], wrench[4], wrench[5]
            };
            const DVec3 f_world = rfa*f_local;
            const DVec3 t_world = rfa*t_local;
            add_directional_force_at_arm(
                force, torque, model, b.b,
                d_rotate(state.q[b.b], b.pb, direction.dtheta[b.b]), f_world
            );
            add_directional_force_at_arm(
                force, torque, model, b.a,
                d_rotate(state.q[b.a], b.pa, direction.dtheta[b.a]), -f_world
            );
            add_directional_torque(torque, model, b.b, t_world);
            add_directional_torque(torque, model, b.a, -t_world);
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
            const DirectionalScalar tau =
                -bar.stiffness*angle-bar.damping*rate;
            add_directional_torque(torque, model, bar.b, axis_world*tau);
            add_directional_torque(torque, model, bar.a, -(axis_world*tau));
        }
    }

    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& t = model.tires[i];
        const bool body_active = directional_body_active(direction, t.body);
        const bool brush_active =
            i < direction.dsx.size() &&
            (direction.dsx[i] != 0.0 || direction.dsy[i] != 0.0);
        if (!body_active && !brush_active) continue;
        const DVec3 center = d_state_point(
            state, direction.dr, direction.dtheta, t.body, t.center
        );
        const DVec3 vc = d_state_point_velocity(
            state, direction.dr, direction.dtheta, direction.dv,
            direction.domega, t.body, t.center
        );
        const double road = i < input.road_z.size() ? input.road_z[i] : 0.0;
        const double road_v = i < input.road_v.size() ? input.road_v[i] : 0.0;
        const DVec3 normal{0.0, 0.0, 1.0};
        const DirectionalScalar delta = t.radius+road-center.z;
        const DirectionalScalar delta_dot = road_v-vc.z;
        const DVec3 forward_raw = d_rotate(
            state.q[t.body], t.forward_axis, direction.dtheta[t.body]
        );
        DVec3 forward = forward_raw;
        forward.z = DirectionalScalar{};
        forward = d_normalized(forward, smooth);
        const DVec3 lateral = d_normalized(d_cross(normal, forward), smooth);
        const DVec3 patch_arm{0.0, 0.0, -t.radius};
        const DVec3 omega{
            {state.omega[t.body].x, direction.domega[t.body].x},
            {state.omega[t.body].y, direction.domega[t.body].y},
            {state.omega[t.body].z, direction.domega[t.body].z}
        };
        const DVec3 patch_velocity = vc+d_cross(omega, patch_arm);
        const DVec3 relative_patch_velocity = patch_velocity-DVec3(0.0,0.0,road_v);
        const DirectionalScalar vx = d_dot(relative_patch_velocity, forward);
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
        const DirectionalScalar trial_normal =
            t.k*delta+t.c*delta_dot;
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
        // The saturated brush projection is piecewise smooth. Mark a
        // directional column non-smooth when the audit perturbation crosses
        // its active-set boundary, not only when the base state is exactly on
        // the boundary.
        constexpr double kJacobianStep = 1e-7;
        const double trial_utilization =
            utilization.value + kJacobianStep*utilization.derivative;
        if (
            std::abs(utilization.value-1.0) <= 1e-12 ||
            (utilization.value-1.0)*(trial_utilization-1.0) <= 0.0
        ) {
            smooth = false;
        }
        if (utilization.value > 1.0) {
            projected.x = sx/utilization;
            projected.y = sy/utilization;
        }
        const DirectionalScalar fx = -t.brush_k_longitudinal*projected.x;
        const DirectionalScalar fy = -t.brush_k_lateral*projected.y;
        if (!brush_only) {
            const DVec3 contact_force = forward*fx+lateral*fy+normal*normal_force;
            // The force arm is relative to the tire body's center of mass;
            // `center` is a world position and must not carry body translation.
            const DVec3 contact_arm = d_rotate(
                state.q[t.body], t.center, direction.dtheta[t.body]
            ) + patch_arm;
            add_directional_force_at_arm(
                force, torque, model, t.body, contact_arm, contact_force
            );
        }
    }

    if (brush_only) return smooth;

    if (!brush_only) {
        for (std::size_t i = 0; i < model.tires.size(); ++i) {
            if (i >= input.torque.size()) continue;
            const Tire& t = model.tires[i];
            if (!directional_orientation_active(direction, t.body)) continue;
            const DVec3 axis = d_normalized(
                d_rotate(state.q[t.body], t.spin_axis, direction.dtheta[t.body]),
                smooth
            );
            add_directional_torque(
                torque, model, t.body, axis*input.torque[i]
            );
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
            c.type == AXLE_FIXED || c.type == AXLE_UNIVERSAL) {
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
                add_relative_rotation(k+2);
            }
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
    int dim{0};

    bool factor(std::vector<double> a, int n) {
        lu = std::move(a);
        dim = n;
        pivot.resize(static_cast<std::size_t>(n));
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
                double* swap_row = lu.data()+static_cast<std::size_t>(best_row*n);
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

    void solve(const std::vector<double>& b, std::vector<double>& x) const {
        const int n = dim;
        x.resize(static_cast<std::size_t>(n));
        std::copy(b.begin(), b.end(), x.begin());
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
    }

};

// Chrono's implicit integrators can keep a Jacobian current across Newton
// solves and refresh it only when the current linearization stops reducing the
// residual.  Keep the same policy at the axle level, with the time-step
// coefficients as the compatibility key.  A failed stale solve invalidates
// this cache before the fresh factorization is attempted.
struct ResidualContext;

struct NewtonLinearizationCache {
    LuFactorization factorization;
    int dim{0};
    double h{0.0};
    double alpha_m{0.0};
    double alpha_f{0.0};
    double beta{0.0};
    double gamma{0.0};
    double alpha_m_z{0.0};
    double alpha_f_z{0.0};
    double gamma_z{0.0};
    bool valid{false};

    bool matches(const ResidualContext& context, int dimension) const;

    void update_key(const ResidualContext& context, int dimension);

    void invalidate() { valid = false; }
};

bool solve_linear(std::vector<double> A, std::vector<double> b, std::vector<double>& x) {
    const int n = static_cast<int>(b.size());
    x = b;
    for (int k = 0; k < n; ++k) {
        int pivot = k;
        double best = std::abs(A[k*n+k]);
        for (int i = k+1; i < n; ++i) {
            if (std::abs(A[i*n+k]) > best) { best = std::abs(A[i*n+k]); pivot = i; }
        }
        if (best < 1e-14) return false;
        if (pivot != k) {
            for (int j = k; j < n; ++j) std::swap(A[k*n+j], A[pivot*n+j]);
            std::swap(x[k], x[pivot]);
        }
        const double diag = A[k*n+k];
        for (int j = k+1; j < n; ++j) A[k*n+j] /= diag;
        x[k] /= diag;
        for (int i = k+1; i < n; ++i) {
            const double f = A[i*n+k];
            if (std::abs(f) < 1e-30) continue;
            for (int j = k+1; j < n; ++j) A[i*n+j] -= f*A[k*n+j];
            x[i] -= f*x[k];
        }
    }
    for (int i = n-1; i >= 0; --i) {
        for (int j = i+1; j < n; ++j) x[i] -= A[i*n+j]*x[j];
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
    const Model& model, const State& state, double tolerance, double& residual
) {
    if (model.rows == 0) {
        residual = 0.0;
        return true;
    }
    const auto jacobian = constraint_jacobian(model, state);
    const std::vector<double> velocity = generalized_velocity(model, state);
    residual = 0.0;
    for (int row = 0; row < model.rows; ++row) {
        double violation = 0.0;
        for (int col = 0; col < model.ndof; ++col) {
            violation += jacobian[row*model.ndof+col] * velocity[col];
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
    interpolate_input(input, input.sample_times[0], sample);
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
            &workspace.body_force, &workspace.body_torque
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
    std::vector<double>& phi_storage = workspace.phi_storage;
    if (!cached) phi_storage = constraint_residual(model,next);
    const std::vector<double>& phi =
        external_cached ? pose_jacobians->position_residual : phi_storage;
    for (int i=0;i<m;++i) out[3*n+i]=phi[i];
    for (int row=0;row<m;++row) {
        double s=0.0;
        for (int col=0;col<n;++col) s += J[row*n+col]*v_next[col];
        out[3*n+m+row]=s;
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
    LuFactorization local_factorization;
    LuFactorization* factorization = &local_factorization;
    ResidualWorkspace primary_workspace;
    std::vector<double> r;
    std::vector<double> rhs(static_cast<std::size_t>(dim), 0.0);
    std::vector<double> dx;
    std::vector<double> trial;
    bool residual_ready = false;
    bool factored=false;
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
        // The dynamics rows carry force units and are gated by the normalized
        // dynamics tolerance; the remaining rows are increments in m, rad, m/s
        // and brush metres, where an absolute tolerance is meaningful.
        double increment_res=0.0;
        for (int i=0;i<dim;++i) {
            if (i>=2*n && i<3*n) continue;
            increment_res=std::max(increment_res,std::abs(r[i]));
        }
        if (pos<=in.position_tolerance && vel<=in.velocity_tolerance &&
            dyn<=in.dynamics_tolerance && increment_res<=in.increment_tolerance) {
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
                    std::move(J), dim
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
                factorization->solve(rhs, dx);
                solved = true;
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
                        max_abs(primary_workspace.output)<rmax) {
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
    interpolate_input(input, input.sample_times[0], sample);
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
    if (!finite_vec(tire_derivatives)) return false;
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
    interpolate_input(input, time+h, sample);
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
    state.tire_sx_dot.resize(model.tires.size(), 0.0);
    state.tire_sy_dot.resize(model.tires.size(), 0.0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        if (!std::isfinite(tire_forces[i])) return false;
        if (tire_forces[i] > 0.0) {
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
    SampleInput previous_sample;
    SampleInput evaluation_sample;
    SampleInput internal_evaluation_sample;
    interpolate_input(input, time, previous_sample);
    interpolate_input(input, time + (1.0-alpha_f)*h, evaluation_sample);
    interpolate_input(
        input, time + alpha_f_z*h, internal_evaluation_sample
    );
    ResidualContext context{
        &model, &input, &previous_sample, &evaluation_sample,
        &internal_evaluation_sample, start, h,
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
    interpolate_input(input, time, sample);
    std::vector<double> penetrations(model.tires.size(), 0.0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& tire = model.tires[i];
        const Vec3 center = state_point(state, tire.body, tire.center);
        const double road = i < sample.road_z.size() ? sample.road_z[i] : 0.0;
        penetrations[i] = tire.radius + road - center.z;
    }
    return penetrations;
}

std::vector<int> contact_modes(
    const Model& model, const AxleInput& input,
    const State& state, double time
) {
    SampleInput sample;
    interpolate_input(input, time, sample);
    std::vector<int> modes(model.tires.size(), 0);
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        const Tire& tire = model.tires[i];
        const Vec3 center = state_point(state, tire.body, tire.center);
        const Vec3 center_velocity =
            state_point_velocity(state, tire.body, tire.center);
        const double road =
            i < sample.road_z.size() ? sample.road_z[i] : 0.0;
        const double road_velocity =
            i < sample.road_v.size() ? sample.road_v[i] : 0.0;
        const double delta = tire.radius + road - center.z;
        const double delta_dot = road_velocity - center_velocity.z;
        const double raw_normal_force = tire.k*delta + tire.c*delta_dot;
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
        if (!solve_one_step(
                model, input, start, time, mid,
                (2*input.rho_inf-1)/(input.rho_inf+1),
                input.rho_inf/(input.rho_inf+1),
                0.25*(1.0-(2*input.rho_inf-1)/(input.rho_inf+1)
                      +input.rho_inf/(input.rho_inf+1))
                * (1.0-(2*input.rho_inf-1)/(input.rho_inf+1)
                   +input.rho_inf/(input.rho_inf+1)),
                0.5-(2*input.rho_inf-1)/(input.rho_inf+1)
                    +input.rho_inf/(input.rho_inf+1),
                (3.0-input.rho_inf)/(2.0*(1.0+input.rho_inf)),
                1.0/(1.0+input.rho_inf),
                0.5+(3.0-input.rho_inf)/(2.0*(1.0+input.rho_inf))
                    -1.0/(1.0+input.rho_inf),
                mid_step
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

std::vector<double> static_residual(
    const Model& model, const State& base, const SampleInput& sample,
    double gravity_x, double gravity_y, double gravity_z,
    const std::vector<int>& active_tires, const std::vector<double>& x,
    double& force_residual, double& position_residual
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
        &active_mask, &static_compression
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
    for(int col=0;col<n;++col){
        double reaction=0.0;
        for(int row=0;row<m;++row) reaction+=J[row*n+col]*lambda[row];
        out[col]=-force[col]-reaction;
    }
    const auto phi=constraint_residual(model,candidate);
    for(int row=0;row<m;++row) out[n+row]=phi[row];
    for (std::size_t i = 0; i < active_tires.size(); ++i) {
        const int tire_index = active_tires[i];
        const Tire& tire = model.tires[static_cast<std::size_t>(tire_index)];
        const Vec3 center =
            state_point(candidate, tire.body, tire.center);
        const double road =
            static_cast<std::size_t>(tire_index) < sample.road_z.size()
                ? sample.road_z[static_cast<std::size_t>(tire_index)]
                : 0.0;
        const double geometric_compression = tire.radius + road - center.z;
        out[static_cast<std::size_t>(n+m)+i] =
            static_compression[static_cast<std::size_t>(tire_index)]
            - geometric_compression;
    }
    force_residual=0.0;
    for(int i=0;i<n;++i) force_residual=std::max(force_residual,std::abs(out[i]));
    // The frozen contract gates a *normalized* dynamics residual, and the
    // transient path already divides by the largest force entering its rows.
    // Doing the same here keeps one definition of "small" across both paths:
    // an absolute newton tolerance is unreachable for a multi-kilonewton
    // corner, so the trim would stall at its own rounding floor instead of
    // converging.
    double force_scale = 1.0;
    for (int i = 0; i < n; ++i) {
        force_scale = std::max(force_scale, std::abs(force[i]));
        double reaction = 0.0;
        for (int row = 0; row < m; ++row) {
            reaction = std::max(
                reaction, std::abs(J[row*n+i]*lambda[static_cast<std::size_t>(row)])
            );
        }
        force_scale = std::max(force_scale, reaction);
    }
    force_residual /= force_scale;
    position_residual=max_abs(phi);
    for (std::size_t i = 0; i < active_tires.size(); ++i) {
        position_residual = std::max(
            position_residual,
            std::abs(out[static_cast<std::size_t>(n+m)+i])
        );
    }
    return out;
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

bool static_trim(const Model& model, const AxleInput& input, State& state,
                 double& force_residual, double& position_residual, int& iterations,
                 std::vector<double>& constraint_multiplier, int& pinned_directions) {
    SampleInput sample;
    interpolate_input(input,input.sample_times[0],sample);
    const State base = state;
    std::vector<int> active_tires;
    for (std::size_t i = 0; i < model.tires.size(); ++i) {
        if (!model.bodies[model.tires[i].body].fixed) {
            active_tires.push_back(static_cast<int>(i));
        }
    }
    const int active_set_limit =
        std::max(2, static_cast<int>(model.tires.size())+2);
    pinned_directions = 0;
    std::vector<double> x(
        static_cast<std::size_t>(model.ndof+model.rows)
            + active_tires.size(),
        0.0
    );
    iterations = 0;
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
                position_residual
            );
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
            // Central differences on a step large enough to stay above the
            // residual's own rounding.  A one-sided 1e-7 difference leaves an
            // O(1e-7) error in the trim Jacobian, which produces steps that no
            // longer reduce the residual once it approaches that floor and the
            // line search then stalls on an otherwise converging solve.
            const double eps=1e-6;
            for(int j=0;j<dim;++j){
                std::vector<double> xp=x;xp[j]+=eps;
                std::vector<double> xm=x;xm[j]-=eps;
                double f2,p2,f4,p4;
                const auto rp=static_residual(
                    model, base, sample, input.gravity_x, input.gravity_y,
                    input.gravity_z, active_tires, xp, f2, p2
                );
                const auto rm=static_residual(
                    model, base, sample, input.gravity_x, input.gravity_y,
                    input.gravity_z, active_tires, xm, f4, p4
                );
                for(int i=0;i<dim;++i) {
                    jac[static_cast<std::size_t>(i*dim+j)]=
                        (rp[i]-rm[i])/(2.0*eps);
                }
            }
            pinned_directions = pin_null_pose_directions(
                jac, r, dim, model.ndof,
                std::max(input.dynamics_tolerance, input.position_tolerance)
            );
            std::vector<double> rhs(static_cast<std::size_t>(dim));
            for(int i=0;i<dim;++i) rhs[i]=-r[i];
            std::vector<double> dx;
            if(!solve_linear(jac,rhs,dx)||!finite_vec(dx)) return false;
            double scale=1.0;
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
                double f2,p2;
                const auto rr=static_residual(
                    model, base, sample, input.gravity_x, input.gravity_y,
                    input.gravity_z, active_tires, trial, f2, p2
                );
                const double merit =
                    p2/input.position_tolerance + f2/input.dynamics_tolerance;
                if(finite_vec(rr)&&merit<base_merit){
                    x=std::move(trial);
                    accepted=true;
                    break;
                }
                scale*=0.5;
            }
            if(!accepted) return false;
        }
        if (!converged) return false;

        std::vector<double> dy(x.begin(),x.begin()+model.ndof);
        const State candidate=pose_candidate(base,model,dy);
        std::vector<int> next_active;
        std::vector<double> geometric_compression(model.tires.size(), 0.0);
        for (std::size_t i = 0; i < model.tires.size(); ++i) {
            const Tire& tire = model.tires[i];
            const Vec3 center =
                state_point(candidate, tire.body, tire.center);
            const double road =
                i < sample.road_z.size() ? sample.road_z[i] : 0.0;
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
                if (compression > input.position_tolerance) {
                    if (compression >
                        tire.maximum_compression+input.position_tolerance) {
                        return false;
                    }
                    next_active.push_back(static_cast<int>(i));
                }
            } else if (geometric_compression[i] >
                       input.position_tolerance) {
                next_active.push_back(static_cast<int>(i));
            }
        }
        if (next_active == active_tires) {
            for (std::size_t i = 0; i < model.tires.size(); ++i) {
                if (std::find(
                        active_tires.begin(), active_tires.end(),
                        static_cast<int>(i)
                    ) == active_tires.end() &&
                    geometric_compression[i] > input.position_tolerance) {
                    return false;
                }
            }
            state=candidate;
            constraint_multiplier.assign(
                x.begin()+model.ndof,
                x.begin()+model.ndof+model.rows
            );
            return true;
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
        x=std::move(next_x);
    }
    return false;
}

Model build_model(const AxleInput& in, std::string& error) {
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
        if(norm(c.axis_a)<kEps||norm(c.axis_b)<kEps){error="constraint axis must be nonzero";return m;}
        c.row=m.rows; const int rows=constraint_rows(c.type); if(rows<0){error="unsupported constraint type";return m;}
        m.rows+=rows; m.constraints.push_back(c);
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
    interpolate_input(input, time+weight*h, sample);
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
    interpolate_input(input, time, sample_input);
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

extern "C" int axle_run(const AxleInput* input, AxleOutput* output,
                         char* error_buffer, std::size_t error_capacity) {
    if (!input || !output) { set_error(error_buffer,error_capacity,"input/output is null"); return 1; }
    if (input->sample_count<2 || !input->sample_times || !output->body_state || !output->diagnostics) {
        set_error(error_buffer,error_capacity,"sample arrays are missing or too short"); return 1;
    }
    std::string error;
    Model model=build_model(*input,error);
    if(!error.empty()){set_error(error_buffer,error_capacity,error);return 2;}
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
                current_constraint_multiplier,trim_pinned_directions
            )){
            set_error(error_buffer,error_capacity,"static equilibrium initialization failed");
            return 6;
        }
    }else{
        const auto position_residual=constraint_residual(model,state);
        trim_position=max_abs(position_residual);
        if(trim_position>input->position_tolerance){
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
         model, state, input->velocity_tolerance, initial_velocity_residual
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
        while(t<target-1e-14){
            const double remaining=target-t;
            double h=std::min(
                input->adaptive_step!=0 ? suggested_step : input->internal_step,
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
                if(input->adaptive_step!=0){
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
                    if(input->adaptive_step==0){
                        failed=true;
                        break;
                    }
                    if(h<=reduction_floor*(1.0+1e-12)){
                        failed=true;
                        break;
                    }
                    h=std::max(reduction_floor,h*reduction);
                    if(input->adaptive_step!=0) suggested_step=h;
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
