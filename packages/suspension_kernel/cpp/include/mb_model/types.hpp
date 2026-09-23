#pragma once

/// The model's element and topology types (MB_MODEL).
///
/// Bodies, joints, the five force-element families and the singleton
/// registration records.  `Tire` is here rather than in the tire layer
/// because `Model` holds a `std::vector<Tire>` and the shared vertical
/// force takes a `const Tire&`; putting it in the tire layer would make
/// `mb_model` and `mb_tire` each depend on the other.

#include "mb_numeric/vector.hpp"
#include "mb_model/enums.hpp"
#include <array>
#include <string>
#include <vector>

namespace axle_kernel {

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
    // Driven coordinates only (AXLE_DRIVEN_TRANSLATION/ROTATION): the zero-angle
    // pose of body ``a`` relative to ``b`` that the rotational row measures from,
    // and the index of this coordinate's target in the case's driven-signal
    // arrays.  A negative signal means a constant zero target (Adams' MOTION/34).
    Quat reference{};
    int signal{-1};
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
    // Per-tire width of the tire-state block, resolved once at registration time
    // from the USE_MODE-derived PAC2002 mode table (D10).
    //
    // The base value of 2 is the linear transient `sx`/`sy` pair and applies to
    // every tire kind: a non-PAC2002 tire must still get its two slots, so the
    // default is never zero.  Caching this on the tire is what lets the tire-state
    // layer compute a block width without calling into the PAC2002 layer, which
    // would otherwise make `mb_tire_state` depend on `mb_tire_pac2002`.
    int state_slot_width{2};
    // Set when the tire's USE_MODE selects the non-linear (advanced) transient
    // contact-mass model (21-25).  Those modes carry four extra tire states
    // (u, v, beta, beta_dot) on top of the linear transient sx/sy pair, so the
    // unknown-vector width depends on it.  See tire_state_width().
    bool contact_mass{false};
    // Non-rolling vertical Maxwell element (A5).  Switched on by the tire file's
    // USE_DYNAMIC_STIFFNESS and a non-zero DYNAMIC_STIFFNESS; with either absent the
    // vertical force is exactly the pre-Maxwell law.
    bool maxwell_enabled{false};
    // --- State semantics (K5, D14) -------------------------------------------
    //
    // The integrator asks these three questions instead of comparing `model_kind`.
    // They are what makes a new tire model a registration-time change rather than
    // an edit to the residual, the Jacobian and the step driver: a model that
    // differs in one of these respects sets the flag, and a model that differs in
    // none of them needs no integrator change at all.
    //
    // They stay booleans on purpose.  `model_kind` is the tire model's own
    // business; the integrator only needs to know which of its three special
    // treatments applies.
    //
    /// The generic BDF2 relaxation row is replaced by the exact exponential of the
    /// affine relaxation ODE.  True for Fiala, whose relaxation is linear in the
    /// slip with frozen coefficients within a step.
    bool uses_exact_relaxation{false};
    /// The tire state is projected back onto the model's admissible set after an
    /// accepted step.  True for the brush model, which is the only kind with a
    /// friction-ellipse return mapping.
    bool has_state_return_mapping{false};
    /// The force responds to the normal load and the slip within a step, so the
    /// Newton linearization may be reused across several steps.  True for both
    /// PAC2002 kinds.
    bool uses_pac2002_law{false};
    /// Contact kinematics are evaluated at the wheel/spindle centre rather than at
    /// the carrier centre (D8).  True for the Adams PAC2002 source and for Fiala;
    /// the carrier stays the orientation frame either way.  Registering this keeps
    /// the two contact-geometry helpers in the model layer instead of making them
    /// reach into the tire models to ask which kind they are.
    bool evaluates_at_wheel_center{false};
    /// The compression is the wheel-plane/road-plane intersection rather than the
    /// plain vertical gap: the gap divided by the spin axis' vertical projection
    /// (D8, §2.5's `project_compression_on_spin`).  True for Fiala.
    bool projects_compression_on_spin{false};
    std::array<double, VEHICLE_PAC2002_PARAMETER_COUNT> pac2002_parameters{};
    // `[DEFLECTION_LOAD_CURVE]` as flattened (deflection_m, load_n) rows.  Empty means
    // the tire file has no such curve and the vertical force keeps the polynomial law.
    std::vector<double> deflection_curve;
    // `[BOTTOMING_CURVE]` as flattened (rim_penetration_m, load_n) rows.  Empty means
    // the tire file has no such curve and no rim reaction is added.
    std::vector<double> bottoming_curve;
    // --- Tire mass ownership (subtask 08, D2 option B) -----------------------
    //
    // The tire's own mass and inertia, in kernel SI units.  Before these two
    // fields the wheel-end body carried the tire's mass, so the tire had no
    // independent inertia to couple.  Zero mass means "no independent inertia":
    // the wheel-end body still carries it, and every document that does not
    // declare the fields keeps its historical answer bit for bit.
    //
    // They are appended at the end of the structure on purpose.  `Tire` is not
    // part of the frozen C ABI (`AxleInput`/`VehicleInput` are, and neither
    // carries a tires-by-value array), but the assembly and the solver read it
    // through offsets the compiler is free to lay out again -- keeping the new
    // fields last is what makes every pre-existing field offset provably
    // unchanged, which the kernel test asserts.
    //
    // The document reaches these fields through `ContractModel`, not through a
    // new `AxleInput` array: adding one to a frozen structure would be an ABI
    // freeze release, and this step deliberately does not open one.  See
    // `contract_model.cpp` (`tire_mass_`/`tire_inertia_`) and
    // `kernel_contract_run.cpp`, which installs them on the built model the
    // same way it installs `body_wrench_point_local`.
    /// The tire's own mass in kg.  Zero keeps the wheel-end body's ownership.
    double mass{0.0};
    /// The tire's own inertia tensor about its centre, in kg*m^2.
    Mat3 inertia{};
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


struct ContactEventRecord {
    double time{0.0};
    int tire_index{-1};
    int transition{0};
};

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

struct DrivenSignal {
    const double* values{nullptr};
    const double* rates{nullptr};
    int column{0};
    int stride{0};
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
    // Driven coordinates store their zero-angle reference and signal index on the
    // constraint descriptor itself.  ``driven_signals`` is the per-signal source
    // table: a driven row reads ``SampleInput::driven_target[signal]``, and
    // ``interpolate_input`` fills that vector from whichever per-sample table the
    // signal points at.  Keeping the source explicit is what lets the generic
    // driver and the prescribed steering actuators share one row implementation
    // instead of the steering path carrying its own copy of the constraint math.
    bool has_driven_coordinates{false};
    std::vector<DrivenSignal> driven_signals;
    std::vector<StaticRotationGauge> static_rotation_gauges;
    RoadProfile road_profile{};
    const double* vehicle_brake_torque{nullptr};
    int static_gauge_body{-1};
    std::uint32_t static_gauge_dof_mask{0};
    bool static_trim_then_release{false};
    double initial_state_angle_tolerance{0.0};
    std::vector<Vec3> release_velocity;
    std::vector<Vec3> release_omega;
    // Body-fixed points at which the per-sample ``body_wrench`` acts, in metres
    // and in body order, one entry per body.  Empty means every wrench acts at
    // its body origin.  A force applied at a body-fixed point makes a moment
    // about the origin equal to ``cross(R*p_local, F)``, and that lever arm is
    // a function of the pose being solved for: the solver has to resolve it
    // rather than accept a wrench already transferred at a reference pose.
    // Frozen at the reference pose the arm loses the first-order geometry of
    // the swept body -- measured as 2.8e-8 rad on a 2.5e-5 rad response,
    // against a 1e-8 rad acceptance tolerance.
    std::vector<Vec3> body_wrench_point_local;
    // Per-body inertia after the tires are taken into account, in body order.
    //
    // A tire used to carry no mass of its own: the wheel-end body owned the
    // whole wheel, and the solver read `Body::mass` / `Body::inertia_body`
    // directly.  When a document declares the tire's own mass (decision D2), the
    // inertia has two owners, and the solver must see their sum.  Summing here,
    // once, at build time is what keeps that a property of the *model*: every
    // consumer reads the same number, and no consumer has to know a tire exists.
    //
    // Empty means "not computed": a consumer then falls back to the body's own
    // values, which is both the historical answer and the answer for a document
    // that declares no tire mass.  `compute_effective_body_inertia` fills them.
    std::vector<double> body_effective_mass;
    std::vector<Mat3> body_effective_inertia_body;
    std::vector<int> free_body;
    std::vector<int> body_to_free;
    int rows{0};
    int ndof{0};
};


/// Fill `Model::body_effective_mass` / `body_effective_inertia_body` from the
/// bodies and the tires the model already carries.
///
/// `m_eff = m_body + sum(tire mass)` and
/// `I_eff = I_body + sum(tire inertia + tire mass * ((c.c)E - c (x) c))` with
/// `c` the tire centre in the carrying body's frame -- the parallel-axis term the
/// wheel end needs once its tire is a separate inertia source.  A body with no
/// massive tire gets its own values back bit for bit, so the historical path is
/// untouched.
///
/// Returns false, with `error` naming the tire, when a massive tire is mounted
/// away from its body's origin.  The residual evaluates a body's inertia with no
/// lever arm (it treats the body origin as the centre of mass); honouring an
/// offset would change the translational/rotational coupling, which is new
/// physics rather than a change of ownership and needs its own acceptance.  The
/// caller reports it instead of silently dropping the term.
bool compute_effective_body_inertia(Model& model, std::string& error);

struct State {
    std::vector<Vec3> r, v, a, omega, alpha;
    std::vector<Quat> q;
    // Linear transient relaxation pair, present for every PAC2002 tire.
    std::vector<double> tire_sx, tire_sy;
    std::vector<double> tire_sx_dot, tire_sy_dot;
    // Non-linear (advanced) transient contact-mass states, USE_MODE 21-25:
    // the contact body's in-plane deflection (u, v) and its yaw angle and rate.
    // The deflection pair shares the meaning of sx/sy, so the contact body's
    // second-order dynamics govern the same quantities the linear model
    // relaxes.  Empty for the other tire models and for USE_MODE <= 14.
    std::vector<double> tire_u, tire_v;
    std::vector<double> tire_u_dot, tire_v_dot;
    std::vector<double> tire_beta, tire_beta_dot;
    // Time derivatives of the contact-body slots above.  The generalized-alpha
    // z-state update needs the previous derivative of every slot, and only the
    // slip pair carries its own in the unknown vector (``tire_sx_dot``); the
    // rate slots' derivatives are the contact-body accelerations and are kept
    // here after each accepted step.
    std::vector<double> tire_u_ddot, tire_v_ddot, tire_beta_ddot;
    // Turn-slip relaxation states of USE_MODE 25 (Eq3963-Eq3966).  Four per tire,
    // packed as ``4*i + k`` with k = 0 phi'_c, 1 phi'_F2, 2 phi'_1, 3 phi'_2, plus
    // their time derivatives for the generalized-alpha z update.  Packing them keeps
    // the plumbing one vector wide instead of four while the state slots (8..11)
    // stay individually addressable through tire_state_value().  Empty below mode 25.
    std::vector<double> tire_phi;
    std::vector<double> tire_phi_dot;
    // Maxwell element of the non-rolling vertical model (A5): the internal
    // displacement of its dynamic branch, one per tire, carried between steps and
    // updated after each accepted step.  Empty unless a tire switches the element on.
    std::vector<double> tire_maxwell;
    // Size of the step being evaluated.  The Maxwell branch is integrated
    // analytically over a step, so its force is a function of the step size; the
    // residual stamps the step it is integrating onto the states it evaluates.
    double step_size{0.0};
};

struct SampleInput {
    std::vector<double> body_wrench, road_z, road_v, torque, brake_torque;
    std::vector<double> steering_target, steering_target_rate,
        steering_target_acceleration;
    // Targets of the model's driven coordinates (AXLE_DRIVEN_TRANSLATION /
    // AXLE_DRIVEN_ROTATION) at this sample, indexed by Constraint::signal.  The
    // rate is what the velocity-level rows and the energy ledger need: the
    // position row is ``coordinate - target``, so its time derivative carries
    // ``-target_rate``.
    std::vector<double> driven_target, driven_target_rate,
        driven_target_acceleration;
    // The time this sample was interpolated at.  The tire force law needs it for
    // the Fiala startup smoothing, and the force functions have no time argument
    // of their own.
    double time{0.0};
};


inline double slot_value(const std::vector<double>& values, std::size_t tire) {
    return tire < values.size() ? values[tire] : 0.0;
}

} // namespace axle_kernel
