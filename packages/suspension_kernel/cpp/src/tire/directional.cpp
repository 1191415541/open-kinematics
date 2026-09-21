// The tire half of the directional (dual-number) force pass: the contact
// frame, the carrier origin and contact arm, the Fiala slips, the
// static-contact override, and the PAC2002, Fiala and brush force laws.
// Split out of `src/vehicle/kernel_directional.cpp`; the bodies are
// unchanged.

#include "mb_tire/functions.hpp"

// Direct dependencies of this translation unit.  The module headers no
// longer aggregate each other's declarations, so each unit includes the
// modules whose functions it actually calls.
#include "mb_config/functions.hpp"
#include "mb_dual/functions.hpp"
#include "mb_numeric/functions.hpp"
#include "mb_joint/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_tire/fiala/functions.hpp"
#include "mb_tire/pac2002/functions.hpp"
#include "mb_tire_state/functions.hpp"

namespace axle_kernel {

DirectionalTireFrame directional_tire_frame(
    const Tire& t,
    const Model& model,
    const State& state,
    const SampleInput& input,
    const DirectionalState& direction,
    std::size_t i,
    int frame_body,
    int center_body,
    const Vec3& center_local,
    const DVec3& center,
    const DVec3& normal,
    const StaticContactOverride* static_contact,
    bool pac2002,
    bool fiala,
    bool& smooth
) {
    DirectionalTireFrame frame;
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
    frame.vc = vc;
    frame.road_profile = road_profile;
    frame.road = road;
    frame.road_v = road_v;
    frame.delta = delta;
    frame.delta_dot = delta_dot;
    frame.loaded_radius = loaded_radius;
    frame.camber = camber;
    frame.spin_rate = spin_rate;
    frame.rolling_radius = rolling_radius;
    frame.force_application_radius = force_application_radius;
    frame.patch_arm = patch_arm;
    frame.rolling_arm = rolling_arm;
    frame.patch_velocity = patch_velocity;
    frame.forward = forward;
    frame.lateral = lateral;
    frame.vx = vx;
    frame.vy = vy;
    frame.sx = sx;
    frame.sy = sy;
    return frame;
}

// K4 (epic MODULES.md section 3.2): the directional carrier origin and contact
// arm.  Every branch of the tire loop needs the carrier's origin in world
// coordinates and the arm from it to the contact point; both are pure functions of
// the state and the tire, so they are computed once here instead of four times in
// the loop.
DVec3 directional_body_origin(
    const State& state,
    const DirectionalState& direction,
    const Tire& t
) {
    return DVec3{
        {state.r[t.body].x, direction.dr[t.body].x},
        {state.r[t.body].y, direction.dr[t.body].y},
        {state.r[t.body].z, direction.dr[t.body].z}
    };
}

DVec3 directional_contact_arm(
    const DVec3& center,
    const DVec3& body_origin,
    const DVec3& patch_arm
) {
    return center-body_origin+patch_arm;
}

DirectionalFialaSlips assemble_directional_fiala_slips(
    const Tire& t,
    std::size_t i,
    int stride,
    const DirectionalScalar& rolling_speed,
    const DirectionalScalar& sx,
    const DirectionalScalar& sy,
    const DirectionalScalar& vx,
    const DirectionalScalar& vy,
    bool& smooth,
    std::vector<double>& tire_state_derivatives
) {
    const double low_speed_threshold = std::max(
        fiala_parameter(t, FIALA_LOW_SPEED_THRESHOLD, 1e-3), 1e-3
    );
    DirectionalScalar slip_speed = rolling_speed;
    const double slip_reference = pac2002_slip_reference_speed(
        rolling_speed.value, low_speed_threshold
    );
    if (slip_speed.value < slip_reference) {
        smooth = false;
        slip_speed = {slip_reference, 0.0};
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
    tire_state_derivatives[stride*i] = state_rate_x.derivative;
    tire_state_derivatives[stride*i+1] = state_rate_y.derivative;
    DirectionalFialaSlips slips;
    slips.longitudinal = longitudinal_slip;
    slips.lateral = lateral_slip;
    return slips;
}

// K4 (epic MODULES.md section 3.2): the directional static-contact override.  It
// is the leading block of the tire loop, taken when a static solve supplies the
// contact state instead of the integrated tire states: the vertical force comes
// from the imposed compression, the arm is corrected for the spin axis when the
// tire has a directional vertical law, and the wrench is applied at that arm.
// Every exit is a skip -- the caller keeps the `continue` -- so the helper is
// `void`.  `smooth` is written through by the two `d_normalized` calls and by the
// vertical-projection floor.
void apply_static_contact_directional(
    std::vector<Vec3>& force, std::vector<Vec3>& torque, const Model& model,
    const State& state, const DirectionalState& direction,
    const StaticContactOverride* static_contact, const Tire& t, std::size_t i,
    const DVec3& normal, bool pac2002, bool fiala, double internal_force_scale,
    bool& smooth) {
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
                {},
                {},
                {},
                t.maxwell_enabled
                    ? DirectionalScalar{pac2002_maxwell_end_displacement(
                          t, slot_value(state.tire_maxwell, i),
                          compression, state.step_size
                      )}
                    : DirectionalScalar{},
                smooth
            )
            : DirectionalScalar{
                t.k*compression,
                t.k*compression_derivative
            }
    );
    if (normal_force.value < 0.0) return;
    const DVec3 body_origin =
        directional_body_origin(state, direction, t);
    const DVec3 contact_center = d_state_point(
        state, direction.dr, direction.dtheta, t.body, t.center
    );
    const DirectionalScalar contact_radius =
        DirectionalScalar{t.radius}
        -DirectionalScalar{compression, compression_derivative};
    DVec3 contact_arm = contact_center-body_origin
        +normal*(-contact_radius);
    if (pac2002 || fiala) {
        const int frame_body = tire_frame_body(t);
        const DVec3 spin_axis = d_normalized(
            d_rotate(
                state.q[frame_body], t.spin_axis,
                direction.dtheta[frame_body]
            ),
            smooth
        );
        const DVec3 radial_down = d_normalized(
            (normal-spin_axis*d_dot(spin_axis, normal))*(-1.0),
            smooth
        );
        DirectionalScalar vertical_projection = -d_dot(
            radial_down, normal
        );
        if (vertical_projection.value <= 1.0e-9) {
            smooth = false;
            vertical_projection = {1.0e-9, 0.0};
        }
        contact_arm = contact_center-body_origin+radial_down*(
            contact_radius/vertical_projection
        );
    }
    add_directional_force_at_arm(
        force, torque, model, t.body, contact_arm,
        normal*normal_force
    );
    return;
}

// K4 (epic MODULES.md section 3.2): the directional PAC2002 force law.  It is the
// `if (pac2002) { ... continue; }` run of the tire loop: the USE_MODE gates, the
// slip speeds and relaxed slips, the transient state rates written into the
// brush-derivative slots, the turn slip, the two pure forces with their combined
// slip correction and utilisation cap, the three moments, and the wrench
// application.  Its only exit is the trailing `continue`, so it is `void`.
void assemble_directional_pac2002_force(
    std::vector<Vec3>& force,
    std::vector<Vec3>& torque,
    const Model& model,
    const State& state,
    const DirectionalState& direction,
    const Tire& t,
    std::size_t i,
    int stride,
    int frame_body,
    const DVec3& center,
    const DVec3& normal,
    const DirectionalScalar& normal_force,
    const DirectionalScalar& rolling_speed,
    const DVec3& vc,
    const DVec3& forward,
    const DVec3& lateral,
    const DirectionalScalar& camber,
    const DirectionalScalar& spin_rate,
    const DirectionalScalar& rolling_radius,
    const DirectionalScalar& loaded_radius,
    const DVec3& patch_arm,
    const DirectionalScalar& sx,
    const DirectionalScalar& sy,
    const DirectionalScalar& vx,
    const DirectionalScalar& vy,
    bool brush_only,
    bool& smooth,
    std::vector<double>& tire_state_derivatives) {
    // 与标量分支相同，Adams 高性能轮胎的局部求解器包含
    // USE_MODE=14 的纵向和侧向一阶松弛状态。
    const int use_mode = pac2002_use_mode(t);
    // Keep the directional derivatives consistent with the scalar
    // branch: USE_MODE 0 is vertical-only, so no slip force or moment
    // takes a derivative with respect to the tire state.  Leaving this
    // unguarded would let the Jacobian disagree with the residual.
    const bool vertical_only = pac2002_mode_is_vertical_only(use_mode);
    const bool allow_longitudinal =
        !vertical_only && pac2002_mode_allows_longitudinal(use_mode);
    const bool allow_lateral =
        !vertical_only && pac2002_mode_allows_lateral(use_mode);
    const bool transient_state =
        pac2002_mode_uses_transient_state(use_mode);
    const double low_speed = pac2002_low_speed_threshold(t);
    DirectionalScalar slip_speed = rolling_speed;
    // The floor on the slip denominator is the disputed part of the low-speed
    // handling; pac2002_slip_reference_speed() decides whether it applies.
    const double slip_reference = pac2002_slip_reference_speed(
        rolling_speed.value, low_speed
    );
    if (slip_speed.value < slip_reference) {
        smooth = false;
        slip_speed = {slip_reference, 0.0};
    }
    const DirectionalScalar longitudinal_slip = d_clamp(
        -vx/slip_speed, -1.0, 1.0, smooth
    );
    const DirectionalScalar lateral_slip = d_clamp(
        d_atan2(vy, slip_speed),
        -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
    );
    const DirectionalScalar effective_longitudinal_slip = transient_state
        ? sx : longitudinal_slip;
    const DirectionalScalar effective_lateral_slip = transient_state
        ? sy : lateral_slip;
    DirectionalScalar longitudinal_state_rate{};
    DirectionalScalar lateral_state_rate{};
    if (transient_state) {
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
        longitudinal_state_rate =
            longitudinal_relaxation*(longitudinal_slip-sx);
        lateral_state_rate =
            lateral_relaxation*(lateral_slip-sy);
    }
    tire_state_derivatives[stride*i] = longitudinal_state_rate.derivative;
    tire_state_derivatives[stride*i+1] = lateral_state_rate.derivative;
    // Turn slip of the directional pass.  It must be the *same value* the
    // residual uses, or the analytic Jacobian differentiates a different
    // force than the one the residual evaluates; the turn-slip states come
    // from the state vector and enter without a directional perturbation,
    // because a contact-mass tire keeps every force column numeric
    // (see nonlinear_relaxation in fill_analytic_jacobian_columns).
    Pac2002TurnSlipDirectional turn_slip{};
    DirectionalScalar total_spin_rate{};
    if (pac2002_mode_uses_turn_slip(use_mode)) {
        const DirectionalScalar yaw_rate = d_dot(
            DVec3{
                {state.omega[frame_body].x, direction.domega[frame_body].x},
                {state.omega[frame_body].y, direction.domega[frame_body].y},
                {state.omega[frame_body].z, direction.domega[frame_body].z}
            },
            normal
        );
        const DirectionalScalar epsilon_gamma =
            pac2002_parameter(t, PAC_PECP1, 0.0)
            *(1.0+pac2002_parameter(t, PAC_PECP2, 0.0)
                *((normal_force-pac2002_reference_load(t))
                    /pac2002_reference_load(t)));
        // This block must reproduce the residual's turn slip exactly, or the
        // analytic Jacobian differentiates a different force than the one the
        // residual evaluates.  It previously omitted camber_spin_sign, the
        // output-side spin_sign and both isolation switches, so the two paths
        // disagreed whenever any of them was moved; a contact-mass tire keeps
        // every force column numeric, which is the only reason that never
        // showed up in a measurement.
        const double camber_spin_sign = pac2002_turn_slip_switch(
            "PAC2002_TURN_SLIP_CAMBER_SIGN", 1.0
        );                total_spin_rate = yaw_rate
            -camber_spin_sign*(1.0-epsilon_gamma)*spin_rate*d_sin(camber);
        const DirectionalScalar phi_c{
            tire_state_value(state, i, 8), 0.0
        };
        const DirectionalScalar phi_f2{
            tire_state_value(state, i, 9), 0.0
        };
        const DirectionalScalar phi_1{
            tire_state_value(state, i, 10), 0.0
        };
        const DirectionalScalar phi_2{
            tire_state_value(state, i, 11), 0.0
        };
        const double spin_sign = pac2002_turn_slip_switch(
            "PAC2002_TURN_SLIP_SPIN_SIGN", 1.0
        );
        turn_slip.force = spin_sign*(2.0*phi_c-phi_f2);
        turn_slip.force = turn_slip.force * DirectionalScalar{
            pac2002_turn_slip_switch("PAC2002_TURN_SLIP_FORCE_SIGN", 1.0)
        };
        turn_slip.moment = spin_sign*(
            pac2002_parameter(t, PAC_EP, 1.0)*phi_c
            +pac2002_parameter(t, PAC_EP12, 3.0)*(phi_1-phi_2)
        );
        if (pac2002_turn_slip_switch(
                "PAC2002_TURN_SLIP_DISABLE", 0.0
            ) > 0.5) {
            turn_slip.force = DirectionalScalar{};
            turn_slip.moment = DirectionalScalar{};
        }
        turn_slip.travel_sign =
            d_dot(vc, forward).value < 0.0 ? -1.0 : 1.0;
    }
    DirectionalScalar fx = allow_longitudinal
        ? pac2002_pure_force_directional(
            t, effective_longitudinal_slip, normal_force, false,
            camber, turn_slip, smooth
        )
        : DirectionalScalar{};
    DirectionalScalar fy = allow_lateral
        ? pac2002_pure_force_directional(
            t, effective_lateral_slip, normal_force, true,
            camber, turn_slip, smooth
        )
        : DirectionalScalar{};
    const bool combined_slip =
        pac2002_mode_allows_combined(use_mode)
        && pac2002_has_combined_slip_terms(t);
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
    DirectionalScalar aligning_moment = allow_lateral
        ? pac2002_aligning_moment_directional(
            t, effective_longitudinal_slip,
            effective_lateral_slip, normal_force, camber, fx, fy,
            turn_slip, smooth
        )
        : DirectionalScalar{};
    if (allow_lateral &&
        t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
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
    const DirectionalScalar overturning_moment = allow_lateral
        ? pac2002_overturning_moment_directional(
            t, fy, normal_force, camber, smooth
        )
        : DirectionalScalar{};
    DirectionalScalar rolling_resistance_moment = allow_longitudinal
        ? pac2002_rolling_resistance_moment_directional(
            t, fx, normal_force, camber, rolling_speed, smooth
        )
        : DirectionalScalar{};
    if (allow_longitudinal &&
        t.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
        rolling_resistance_moment +=
            -fx*(rolling_radius-loaded_radius);
    }
    if (!brush_only) {
        const DVec3 contact_force =
            forward*fx+lateral*fy+normal*normal_force;
        const DVec3 body_origin =
            directional_body_origin(state, direction, t);
        const DVec3 contact_arm =
            directional_contact_arm(center, body_origin, patch_arm);
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
    if (pac2002_mode_uses_turn_slip(use_mode)) {
        // Turn-slip slots in the directional pass.  Their derivative with
        // respect to the perturbation direction is deliberately left zero:
        // a contact-mass tire marks every force column numeric (see
        // nonlinear_relaxation), so nothing reads these entries, and the
        // values the residual integrates are written by the double pass.
        for (int k = 0; k < 4; ++k) {
            tire_state_derivatives[
                static_cast<std::size_t>(stride)*i + 8
                    + static_cast<std::size_t>(k)
            ] = 0.0;
        }
    }
}

// K4 (epic MODULES.md section 3.2): the directional Fiala wrench.  The slip and
// state-rate half stays in the loop (`assemble_directional_fiala_slips`); this is
// the `if (!brush_only) { ... }` force half: the Fiala force law on the transient
// or the relaxed slips, the startup scale, the contact wrench at the patch arm and
// the rolling-resistance moment with its non-smooth fade window.  The guard is
// kept inside, so the helper is a no-op for the brush-only pass.
void apply_directional_fiala_force(
    std::vector<Vec3>& force,
    std::vector<Vec3>& torque,
    const Model& model,
    const State& state,
    const DirectionalState& direction,
    const SampleInput& input,
    const Tire& t,
    const DVec3& center,
    const DVec3& normal,
    const DirectionalScalar& normal_force,
    const DVec3& forward,
    const DVec3& lateral,
    const DVec3& patch_arm,
    const DirectionalScalar& spin_rate,
    const DirectionalScalar& sx,
    const DirectionalScalar& sy,
    const DirectionalScalar& longitudinal_slip,
    const DirectionalScalar& lateral_slip,
    bool brush_only,
    bool& smooth) {
    if (!brush_only) {
        DirectionalScalar fx, fy, mz;
        // Same rule as the scalar path: bypass the relaxed states when the
        // tire's USE_MODE selects no slip transient.  Bypassing also makes
        // the force independent of those states, so the analytic columns
        // for the state slots correctly become zero.
        const bool transient = fiala_transient_enabled(fiala_use_mode(t));
        fiala_forces_directional(
            t, transient ? sx : longitudinal_slip,
            transient ? sy : lateral_slip,
            normal_force, fx, fy, mz, smooth
        );
        // Same startup step as the scalar path.  The factor depends on
        // time only, so scaling the directional values scales their
        // derivatives by the same amount, which is what the chain rule
        // requires.
        const double startup = fiala_startup_scale(
            fiala_use_mode(t), input.time
        );
        fx = fx * startup;
        fy = fy * startup;
        mz = mz * startup;
        const DVec3 contact_force = forward*fx+lateral*fy
            +normal*normal_force;
        const DVec3 body_origin =
            directional_body_origin(state, direction, t);
        add_directional_force_at_arm(
            force, torque, model, t.body,
            center-body_origin+patch_arm, contact_force
        );
        // The rolling-resistance factor is built from the spin rate's
        // value; its only state dependence is sign(omega), which is flat
        // except inside the narrow fade window, so the column is marked
        // non-smooth there instead of differentiating a kink.  The same
        // startup step as the scalar path scales the moment, so the two
        // paths stay consistent.
        const double rolling_speed_abs = std::abs(spin_rate.value);
        if (rolling_speed_abs > 0.125 && rolling_speed_abs < 0.5) {
            smooth = false;
        }
        add_directional_torque(
            torque, model, t.body,
            normal*mz
            +lateral*(fiala_parameter(t, FIALA_ROLLING_RESISTANCE, 0.0)
                *fiala_rolling_resistance_factor(spin_rate.value)
                *normal_force*startup)
        );
    }
}

// K4 (epic MODULES.md section 3.2): the directional brush-model tail.  It is the
// fall-through path of the tire loop and the only branch with no `continue`: the
// two relaxation rates written into the brush-derivative slots, the projected slip
// with its active-set non-smooth marker, and -- unless this is the brush-only pass
// -- the contact wrench at the patch arm.
void apply_directional_brush_force(
    std::vector<Vec3>& force,
    std::vector<Vec3>& torque,
    const Model& model,
    const State& state,
    const DirectionalState& direction,
    const Tire& t,
    std::size_t i,
    int stride,
    const DVec3& center,
    const DVec3& normal,
    const DirectionalScalar& normal_force,
    const DirectionalScalar& rolling_speed,
    const DVec3& forward,
    const DVec3& lateral,
    const DVec3& patch_arm,
    const DirectionalScalar& sx,
    const DirectionalScalar& sy,
    const DirectionalScalar& vx,
    const DirectionalScalar& vy,
    bool brush_only,
    bool& smooth,
    std::vector<double>& tire_state_derivatives) {
    const DirectionalScalar brush_x =
        vx-rolling_speed/t.relaxation_length_longitudinal*sx;
    const DirectionalScalar brush_y =
        vy-rolling_speed/t.relaxation_length_lateral*sy;
    tire_state_derivatives[stride*i] = brush_x.derivative;
    tire_state_derivatives[stride*i+1] = brush_y.derivative;
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
        const DVec3 body_origin =
            directional_body_origin(state, direction, t);
        const DVec3 contact_arm =
            directional_contact_arm(center, body_origin, patch_arm);
        add_directional_force_at_arm(
            force, torque, model, t.body, contact_arm, contact_force
        );
    }
}

} // namespace axle_kernel
