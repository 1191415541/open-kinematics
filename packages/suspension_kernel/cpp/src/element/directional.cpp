// The element half of the directional (dual-number) force pass: the
// aerodynamic drag, the spring, the bushing and the anti-roll bar, the
// steering actuators, the pack that calls them, the drive/brake torques
// and the detached-tire rate write.  Split out of
// `src/vehicle/kernel_directional.cpp`; the bodies are unchanged.

#include "mb_element/functions.hpp"

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

// K4 (epic MODULES.md section 3.2): The aerodynamic drag of the directional pass: the drag axis, the application-point velocity
// and the along-axis force at the application arm.
void external_force_aerodynamic_directional(
    const Model& model,
    const State& state,
    const DirectionalState& direction,
    double external_load_scale,
    std::vector<Vec3>& force,
    std::vector<Vec3>& torque,
    bool& smooth) {
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
}

// K4 (epic MODULES.md section 3.2): The spring elements of the directional pass: the elastic, damper and two stop branches,
// each with its analytic derivative and its own smoothness probe.
void external_force_spring_directional(
    const Model& model,
    const State& state,
    const DirectionalState& direction,
    const StaticContactOverride* static_contact,
    double internal_force_scale,
    std::vector<Vec3>& force,
    std::vector<Vec3>& torque,
    bool& smooth) {
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
}

// K4 (epic MODULES.md section 3.2): The bushing elements of the directional pass: the frame-relative pose, the elastic and
// viscous wrenches and the force/moment application.
void external_force_bushing_directional(
    const Model& model,
    const State& state,
    const DirectionalState& direction,
    double internal_force_scale,
    std::vector<Vec3>& force,
    std::vector<Vec3>& torque,
    bool& smooth) {
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
}

// K4 (epic MODULES.md section 3.2): The anti-roll bars of the directional pass.  It applies a torque only, so it takes no
// force buffer.
void external_force_anti_roll_directional(
    const Model& model,
    const State& state,
    const DirectionalState& direction,
    double internal_force_scale,
    std::vector<Vec3>& torque,
    bool& smooth) {
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
}

// K4 (epic MODULES.md section 3.2): The steering actuators of the directional pass: the prescribed target and its rate, the
// axis reference, the elastic and damping terms and the reaction wrench.
void external_force_steering_directional(
    const Model& model,
    const State& state,
    const SampleInput& input,
    const DirectionalState& direction,
    std::vector<Vec3>& force,
    std::vector<Vec3>& torque,
    bool& smooth) {
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

DirectionalElementForces assemble_directional_elements(
    const Model& model,
    const State& state,
    const SampleInput& input,
    const DirectionalState& direction,
    const StaticContactOverride* static_contact,
    bool brush_only,
    double external_load_scale,
    double internal_force_scale
) {
    DirectionalElementForces out;
    out.force.resize(static_cast<std::size_t>(model.bodies.size()));
    out.torque.resize(static_cast<std::size_t>(model.bodies.size()));
    std::vector<Vec3>& force = out.force;
    std::vector<Vec3>& torque = out.torque;
    bool& smooth = out.smooth;
    if (!brush_only) {
        external_force_aerodynamic_directional(
            model, state, direction, external_load_scale, force, torque, smooth
        );
        external_force_spring_directional(
            model, state, direction, static_contact, internal_force_scale, force, torque, smooth
        );

        external_force_bushing_directional(
            model, state, direction, internal_force_scale, force, torque, smooth
        );

        external_force_anti_roll_directional(
            model, state, direction, internal_force_scale, torque, smooth
        );

        external_force_steering_directional(
            model, state, input, direction, force, torque, smooth
        );
    }
    return out;
}

DirectionalDriveTorques assemble_directional_drive_torques(
    const Model& model,
    const State& state,
    const SampleInput& input,
    const DirectionalState& direction,
    const std::vector<Vec3>& torque_in
) {
    DirectionalDriveTorques out;
    out.torque = torque_in;
    std::vector<Vec3>& torque = out.torque;
    bool& smooth = out.smooth;
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
                }
                // 与标量路径逐字一致：静止/近静止轮（`|axial_rate| <= kEps`）不加制动矩。
                // 旧写法是裸 `axial_rate.value > 0.0`，于是静止轮在标量（残差）路径得到 0、
                // 在方向（Jacobian）路径得到满额制动矩——两条路径对同一状态给出不同的力矩，
                // Newton 的下降方向因此与残差不一致，表现为 `no descent` 且残差停在一个与
                // 制动矩成正比的平台上。
                if (axial_rate.value > kEps) {
                    brake_torque = -brake_magnitude;
                } else if (axial_rate.value < -kEps) {
                    brake_torque = brake_magnitude;
                }
                add_directional_torque(
                    torque, model, t.body, tire_axis*brake_torque
                );
                // 与标量路径同理：制动矩的反作用属于不旋转的转向节。缺这一项时
                // 整车会获得等于总制动矩的寄生俯仰力偶，抵消掉大部分 `Fx * h`
                // 载荷转移（实测缺口 = Στ_brake / L，逐点吻合 0.1%）。
                if (frame_body != t.body) {
                    add_directional_torque(
                        torque, model, frame_body,
                        tire_axis*(-brake_torque)
                    );
                }
            }
        }
    return out;
}

// K4 (epic MODULES.md section 3.2): the directional detachment write.  Both places
// in the tire loop that abandon a tire -- when the contact compression is
// non-positive and when the vertical force is -- write the same two relaxed rates
// into the brush-derivative slots.  The `continue` stays at the call sites, and
// the one `smooth` write the first site makes is not part of this write.
void write_directional_detachment(
    const Tire& t,
    const DirectionalScalar& sx,
    const DirectionalScalar& sy,
    std::size_t i,
    int stride,
    std::vector<double>& tire_state_derivatives
) {
    const DirectionalScalar detached_x = -sx/t.detached_relaxation;
    const DirectionalScalar detached_y = -sy/t.detached_relaxation;
    tire_state_derivatives[stride*i] = detached_x.derivative;
    tire_state_derivatives[stride*i+1] = detached_y.derivative;
}

} // namespace axle_kernel
