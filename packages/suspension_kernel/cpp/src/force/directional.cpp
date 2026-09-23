// The directional pass's generalized-force assembly and its dispatcher.
// Split out of `src/vehicle/kernel_directional.cpp`; the bodies are
// unchanged.

#include "mb_force/functions.hpp"

// Direct dependencies of this translation unit.  The module headers no
// longer aggregate each other's declarations, so each unit includes the
// modules whose functions it actually calls.
#include "mb_element/functions.hpp"
#include "mb_tire/functions.hpp"
#include "mb_config/functions.hpp"
#include "mb_dual/functions.hpp"
#include "mb_numeric/functions.hpp"
#include "mb_joint/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_tire/fiala/functions.hpp"
#include "mb_tire/pac2002/functions.hpp"
#include "mb_tire_state/functions.hpp"

namespace axle_kernel {

// K4 (epic MODULES.md section 3.2): the directional generalized-force assembly.  It
// maps the per-body wrenches onto the free-body generalized coordinates and, for
// the bodies whose inertial directions are active, subtracts the gyroscopic term
// `omega x (R I R^T omega)`.  It runs only after the `brush_only` early return, so
// `generalized_force` stays unassigned on that path exactly as before.
void assemble_directional_generalized_force(
    std::vector<double>& generalized_force,
    const std::vector<Vec3>& force,
    const std::vector<Vec3>& torque,
    const Model& model,
    const State& state,
    const DirectionalState& direction,
    int n) {
    generalized_force.assign(static_cast<std::size_t>(n), 0.0);
    for (int fi = 0; fi < static_cast<int>(model.free_body.size()); ++fi) {
        const int body_index = model.free_body[fi];
        const bool inertial_active =
            directional_inertial_active(direction, body_index);
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
                inertia_body.a[row][col] =
                    body_effective_inertia_body(model, body_index).a[row][col];
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
}

bool external_force_directional(
    const Model& model, const State& state, const SampleInput& input,
    const DirectionalState& direction, std::vector<double>& generalized_force,
    std::vector<double>& tire_state_derivatives, bool brush_only,
    const StaticContactOverride* static_contact,
    DirectionalForceScratch* scratch) {
    const int body_count = static_cast<int>(model.bodies.size());
    const int n = model.ndof;
    // Slots per tire in the derivative array; see external_force_vector.
    const int stride = tire_block_width(model);
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
    tire_state_derivatives.assign(
        model.tires.size() * static_cast<std::size_t>(tire_block_width(model)), 0.0
    );
    const double external_load_scale = static_contact == nullptr
        ? 1.0 : static_contact->external_load_scale;
    const double internal_force_scale = static_contact == nullptr
        ? 1.0 : static_contact->internal_force_scale;

    DirectionalElementForces elements = assemble_directional_elements(
        model, state, input, direction, static_contact, brush_only,
        external_load_scale, internal_force_scale
    );
    force = std::move(elements.force);
    torque = std::move(elements.torque);
    smooth = elements.smooth;

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
            if (static_active) {
                apply_static_contact_directional(
                    force, torque, model, state, direction, static_contact, t, i,
                    normal, pac2002, fiala, internal_force_scale, smooth
                );
            }
            continue;
        }
        DirectionalTireFrame frame = directional_tire_frame(
            t, model, state, input, direction, i, frame_body, center_body,
            center_local, center, normal, static_contact,
            pac2002, fiala, smooth
        );
        const DVec3& vc = frame.vc;
        [[maybe_unused]] const DirectionalRoadProfile& road_profile = frame.road_profile;
        [[maybe_unused]] const DirectionalScalar& road = frame.road;
        [[maybe_unused]] const DirectionalScalar& road_v = frame.road_v;
        DirectionalScalar& delta = frame.delta;
        DirectionalScalar& delta_dot = frame.delta_dot;
        DirectionalScalar& loaded_radius = frame.loaded_radius;
        const DirectionalScalar& camber = frame.camber;
        const DirectionalScalar& spin_rate = frame.spin_rate;
        const DirectionalScalar& rolling_radius = frame.rolling_radius;
        [[maybe_unused]] const DirectionalScalar& force_application_radius = frame.force_application_radius;
        const DVec3& patch_arm = frame.patch_arm;
        [[maybe_unused]] const DVec3& rolling_arm = frame.rolling_arm;
        [[maybe_unused]] const DVec3& patch_velocity = frame.patch_velocity;
        const DVec3& forward = frame.forward;
        const DVec3& lateral = frame.lateral;
        const DirectionalScalar& vx = frame.vx;
        const DirectionalScalar& vy = frame.vy;
        const DirectionalScalar& sx = frame.sx;
        const DirectionalScalar& sy = frame.sy;
        if (delta.value <= 0.0) {
            if (std::abs(delta.value) <= 1e-12) smooth = false;
            write_directional_detachment(
                t, sx, sy, i, stride, tire_state_derivatives
            );
            continue;
        }
        const DirectionalScalar trial_normal = pac2002
            ? pac2002_vertical_force_directional(
                t, delta, delta_dot, camber, spin_rate, {}, {},
                t.maxwell_enabled
                    ? DirectionalScalar{
                        pac2002_maxwell_end_displacement(
                            t, slot_value(state.tire_maxwell, i), delta.value,
                            state.step_size
                        ),
                        (1.0-pac2002_maxwell_decay(t, state.step_size))
                            *delta.derivative
                    }
                    : DirectionalScalar{},
                smooth
            )
            : fiala_vertical_force_directional(t, delta, delta_dot);
        if (std::abs(trial_normal.value) <= 1e-12) smooth = false;
        if (trial_normal.value <= 0.0) {
            write_directional_detachment(
                t, sx, sy, i, stride, tire_state_derivatives
            );
            continue;
        }
        if (pac2002 && pac2002_has_vertical_force_coupling_terms(t)) {
            smooth = false;
        }
        const DirectionalScalar normal_force = trial_normal;
        const DirectionalScalar rolling_speed = d_abs(
            d_dot(vc, forward), smooth
        );
        if (pac2002) {
                assemble_directional_pac2002_force(
                    force, torque, model, state, direction, t, i, stride, frame_body, center, normal, normal_force, rolling_speed, vc, forward, lateral, camber, spin_rate, rolling_radius, loaded_radius, patch_arm, sx, sy, vx, vy, brush_only, smooth, tire_state_derivatives
                );
                continue;
            }
        if (t.model_kind == VEHICLE_TIRE_FIALA) {
            const DirectionalFialaSlips fiala_slips =
                assemble_directional_fiala_slips(
                    t, i, stride, rolling_speed, sx, sy, vx, vy,
                    smooth, tire_state_derivatives
                );
            const DirectionalScalar& longitudinal_slip =
                fiala_slips.longitudinal;
            const DirectionalScalar& lateral_slip = fiala_slips.lateral;
            apply_directional_fiala_force(
                force, torque, model, state, direction, input, t, center, normal, normal_force, forward, lateral, patch_arm, spin_rate, sx, sy, longitudinal_slip, lateral_slip, brush_only, smooth
            );
            continue;
        }
        apply_directional_brush_force(
            force, torque, model, state, direction, t, i, stride, center, normal, normal_force, rolling_speed, forward, lateral, patch_arm, sx, sy, vx, vy, brush_only, smooth, tire_state_derivatives
        );
    }

    if (brush_only) return smooth;

    DirectionalDriveTorques drive = assemble_directional_drive_torques(
        model, state, input, direction, torque
    );
    torque = std::move(drive.torque);
    smooth = drive.smooth;

    assemble_directional_generalized_force(
        generalized_force, force, torque, model, state, direction, n
    );
    return smooth;
}

} // namespace axle_kernel
