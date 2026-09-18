// The generic core entry point (`mb_core_*`).
//
// This translation unit exists to prove that the generic half of the kernel
// stands on its own: it takes bodies, joints and element blocks, and returns a
// body-state history, without any suspension semantics in view.
//
// It deliberately does **not** re-implement the solver.  `run_model` in
// `kernel_abi.cpp` already accepts a caller-built `Model` (its `model_override`
// parameter, which the contract entry point uses the same way), so the core path
// builds the model and then hands it over.  Writing a second integration and output path
// would mean two copies of the tested numerics, which is the opposite of what
// this ABI is for.

#include "core_abi.hpp"

#include "abi/functions.hpp"

// Direct dependencies of this translation unit.  The module headers no
// longer aggregate each other's declarations, so each unit includes the
// modules whose functions it actually calls.
#include "mb_base/functions.hpp"
#include "mb_model/functions.hpp"

namespace axle_kernel {

namespace {

/// Derive the index maps and row counts a solver-ready `Model` needs.
///
/// This is the same derivation `build_model` performs after it has filled in the
/// bodies and constraints (`kernel_static_output.cpp`), lifted so both paths share
/// one implementation: a second copy would be a place for the two to disagree
/// about how many rows a joint contributes.
bool finalize_indices(
    Model& model,
    std::size_t body_count,
    std::string& error
) {
    model.body_to_free.assign(body_count, -1);
    model.free_body.clear();
    for (std::size_t i = 0; i < body_count; ++i) {
        if (!model.bodies[i].fixed) {
            model.body_to_free[i] = static_cast<int>(model.free_body.size());
            model.free_body.push_back(static_cast<int>(i));
        }
    }
    model.ndof = 6*static_cast<int>(model.free_body.size());
    model.rows = 0;
    for (Constraint& constraint : model.constraints) {
        constraint.row = model.rows;
        const int rows = constraint_rows(constraint.type);
        if (rows < 0) {
            error = "unsupported constraint type";
            return false;
        }
        model.rows += rows;
    }
    return true;
}

/// Fill the settings block of a staging `AxleInput` from the core input.
///
/// `run_model` reads no axle-specific setting when it is given a model, so the
/// staging structure exists only to carry the sampling grid and the solver
/// settings.  The field-by-field assignment is spelled out rather than
/// `memcpy`-ed because the two structures are separate ABI surfaces and are
/// allowed to drift apart in field order.
void stage_settings(const MbCoreInput& input, AxleInput& staged) {
    // The staging structure is never seen by a caller, but `run_model` validates
    // its extension header like any other input, so it has to describe itself
    // honestly.  Leaving these zero made the core path fail with an axle ABI
    // mismatch -- which is the validation working, not a false alarm.
    staged.struct_size = sizeof(AxleInput);
    staged.abi_version = static_cast<std::uint32_t>(kAxleKernelAbiVersion);
    staged.reserved = 0;
    staged.sample_count = input.sample_count;
    staged.sample_times = input.sample_times;
    staged.body_wrench = input.body_wrench;
    staged.gravity_x = input.gravity_x;
    staged.gravity_y = input.gravity_y;
    staged.gravity_z = input.gravity_z;
    staged.rho_inf = input.rho_inf;
    staged.integrator_type = input.integrator_type;
    staged.hht_alpha = input.hht_alpha;
    staged.initialization_mode = input.initialization_mode;
    staged.adaptive_step = input.adaptive_step;
    staged.internal_step = input.internal_step;
    staged.min_step = input.min_step;
    staged.max_step = input.max_step;
    staged.local_relative_tolerance = input.local_relative_tolerance;
    staged.local_position_tolerance = input.local_position_tolerance;
    staged.local_angle_tolerance = input.local_angle_tolerance;
    staged.local_velocity_tolerance = input.local_velocity_tolerance;
    staged.local_angular_velocity_tolerance =
        input.local_angular_velocity_tolerance;
    staged.contact_event_tolerance = input.contact_event_tolerance;
    // Brush-state tolerance is a tire concept: it only matters for the transient
    // brush relaxation, which a core model has no tire for.  The solver still
    // requires a positive value, so a fixed physical default is supplied here
    // rather than exposed as a core setting the caller could not use.
    staged.local_brush_tolerance = 1e-9;
    staged.max_newton_iterations = input.max_newton_iterations;
    staged.max_line_search_iterations = input.max_line_search_iterations;
    staged.position_tolerance = input.position_tolerance;
    staged.velocity_tolerance = input.velocity_tolerance;
    staged.dynamics_tolerance = input.dynamics_tolerance;
    staged.increment_tolerance = input.increment_tolerance;
}

} // namespace

/// Build a model from the core input.  Returns false and sets `error` on failure.
bool build_core_model(
    const MbCoreInput& input,
    Model& model,
    std::string& error
) {
    if (!input.body_mass || !input.body_inertia_body_3x3 ||
        !input.body_pose_position_quaternion || input.body_count == 0) {
        error = "body arrays are missing";
        return false;
    }
    model.bodies.resize(input.body_count);
    for (std::size_t i = 0; i < input.body_count; ++i) {
        Body& body = model.bodies[i];
        body.mass = input.body_mass[i];
        body.fixed = input.body_fixed && input.body_fixed[i] != 0;
        if (!(body.mass > 0.0) && !body.fixed) {
            error = "free body mass must be positive";
            return false;
        }
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                body.inertia_body.a[r][c] =
                    input.body_inertia_body_3x3[(i*9) + r*3 + c];
            }
        }
        if (!finite_symmetric(body.inertia_body)) {
            error = "body inertia must be finite and symmetric";
            return false;
        }
        const double* pose = input.body_pose_position_quaternion + i*7;
        body.r = Vec3{pose[0], pose[1], pose[2]};
        body.q = qnormalize(Quat{pose[3], pose[4], pose[5], pose[6]});
        if (input.body_velocity_omega != nullptr) {
            const double* motion = input.body_velocity_omega + i*6;
            body.v = Vec3{motion[0], motion[1], motion[2]};
            body.omega = Vec3{motion[3], motion[4], motion[5]};
        }
    }

    if (input.joint_count > 0 && (!input.joint_type || !input.joint_body_a ||
                                  !input.joint_body_b)) {
        error = "joint arrays are missing";
        return false;
    }
    model.constraints.resize(input.joint_count);
    for (std::size_t i = 0; i < input.joint_count; ++i) {
        Constraint& constraint = model.constraints[i];
        constraint.type = input.joint_type[i];
        constraint.a = input.joint_body_a[i];
        constraint.b = input.joint_body_b[i];
        if (constraint.a < 0 || constraint.b < 0 ||
            constraint.a >= static_cast<int>(input.body_count) ||
            constraint.b >= static_cast<int>(input.body_count)) {
            error = "joint body index is out of range";
            return false;
        }
        if (input.joint_point_a != nullptr) {
            constraint.pa = Vec3{
                input.joint_point_a[i*3], input.joint_point_a[i*3+1],
                input.joint_point_a[i*3+2]
            };
        }
        if (input.joint_point_b != nullptr) {
            constraint.pb = Vec3{
                input.joint_point_b[i*3], input.joint_point_b[i*3+1],
                input.joint_point_b[i*3+2]
            };
        }
        if (input.joint_axis_a != nullptr) {
            constraint.axis_a = Vec3{
                input.joint_axis_a[i*3], input.joint_axis_a[i*3+1],
                input.joint_axis_a[i*3+2]
            };
        }
        if (input.joint_axis_b != nullptr) {
            constraint.axis_b = Vec3{
                input.joint_axis_b[i*3], input.joint_axis_b[i*3+1],
                input.joint_axis_b[i*3+2]
            };
        }
    }

    if (!finalize_indices(model, input.body_count, error)) return false;

    // The element blocks go through the same reader the axle entry point uses, so
    // a `kind` cannot mean one thing here and another there.
    if (!read_element_blocks(
            input.elements, input.element_count, input.element_curves,
            0, nullptr, model, error)) {
        return false;
    }
    return true;
}

} // namespace axle_kernel

extern "C" int mb_core_abi_version() {
    return static_cast<int>(axle_kernel::kCoreKernelAbiVersion);
}

extern "C" int mb_core_run(
    const MbCoreInput* input,
    MbCoreOutput* output,
    char* error_buffer,
    std::size_t error_capacity
) {
    using namespace axle_kernel;
    if (!input || !output) {
        set_error(error_buffer, error_capacity, "input/output is null");
        return 1;
    }
    // Same extension protocol as the axle and vehicle surfaces: state the size
    // compiled against, and refuse a mismatch rather than reading past the end.
    if (input->struct_size < sizeof(MbCoreInput) ||
        input->abi_version != static_cast<std::uint32_t>(kCoreKernelAbiVersion) ||
        output->struct_size < sizeof(MbCoreOutput) ||
        output->abi_version != static_cast<std::uint32_t>(kCoreKernelAbiVersion)) {
        char detail[192];
        std::snprintf(
            detail, sizeof(detail),
            "core ABI mismatch: expected struct_size>=%zu abi=%d, got %zu/%u",
            sizeof(MbCoreInput), kCoreKernelAbiVersion,
            input->struct_size, input->abi_version
        );
        set_error(error_buffer, error_capacity, detail);
        return 4;
    }
    if (input->sample_count < 2 || !input->sample_times) {
        set_error(error_buffer, error_capacity, "sample arrays are missing or too short");
        return 1;
    }

    std::string error;
    Model model;
    if (!build_core_model(*input, model, error)) {
        set_error(error_buffer, error_capacity, error);
        return 2;
    }

    AxleInput staged{};
    stage_settings(*input, staged);
    AxleOutput staged_output{};
    staged_output.struct_size = sizeof(AxleOutput);
    staged_output.abi_version = static_cast<std::uint32_t>(kAxleKernelAbiVersion);
    staged_output.reserved = 0;
    staged_output.body_state = output->body_state;
    staged_output.body_state_capacity = output->body_state_capacity;
    staged_output.constraint_wrench = output->joint_wrench;
    staged_output.constraint_wrench_capacity = output->joint_wrench_capacity;
    staged_output.spring_output = output->spring_output;
    staged_output.spring_output_capacity = output->spring_output_capacity;
    staged_output.energy_output = output->energy_output;
    staged_output.energy_output_capacity = output->energy_output_capacity;
    staged_output.diagnostics = output->diagnostics;
    staged_output.diagnostics_capacity = output->diagnostics_capacity;
    staged_output.contact_event_count = output->contact_event_count;

    // The channels an axle model fills but a core model cannot name: there are no
    // tires, anti-roll bars or contact events in a core model, so the required
    // capacity for those is zero.  A bushing is different -- it is a generic
    // element a core model may well contain -- but the core output deliberately
    // does not expose a bushing channel, so the solver's writes are absorbed by a
    // scratch buffer here rather than by inventing a suspension-shaped field on
    // the generic surface.
    const std::size_t bushing_need =
        input->sample_count*model.bushings.size()*kBushingOutputWidth;
    std::vector<double> bushing_scratch(bushing_need);
    const std::size_t anti_roll_need =
        input->sample_count*model.anti_roll_bars.size()*kAntiRollOutputWidth;
    std::vector<double> anti_roll_scratch(anti_roll_need);
    const std::size_t tire_need =
        input->sample_count*model.tires.size()*kTireOutputWidth;
    std::vector<double> tire_scratch(tire_need);

    staged_output.bushing_output =
        bushing_scratch.empty() ? nullptr : bushing_scratch.data();
    staged_output.bushing_output_capacity = bushing_need;
    staged_output.anti_roll_output =
        anti_roll_scratch.empty() ? nullptr : anti_roll_scratch.data();
    staged_output.anti_roll_output_capacity = anti_roll_need;
    staged_output.tire_output =
        tire_scratch.empty() ? nullptr : tire_scratch.data();
    staged_output.tire_output_capacity = tire_need;
    staged_output.contact_event_output = nullptr;
    staged_output.contact_event_output_capacity = 0;

    return run_model(
        &staged, &staged_output, error_buffer, error_capacity, &model
    );
}
