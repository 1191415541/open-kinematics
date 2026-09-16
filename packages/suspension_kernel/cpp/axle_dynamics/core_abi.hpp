#pragma once

// The generic core ABI (`mb_core_*`): the surface a non-suspension product uses.
//
// The axle ABI carries suspension semantics -- road profile, static gauges,
// steering actuators, tire registration -- because a vehicle needs them.  A
// double pendulum with a spring needs none of that, and the epic requires an
// example proving the generic part stands on its own.
//
// The core surface is therefore the same generic element block the vehicle entry
// point reads, plus bodies and joints, and nothing else.  It deliberately does not
// restate the element layout: `ElementBlock`, `ElementCurveReference`,
// `ElementKind` and `ElementLayout` come from `axle_kernel.hpp`, so a `kind` has
// exactly one layout across all three entry points.

#include "axle_kernel.hpp"

extern "C" {

/// Generic core input: bodies, joints, element blocks and the sampling grid.
///
/// Everything suspension-specific is absent by construction: there is no road,
/// no tire array, no steering actuator and no static gauge.  A caller that needs
/// those uses the axle or vehicle surface.
struct MbCoreInput {
    /// Size of this structure as the caller compiled it.
    std::size_t struct_size;
    std::uint32_t abi_version;
    std::uint32_t reserved;

    // Bodies.  `body_fixed` may be null, in which case every body is free.
    std::size_t body_count;
    const double* body_mass;
    const double* body_inertia_body_3x3;
    /// Four doubles per body: position then quaternion (w, x, y, z).
    const double* body_pose_position_quaternion;
    /// Six doubles per body: linear velocity then angular velocity.
    const double* body_velocity_omega;
    /// One int per body; non-zero means fixed to ground.
    const int* body_fixed;

    // Joints.  `joint_axis_a`/`joint_axis_b` carry one three-vector per joint and
    // may be null for the types that do not use them.
    std::size_t joint_count;
    /// One of `AxleConstraintType`.
    const int* joint_type;
    const int* joint_body_a;
    const int* joint_body_b;
    /// Three doubles per joint.
    const double* joint_point_a;
    const double* joint_point_b;
    const double* joint_axis_a;
    const double* joint_axis_b;

    // Generic elements, read through the layout table in `axle_kernel.hpp`.
    std::size_t element_count;
    const ElementBlock* elements;
    /// `element_count * kElementCurveSlots` entries, in element order.
    const ElementCurveReference* element_curves;

    // Sampling grid and solver settings.  The names match the axle ABI so the two
    // surfaces read the same way.
    std::size_t sample_count;
    const double* sample_times;
    /// Three doubles per sample, applied to body 0; may be null.
    const double* body_wrench;

    double gravity_x;
    double gravity_y;
    double gravity_z;
    double rho_inf;
    int integrator_type;
    double hht_alpha;
    int initialization_mode;
    int adaptive_step;
    double internal_step;
    double min_step;
    double max_step;
    double local_relative_tolerance;
    double local_position_tolerance;
    double local_angle_tolerance;
    double local_velocity_tolerance;
    double local_angular_velocity_tolerance;
    double contact_event_tolerance;
    int max_newton_iterations;
    int max_line_search_iterations;
    double position_tolerance;
    double velocity_tolerance;
    double dynamics_tolerance;
    double increment_tolerance;
};

/// Generic core output: the body state history, the joint reactions and the
/// per-sample diagnostics.
///
/// The suspension-specific channels (tires, bushings, anti-roll bars, road
/// contact events) are absent by construction -- a core model has none of those
/// elements, so there is nothing to report and no field to report it in.  The
/// spring channel is present because a spring is a generic element.
struct MbCoreOutput {
    std::size_t struct_size;
    std::uint32_t abi_version;
    std::uint32_t reserved;

    /// One row per sample and body: position(3), quaternion(4), velocity(3),
    /// omega(3), linear acceleration(3), angular acceleration(3) -- the same
    /// `kStatePerBody = 19` layout the axle surface uses.
    double* body_state;
    std::size_t body_state_capacity;
    /// One row per sample and joint, six doubles.
    double* joint_wrench;
    std::size_t joint_wrench_capacity;
    /// One row per sample and element, `kSpringOutputWidth = 7` doubles.
    double* spring_output;
    std::size_t spring_output_capacity;
    /// One row per sample, `kEnergyOutputWidth = 21` doubles.
    double* energy_output;
    std::size_t energy_output_capacity;
    /// One row per sample, `kDiagnosticsWidth = 16` doubles.
    double* diagnostics;
    std::size_t diagnostics_capacity;
    /// Written with zero, because a core model has no contact events.
    std::size_t* contact_event_count;
};

} // extern "C"

// Exported entry points.  `mb_core_abi_version` returns `kCoreKernelAbiVersion`.
//
// These are declared inside `extern "C"` for the same reason the structures are:
// a declaration at namespace scope would carry C++ linkage, and the definitions
// below carry C linkage, so the two would conflict at link time.
extern "C" {

AXLE_API int mb_core_abi_version();
AXLE_API int mb_core_run(
    const MbCoreInput* input,
    MbCoreOutput* output,
    char* error_buffer,
    std::size_t error_capacity
);

} // extern "C"
