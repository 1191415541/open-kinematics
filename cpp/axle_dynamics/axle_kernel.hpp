#pragma once

#include <cstddef>

#if defined(_WIN32)
#define AXLE_API __declspec(dllexport)
#else
#define AXLE_API __attribute__((visibility("default")))
#endif

extern "C" {

enum AxleConstraintType {
    AXLE_SPHERICAL = 0,
    AXLE_REVOLUTE = 1,
    AXLE_FIXED = 2,
    AXLE_PRISMATIC = 3,
    // Coincident points plus one orthogonality condition between the two
    // axes: four rows, two rotational degrees of freedom.
    AXLE_UNIVERSAL = 4,
    // Shared axis line, free to slide along and spin about it: four rows.
    AXLE_CYLINDRICAL = 5,
    // Point of body B constrained to the plane of body A: one row.
    AXLE_INPLANE = 6
};

struct AxleInput {
    std::size_t body_count;
    const double* body_mass;
    const double* body_inertia_body_3x3;
    const double* body_pose_position_quaternion;
    const double* body_velocity_omega;
    const int* body_fixed;

    std::size_t constraint_count;
    const int* constraint_type;
    const int* constraint_body_a;
    const int* constraint_body_b;
    const double* constraint_point_a;
    const double* constraint_point_b;
    const double* constraint_axis_a;
    const double* constraint_axis_b;

    std::size_t spring_count;
    const int* spring_body_a;
    const int* spring_body_b;
    const double* spring_point_a;
    const double* spring_point_b;
    const double* spring_stiffness;
    const double* spring_compression_damping;
    const double* spring_rebound_damping;
    const double* spring_free_length;
    const double* spring_minimum_length;
    const double* spring_maximum_length;
    const double* spring_compression_stop_stiffness;
    const double* spring_compression_stop_damping;
    const double* spring_rebound_stop_stiffness;
    const double* spring_rebound_stop_damping;
    // Optional measured force-velocity curves, concatenated across springs.
    // `spring_damper_curve_count[i]` gives the number of points for spring i
    // and `spring_damper_curve_offset[i]` where they start in the two value
    // arrays.  A count of zero means the constant coefficients are used.
    const int* spring_damper_curve_offset;
    const int* spring_damper_curve_count;
    const double* spring_damper_curve_velocity;
    const double* spring_damper_curve_force;

    std::size_t bushing_count;
    const int* bushing_body_a;
    const int* bushing_body_b;
    const double* bushing_point_a;
    const double* bushing_point_b;
    const double* bushing_frame_a_quaternion;
    const double* bushing_frame_b_quaternion;
    const double* bushing_reference_translation;
    const double* bushing_reference_quaternion;
    const double* bushing_stiffness_6x6;
    const double* bushing_damping_6x6;
    const double* bushing_preload_6;

    std::size_t anti_roll_bar_count;
    const int* anti_roll_body_a;
    const int* anti_roll_body_b;
    const double* anti_roll_axis_a;
    const double* anti_roll_reference_quaternion;
    const double* anti_roll_stiffness;
    const double* anti_roll_damping;

    std::size_t tire_count;
    const int* tire_body;
    const double* tire_center_local;
    const double* tire_spin_axis_local;
    const double* tire_forward_axis_local;
    const double* tire_radius;
    const double* tire_maximum_compression;
    const double* tire_stiffness;
    const double* tire_damping;
    const double* tire_mu_longitudinal;
    const double* tire_mu_lateral;
    const double* tire_brush_stiffness_longitudinal;
    const double* tire_brush_stiffness_lateral;
    const double* tire_relaxation_length_longitudinal;
    const double* tire_relaxation_length_lateral;
    const double* tire_detached_relaxation;

    std::size_t sample_count;
    const double* sample_times;
    const double* body_wrench;
    const double* road_z;
    const double* road_z_velocity;
    const double* wheel_torque;

    double gravity_x;
    double gravity_y;
    double gravity_z;
    double rho_inf;
    // 0 = GGL generalized-alpha, 1 = Adams-compatible HHT.
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
    double local_brush_tolerance;
    // Maximum time-bracket width used to localize a contact mode switch.
    // This may be smaller than min_step because event substeps are explicit.
    double contact_event_tolerance;
    int max_newton_iterations;
    int max_line_search_iterations;
    double position_tolerance;
    double velocity_tolerance;
    double dynamics_tolerance;
    double increment_tolerance;
};

struct AxleOutput {
    // One row per sample and body: position(3), quaternion(4), velocity(3),
    // omega(3), linear acceleration(3), angular acceleration(3).
    double* body_state;
    std::size_t body_state_capacity;
    // One row per sample and ideal constraint: world force and moment on
    // body_b, with moment taken about the body_b joint marker.
    double* constraint_wrench;
    std::size_t constraint_wrench_capacity;
    // Per spring: length, length rate, main elastic force, main damping force,
    // compression-stop elastic force, rebound-stop elastic force, and total
    // axial force. The difference between total and all elastic terms is the
    // full dissipative force, including active stop damping.
    double* spring_output;
    std::size_t spring_output_capacity;
    // Per bushing: deformation(6), local wrench on body_b(6).
    double* bushing_output;
    std::size_t bushing_output_capacity;
    // Per anti-roll bar: relative angle, relative rate, torque on body_b.
    double* anti_roll_output;
    std::size_t anti_roll_output_capacity;
    // One row per sample: accepted, accepted steps, rejected attempts,
    // max Newton iterations, min/max/last accepted step, position/velocity/
    // dynamics residual, active contacts, event count, local error ratio,
    // energy residual, failure code.
    // When SUSPENSION_AXLE_PROFILE is enabled and one extra row is provided,
    // an aggregate performance row follows all sample rows.  It is optional
    // and does not alter the 16-column public sample layout.
    double* diagnostics;
    std::size_t diagnostics_capacity;
    // One row per sample and tire:
    // active, gap, penetration, normal velocity, normal force,
    // longitudinal force, lateral force, longitudinal slip velocity,
    // lateral slip velocity, friction utilization, brush sx, brush sy.
    double* tire_output;
    std::size_t tire_output_capacity;
    // One row per sample: kinetic, potential, total, interval residual,
    // external/road/drive work, damper/friction/contact dissipation,
    // algorithmic dissipation, total work, total physical dissipation, status,
    // gravity, spring, stop, bushing, anti-roll, tire-normal, tire-brush energy.
    double* energy_output;
    std::size_t energy_output_capacity;
    // Localized contact transitions: time, tire index, transition
    // (+1 enter, -1 exit). Capacity is measured in doubles.
    double* contact_event_output;
    std::size_t contact_event_output_capacity;
    std::size_t* contact_event_count;
};

AXLE_API int axle_kernel_abi_version();

AXLE_API int axle_run(
    const AxleInput* input,
    AxleOutput* output,
    char* error_buffer,
    std::size_t error_capacity);

}
