#pragma once

#include <cstddef>
#include <cstdint>

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
    AXLE_INPLANE = 6,
    // Adams CONVEL: coincident centers plus the constant-velocity relation.
    AXLE_CONVEL = 7
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

enum VehicleSteeringActuatorType {
    VEHICLE_STEERING_TRANSLATION = 0,
    VEHICLE_STEERING_ROTATION = 1,
    // 受约束的转角输入；目标进入 KKT 约束，不通过高刚度力近似。
    VEHICLE_STEERING_PRESCRIBED_ROTATION = 2,
    // 受约束的齿条平移输入；目标进入 KKT 约束。
    VEHICLE_STEERING_PRESCRIBED_TRANSLATION = 3
};

enum VehicleTireModelKind {
    VEHICLE_TIRE_NATIVE_BRUSH = 0,
    // 使用当前项目实现的 PAC2002 纯滑移/联合滑移与松弛项；
    // 为保持历史行为，零滑移处移除校准偏置。
    VEHICLE_TIRE_PAC2002_PURE_SLIP = 1,
    // Adams 内置 PAC2002 源模式。按照 PAC2002/Chrono 的 Fx0/Fy0
    // 定义保留 PH*/PV* 在零滑移处产生的源偏置。
    VEHICLE_TIRE_PAC2002_ADAMS_SOURCE = 2,
    // Adams FIALA 属性文件对应的瞬态刷模型。
    VEHICLE_TIRE_FIALA = 3
};

enum VehicleBushingRotationCoordinates {
    VEHICLE_BUSHING_ROTATION_VECTOR = 0,
    VEHICLE_BUSHING_CARDAN_XYZ = 1
};

// 前 83 个位置保持历史布局，其余参数追加在末尾；旧参数位置不能改变。
enum { VEHICLE_PAC2002_PARAMETER_COUNT = 167 };

// The vehicle ABI deliberately wraps the stable axle ABI instead of
// appending fields to AxleInput.  This keeps existing ctypes callers binary
// compatible while leaving room for versioned vehicle-only inputs.
struct VehicleInput {
    std::size_t struct_size;
    std::uint32_t abi_version;
    std::uint32_t reserved;
    AxleInput axle;

    // One actuator per steerable body. Target arrays are laid out as
    // sample_count * steering_count. Translation actuators use metres and
    // metres per second; rotation actuators use radians and radians per
    // second.
    std::size_t steering_count;
    const int* steering_type;
    const int* steering_body;
    const int* steering_reaction_body;
    const double* steering_point_local;
    const double* steering_reaction_point_local;
    const double* steering_axis_local;
    const double* steering_reference_quaternion;
    const double* steering_target_angle;
    const double* steering_target_rate;
    const double* steering_stiffness;
    const double* steering_damping;

    // Optional vertical road profile evaluated at the current wheel-center
    // position. `road_kind` is 0 for sampled-only input, 1 for plane, 2 for
    // sine, 3 for bump, 4 for random Fourier, and 5 for four-post bump.
    int road_kind;
    double road_origin_x;
    double road_origin_z;
    double road_amplitude;
    double road_wavelength;
    double road_phase;
    double road_bump_start;
    double road_bump_length;
    const double* road_corner_scale;

    // Optional non-negative brake torque magnitudes, laid out as
    // sample_count * axle.tire_count. The kernel opposes the instantaneous
    // tire spin direction and applies zero braking torque at zero spin.
    const double* brake_torque;

    // Optional static-trim gauge.  The mask uses pose coordinates
    // translation x/y/z = bits 0/1/2 and rotation x/y/z = bits 3/4/5.
    // These equations remove only a declared global coordinate null-space;
    // they do not add a physical reaction or a ground constraint.
    std::size_t static_gauge_body;
    std::uint32_t static_gauge_dof_mask;

    // Optional vehicle-only initialization mode.  When non-zero, the kernel
    // trims the model with zero velocity and restores the body velocities
    // supplied through AxleInput immediately before dynamic integration.
    int static_trim_then_release;

    // Optional vehicle-only tire frame mapping. The stable axle ABI keeps
    // tire_body as both the force body and contact frame. A vehicle wheel
    // has a separate spinning force body and a non-spinning upright frame;
    // these arrays select the latter. The fields are optional for backward
    // compatibility and fall back to AxleInput.tire_body/tire_center_local.
    const int* tire_frame_body;
    const double* tire_frame_center_local;

    // 可选轮胎模型扩展。参数按 VEHICLE_PAC2002_PARAMETER_COUNT 的固定顺序
    // 展开为 tire_count * parameter_count；省略这些字段时仍使用刷胎。
    const int* tire_model_kind;
    const double* tire_pac2002_parameters;
    const int* tire_pac2002_mirror;

    // 可选整车力元曲线。每条曲线使用对应的 offset/count 数组，坐标和力
    // 分别为米、牛顿；弹簧坐标是压缩挠度，限位块坐标是穿透量。
    const int* vehicle_spring_elastic_curve_offset;
    const int* vehicle_spring_elastic_curve_count;
    const double* vehicle_spring_elastic_curve_deflection;
    const double* vehicle_spring_elastic_curve_force;
    const int* vehicle_spring_compression_stop_curve_offset;
    const int* vehicle_spring_compression_stop_curve_count;
    const double* vehicle_spring_compression_stop_curve_penetration;
    const double* vehicle_spring_compression_stop_curve_force;
    const int* vehicle_spring_rebound_stop_curve_offset;
    const int* vehicle_spring_rebound_stop_curve_count;
    const double* vehicle_spring_rebound_stop_curve_penetration;
    const double* vehicle_spring_rebound_stop_curve_force;

    // 可选衬套六轴弹性曲线。每个衬套占用连续的六个 offset/count 槽位；
    // 平移坐标/力为 m/N，转动坐标/力矩为 rad/N*m。
    const int* vehicle_bushing_force_curve_offset;
    const int* vehicle_bushing_force_curve_count;
    const double* vehicle_bushing_force_curve_coordinate;
    const double* vehicle_bushing_force_curve_force;

    // Adams CONVEL marker副轴。主轴数组分别保存 I 的 x 轴和 J 的 y 轴，
    // 副轴数组保存 I 的 y 轴和 J 的 x 轴。
    const double* constraint_axis_a_secondary;
    const double* constraint_axis_b_secondary;
    // 可选的 Adams CONVEL 初始交叉轴关系，按 dot(I.x, J.y) 给出；
    // 未提供时使用严格正交目标 0。
    const double* constraint_convel_angle_target;

    // 仅用于静态配平的被动转轴。每个条目由一个自由体和一个局部轴组成；
    // 动态积分仍使用完整刚体自由度，不添加任何物理约束。
    std::size_t static_rotation_gauge_count;
    const int* static_rotation_gauge_body;
    const double* static_rotation_gauge_axis_local;

    // 仅用于 provided_consistent_state 的初始 CONVEL 角度残差检查；
    // 不改变动态积分阶段的 local_angle_tolerance。
    double initial_state_angle_tolerance;

    // 每个衬套的转动坐标：0 为旋转向量，1 为 XYZ Cardan 角。
    const int* bushing_rotation_coordinates;

    // Adams 线性关节坐标耦合器。每个耦合器增加一行
    // scale_a * delta_a + scale_b * delta_b = 0；关节编号使用
    // `constraint_*` 数组中的零基索引，坐标类型 0 为转动、1 为平移。
    std::size_t coordinate_coupler_count;
    const int* coordinate_coupler_joint_a;
    const int* coordinate_coupler_coordinate_a;
    const double* coordinate_coupler_scale_a;
    const int* coordinate_coupler_joint_b;
    const int* coordinate_coupler_coordinate_b;
    const double* coordinate_coupler_scale_b;

    // Quadratic aerodynamic drag elements.  Points and axes are local to the
    // selected body; coefficients multiply longitudinal speed squared.
    std::size_t aerodynamic_drag_count;
    const int* aerodynamic_drag_body;
    const double* aerodynamic_drag_application_point;
    const double* aerodynamic_drag_forward_axis;
    const double* aerodynamic_drag_coefficient;

    // Optional drive-torque actuator mapping per tire. A negative drive body
    // retains the historical wheel-body application. The axis is local to
    // the selected drive body; reaction body may be negative.
    const int* tire_drive_torque_body;
    const int* tire_drive_torque_reaction_body;
    const double* tire_drive_torque_axis_local;
    const int* bushing_force_curve_interpolation;
};

struct VehicleOutput {
    std::size_t struct_size;
    std::uint32_t abi_version;
    std::uint32_t reserved;
    AxleOutput axle;

    // One row per sample and steering actuator:
    // measured angle, measured rate, target angle, applied torque.
    double* steering_output;
    std::size_t steering_output_capacity;
};

AXLE_API int axle_kernel_abi_version();

AXLE_API int axle_run(
    const AxleInput* input,
    AxleOutput* output,
    char* error_buffer,
    std::size_t error_capacity);

AXLE_API int vehicle_kernel_abi_version();

AXLE_API int vehicle_run(
    const VehicleInput* input,
    VehicleOutput* output,
    char* error_buffer,
    std::size_t error_capacity);

}
