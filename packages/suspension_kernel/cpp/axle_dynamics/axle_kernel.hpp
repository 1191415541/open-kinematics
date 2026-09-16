#pragma once

#include <cstddef>
#include <cstdint>

#if defined(_WIN32)
#define AXLE_API __declspec(dllexport)
#else
#define AXLE_API __attribute__((visibility("default")))
#endif

// The joint/tire/steering enumerations and the ABI version constants live in a
// dependency-free layer header so the model layer can use them without pulling
// in this public header, and so the version literals have one home.
#include "mb_model/enums.hpp"

extern "C" {

// ---------------------------------------------------------------------------
// Generic element blocks
// ---------------------------------------------------------------------------
//
// The five element families (spring, bushing, anti-roll bar, tire, aerodynamic
// drag) used to be five sets of parallel arrays: one array per scalar field, per
// family, on `AxleInput` and again on `VehicleInput`.  Adding a field to a family
// therefore changed both structures, and a reader on either entry point had to
// know every family's private layout.
//
// These blocks replace that with one layout shared by both entry points: an
// element is a `kind` plus a body pair plus a fixed-size parameter block, and the
// reader consults one table of offsets per kind.  Curves are referenced through
// `ElementCurveReference`, which is how a family declares its fixed number of
// curve slots without any of them living in the element block itself.
//
// The sizes below are part of the ABI.  They are deliberately generous: the
// largest family (a bushing, with two 6x6 matrices and a 6-vector preload) has to
// fit, and a uniform stride is what keeps the block indexable without a per-kind
// width.

/// The uniform parameter run each family may use.  This is the ABI width.
inline constexpr std::size_t kElementBlockSize = 176;

/// Capacity of one element's parameter cache.
///
/// Larger than `kElementBlockSize` because a tire needs its whole PAC2002
/// parameter table -- 226 doubles -- and that cannot fit in the uniform block.
/// The block carries the scalars every family shares; a family that needs more
/// points `cached_parameters` at a caller-owned array whose length
/// `cached_parameter_count` states.
inline constexpr std::size_t kElementParameterCacheSize = 256;

/// Number of ints in one element's integer block.
inline constexpr std::size_t kElementIntBlockSize = 16;

/// Curve slots a family may reference.  Every family declares a fixed number of
/// slots; an unused slot has `count == 0` and a null pointer.
inline constexpr std::size_t kElementCurveSlots = 8;

/// Element families.  These values are ABI: they appear in `elements[i].kind`.
enum ElementKind {
    ELEMENT_SPRING = 0,
    ELEMENT_BUSHING = 1,
    ELEMENT_ANTI_ROLL = 2,
    ELEMENT_TIRE = 3,
    ELEMENT_AERODYNAMIC_DRAG = 4
};

/// Curve slot roles a family may use, indexed within its own slot list.
enum ElementCurveSlot {
    ELEMENT_CURVE_PRIMARY = 0,
    ELEMENT_CURVE_SECONDARY = 1,
    ELEMENT_CURVE_TERTIARY = 2,
    ELEMENT_CURVE_QUATERNARY = 3
};

/// One element: its family, the bodies it acts between, and its parameters.
///
/// `body_b` may be -1 for a ground-referenced element; `body_a` must always name
/// a body.  How many entries of `parameters` and `ints` a given `kind` actually
/// uses is stated by the layout table the reader consults, and the remainder must
/// be zero so that a block copied from a shorter source stays comparable.
struct ElementBlock {
    /// One of `ElementKind`.
    int kind;
    /// Reserved for alignment and future per-element flags; must be zero.
    int flags;
    int body_a;
    int body_b;

    double parameters[kElementBlockSize];
    int ints[kElementIntBlockSize];

    // Parameter cache for families whose data does not fit the uniform block.
    //
    // A tire's PAC2002 table is 226 doubles -- larger than the block -- and is
    // therefore carried beside it: `cached_parameters` points at a caller-owned
    // array of `cached_parameter_count` doubles.  A family that fits in
    // `parameters` leaves both at zero; a family that does not must set them, and
    // the reader rejects a count without a pointer rather than reading a table
    // that is not there.
    const double* cached_parameters;
    std::size_t cached_parameter_count;
};

/// One curve slot of one element: a flattened `(abscissa, ordinate)` table.
///
/// A slot with `count == 0` is "not supplied"; `values` may then be null.  The
/// table is `2 * count` doubles, abscissa first, exactly as the replaced parallel
/// arrays carried it.
struct ElementCurveReference {
    const double* values;
    std::size_t count;
};

/// One row-registering topology extension, mirroring the `driven_*` mechanism.
///
/// `kind` selects the extension and states how many rows it adds; `parameters`
/// and `ints` carry its scalars and indices.  A `CoordinateCoupler` couples two
/// joint coordinates, a `SteeringActuator` adds one driven row per sample, and a
/// `StaticRotationGauge` changes no row count at all.
///
/// **Not read yet.**  The field exists so the layout is fixed and the two C entry
/// points carry the same surface, but no reader consumes it, and a run that
/// supplies one is refused with `AXLE_TOPOLOGY_EXTENSION_ERROR` rather than
/// quietly building a model without the extension.  Until the readers land, the
/// three families come from their existing fields: `coordinate_coupler_*` on the
/// vehicle structure for the coupler, `steering_actuator_*` for the steering
/// actuator, and `static_rotation_gauge_*` for the gauge.
struct TopologyExtensionBlock {
    int kind;
    int flags;
    double parameters[kElementBlockSize];
    int ints[kElementIntBlockSize];
};

/// The refusal a run that supplies a topology extension gets today.
inline constexpr const char* AXLE_TOPOLOGY_EXTENSION_ERROR =
    "topology_extensions is not readable yet: no reader consumes it, so supplying "
    "one is refused instead of building a model without the extension";

/// Topology extension families.  These values are ABI.
enum TopologyExtensionKind {
    TOPOLOGY_COORDINATE_COUPLER = 0,
    TOPOLOGY_STEERING_ACTUATOR = 1,
    TOPOLOGY_STATIC_ROTATION_GAUGE = 2
};

/// Where one family keeps each of its scalars inside the uniform block.
///
/// This table is the point of the whole exercise: it is the single description of
/// a family's layout, so `build_model` and the vehicle registration path read the
/// same offsets and a `kind` cannot mean two different things on the two entry
/// points.
///
/// `parameter_index` values are indices into `ElementBlock::parameters`; an index
/// of `kUnusedParameter` marks a slot the family does not use, and the remaining
/// entries must be zero.  `curve_slots` is how many `ElementCurveReference`
/// entries the family consumes, in slot order from `ELEMENT_CURVE_PRIMARY`.
struct ElementLayout {
    /// One of `ElementKind`.
    int kind;
    /// See `ElementParameter` values below.
    int primary_parameter;
    int secondary_parameter;
    int tertiary_parameter;
    int quaternary_parameter;
    int curve_slots;
    /// Number of leading entries of `ElementBlock::ints` this family uses.
    int int_count;
};

/// Marks an `ElementLayout` slot the family does not use.
inline constexpr int kUnusedParameter = -1;

/// Named offsets into an element's parameter block, grouped by family.
///
/// Each family owns a contiguous, non-overlapping run so that two families can
/// never silently share a slot, and each run starts at a multiple of 8 so it can
/// be appended to without moving the families after it.  `static_assert`s below
/// pin the runs: the bushing's preload vector is what fixes the block size.
enum ElementParameter {
    // Spring: 0..15.  Points are three doubles each, so six slots per pair.
    ELEMENT_SPRING_STIFFNESS = 0,
    ELEMENT_SPRING_COMPRESSION_DAMPING = 1,
    ELEMENT_SPRING_REBOUND_DAMPING = 2,
    ELEMENT_SPRING_FREE_LENGTH = 3,
    ELEMENT_SPRING_MINIMUM_LENGTH = 4,
    ELEMENT_SPRING_MAXIMUM_LENGTH = 5,
    ELEMENT_SPRING_COMPRESSION_STOP_STIFFNESS = 6,
    ELEMENT_SPRING_COMPRESSION_STOP_DAMPING = 7,
    ELEMENT_SPRING_REBOUND_STOP_STIFFNESS = 8,
    ELEMENT_SPRING_REBOUND_STOP_DAMPING = 9,
    ELEMENT_SPRING_POINT_A = 10,
    ELEMENT_SPRING_POINT_B = 13,

    // Bushing: 16..143.  Two 6x6 matrices dominate the run; the attachment
    // geometry follows, which is what makes a bushing expressible completely by
    // its block.
    ELEMENT_BUSHING_STIFFNESS_6X6 = 16,
    ELEMENT_BUSHING_DAMPING_6X6 = 52,
    ELEMENT_BUSHING_PRELOAD_6 = 88,
    ELEMENT_BUSHING_POINT_A = 94,
    ELEMENT_BUSHING_POINT_B = 97,
    ELEMENT_BUSHING_REFERENCE_TRANSLATION = 100,
    ELEMENT_BUSHING_FRAME_A_QUATERNION = 103,
    ELEMENT_BUSHING_FRAME_B_QUATERNION = 107,
    ELEMENT_BUSHING_REFERENCE_QUATERNION = 111,

    // Anti-roll bar: 144..153 (stiffness pair, axis, reference quaternion).
    ELEMENT_ANTI_ROLL_STIFFNESS = 144,
    ELEMENT_ANTI_ROLL_DAMPING = 145,
    ELEMENT_ANTI_ROLL_AXIS_A = 146,
    ELEMENT_ANTI_ROLL_REFERENCE_QUATERNION = 149,

    // Tire: 154..167.
    ELEMENT_TIRE_STIFFNESS = 154,
    ELEMENT_TIRE_DAMPING = 155,
    ELEMENT_TIRE_MAXIMUM_COMPRESSION = 156,
    ELEMENT_TIRE_RADIUS = 157,
    ELEMENT_TIRE_MU_LONGITUDINAL = 158,
    ELEMENT_TIRE_MU_LATERAL = 159,
    ELEMENT_TIRE_BRUSH_STIFFNESS_LONGITUDINAL = 160,
    ELEMENT_TIRE_BRUSH_STIFFNESS_LATERAL = 161,
    ELEMENT_TIRE_RELAXATION_LENGTH_LONGITUDINAL = 162,
    ELEMENT_TIRE_RELAXATION_LENGTH_LATERAL = 163,
    ELEMENT_TIRE_DETACHED_RELAXATION = 164,

    // Aerodynamic drag: 168..175.  One body (the other body slot must be -1),
    // its application point, its forward axis and the coefficient.
    ELEMENT_AERODYNAMIC_DRAG_APPLICATION_POINT = 168,
    ELEMENT_AERODYNAMIC_DRAG_FORWARD_AXIS = 171,
    ELEMENT_AERODYNAMIC_DRAG_COEFFICIENT = 174
};

/// Index into an element's integer block.
enum ElementInteger {
    ELEMENT_INT_MODEL_KIND = 0,
    ELEMENT_INT_MIRROR_SIDE = 1,
    ELEMENT_INT_ROTATION_COORDINATES = 2,
    ELEMENT_INT_CURVE_INTERPOLATION = 3,
    ELEMENT_INT_DRIVE_TORQUE_BODY = 4,
    ELEMENT_INT_DRIVE_TORQUE_REACTION_BODY = 5
};

/// The one layout table both entry points read.
///
/// A `static_assert` pair below pins the largest family inside the uniform block:
/// a bushing's preload ends at index 93, so the block size cannot be reduced
/// without moving that family.
inline constexpr ElementLayout kElementLayouts[] = {
    {ELEMENT_SPRING, ELEMENT_SPRING_STIFFNESS, ELEMENT_SPRING_COMPRESSION_DAMPING,
     ELEMENT_SPRING_REBOUND_DAMPING, ELEMENT_SPRING_FREE_LENGTH,
     /*curve_slots=*/2, /*int_count=*/0},
    {ELEMENT_BUSHING, ELEMENT_BUSHING_STIFFNESS_6X6, ELEMENT_BUSHING_DAMPING_6X6,
     ELEMENT_BUSHING_PRELOAD_6, ELEMENT_BUSHING_POINT_A,
     /*curve_slots=*/1, /*int_count=*/2},
    {ELEMENT_ANTI_ROLL, ELEMENT_ANTI_ROLL_STIFFNESS, ELEMENT_ANTI_ROLL_DAMPING,
     ELEMENT_ANTI_ROLL_AXIS_A, ELEMENT_ANTI_ROLL_REFERENCE_QUATERNION,
     /*curve_slots=*/0, /*int_count=*/0},
    {ELEMENT_TIRE, ELEMENT_TIRE_STIFFNESS, ELEMENT_TIRE_DAMPING,
     ELEMENT_TIRE_BRUSH_STIFFNESS_LONGITUDINAL,
     ELEMENT_TIRE_BRUSH_STIFFNESS_LATERAL,
     /*curve_slots=*/2, /*int_count=*/3},
    {ELEMENT_AERODYNAMIC_DRAG, ELEMENT_AERODYNAMIC_DRAG_APPLICATION_POINT,
     ELEMENT_AERODYNAMIC_DRAG_FORWARD_AXIS, ELEMENT_AERODYNAMIC_DRAG_COEFFICIENT,
     kUnusedParameter,
     /*curve_slots=*/0, /*int_count=*/0},
};

/// Field-by-field map from a spring's parameters into the uniform block.
///
/// Each row says *which* spring field a block slot feeds, by name, plus how many
/// consecutive doubles it carries.  Naming the field rather than deriving a byte
/// offset is deliberate: `Spring` starts with two ints, so a hand-computed double
/// index would be silently wrong the first time a field is inserted, whereas a
/// `switch` on the name fails to compile when a field is renamed.
struct ElementFieldMap {
    /// Index into `ElementBlock::parameters`.
    int element_offset;
    /// Which family field this slot feeds; see `SpringField`.
    int field;
    /// Consecutive doubles moved: 1 for a scalar, 3 for a body-local point.
    int width;
};

/// The spring fields a block slot may feed.
enum SpringField {
    SPRING_FIELD_K = 0,
    SPRING_FIELD_C_COMPRESSION,
    SPRING_FIELD_C_REBOUND,
    SPRING_FIELD_FREE_LENGTH,
    SPRING_FIELD_MINIMUM_LENGTH,
    SPRING_FIELD_MAXIMUM_LENGTH,
    SPRING_FIELD_COMPRESSION_STOP_K,
    SPRING_FIELD_COMPRESSION_STOP_C,
    SPRING_FIELD_REBOUND_STOP_K,
    SPRING_FIELD_REBOUND_STOP_C,
    SPRING_FIELD_POINT_A,
    SPRING_FIELD_POINT_B
};

inline constexpr ElementFieldMap kSpringFieldMap[] = {
    {ELEMENT_SPRING_STIFFNESS, SPRING_FIELD_K, 1},
    {ELEMENT_SPRING_COMPRESSION_DAMPING, SPRING_FIELD_C_COMPRESSION, 1},
    {ELEMENT_SPRING_REBOUND_DAMPING, SPRING_FIELD_C_REBOUND, 1},
    {ELEMENT_SPRING_FREE_LENGTH, SPRING_FIELD_FREE_LENGTH, 1},
    {ELEMENT_SPRING_MINIMUM_LENGTH, SPRING_FIELD_MINIMUM_LENGTH, 1},
    {ELEMENT_SPRING_MAXIMUM_LENGTH, SPRING_FIELD_MAXIMUM_LENGTH, 1},
    {ELEMENT_SPRING_COMPRESSION_STOP_STIFFNESS, SPRING_FIELD_COMPRESSION_STOP_K, 1},
    {ELEMENT_SPRING_COMPRESSION_STOP_DAMPING, SPRING_FIELD_COMPRESSION_STOP_C, 1},
    {ELEMENT_SPRING_REBOUND_STOP_STIFFNESS, SPRING_FIELD_REBOUND_STOP_K, 1},
    {ELEMENT_SPRING_REBOUND_STOP_DAMPING, SPRING_FIELD_REBOUND_STOP_C, 1},
    {ELEMENT_SPRING_POINT_A, SPRING_FIELD_POINT_A, 3},
    {ELEMENT_SPRING_POINT_B, SPRING_FIELD_POINT_B, 3},
};

inline constexpr std::size_t kSpringFieldMapCount =
    sizeof(kSpringFieldMap) / sizeof(kSpringFieldMap[0]);

/// The bushing fields a block slot may feed.
enum BushingField {
    BUSHING_FIELD_STIFFNESS = 0,
    BUSHING_FIELD_DAMPING,
    BUSHING_FIELD_PRELOAD,
    BUSHING_FIELD_POINT_A,
    BUSHING_FIELD_POINT_B,
    BUSHING_FIELD_REFERENCE_TRANSLATION,
    BUSHING_FIELD_FRAME_A_QUATERNION,
    BUSHING_FIELD_FRAME_B_QUATERNION,
    BUSHING_FIELD_REFERENCE_QUATERNION
};

inline constexpr ElementFieldMap kBushingFieldMap[] = {
    {ELEMENT_BUSHING_STIFFNESS_6X6, BUSHING_FIELD_STIFFNESS, 36},
    {ELEMENT_BUSHING_DAMPING_6X6, BUSHING_FIELD_DAMPING, 36},
    {ELEMENT_BUSHING_PRELOAD_6, BUSHING_FIELD_PRELOAD, 6},
    {ELEMENT_BUSHING_POINT_A, BUSHING_FIELD_POINT_A, 3},
    {ELEMENT_BUSHING_POINT_B, BUSHING_FIELD_POINT_B, 3},
    {ELEMENT_BUSHING_REFERENCE_TRANSLATION, BUSHING_FIELD_REFERENCE_TRANSLATION, 3},
    {ELEMENT_BUSHING_FRAME_A_QUATERNION, BUSHING_FIELD_FRAME_A_QUATERNION, 4},
    {ELEMENT_BUSHING_FRAME_B_QUATERNION, BUSHING_FIELD_FRAME_B_QUATERNION, 4},
    {ELEMENT_BUSHING_REFERENCE_QUATERNION, BUSHING_FIELD_REFERENCE_QUATERNION, 4},
};

inline constexpr std::size_t kBushingFieldMapCount =
    sizeof(kBushingFieldMap) / sizeof(kBushingFieldMap[0]);

/// Every spring slot the map writes must land inside the uniform block.
static_assert(
    ELEMENT_SPRING_POINT_B + 3 <= kElementBlockSize,
    "a mapped spring slot falls outside the uniform element block"
);

inline constexpr std::size_t kElementLayoutCount =
    sizeof(kElementLayouts) / sizeof(kElementLayouts[0]);

/// The bushing is the widest family: it ends at the reference quaternion, and the
/// aerodynamic coefficient is the highest index in use.  These assertions are what
/// make the block size a checked consequence of the layouts rather than a number
/// someone has to remember to raise.
static_assert(
    ELEMENT_BUSHING_REFERENCE_QUATERNION + 4 <= kElementBlockSize,
    "the uniform element block is too small for the widest family (bushing)"
);
static_assert(
    ELEMENT_AERODYNAMIC_DRAG_COEFFICIENT < kElementBlockSize,
    "an element parameter index falls outside the uniform block"
);
static_assert(
    ELEMENT_SPRING_POINT_B + 3 <= ELEMENT_BUSHING_STIFFNESS_6X6,
    "the spring and bushing runs must not overlap"
);
static_assert(
    ELEMENT_BUSHING_REFERENCE_QUATERNION + 4 <= ELEMENT_ANTI_ROLL_STIFFNESS,
    "the bushing and anti-roll runs must not overlap"
);
static_assert(
    ELEMENT_ANTI_ROLL_REFERENCE_QUATERNION + 4 <= ELEMENT_TIRE_STIFFNESS,
    "the anti-roll and tire runs must not overlap"
);
static_assert(
    ELEMENT_TIRE_DETACHED_RELAXATION < ELEMENT_AERODYNAMIC_DRAG_COEFFICIENT,
    "the tire and aerodynamic runs must not overlap"
);
static_assert(
    ELEMENT_INT_DRIVE_TORQUE_REACTION_BODY < kElementIntBlockSize,
    "the element integer block is too small"
);

struct AxleInput {
    // Extension protocol, matching `VehicleInput`: `struct_size` is the size of
    // the structure the caller compiled against, and `abi_version` must equal
    // `axle_kernel_abi_version()`.  `run_model` rejects a mismatch instead of
    // reading fields the caller never provided.
    std::size_t struct_size;
    std::uint32_t abi_version;
    std::uint32_t reserved;

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

    // Generic element surface, appended so every existing field keeps its offset.
    //
    // When `element_count` is zero the reader falls back to the per-family arrays
    // above, which is what keeps this change additive.  A caller that fills the
    // blocks must leave the per-family arrays empty; supplying both is an error
    // rather than a silent override.
    std::size_t element_count;
    const ElementBlock* elements;
    /// `element_count * kElementCurveSlots` entries, in element order.
    const ElementCurveReference* element_curves;
    std::size_t topology_extension_count;
    const TopologyExtensionBlock* topology_extensions;
};

struct AxleOutput {
    // Extension protocol, matching `VehicleOutput`; see `AxleInput`.
    std::size_t struct_size;
    std::uint32_t abi_version;
    std::uint32_t reserved;

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

    // Optional per-tire vertical deflection-load curve (`[DEFLECTION_LOAD_CURVE]`):
    // an offset and a count per tire into a flattened (deflection_m, load_n) table.
    const int* tire_deflection_curve_offset;
    const int* tire_deflection_curve_count;
    const double* tire_deflection_curve_deflection;
    const double* tire_deflection_curve_force;

    // Optional per-tire wheel-bottoming curve (`[BOTTOMING_CURVE]`): the same
    // offset/count layout, rows being (rim_penetration_m, load_n).  The rim is hit
    // once the deflection exceeds UNLOADED_RADIUS - BOTTOMING_RADIUS, and its force is
    // added on top of the tire's own vertical force (Adams documents
    // Fz = min(0, Fzk + Fzc) + min(0, Fzrim)).
    const int* tire_bottoming_curve_offset;
    const int* tire_bottoming_curve_count;
    const double* tire_bottoming_curve_penetration;
    const double* tire_bottoming_curve_force;

    // Driven coordinates: one row each, prescribing a single relative degree of
    // freedom between two bodies as a function of time (Adams' joint MOTION).
    // These become AXLE_DRIVEN_TRANSLATION / AXLE_DRIVEN_ROTATION constraints, so
    // they append rows to the model and their reaction is reported through the
    // ordinary constraint-wrench output.
    //
    // ``driven_type`` is the AxleConstraintType value.  ``driven_axis_local`` is
    // expressed in the *reaction* body's frame, and ``driven_reference_quaternion``
    // is the zero-angle pose of the driven body relative to it (used by the
    // rotational variant only).  ``driven_target`` and ``driven_target_rate`` are
    // flattened ``sample_count x driven_count`` tables in the case's order, exactly
    // like the steering targets.
    //
    // A driven coordinate always needs a reaction body: the row measures a
    // relative quantity, and the constraint-wrench output indexes both endpoints.
    std::size_t driven_count;
    const int* driven_type;
    const int* driven_body;
    const int* driven_reaction_body;
    const double* driven_point_local;
    const double* driven_reaction_point_local;
    const double* driven_axis_local;
    const double* driven_reference_quaternion;
    const double* driven_target;
    const double* driven_target_rate;

    // Generic element surface, shared verbatim with `AxleInput` so a `kind` has
    // one layout on both entry points.  See the comment there for the fallback
    // rule.
    std::size_t element_count;
    const ElementBlock* elements;
    /// `element_count * kElementCurveSlots` entries, in element order.
    const ElementCurveReference* element_curves;
    std::size_t topology_extension_count;
    const TopologyExtensionBlock* topology_extensions;
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



