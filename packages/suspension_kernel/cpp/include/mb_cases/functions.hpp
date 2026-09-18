#pragma once

/// The free functions of the `mb_cases` module.
///
/// The case layer is the only place that knows how a *declarative* contract
/// document becomes a concrete solver run.  It owns three translations:
///
/// * a `multibody-model` document into the flat arrays an `AxleInput` points
///   at, including the two conversions the wire format deliberately keeps on
///   this side -- millimetres to metres, and world-frame points to body-local
///   points;
/// * a `multibody-case` document into a list of concrete cases, each carrying
///   only the tables that differ between cases;
/// * the description of the result blocks the caller allocates.
///
/// It never calls the solver.  `abi` owns the call sequence, which keeps this
/// module a pure translation: documents in, array tables out.  That is also
/// what makes a new case family a new file here plus one row in the dispatch,
/// with no change to the solver or to the ABI.

#include "mb_base/prelude.hpp"
#include "mb_base/vector.hpp"
#include "mb_contract/types.hpp"
#include "mb_input/types.hpp"

namespace axle_kernel {

/// A named body-local point of the model, in metres.  Markers are how a case
/// document refers to a geometric location ("the left wheel centre") without
/// knowing body indices or the frame the assembly was authored in.
struct ContractMarker {
  std::string name;
  int body = -1;
  Vec3 point{};
};

/// One concrete case: the tables that vary from case to case.
///
/// Everything else the solver needs lives in `ContractModel`: one model is
/// shared by every case in a run.
struct ContractCase {
  std::string name;
  std::size_t sample_count = 0;
  /// The output grid of this case, in seconds.
  std::vector<double> sample_times;
  /// `sample_count * body_count * 6`, flat, in body order.
  std::vector<double> body_wrench;
  /// Per-tire road height and vertical speed, `sample_count * tire_count`.
  std::vector<double> road_z;
  std::vector<double> road_velocity;
  /// Per-tire applied torque, `sample_count * tire_count`.
  std::vector<double> wheel_torque;
  /// `sample_count * driven_count`, flat, in driven-coordinate order.
  std::vector<double> driven_target;
  std::vector<double> driven_target_rate;
  /// Per-actuator prescribed steering, `sample_count * steering_count`.
  std::vector<double> steering_target;
  std::vector<double> steering_rate;
  /// Per-tire brake torque, `sample_count * tire_count`.
  std::vector<double> brake_torque;

  /// The road the case runs on, when it overrides the model's own profile.
  /// A road is a case input, not a model one: the same vehicle is driven over
  /// different surfaces, and the case document is what says which.
  bool has_road = false;
  int road_kind = 0;
  double road_origin_x = 0.0;
  double road_origin_z = 0.0;
  double road_amplitude = 0.0;
  double road_wavelength = 0.0;
  double road_phase = 0.0;
  double road_bump_start = 0.0;
  double road_bump_length = 0.0;
  double road_corner_scale[4] = {1.0, 1.0, 1.0, 1.0};

  /// The static-trim gauge, when the case asks for one.  A gauge removes a
  /// declared global null-space direction; it is not a physical constraint, so
  /// which directions it pins is a property of the case rather than the model.
  bool has_static_gauge = false;
  /// The gauge body as the document names it; a family resolves it to an index.
  std::string static_gauge_body_name;
  std::size_t static_gauge_body = 0;
  std::uint32_t static_gauge_dof_mask = 0;
  bool static_trim_then_release = false;
  /// Residual tolerance for a provided consistent initial state.
  double initial_state_angle_tolerance = 1e-6;
};

/// A model read from a `multibody-model` document, with the owned storage that
/// backs the pointers of the `-Input` structures it fills.
///
/// The class exists so that lifetime is not a matter of discipline: the arrays
/// cannot be freed while a structure still points at them, because the
/// structure is filled by a method of the object that owns the arrays.
class ContractModel {
 public:
  /// Read `document` into this object.  On failure returns false and sets
  /// `error`; the object is left with no usable model.
  ///
  /// `blob` is the payload the document's own descriptors point into.  A tire
  /// model with a large parameter vector carries it there rather than in JSON,
  /// and the reader checks every descriptor against this payload before it
  /// reads, so a truncated model is an error rather than a short read.
  bool read(const JsonValue& document, const std::string& blob, std::string& error);

  /// Point the model-level fields of `input` at the owned arrays.  Per-case
  /// fields (sample times, wrenches, driven targets) are filled by
  /// `fill_case`, which must be called after this.
  void fill(VehicleInput& input) const;

  /// Point the per-case fields of `input` at `run`'s arrays.
  void fill_case(const ContractCase& run, VehicleInput& input) const;

  /// Index of a named body, or -1.
  int body_index(const std::string& name) const;
  /// Index of a named driven coordinate, or -1.
  int driven_index(const std::string& name) const;
  /// Index of a named tire, or -1.
  int tire_index(const std::string& name) const;
  /// Tire names, in the order every per-tire block uses.
  const std::vector<std::string>& tire_order() const { return tire_names_; }
  /// Index of a named steering actuator, or -1.
  int steering_index(const std::string& name) const;
  /// Actuator names, in the order the steering-target columns use.
  const std::vector<std::string>& steering_order() const { return steering_names_; }
  /// The body each tire is mounted on, and its centre in that body's frame.
  /// A caller that has no separate wheel frame uses these as the fallback.
  const std::vector<int>& tire_bodies() const { return tire_body_; }
  const std::vector<double>& tire_centres() const { return tire_center_; }
  /// The frame each tire is measured against, and its centre in that frame.
  /// The document always resolves this, so there is no fallback to apply.
  const std::vector<int>& tire_frame_bodies() const { return tire_frame_body_; }
  const std::vector<double>& tire_frame_centres() const { return tire_frame_center_; }
  /// Optional per-tire drive-torque application; -1 means the tire's own body.
  const std::vector<int>& tire_drive_bodies() const { return tire_drive_body_; }
  const std::vector<int>& tire_drive_reactions() const { return tire_drive_reaction_; }
  const std::vector<double>& tire_drive_axes() const { return tire_drive_axis_; }
  /// The ABI's per-tire model kind: 0 brush, 1 PAC2002, 2 PAC2002 Adams, 3 Fiala.
  const std::vector<int>& tire_model_kinds() const { return tire_model_kind_; }
  const std::vector<int>& tire_mirrors() const { return tire_mirror_; }
  /// The per-tire parameter block, laid out `tire * VEHICLE_PAC2002_PARAMETER_COUNT`.
  const std::vector<double>& tire_parameters() const { return tire_parameters_; }
  /// Optional per-tire vertical tables: an offset and a count per tire into a
  /// flattened `(abscissa, value)` list, in tire order.  A count of zero means
  /// the tire uses its stiffness polynomial instead of a measured table.
  const std::vector<int>& tire_deflection_offsets() const { return tire_deflection_offset_; }
  const std::vector<int>& tire_deflection_counts() const { return tire_deflection_count_; }
  const std::vector<double>& tire_deflection_abscissa() const { return tire_deflection_x_; }
  const std::vector<double>& tire_deflection_value() const { return tire_deflection_y_; }
  const std::vector<int>& tire_bottoming_offsets() const { return tire_bottoming_offset_; }
  const std::vector<int>& tire_bottoming_counts() const { return tire_bottoming_count_; }
  const std::vector<double>& tire_bottoming_abscissa() const { return tire_bottoming_x_; }
  const std::vector<double>& tire_bottoming_value() const { return tire_bottoming_y_; }
  /// Optional per-bushing six-axis force curves.  The offset/count tables are
  /// laid out ``bushing * 6 + axis``; a zero count means the linear stiffness
  /// for that axis is used.  Coordinates are already in kernel SI units.
  const std::vector<int>& bushing_force_curve_interpolations() const {
    return bushing_force_curve_interpolation_;
  }
  const std::vector<int>& bushing_force_curve_offsets() const {
    return bushing_force_curve_offset_;
  }
  const std::vector<int>& bushing_force_curve_counts() const {
    return bushing_force_curve_count_;
  }
  const std::vector<double>& bushing_force_curve_coordinates() const {
    return bushing_force_curve_coordinate_;
  }
  const std::vector<double>& bushing_force_curve_values() const {
    return bushing_force_curve_force_;
  }
  /// Per-bushing rotation convention: 0 rotation-vector, 1 Cardan xyz.
  const std::vector<int>& bushing_rotation_coordinates() const {
    return bushing_rotation_coordinate_;
  }

  /// Look up a marker by name; returns false when the model has no such marker.
  bool find_marker(const std::string& name, ContractMarker& out) const;

  /// The body-fixed point each body's external `body_wrench` acts at, in
  /// metres, one entry per body in body order.  Empty when no body declares
  /// one, which is the historical convention every dynamics family uses: the
  /// wrench then acts at the body origin.
  ///
  /// A family that means "a force at a marker" -- the C load path is the one
  /// that does -- names the marker in `body_wrench_markers`, because the lever
  /// arm that turns such a force into a moment about the body origin is a
  /// function of the pose being solved for.  Pre-transferring it at the
  /// reference pose freezes the arm and drops the first-order geometry of the
  /// swept body (measured at 2.8e-8 rad against a 1e-8 rad tolerance).
  const std::vector<Vec3>& body_wrench_points() const { return body_wrench_point_; }

  /// Whether `body` declared an application point, so a reader that transfers a
  /// load by hand can refuse rather than silently assume the origin.
  bool has_body_wrench_point(int body) const {
    return body >= 0 && static_cast<std::size_t>(body) < body_wrench_declared_.size()
        && body_wrench_declared_[static_cast<std::size_t>(body)] != 0;
  }

  /// Name of a driven coordinate, in declaration order.
  const std::string& driven_name(std::size_t index) const { return driven_names_[index]; }

  /// The separation of a driven coordinate at the assembling pose, in metres.
  /// This is the target that leaves the neutral case in static equilibrium.
  double driven_separation(std::size_t index) const;

  /// Whether a driven coordinate is a rotation rather than a translation.
  ///
  /// The two do not scale alike -- a translation is a length and follows the
  /// document's length unit, a rotation is an angle -- and only a translation
  /// has an assembling *separation* for an offset to be measured from.
  bool driven_is_rotation(std::size_t index) const;

  /// The seven pose doubles `[x, y, z, qw, qx, qy, qz]` of a body, in metres.
  /// The storage stays owned here, so the pointer is valid for the lifetime of
  /// the model.
  const double* body_pose(std::size_t index) const { return &body_pose_[index * 7]; }

  const std::string& name() const { return name_; }
  /// The body names, in the index order every block of the result uses.
  const std::vector<std::string>& body_order() const { return body_names_; }
  std::size_t body_count() const { return body_names_.size(); }
  std::size_t driven_count() const { return driven_names_.size(); }

  /// Whether the model needs the vehicle-level registration stages at all.
  ///
  /// The kernel has two entry points and they do not build the same model: the
  /// axle one stops after `build_model`, the vehicle one then registers tires,
  /// curves, actuators and driven coordinates.  Which one a caller would have
  /// picked is a property of the model, so the contract entry point has to
  /// decide it the same way -- running the vehicle stages over a model that has
  /// no vehicle-level content would quietly re-derive that model's tire
  /// semantics and change the answer.
  bool needs_vehicle_stages() const { return vehicle_stages_ || !driven_names_.empty(); }

  /// The document's length unit, in metres.
  ///
  /// The conversions the wire format needs are all powers of this number, so
  /// they live here rather than being scattered through the readers: a length
  /// scales by it, a force per length by its inverse, an inertia by its square,
  /// and a moment -- including a torque -- by it.  A document written in metres
  /// declares `"m"` and every one of those becomes 1, which is what a caller
  /// that wants to hand the kernel its own numbers verbatim should do.
  double length_scale() const { return length_scale_; }
  double force_per_length_scale() const { return 1.0 / length_scale_; }
  double inertia_scale() const { return length_scale_ * length_scale_; }
  double moment_scale() const { return length_scale_; }

 private:
  std::string name_;
  double length_scale_ = 1e-3;

  std::vector<std::string> body_names_;
  std::vector<double> body_mass_;
  std::vector<double> body_inertia_;
  std::vector<double> body_pose_;
  std::vector<double> body_velocity_;
  std::vector<int> body_fixed_;

  std::vector<int> constraint_type_;
  std::vector<int> constraint_body_a_;
  std::vector<int> constraint_body_b_;
  std::vector<double> constraint_point_a_;
  std::vector<double> constraint_point_b_;
  std::vector<double> constraint_axis_a_;
  std::vector<double> constraint_axis_b_;
  std::vector<double> constraint_axis_a_secondary_;
  std::vector<double> constraint_axis_b_secondary_;
  std::vector<double> constraint_convel_angle_target_;

  std::vector<int> driven_type_;
  std::vector<int> driven_body_;
  std::vector<int> driven_reaction_body_;
  std::vector<double> driven_point_;
  std::vector<double> driven_reaction_point_;
  std::vector<double> driven_axis_;
  std::vector<double> driven_reference_;
  std::vector<std::string> driven_names_;
  std::vector<double> driven_separation_;

  std::vector<int> bushing_body_a_;
  std::vector<int> bushing_body_b_;
  std::vector<double> bushing_point_a_;
  std::vector<double> bushing_point_b_;
  std::vector<double> bushing_frame_a_;
  std::vector<double> bushing_frame_b_;
  std::vector<double> bushing_reference_translation_;
  std::vector<double> bushing_reference_quaternion_;
  std::vector<double> bushing_stiffness_;
  std::vector<double> bushing_damping_;
  std::vector<double> bushing_preload_;
  std::vector<int> bushing_force_curve_interpolation_;
  std::vector<int> bushing_force_curve_offset_;
  std::vector<int> bushing_force_curve_count_;
  std::vector<double> bushing_force_curve_coordinate_;
  std::vector<double> bushing_force_curve_force_;
  std::vector<int> bushing_rotation_coordinate_;

  std::vector<int> spring_body_a_;
  std::vector<int> spring_body_b_;
  std::vector<double> spring_point_a_;
  std::vector<double> spring_point_b_;
  std::vector<double> spring_stiffness_;
  std::vector<double> spring_compression_damping_;
  std::vector<double> spring_rebound_damping_;
  std::vector<double> spring_free_length_;
  std::vector<double> spring_minimum_length_;
  std::vector<double> spring_maximum_length_;
  std::vector<double> spring_compression_stop_stiffness_;
  std::vector<double> spring_compression_stop_damping_;
  std::vector<double> spring_rebound_stop_stiffness_;
  std::vector<double> spring_rebound_stop_damping_;
  // Zero-length vectors may legally be null, and the registration path tests
  // pointers rather than counts, so the fallback tables keep a valid address.
  mutable std::vector<double> spring_fallback_minimum_;
  mutable std::vector<double> spring_fallback_maximum_;
  mutable std::vector<int> spring_fallback_offset_;
  mutable std::vector<int> spring_fallback_count_;

  std::vector<int> spring_damper_curve_offset_;
  std::vector<int> spring_damper_curve_count_;
  std::vector<double> spring_damper_curve_velocity_;
  std::vector<double> spring_damper_curve_force_;

  std::vector<int> spring_elastic_curve_offset_;
  std::vector<int> spring_elastic_curve_count_;
  std::vector<double> spring_elastic_curve_deflection_;
  std::vector<double> spring_elastic_curve_force_;
  std::vector<int> spring_compression_stop_curve_offset_;
  std::vector<int> spring_compression_stop_curve_count_;
  std::vector<double> spring_compression_stop_curve_penetration_;
  std::vector<double> spring_compression_stop_curve_force_;
  std::vector<int> spring_rebound_stop_curve_offset_;
  std::vector<int> spring_rebound_stop_curve_count_;
  std::vector<double> spring_rebound_stop_curve_penetration_;
  std::vector<double> spring_rebound_stop_curve_force_;

  std::vector<std::string> tire_names_;
  std::vector<int> tire_body_;
  std::vector<double> tire_center_;
  std::vector<double> tire_spin_axis_;
  std::vector<double> tire_forward_axis_;
  std::vector<double> tire_radius_;
  std::vector<double> tire_maximum_compression_;
  std::vector<double> tire_stiffness_;
  std::vector<double> tire_damping_;
  std::vector<double> tire_mu_longitudinal_;
  std::vector<double> tire_mu_lateral_;
  std::vector<double> tire_brush_longitudinal_;
  std::vector<double> tire_brush_lateral_;
  std::vector<double> tire_relaxation_longitudinal_;
  std::vector<double> tire_relaxation_lateral_;
  std::vector<double> tire_detached_relaxation_;

  std::vector<int> anti_roll_body_a_;
  std::vector<int> anti_roll_body_b_;
  std::vector<double> anti_roll_axis_a_;
  std::vector<double> anti_roll_reference_;
  std::vector<double> anti_roll_stiffness_;
  std::vector<double> anti_roll_damping_;

  std::vector<int> aero_body_;
  std::vector<double> aero_point_;
  std::vector<double> aero_axis_;
  std::vector<double> aero_coefficient_;

  std::vector<std::string> steering_names_;
  std::vector<int> steering_type_;
  std::vector<int> steering_body_;
  std::vector<int> steering_reaction_body_;
  std::vector<double> steering_point_;
  std::vector<double> steering_reaction_point_;
  std::vector<double> steering_axis_;
  std::vector<double> steering_reference_;
  std::vector<double> steering_stiffness_;
  std::vector<double> steering_damping_;

  std::vector<int> tire_model_kind_;
  std::vector<int> tire_mirror_;
  /// `tire_count * VEHICLE_PAC2002_PARAMETER_COUNT`, or empty when every tire
  /// is a brush model that carries no parameter block.
  std::vector<double> tire_parameters_;

  std::vector<int> tire_deflection_offset_;
  std::vector<int> tire_deflection_count_;
  std::vector<double> tire_deflection_x_;
  std::vector<double> tire_deflection_y_;
  std::vector<int> tire_bottoming_offset_;
  std::vector<int> tire_bottoming_count_;
  std::vector<double> tire_bottoming_x_;
  std::vector<double> tire_bottoming_y_;

  std::vector<int> tire_frame_body_;
  std::vector<double> tire_frame_center_;
  std::vector<int> tire_drive_body_;
  std::vector<int> tire_drive_reaction_;
  std::vector<double> tire_drive_axis_;

  std::vector<ContractMarker> markers_;

  /// Dense per-body application points for `body_wrench`; empty means "every
  /// wrench acts at its body's origin".  A declared point of (0,0,0) is the
  /// origin too, so a declared table adds nothing for a body that has no
  /// offset -- which is what keeps the change inert for every other family.
  std::vector<Vec3> body_wrench_point_;
  std::vector<unsigned char> body_wrench_declared_;

  std::vector<int> coupler_joint_a_;
  std::vector<int> coupler_coordinate_a_;
  std::vector<double> coupler_scale_a_;
  std::vector<int> coupler_joint_b_;
  std::vector<int> coupler_coordinate_b_;
  std::vector<double> coupler_scale_b_;

  std::vector<int> gauge_body_;
  std::vector<double> gauge_axis_;

  bool vehicle_stages_ = false;
  int road_kind_ = 0;
  double road_origin_x_ = 0.0;
  double road_origin_z_ = 0.0;
  double road_amplitude_ = 0.0;
  double road_wavelength_ = 0.0;
  double road_phase_ = 0.0;
  double road_bump_start_ = 0.0;
  double road_bump_length_ = 0.0;
  double road_corner_scale_[4] = {1.0, 1.0, 1.0, 1.0};

  double gravity_[3] = {0.0, 0.0, 0.0};
};

/// The plan a `multibody-case` document expands into.
struct ContractPlan {
  std::string family;
  std::string name;
  std::string solver_integrator = "ggl_generalized_alpha";
  double rho_inf = 0.8;
  double hht_alpha = -0.3;
  int initialization_mode = 0;
  int adaptive_step = 1;
  double internal_step = 2.5e-4;
  double min_step = 1e-6;
  double max_step = 1e-3;
  double local_relative_tolerance = 1e-5;
  double local_position_tolerance = 1e-7;
  double local_angle_tolerance = 1e-7;
  double local_velocity_tolerance = 1e-6;
  double local_angular_velocity_tolerance = 1e-6;
  double local_brush_tolerance = 1e-7;
  double contact_event_tolerance = 1e-6;
  int max_newton_iterations = 20;
  int max_line_search_iterations = 10;
  double position_tolerance = 1e-8;
  double velocity_tolerance = 1e-7;
  double dynamics_tolerance = 1e-8;
  double increment_tolerance = 1e-8;
  std::vector<ContractCase> cases;
};

/// Expand `document` against `model`, reading the sample tables the document
/// describes out of `blob`.  Only the families this build implements are
/// accepted; anything else fails closed with a message naming the family.
bool contract_expand_case(const JsonValue& document, const std::string& blob,
                          const ContractModel& model, ContractPlan& out,
                          std::string& error);

/// Apply the solver settings of `plan` to the solver scalars of `input`.
void contract_apply_solver(const ContractPlan& plan, AxleInput& input);

/// The case families `contract_expand_case` currently accepts.
bool contract_case_supported(const std::string& family);

}  // namespace axle_kernel
