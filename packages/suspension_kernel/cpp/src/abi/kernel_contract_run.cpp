/// The contract boundary: one entry point, two documents, one result.
///
/// This is the only ABI surface the product grows against.  The flat entry
/// points that preceded it (`axle_run`, `vehicle_run`) are no longer exported:
/// their kernels survive as internal helpers in `kernel_abi.cpp`, so the
/// numerics they exercised are still reachable, but nothing outside the library
/// can bind to them.  Every new capability arrives here instead: a case family
/// is a new file in `mb_cases`, a new element is a row in the element registry,
/// and neither changes this signature.
///
/// The call sequence is deliberately the one the flat vehicle entry already used,
/// so the model the contract describes is registered exactly like the model that
/// entry used to describe.  What is new is where the numbers come from: not from
/// a struct the caller filled, but from a document the kernel parsed.

#include "abi/functions.hpp"

#include "axle_kernel.hpp"

#include "mb_config/version.hpp"
#include "mb_config/element_wrench.hpp"
#include "mb_config/constraint_output.hpp"
#include "mb_config/controller_output.hpp"
#include "mb_cases/functions.hpp"
#include "mb_contract/functions.hpp"
#include "mb_input/functions.hpp"
#include "mb_model/enums.hpp"
#include "mb_tire/pac2002/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_output/functions.hpp"
#include "mb_solve_static/functions.hpp"
#include "mb_joint/functions.hpp"
#include "mb_assembly/functions.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

namespace axle_kernel {
namespace {

using Json = JsonValue;

Json json_integer(long long value) {
  Json out;
  out.kind = JsonKind::Number;
  out.number = static_cast<double>(value);
  out.number_is_integer = true;
  out.integer = value;
  return out;
}

Json json_number(double value) {
  Json out;
  out.kind = JsonKind::Number;
  out.number = value;
  out.number_is_integer = false;
  return out;
}

Json json_string(const std::string& value) {
  Json out;
  out.kind = JsonKind::String;
  out.text = value;
  return out;
}

Json json_array(std::vector<Json> items) {
  Json out;
  out.kind = JsonKind::Array;
  out.items = std::move(items);
  return out;
}

Json json_object(std::vector<std::pair<std::string, Json>> fields) {
  Json out;
  out.kind = JsonKind::Object;
  out.fields = std::move(fields);
  return out;
}

/// Resolve one characteristics request's explicit entity bindings into the
/// solver's own frozen measurement and load-mode descriptions.
///
/// This is the one place the declarative vocabulary of a request meets the
/// numeric vocabulary of the solver, and it is deliberately a *lookup* rather
/// than an interpretation: the contract bans recovering a metric or a frame from
/// an entity name (`mb_cases/functions.hpp`), so a marker is looked up in the
/// model's own marker table and a body name in its own body order, and nothing
/// is ever derived from how either one is spelled.
///
/// It lives in the ABI entry point because that is the only level that may see
/// both sides: `mb_cases` owns the documents and deliberately cannot reach the
/// solver, while `mb_solve_static` owns the frozen descriptions and cannot read
/// a document.  Splitting it would either break the module order or duplicate
/// the binding rules in two places that could then disagree.
///
/// Returns false with a message naming the offending declaration when a binding
/// is missing or cannot be resolved: an unknown marker, an unknown body, or a
/// measurement on a body the model fixes.  A fixed body has no free coordinate,
/// so no measurement of it can ever be answered; letting such a declaration
/// through would refuse the whole request later for a reason that names none of
/// the declarations, which is exactly the report this project refuses to lose.
bool resolve_local_response_bindings(
    const ContractModel& document, const Model& model,
    const ContractCharacteristics& request,
    std::vector<FrozenMeasurement>& measurements,
    std::vector<FrozenLoadMode>& modes, std::string& error) {
  if (document.body_count() != model.bodies.size()) {
    error = "characteristics: the document's bodies and the built model disagree";
    return false;
  }
  // A binding names either a marker (a point on a body) or a body.  Both are
  // explicit and neither is inferred; the marker table is consulted first
  // because a model that declares a marker of that name means that point, and a
  // body name only falls through when no marker carries it.
  const auto bind_entity = [&](const std::string& name, int& body, Vec3& point,
                               bool& has_point, const std::string& where) {
    ContractMarker marker;
    if (document.find_marker(name, marker)) {
      body = marker.body;
      point = marker.point;
      has_point = true;
      return true;
    }
    const int index = document.body_index(name);
    if (index < 0) {
      error = where + " names neither a declared marker nor a declared body: \"" +
              name + "\"";
      return false;
    }
    body = index;
    point = Vec3{};
    has_point = false;
    return true;
  };
  for (std::size_t index = 0; index < request.measurements.size(); ++index) {
    const ContractDeclaration& declared = request.measurements[index];
    const std::string where =
        "characteristics measurement[" + std::to_string(index) + "]";
    if (!declared.measurement_binding.present) {
      error = where +
              " declares no binding, so the request does not say which entity it"
              " measures";
      return false;
    }
    FrozenMeasurement measurement;
    measurement.id = declared.id.empty() ? declared.measurement_id : declared.id;
    measurement.units = declared.units;
    measurement.component = declared.measurement_binding.component;
    if (!bind_entity(declared.measurement_binding.marker, measurement.body,
                     measurement.point, measurement.has_point,
                     where + " binding marker")) {
      return false;
    }
    if (declared.measurement_binding.reference.empty()) {
      // An absent reference is the world, which is the one reference with no
      // body to name.  An explicitly empty reference is refused by the reader,
      // so "absent" and "empty" cannot reach here as the same thing.
      measurement.reference = -1;
    } else if (!bind_entity(declared.measurement_binding.reference,
                            measurement.reference, measurement.reference_point,
                            measurement.has_reference_point,
                            where + " binding reference")) {
      return false;
    }
    if (model.body_to_free[static_cast<std::size_t>(measurement.body)] < 0) {
      error = where + " measures \"" + declared.measurement_binding.marker +
              "\", which the model fixes, so it has no free coordinate";
      return false;
    }
    measurements.push_back(std::move(measurement));
  }
  for (std::size_t index = 0; index < request.perturbations.size(); ++index) {
    const ContractDeclaration& declared = request.perturbations[index];
    const std::string where =
        "characteristics perturbation[" + std::to_string(index) + "]";
    if (!declared.load_binding.present) {
      error = where +
              " declares no binding, so the request does not say which load it"
              " applies";
      return false;
    }
    FrozenLoadMode mode;
    mode.id = declared.id.empty() ? declared.measurement_id : declared.id;
    // The amplitudes are dense over *bodies* in body order, which is the layout
    // `frozen_local_response` reads and `frozen_load_column` indexes: six entries
    // per body in `[Fx,Fy,Fz,Mx,My,Mz]` order.  Two terms on the same body and
    // component add, which is how a mode that names the same wrench twice is
    // read as one amplitude rather than silently losing a term.
    mode.amplitudes.assign(model.bodies.size()*6, 0.0);
    for (const ContractLoadTerm& term : declared.load_binding.terms) {
      const int body = document.body_index(term.body);
      if (body < 0) {
        error = where + " names an unknown body \"" + term.body + "\"";
        return false;
      }
      mode.amplitudes[static_cast<std::size_t>(body)*6 +
                      static_cast<std::size_t>(term.wrench)] += term.amplitude;
    }
    modes.push_back(std::move(mode));
  }
  return true;
}

/// The contract version a document reports.  The optional element-wrench channel
/// is the only reason it moves, and it is off unless a caller asks for it, so a
/// result that carries no channel keeps reporting version 1.
long long document_contract_version(bool observed = false) {
  return observed || element_wrench_output_enabled() ? 2 : 1;
}

/// The `local_response_status` vocabulary.
///
/// It is a small frozen enum rather than a string because the channel is a
/// numeric block (`block_descriptor` carries numbers only), and because the
/// reason a cell is unanswered is the part a caller must not have to infer: an
/// all-NaN column would otherwise mean "singular", "unidentifiable" and "this
/// sample was never reached" at the same time.
enum LocalResponseStatus {
  /// The mode's right-hand side is in `A`'s range but the reading is not
  /// determined by it, so no value is published.
  kLocalResponseUnidentifiable = 1,
  /// The mode's actual right-hand side is outside `A`'s range: no solution.
  kLocalResponseSingular = 2,
  /// The sample could not be prepared at all (frozen state, tangent or solve).
  kLocalResponseRefused = 3,
  /// A value is present and the measurement is identifiable for this sample.
  kLocalResponseValid = 0,
};

/// The block descriptor the result schema requires.
Json block_descriptor(const std::string& name, std::size_t offset, std::size_t length,
                      const std::vector<std::size_t>& shape) {
  std::vector<Json> dimensions;
  dimensions.reserve(shape.size());
  for (std::size_t extent : shape) {
    dimensions.push_back(json_integer(static_cast<long long>(extent)));
  }
  return json_object({
      {"dtype", json_string("float64")},
      {"length", json_integer(static_cast<long long>(length))},
      {"name", json_string(name)},
      {"offset", json_integer(static_cast<long long>(offset))},
      {"order", json_string("C")},
      {"shape", json_array(std::move(dimensions))},
  });
}


/// Point every optional vehicle-extension array at a valid buffer.
///
/// The registration stages were written against the ctypes caller, which always
/// passes a real pointer -- for it, "no data" is an empty NumPy array, not a
/// null pointer -- and several of them validate the pointer rather than the
/// count.  A contract caller has no such arrays, so it supplies the same thing
/// the mirror did: a one-element buffer whose value is never read when the
/// count is zero.  The exception is the per-bushing curve table, whose offsets
/// are indexed once per bushing and therefore has to be as long as the table it
/// indexes.
void provide_empty_option_arrays(VehicleInput& input, std::size_t bushing_count,
                                 int* dummy_int, double* dummy_double) {
  input.steering_type = dummy_int;
  input.steering_body = dummy_int;
  input.steering_reaction_body = dummy_int;
  input.steering_point_local = dummy_double;
  input.steering_reaction_point_local = dummy_double;
  input.steering_axis_local = dummy_double;
  input.steering_reference_quaternion = dummy_double;
  input.steering_target_angle = dummy_double;
  input.steering_target_rate = dummy_double;
  input.steering_stiffness = dummy_double;
  input.steering_damping = dummy_double;
  input.road_corner_scale = dummy_double;
  input.tire_frame_body = dummy_int;
  input.tire_frame_center_local = dummy_double;
  input.tire_model_kind = dummy_int;
  input.tire_pac2002_parameters = dummy_double;
  input.tire_pac2002_mirror = dummy_int;
  input.tire_drive_torque_body = dummy_int;
  input.tire_drive_torque_reaction_body = dummy_int;
  input.tire_drive_torque_axis_local = dummy_double;
  input.tire_deflection_curve_offset = dummy_int;
  input.tire_deflection_curve_count = dummy_int;
  input.tire_deflection_curve_deflection = dummy_double;
  input.tire_deflection_curve_force = dummy_double;
  input.tire_bottoming_curve_offset = dummy_int;
  input.tire_bottoming_curve_count = dummy_int;
  input.tire_bottoming_curve_penetration = dummy_double;
  input.tire_bottoming_curve_force = dummy_double;
  input.vehicle_spring_elastic_curve_offset = dummy_int;
  input.vehicle_spring_elastic_curve_count = dummy_int;
  input.vehicle_spring_elastic_curve_deflection = dummy_double;
  input.vehicle_spring_elastic_curve_force = dummy_double;
  input.vehicle_spring_compression_stop_curve_offset = dummy_int;
  input.vehicle_spring_compression_stop_curve_count = dummy_int;
  input.vehicle_spring_compression_stop_curve_penetration = dummy_double;
  input.vehicle_spring_compression_stop_curve_force = dummy_double;
  input.vehicle_spring_rebound_stop_curve_offset = dummy_int;
  input.vehicle_spring_rebound_stop_curve_count = dummy_int;
  input.vehicle_spring_rebound_stop_curve_penetration = dummy_double;
  input.vehicle_spring_rebound_stop_curve_force = dummy_double;
  input.vehicle_bushing_force_curve_coordinate = dummy_double;
  input.vehicle_bushing_force_curve_force = dummy_double;
  input.static_rotation_gauge_body = dummy_int;
  input.static_rotation_gauge_axis_local = dummy_double;
  input.coordinate_coupler_joint_a = dummy_int;
  input.coordinate_coupler_coordinate_a = dummy_int;
  input.coordinate_coupler_joint_b = dummy_int;
  input.coordinate_coupler_coordinate_b = dummy_int;
  input.coordinate_coupler_scale_a = dummy_double;
  input.coordinate_coupler_scale_b = dummy_double;
  input.aerodynamic_drag_body = dummy_int;
  input.aerodynamic_drag_application_point = dummy_double;
  input.aerodynamic_drag_forward_axis = dummy_double;
  input.aerodynamic_drag_coefficient = dummy_double;
  (void)bushing_count;
}

void append_doubles(const double* values, std::size_t count, std::string& out) {
  const char* bytes = reinterpret_cast<const char*>(values);
  out.append(bytes, count * sizeof(double));
}

int fail(char* error_buffer, std::size_t error_capacity, int code,
         const std::string& message) {
  set_error(error_buffer, error_capacity, message);
  return code;
}

}  // namespace
}  // namespace axle_kernel

extern "C" AXLE_API int32_t suspension_kernel_contract_version() {
  return 1;
}

namespace axle_kernel {
namespace {

/// The capability document: what this kernel computes exactly, and which PAC2002
/// coefficient families it refuses.
Json capability_document() {
  std::vector<Json> modes;
  for (int mode : pac2002_supported_use_modes()) {
    modes.push_back(json_integer(static_cast<long long>(mode)));
  }
  std::vector<Json> parameters;
  for (const char* name : pac2002_refused_parameters()) {
    parameters.push_back(json_string(name));
  }
  std::vector<Json> flags;
  for (const char* name : pac2002_refused_feature_flags()) {
    flags.push_back(json_string(name));
  }
  std::vector<Json> families;
  for (const Pac2002RefusedFamily& family : pac2002_refused_families()) {
    std::vector<Json> coefficients;
    for (const char* name : family.coefficients) {
      coefficients.push_back(json_string(name));
    }
    families.push_back(json_object({
        {"name", json_string(family.name)},
        {"reason", json_string(family.reason)},
        {"coefficients", json_array(std::move(coefficients))},
    }));
  }
  std::vector<Json> case_families;
  for (int index = 0; index < contract_case_family_count(false); ++index) {
    const char* name = contract_case_family_name(index);
    if (contract_case_supported(name)) case_families.push_back(json_string(name));
  }
  // The characteristics declaration is deliberately two fields rather than one.
  // Reading a request and *answering* it are different capabilities, and this
  // build does the first for every request and the second for exactly one
  // response kind: `local_response`, the load-driven frozen projection the
  // `frozen_load_compliance_v1` protocol's own `response_contract` names.  That
  // is also the kind the Python compiler checks, so a request naming it is
  // answerable and every other kind is refused before submission.  Listing a
  // kind the kernel does not compute would be the empty-shell capability the
  // contract forbids; listing nothing while it does compute one would refuse a
  // request the kernel can answer.
  std::vector<Json> characteristics_kinds;
  characteristics_kinds.push_back(json_string("local_response"));
  return json_object({
      {"contract", json_string("multibody-capabilities")},
      {"contract_version", json_integer(document_contract_version())},
      {"function_program_versions", json_array({json_integer(1)})},
      {"joint_coordinate_motion_versions", json_array({json_integer(1)})},
      {"constraint_channel_versions", json_array({json_integer(1)})},
      {"characteristics_request_versions", json_array({json_integer(1)})},
      {"characteristics_response_kinds", json_array(std::move(characteristics_kinds))},
      {"case_families", json_array(std::move(case_families))},
      // A time-history family may declare its cases as a top-level `runs` list,
      // which lets one document run several excitations on the shared grid.  The
      // version names the merge and refusal rules the case layer implements, so a
      // caller that writes a runs list can refuse a build that would ignore it.
      {"case_runs_versions", json_array({json_integer(1)})},
      {"function_element_types", json_array({json_string("force"), json_string("torque"), json_string("wrench")})},
      {"pac2002_refused_families", json_array(std::move(families))},
      {"pac2002_refused_feature_flags", json_array(std::move(flags))},
      {"pac2002_refused_parameters", json_array(std::move(parameters))},
      {"pac2002_supported_use_modes", json_array(std::move(modes))},
  });
}

}  // namespace
}  // namespace axle_kernel

extern "C" AXLE_API int32_t suspension_kernel_capabilities(
    char* buffer, size_t capacity, size_t* written) {
  using namespace axle_kernel;
  if (written == nullptr) return 1;
  std::string canonical;
  std::string error;
  if (!contract_write_canonical(capability_document(), canonical, error)) {
    *written = 0;
    return 3;
  }
  *written = canonical.size();
  if (buffer == nullptr || capacity < canonical.size()) return 11;
  std::memcpy(buffer, canonical.data(), canonical.size());
  return 0;
}

extern "C" AXLE_API int32_t suspension_kernel_run(
    const std::uint8_t* model_payload, std::size_t model_length,
    const std::uint8_t* case_payload, std::size_t case_length,
    std::uint8_t* result_out, std::size_t* result_length_in_out,
    char* error_buffer, std::size_t error_capacity
) try {
  using namespace axle_kernel;
  if (model_payload == nullptr || case_payload == nullptr ||
      result_length_in_out == nullptr) {
    return fail(error_buffer, error_capacity, 1,
                "model payload, case payload and result length are required");
  }

  // The element-wrench sink is process-global because the element laws reach it
  // through the force-assembly primitives rather than through their arguments.
  // It holds a pointer into a block owned by this call, so every return path has
  // to hand the target back: a later entry point that does not configure it
  // (mb_core_run) would otherwise write through a pointer into this frame's dead
  // storage.  The guard runs on all of them, the early failures included.
  struct ElementWrenchGuard {
    ~ElementWrenchGuard() {
      if (ElementWrenchSink* const sink = element_wrench_sink()) sink->clear();
    }
  } element_wrench_guard;
  struct ConstraintOutputGuard {
    ~ConstraintOutputGuard() { constraint_output_sink().clear(); }
  } constraint_output_guard;
  // The controller ledger's recorder is process-global for the same reason and
  // holds a pointer into a block owned by this call, so every return path has to
  // hand its target back.  One guard beside the other: the two channels share
  // every property that made the first one necessary.
  struct ControllerGuard {
    ~ControllerGuard() {
      if (ControllerSink* const sink = controller_sink()) sink->clear();
    }
  } controller_guard;

  ContractPayload model_payload_parsed;
  ContractPayload case_payload_parsed;
  std::string error;
  if (!contract_parse_container(model_payload, model_length, model_payload_parsed, error)) {
    return fail(error_buffer, error_capacity, 2, "model container: " + error);
  }
  if (!contract_parse_container(case_payload, case_length, case_payload_parsed, error)) {
    return fail(error_buffer, error_capacity, 2, "case container: " + error);
  }

  ContractModel model;
  if (!model.read(model_payload_parsed.document, model_payload_parsed.blob, error)) {
    return fail(error_buffer, error_capacity, 2, "model document: " + error);
  }
  ContractPlan plan;
  if (!contract_expand_case(case_payload_parsed.document, case_payload_parsed.blob,
                            model, plan, error)) {
    return fail(error_buffer, error_capacity, 2, "case document: " + error);
  }

  // Every case in one document shares its time grid, so the driven target
  // tables all have the same shape.  They live in one buffer because the
  // registration stage keeps the pointer rather than a copy: a per-case vector
  // would leave the model pointing at a freed allocation.
  const std::size_t samples = plan.cases.front().sample_count;
  std::vector<double> driven_target(samples * model.driven_count(), 0.0);
  std::vector<double> driven_target_rate(samples * model.driven_count(), 0.0);
  // The steering targets and the brake torque are per-sample tables too, and
  // the registration stage keeps the pointer it is handed for both -- so they
  // live in buffers whose address never changes, exactly like the driven
  // targets, and each case overwrites the contents.
  const std::size_t actuator_count = model.steering_order().size();
  std::vector<double> steering_target(samples * actuator_count, 0.0);
  std::vector<double> steering_rate(samples * actuator_count, 0.0);
  std::vector<double> brake_torque(
      samples * (model.tire_order().empty() ? 1 : model.tire_order().size()), 0.0);
  // The normalized demands, in buffers of the same shape and the same lifetime
  // rule as the tables above (subtask p2-10).  They are always allocated so a
  // case can point the model at them; a run that declares none leaves them
  // empty-valued and, crucially, leaves the model's pointers null, which is
  // what tells every demand reader that no demand was declared.
  std::vector<double> wheel_demand(
      samples * (model.tire_order().empty() ? 1 : model.tire_order().size()), 0.0);
  std::vector<double> brake_demand(
      samples * (model.tire_order().empty() ? 1 : model.tire_order().size()), 0.0);

  VehicleInput input{};
  input.struct_size = sizeof(VehicleInput);
  input.abi_version = static_cast<std::uint32_t>(kVehicleKernelAbiVersion);
  input.reserved = 0;
  input.axle.struct_size = sizeof(AxleInput);
  input.axle.abi_version = static_cast<std::uint32_t>(kAxleKernelAbiVersion);
  input.axle.reserved = 0;
  // The pass installs a valid pointer for every optional array so that a
  // registration stage which validates the pointer rather than the count has
  // something to validate.  It runs *before* the model fills the structure, so
  // every array the document actually carries replaces its placeholder.
  int dummy_int = 0;
  double dummy_double = 0.0;
  provide_empty_option_arrays(input, 0, &dummy_int, &dummy_double);

  model.fill(input);
  input.initial_state_angle_tolerance = 1e-6;
  input.axle.sample_count = samples;
  input.axle.sample_times = plan.cases.front().sample_times.data();
  input.axle.body_wrench = plan.cases.front().body_wrench.data();
  input.driven_target = driven_target.data();
  input.driven_target_rate = driven_target_rate.data();
  if (actuator_count != 0) {
    input.steering_target_angle = steering_target.data();
    input.steering_target_rate = steering_rate.data();
  }
  input.brake_torque = model.tire_order().empty() ? nullptr : brake_torque.data();

  // A tire needs more than the geometry the model document carries.  The frame
  // it is measured against defaults to its own body, and the PAC2002 parameter
  // block exists on every tire even when the model selects the brush law, which
  // reads none of it -- the registration stage validates it for finiteness
  // regardless, so a zero block is both correct and sufficient here.
  const std::size_t tire_count = model.tire_order().size();
  const std::size_t tire_slots = tire_count == 0 ? 1 : tire_count;
  // A model that carries tire model kinds publishes its own arrays; the zero
  // block below is the brush-model default the model fills over.
  std::vector<int> tire_kind(tire_slots, 0);
  std::vector<int> tire_mirror(tire_slots, 0);
  // A negative drive body is the documented "apply to the tire's own body"
  // mapping, and it is only valid with a negative reaction and a zero axis.
  std::vector<int> tire_drive_body(tire_slots, -1);
  std::vector<int> tire_drive_reaction(tire_slots, -1);
  std::vector<double> tire_drive_axis(tire_slots * 3, 0.0);
  std::vector<double> tire_parameters(
      tire_count == 0 ? 1 : tire_count * VEHICLE_PAC2002_PARAMETER_COUNT, 0.0);
  std::vector<int> tire_frame_body(tire_slots, 0);
  std::vector<double> tire_frame_centre(tire_slots * 3, 0.0);
  if (tire_count != 0) {
    // The document resolves the frame a tire is measured against and any
    // drive-torque mapping, so the model carries both; no fallback applies.
    tire_frame_body = model.tire_frame_bodies();
    tire_frame_centre = model.tire_frame_centres();
    tire_drive_body = model.tire_drive_bodies();
    tire_drive_reaction = model.tire_drive_reactions();
    tire_drive_axis = model.tire_drive_axes();
  }
  const std::vector<int>& model_kinds = model.tire_model_kinds();
  const std::vector<int>& model_mirrors = model.tire_mirrors();
  const std::vector<double>& model_parameters = model.tire_parameters();
  if (model_kinds.size() == tire_count) tire_kind = model_kinds;
  if (model_mirrors.size() == tire_count) tire_mirror = model_mirrors;
  if (model_parameters.size() == tire_count * VEHICLE_PAC2002_PARAMETER_COUNT) {
    tire_parameters = model_parameters;
  }
  input.tire_model_kind = tire_kind.data();
  input.tire_pac2002_mirror = tire_mirror.data();
  input.tire_pac2002_parameters = tire_parameters.data();
  input.tire_frame_body = tire_frame_body.data();
  input.tire_frame_center_local = tire_frame_centre.data();
  input.tire_drive_torque_body = tire_drive_body.data();
  input.tire_drive_torque_reaction_body = tire_drive_reaction.data();
  input.tire_drive_torque_axis_local = tire_drive_axis.data();
  // The tire tables are optional and already owned by `ContractModel::fill`;
  // zero-filled arrays here would erase every measured vertical curve before
  // registration sees it.

  Model built = build_model(
      input.axle,
      error,
      input.constraint_axis_a_secondary,
      input.constraint_axis_b_secondary,
      input.constraint_convel_angle_target,
      input.coordinate_coupler_count,
      input.coordinate_coupler_joint_a,
      input.coordinate_coupler_coordinate_a,
      input.coordinate_coupler_scale_a,
      input.coordinate_coupler_joint_b,
      input.coordinate_coupler_coordinate_b,
      input.coordinate_coupler_scale_b
  );
  // A model that declares application points for its external loads says the
  // load is a force at a body-fixed marker, so the lever arm is the deformed
  // one.  `build_model` cannot know that -- the declaration is contract-level,
  // not part of the flat element table -- so it is installed here, once, on
  // the model every case of this run shares.  An empty table leaves the
  // historical origin convention untouched.
  built.body_wrench_point_local = model.body_wrench_points();
  built.function_programs = model.function_programs();
  built.function_elements = model.function_elements();
  if (!error.empty()) {
    return fail(error_buffer, error_capacity, 2, "model build: " + error);
  }
  // The tire's own mass and inertia are contract-level declarations, like the
  // body-wrench points above: `build_model` cannot see them because they are
  // deliberately not an `AxleInput` array (adding one to the frozen structure
  // would be an ABI freeze release).  `ContractModel` read them out of the
  // document, so they are installed here on the one model every case of this
  // run shares.  Zero -- what a document that declares neither field gets --
  // leaves the historical convention in place, where the wheel-end body owns
  // the inertia.
  if (!install_tire_mass(model, built, error)) {
    return fail(error_buffer, error_capacity, 2, "tire mass: " + error);
  }
  // With the tires settled, the per-body inertia the solver reads is summed once,
  // here.  Every consumer goes through `body_effective_mass` /
  // `body_effective_inertia_body`, and a document that declares no tire mass gets
  // its bodies' own values back unchanged.
  if (!compute_effective_body_inertia(built, error)) {
    return fail(error_buffer, error_capacity, 2, "tire inertia: " + error);
  }
  // The rotational-torque family is the one element family `AxleInput` carries
  // no field for, and it has to stay that way: a new field on that structure is
  // an ABI release.  So `ContractModel` kept the family as the uniform
  // `ElementBlock` the generic reader already accepts, and it is appended here,
  // after `build_model` has built the model the document's other families
  // describe.  Reading it through the same reader the C ABI uses is what makes
  // the two routes agree on what one element means.
  //
  // The two input forms stay mutually exclusive: these blocks never touch
  // `input.axle.elements`, so `build_model`'s guard sees the per-family tables
  // only.  `read_element_blocks` refuses a null curve array when blocks are
  // supplied, and this family declares no curve slot at all
  // (`curve_slots = 0`), so the array is real but nothing in it is read.
  if (!model.rotational_torques().empty()) {
    const std::vector<ElementBlock>& torque_blocks = model.rotational_torques();
    std::vector<ElementCurveReference> torque_curves(
        torque_blocks.size() * kElementCurveSlots);
    for (ElementCurveReference& slot : torque_curves) {
      slot.values = nullptr;
      slot.count = 0;
    }
    if (!read_element_blocks(torque_blocks.data(), torque_blocks.size(),
                             torque_curves.data(), 0, nullptr, built, error)) {
      return fail(error_buffer, error_capacity, 2, "model element: " + error);
    }
  }
  // The registration order is the one the vehicle entry point established; the
  // stages whose counts are zero return immediately, so a family that does not
  // use tires or springs pays nothing for their presence here.
  // The registration stages read their option arrays, so the "no explicit
  // choice" values have to exist before the first of them runs.  A zero array
  // is the documented default in both cases: the rotation-vector convention
  // and piecewise-linear interpolation.
  // `std::vector::data()` is allowed to be null when the vector is empty, and
  // the registration stages test the pointer rather than the count, so every
  // one of these is at least one element long even when the model has no
  // bushings at all.
  const std::size_t bushing_slots = built.bushings.empty() ? 1 : built.bushings.size();
  if (model.needs_vehicle_stages()) {
    std::vector<int> bushing_rotation_fallback(bushing_slots, 0);
    // The registration stages treat a null pointer as a malformed table even
    // when the count is zero, so every optional table has a one-element
    // fallback.  When the document carries curves, the model's owned arrays
    // replace these placeholders.
    std::vector<int> bushing_interpolation_fallback(bushing_slots, 0);
    std::vector<int> bushing_curve_offset_fallback(bushing_slots * 6, 0);
    std::vector<int> bushing_curve_count_fallback(bushing_slots * 6, 0);
    std::vector<double> bushing_curve_coordinate_fallback(1, 0.0);
    std::vector<double> bushing_curve_force_fallback(1, 0.0);
    const auto& bushing_rotation = model.bushing_rotation_coordinates();
    const auto& bushing_interpolation =
        model.bushing_force_curve_interpolations();
    const auto& bushing_curve_offset = model.bushing_force_curve_offsets();
    const auto& bushing_curve_count = model.bushing_force_curve_counts();
    const auto& bushing_curve_coordinate =
        model.bushing_force_curve_coordinates();
    const auto& bushing_curve_force = model.bushing_force_curve_values();
    input.bushing_rotation_coordinates =
        bushing_rotation.empty() ? bushing_rotation_fallback.data()
                                 : bushing_rotation.data();
    input.bushing_force_curve_interpolation =
        bushing_interpolation.empty()
            ? bushing_interpolation_fallback.data()
            : bushing_interpolation.data();
    input.vehicle_bushing_force_curve_offset =
        bushing_curve_offset.empty()
            ? bushing_curve_offset_fallback.data()
            : bushing_curve_offset.data();
    input.vehicle_bushing_force_curve_count =
        bushing_curve_count.empty()
            ? bushing_curve_count_fallback.data()
            : bushing_curve_count.data();
    input.vehicle_bushing_force_curve_coordinate =
        bushing_curve_coordinate.empty()
            ? bushing_curve_coordinate_fallback.data()
            : bushing_curve_coordinate.data();
    input.vehicle_bushing_force_curve_force =
        bushing_curve_force.empty()
            ? bushing_curve_force_fallback.data()
            : bushing_curve_force.data();

    if (!add_vehicle_tire_frames(input, built, error) ||
        !add_vehicle_tire_models(input, built, error) ||
        !add_vehicle_drive_torque_mappings(input, built, error) ||
        !add_vehicle_spring_curves(input, built, error) ||
        !add_vehicle_bushing_curves(input, built, error) ||
        !add_vehicle_bushing_rotation_coordinates(input, built, error) ||
        !add_vehicle_aerodynamic_drags(input, built, error) ||
        !add_vehicle_steering_actuators(input, built, error)) {
      return fail(error_buffer, error_capacity, 2, "model registration: " + error);
    }

    // Driven coordinates and prescribed actuators append constraint rows after
    // `build_model` has audited the system, so the rank audit runs again -- the
    // same reason the ctypes vehicle entry point repeats it.
    if (!add_driven_coordinates(input, built, error)) {
      return fail(error_buffer, error_capacity, 2, "driven coordinates: " + error);
    }
    if (built.rows > 0 && !audit_constraint_system(built, error)) {
      return fail(error_buffer, error_capacity, 2, "constraint audit: " + error);
    }
    if (!add_vehicle_road_profile(input, built, error) ||
        !add_vehicle_static_rotation_gauges(input, built, error)) {
      return fail(error_buffer, error_capacity, 2, "model registration: " + error);
    }
  }

  // A static gauge removes a declared global null space rather than adding a
  // physical constraint, so it belongs to the model the solver sees and has to
  // be installed before the first run.  Every case in one document shares it.
  if (plan.cases.front().has_static_gauge) {
    const std::uint32_t mask = plan.cases.front().static_gauge_dof_mask;
    const std::size_t body = plan.cases.front().static_gauge_body;
    if ((mask & ~static_cast<std::uint32_t>(0x3F)) != 0) {
      return fail(error_buffer, error_capacity, 4,
                  "static gauge mask must use pose bits 0 through 5");
    }
    if (body >= built.bodies.size() || built.bodies[body].fixed) {
      return fail(error_buffer, error_capacity, 4,
                  "static gauge names a body that cannot be gauged");
    }
    if (plan.cases.front().static_trim_then_release &&
        plan.cases.front().initial_state_angle_tolerance <= 0.0) {
      return fail(error_buffer, error_capacity, 4,
                  "static-trim release needs a positive angle tolerance");
    }
    built.static_gauge_body = static_cast<int>(body);
    built.static_gauge_dof_mask = mask;
  }

  // `run_model` reads the brake torque through the model rather than the input,
  // so the bus has to be pointed at the buffer before the first case runs.
  if (!model.tire_order().empty()) {
    built.vehicle_brake_torque = input.brake_torque;
  }
  // The normalized demands (subtask p2-10).  Registered only when a case
  // actually declares one: a run that declares none leaves both pointers null,
  // which is what makes every demand reader behave exactly as it did before the
  // channel existed.  The per-case contents are copied in the case loop.
  bool declares_wheel_demand = false;
  bool declares_brake_demand = false;
  for (const ContractCase& run : plan.cases) {
    declares_wheel_demand = declares_wheel_demand || !run.wheel_demand.empty();
    declares_brake_demand = declares_brake_demand || !run.brake_demand.empty();
  }
  if (declares_wheel_demand) built.vehicle_wheel_demand = wheel_demand.data();
  if (declares_brake_demand) built.vehicle_brake_demand = brake_demand.data();

  const std::size_t bodies = built.bodies.size();
  const std::size_t wrench_rows = built.constraints.size();
  const std::size_t cases = plan.cases.size();
  std::size_t total_samples = 0;
  for (const ContractCase& run : plan.cases) total_samples += run.sample_count;
  const std::size_t diagnostics_rows = total_samples + 2 * cases;
  const Json* requested_outputs = case_payload_parsed.document.find("outputs");
  const auto requested = [&](const char* name) {
    const Json* value = requested_outputs == nullptr ? nullptr : requested_outputs->find(name);
    return value != nullptr && value->kind == JsonKind::Bool && value->boolean;
  };
  const bool constraint_output_enabled = requested("constraint_reaction");
  const bool jacobian_output_enabled = requested("constraint_jacobian");
  const bool accepted_state_enabled = requested("accepted_state");
  if (accepted_state_enabled &&
      (plan.family == "kc_quasi_static" || plan.family == "vehicle_kc")) {
    return fail(error_buffer, error_capacity, 2,
                "accepted_state requires a dynamic case");
  }
  std::vector<double> accepted_identity, accepted_output_map, accepted_states, accepted_multipliers,
      accepted_internal;
  // All State-owned internal vectors, including their derivatives, are copied.
  // An absent vector has count zero, not a fabricated tire-sized zero vector.
  const std::vector<std::pair<const char*, std::vector<double> State::*>> internal_fields = {
      {"tire_sx", &State::tire_sx}, {"tire_sy", &State::tire_sy},
      {"tire_sx_dot", &State::tire_sx_dot}, {"tire_sy_dot", &State::tire_sy_dot},
      {"tire_u", &State::tire_u}, {"tire_v", &State::tire_v},
      {"tire_u_dot", &State::tire_u_dot}, {"tire_v_dot", &State::tire_v_dot},
      {"tire_beta", &State::tire_beta}, {"tire_beta_dot", &State::tire_beta_dot},
      {"tire_u_ddot", &State::tire_u_ddot}, {"tire_v_ddot", &State::tire_v_ddot},
      {"tire_beta_ddot", &State::tire_beta_ddot}, {"tire_phi", &State::tire_phi},
      {"tire_phi_dot", &State::tire_phi_dot}, {"tire_maxwell", &State::tire_maxwell}};
  std::vector<std::size_t> internal_counts;
  std::size_t internal_width = 1; // State::step_size


  const std::size_t state_length = total_samples * bodies * kStatePerBody;
  const std::size_t wrench_length = total_samples * wrench_rows * kConstraintOutputWidth;
  const std::size_t diagnostics_length = diagnostics_rows * kDiagnosticsWidth;

  std::vector<double> states(state_length, std::numeric_limits<double>::quiet_NaN());
  std::vector<double> wrenches(wrench_length, std::numeric_limits<double>::quiet_NaN());
  std::vector<double> diagnostics(diagnostics_length,
                                  std::numeric_limits<double>::quiet_NaN());
  std::vector<double> constraint_multipliers(
      constraint_output_enabled ? total_samples*built.rows : 0,
      std::numeric_limits<double>::quiet_NaN());
  std::vector<double> constraint_reactions(
      constraint_output_enabled ? total_samples*wrench_rows*2*kConstraintReactionWidth : 0,
      std::numeric_limits<double>::quiet_NaN());
  std::vector<double> coupler_reactions(
      constraint_output_enabled ? total_samples*built.coordinate_couplers.size()*4*kConstraintReactionWidth : 0,
      std::numeric_limits<double>::quiet_NaN());
  std::vector<double> constraint_jacobians(
      jacobian_output_enabled ? total_samples*built.rows*built.ndof : 0,
      std::numeric_limits<double>::quiet_NaN());
  std::vector<double> contact_events;
  // The element, tire and energy ledgers are *result*, not scratch: the dynamic
  // families report tire forces, spring and bushing loads and the energy
  // balance, and a caller that had to reach the flat ABI for them would keep
  // that ABI alive.  Each is one block per sample in element order, sized for
  // the whole run so a multi-case plan can offset into it like `body_state`.
  const std::size_t spring_count = built.springs.size();
  const std::size_t damper_count = built.dampers.size();
  const std::size_t bump_stop_count = built.bump_stops.size();
  const std::size_t bushing_count = built.bushings.size();
  const std::size_t anti_roll_count = built.anti_roll_bars.size();
  const double kNan = std::numeric_limits<double>::quiet_NaN();
  // --- the load-driven frozen local response ----------------------------------
  //
  // A characteristics request is inert data until something answers it.  This
  // build answers exactly one definition: `frozen_load_compliance` projected
  // through the `local_response` operator, which is the request the frozen
  // protocol's own `response_contract` names.  Any other definition keeps the
  // read-and-echo behaviour it had -- publishing a block for a definition the
  // kernel does not compute would be the empty-shell capability the contract
  // forbids, and the capability document below says the same thing in the
  // caller's own vocabulary.
  //
  // The bindings are resolved *once*, here, rather than inside the observer:
  // a request whose bindings cannot be resolved is refused before a single
  // sample runs, with a message naming the declaration, instead of failing
  // halfway through a dynamic integration.
  const bool answers_local_response =
      plan.characteristics.present &&
      plan.characteristics.definition_kind == "frozen_load_compliance" &&
      plan.characteristics.operator_kind == "local_response" &&
      // The response kind is a property of each *measurement* declaration, and
      // it is the same field the Python compiler checks against the capability
      // document.  Reading it here is what keeps the two gates from disagreeing:
      // a request the compiler admits must be one this build answers, and a
      // request whose measurements ask for something else keeps the read-only
      // behaviour it had.
      std::all_of(plan.characteristics.measurements.begin(),
                  plan.characteristics.measurements.end(),
                  [](const ContractDeclaration& declared) {
                    return declared.response_kind == "local_response";
                  });
  std::vector<FrozenMeasurement> local_measurements;
  std::vector<FrozenLoadMode> local_modes;
  std::vector<double> local_values, local_status, local_modes_per_sample,
      local_audit, local_identity;
  std::size_t local_records = 0;
  // Counted rather than derived, because a refused sample is the one fact the
  // status block cannot express on its own: an all-NaN row looks the same
  // whether the sample was refused or simply never reached.
  std::size_t local_refused_samples = 0;
  if (answers_local_response) {
    std::string binding_error;
    if (!resolve_local_response_bindings(model, built, plan.characteristics,
                                         local_measurements, local_modes,
                                         binding_error)) {
      return fail(error_buffer, error_capacity, 2, binding_error);
    }
    if (local_measurements.empty() || local_modes.empty()) {
      return fail(error_buffer, error_capacity, 2,
                  "characteristics: a load-driven response needs at least one"
                  " measurement and one perturbation");
    }
    local_values.resize(total_samples * local_modes.size() *
                            local_measurements.size(),
                        kNan);
    local_status.resize(total_samples * local_modes.size() *
                            local_measurements.size(),
                        kNan);
    local_modes_per_sample.assign(total_samples, kNan);
    // Three audit columns per sample: the tangent dimension, its rank, and the
    // number of numeric factorizations the sample actually spent.  They are
    // numbers rather than prose so a caller can see that one sample reused one
    // factorization for every mode, which is the cost claim the design makes.
    local_audit.assign(total_samples*3, kNan);
    // [case_index, accepted_step_index, time_s, last_output_sample_id].  A
    // sample the run never reached (a case that failed part way) leaves this
    // row NaN, so "no response" cannot be confused with "response of zero".
    local_identity.assign(total_samples*4, kNan);
    local_records = total_samples;
  }
  std::vector<double> energy_block(total_samples * kEnergyOutputWidth, kNan);
  std::vector<double> steering_block(
      total_samples * actuator_count * kSteeringOutputWidth, kNan);
  std::vector<double> spring_block(total_samples * spring_count * kSpringOutputWidth, kNan);
  std::vector<double> damper_block(total_samples * damper_count * kDamperOutputWidth, kNan);
  std::vector<double> bump_stop_block(
      total_samples * bump_stop_count * kBumpStopOutputWidth, kNan);
  std::vector<double> bushing_block(total_samples * bushing_count * kBushingOutputWidth, kNan);
  std::vector<double> anti_roll_block(total_samples * anti_roll_count * kAntiRollOutputWidth, kNan);
  std::vector<double> tire_block(total_samples * tire_count * kTireOutputWidth, kNan);
  // The optional element-wrench channel: one row per (element, end) per sample.
  // While the switch is off nothing is allocated, nothing is described and
  // nothing is appended, so the default path's blocks and blob stay
  // byte-for-byte what they were.  The rows are fed by the observer pass, which
  // is the only place one accepted sample's element wrenches exist as a coherent
  // set; the sink owns the row layout so the element laws do not have to.
  ElementWrenchCounts element_wrench_counts;
  element_wrench_counts.springs = spring_count;
  element_wrench_counts.dampers = damper_count;
  element_wrench_counts.bump_stops = bump_stop_count;
  element_wrench_counts.bushings = bushing_count;
  element_wrench_counts.anti_rolls = anti_roll_count;
  element_wrench_counts.rotational_torques = built.rotational_torques.size();
  element_wrench_counts.functions = built.function_elements.size();
  element_wrench_counts.steering = built.steering_actuators.size();
  element_wrench_counts.tires = tire_count;
  element_wrench_counts.bodies = bodies;
  element_wrench_counts.drags = built.aerodynamic_drags.size();
  const bool element_wrench_enabled = element_wrench_output_enabled() || requested("element_wrench");
  const std::size_t element_wrench_records = element_wrench_enabled
      ? element_wrench_record_count(element_wrench_counts) : 0;
  std::vector<double> element_wrench_block(
      element_wrench_records == 0
          ? 0
          : total_samples * element_wrench_records * kElementWrenchOutputWidth,
      kNan);
  // The optional closed-loop controller ledger (p5-04): one row per sample,
  // four columns, written by the element law as it derives the ABS demand.
  // Off unless `SUSPENSION_KERNEL_CONTROLLER_OUTPUT` asks for it, so the
  // default path allocates nothing, describes nothing and appends nothing.
  const bool controller_enabled = controller_output_enabled();
  std::vector<double> controller_block(
      controller_enabled ? total_samples * kControllerOutputWidth : 0, kNan);
  // A block descriptor cannot express a zero extent, so an empty ledger is an
  // absent block rather than a block of nothing.
  const auto block_slot = [](std::vector<double>& block, std::size_t stride,
                             std::size_t sample) {
    return block.empty() ? static_cast<double*>(nullptr) : block.data() + sample * stride;
  };
  struct CaseSpan {
    std::string name;
    std::size_t offset = 0;
    std::size_t sample_count = 0;
    std::size_t contact_events = 0;
  };
  std::vector<CaseSpan> case_spans;
  // A case that fails does not erase the run.  `run_model` wrote the samples it
  // did converge -- including the one that failed, with its accepted flag clear
  // -- and that is exactly what a caller needs to see: the CLI prints it, and
  // the acceptance evidence is written from it.  So a failure stops the loop and
  // is *reported* in the result document rather than thrown away, and the caller
  // decides what a failed run means.
  struct CaseFailure {
    bool failed = false;
    std::string case_name;
    std::string message;
    std::size_t sample_index = 0;
    double time_s = 0.0;
    // The status `run_model` returned.  A caller maps failures onto its own
    // error type, and the code is part of that mapping.
    int status = 0;
  } failure;
  std::size_t sample_offset = 0;
  for (const ContractCase& run : plan.cases) {
    model.fill_case(run, input);
    // A road is a *case* input: the same vehicle is driven over different
    // surfaces, and a case that declares one overrides whatever the model
    // carried.  The override used to be read into the case and then never
    // applied, which showed up as a vehicle that never touched the road it was
    // given -- invisible while every fixture's road was the default flat one.
    //
    // It is applied to the built model rather than through `VehicleInput`
    // because the registration stage that reads a road runs once per run, while
    // this one can change from case to case.
    if (run.has_road) {
      built.road_profile.kind = run.road_kind;
      built.road_profile.origin_x = run.road_origin_x;
      built.road_profile.origin_z = run.road_origin_z;
      built.road_profile.amplitude = run.road_amplitude;
      built.road_profile.wavelength = run.road_wavelength;
      built.road_profile.phase = run.road_phase;
      built.road_profile.bump_start = run.road_bump_start;
      built.road_profile.bump_length = run.road_bump_length;
      for (std::size_t index = 0; index < 4; ++index) {
        built.road_profile.corner_scale[index] = run.road_corner_scale[index];
      }
    }
    input.axle.sample_times = run.sample_times.data();
    // Same address, new contents: the model's driven signals point here.
    std::memcpy(driven_target.data(), run.driven_target.data(),
                run.driven_target.size() * sizeof(double));
    std::memcpy(driven_target_rate.data(), run.driven_target_rate.data(),
                run.driven_target_rate.size() * sizeof(double));
    if (actuator_count != 0) {
      std::memcpy(steering_target.data(), run.steering_target.data(),
                  run.steering_target.size() * sizeof(double));
      std::memcpy(steering_rate.data(), run.steering_rate.data(),
                  run.steering_rate.size() * sizeof(double));
    }
    if (!run.brake_torque.empty()) {
      std::memcpy(brake_torque.data(), run.brake_torque.data(),
                  run.brake_torque.size() * sizeof(double));
    }
    // The normalized demands, same address new contents (subtask p2-10).  A
    // case that declares none leaves its buffer at whatever the previous case
    // wrote only if the model pointer is live for the other table, so each
    // declared table is copied in full and each undeclared one is explicitly
    // cleared: a demand must never leak from one case into the next.
    if (!run.wheel_demand.empty()) {
      std::memcpy(wheel_demand.data(), run.wheel_demand.data(),
                  run.wheel_demand.size() * sizeof(double));
    } else if (declares_wheel_demand) {
      std::fill(wheel_demand.begin(), wheel_demand.end(), 0.0);
    }
    if (!run.brake_demand.empty()) {
      std::memcpy(brake_demand.data(), run.brake_demand.data(),
                  run.brake_demand.size() * sizeof(double));
    } else if (declares_brake_demand) {
      std::fill(brake_demand.begin(), brake_demand.end(), 0.0);
    }
    // A driven coordinate that a family does not drive is held at its design
    // separation, which is the state the model was assembled in.  Leaving the
    // buffer at zero would pin it at the world origin instead, and the run would
    // look like a plausible answer to a different question.
    if (run.driven_target.empty() && model.driven_count() != 0) {
      for (std::size_t index = 0; index < model.driven_count(); ++index) {
        const double separation = model.driven_separation(index);
        for (std::size_t sample = 0; sample < run.sample_count; ++sample) {
          driven_target[sample * model.driven_count() + index] = separation;
        }
      }
      std::fill(driven_target_rate.begin(), driven_target_rate.end(), 0.0);
    }
    contract_apply_solver(plan, input.axle);
    if (constraint_output_enabled || jacobian_output_enabled) {
      constraint_output_sink() = {
          constraint_output_enabled ? constraint_multipliers.data()+sample_offset*built.rows : nullptr,
          constraint_output_enabled ? constraint_reactions.data()+sample_offset*wrench_rows*2*kConstraintReactionWidth : nullptr,
          static_cast<std::size_t>(built.rows), wrench_rows,
          jacobian_output_enabled ? constraint_jacobians.data()+sample_offset*built.rows*built.ndof : nullptr,
          static_cast<std::size_t>(built.ndof),
          constraint_output_enabled ? coupler_reactions.data()+sample_offset*built.coordinate_couplers.size()*4*kConstraintReactionWidth : nullptr,
          built.coordinate_couplers.size()};
    }

    // This case's rows start at its slice of the block; the observer adds the
    // sample offset, exactly like `body_state`.
    if (element_wrench_enabled) {
      ElementWrenchSink* const sink = element_wrench_sink();
      if (sink != nullptr) {
        sink->configure(
            element_wrench_block.data() +
                sample_offset * element_wrench_records * kElementWrenchOutputWidth,
            element_wrench_records);
      }
    }
    // This case's rows of the controller ledger, bound the same way and with
    // the same lifetime: the sink is handed the slice the observer writes into.
    if (controller_enabled) {
      ControllerSink* const sink = controller_sink();
      if (sink != nullptr) {
        sink->configure(
            controller_block.data() + sample_offset * kControllerOutputWidth,
            run.sample_count);
      }
    }
    AxleOutput axle_output{};
    axle_output.struct_size = sizeof(AxleOutput);
    axle_output.abi_version = static_cast<std::uint32_t>(kAxleKernelAbiVersion);
    axle_output.body_state = states.data() + sample_offset * bodies * kStatePerBody;
    axle_output.body_state_capacity = run.sample_count * bodies * kStatePerBody;
    axle_output.constraint_wrench =
        wrenches.data() + sample_offset * wrench_rows * kConstraintOutputWidth;
    axle_output.constraint_wrench_capacity =
        run.sample_count * wrench_rows * kConstraintOutputWidth;
    axle_output.diagnostics =
        diagnostics.data() + sample_offset * kDiagnosticsWidth + 2 * case_spans.size() * kDiagnosticsWidth;
    axle_output.diagnostics_capacity = (run.sample_count + 2) * kDiagnosticsWidth;
    axle_output.energy_output = block_slot(energy_block, kEnergyOutputWidth, sample_offset);
    axle_output.energy_output_capacity = run.sample_count * kEnergyOutputWidth;
    axle_output.spring_output =
        block_slot(spring_block, spring_count * kSpringOutputWidth, sample_offset);
    axle_output.spring_output_capacity =
        run.sample_count * spring_count * kSpringOutputWidth;
    axle_output.damper_output =
        block_slot(damper_block, damper_count * kDamperOutputWidth, sample_offset);
    axle_output.damper_output_capacity =
        run.sample_count * damper_count * kDamperOutputWidth;
    axle_output.bump_stop_output =
        block_slot(bump_stop_block, bump_stop_count * kBumpStopOutputWidth,
                   sample_offset);
    axle_output.bump_stop_output_capacity =
        run.sample_count * bump_stop_count * kBumpStopOutputWidth;
    axle_output.bushing_output =
        block_slot(bushing_block, bushing_count * kBushingOutputWidth, sample_offset);
    axle_output.bushing_output_capacity =
        run.sample_count * bushing_count * kBushingOutputWidth;
    axle_output.anti_roll_output =
        block_slot(anti_roll_block, anti_roll_count * kAntiRollOutputWidth, sample_offset);
    axle_output.anti_roll_output_capacity =
        run.sample_count * anti_roll_count * kAntiRollOutputWidth;
    axle_output.tire_output =
        block_slot(tire_block, tire_count * kTireOutputWidth, sample_offset);
    axle_output.tire_output_capacity = run.sample_count * tire_count * kTireOutputWidth;
    // A contact event is three doubles, and the kernel reports how many it
    // found rather than truncating when the buffer is short.  Starting empty
    // and growing on that report is the protocol the ctypes caller already
    // uses, and it costs nothing for a case that has no events.
    std::vector<double> events;
    std::size_t event_capacity = 0;
    std::size_t contact_count = 0;
    int status = 0;
    const std::size_t accepted_start = accepted_identity.size()/4;
    const std::size_t output_map_start = accepted_output_map.size();
    const RunSampleObserver accepted_observer = [&](const RunSample& sample) {
      if (sample.kind == RunSampleKind::output) {
        // This is an explicit observation of the same committed state, rather
        // than a floating-point time match between independent grids.
        if (accepted_identity.size()/4 > accepted_start) {
          if (accepted_identity.back() < 0.0) accepted_identity.back() =
              static_cast<double>(sample.output_sample_id);
        }
        accepted_output_map.insert(accepted_output_map.end(), {
            static_cast<double>(case_spans.size()),
            static_cast<double>(sample.output_sample_id),
            static_cast<double>(sample.accepted_step_id), sample.time});
        return;
      }
      if (sample.kind == RunSampleKind::initial) accepted_output_map.insert(
          accepted_output_map.end(), {static_cast<double>(case_spans.size()), 0.0, 0.0, sample.time});
      accepted_identity.insert(accepted_identity.end(), {
          static_cast<double>(case_spans.size()),
          static_cast<double>(sample.accepted_step_id), sample.time,
          sample.kind == RunSampleKind::initial ? 0.0 : -1.0});
      const std::size_t offset = accepted_states.size();
      accepted_states.resize(offset+bodies*kStatePerBody);
      copy_state(sample.state, sample.model, accepted_states.data()+offset, 0);
      accepted_multipliers.insert(accepted_multipliers.end(),
          sample.constraint_multiplier.begin(), sample.constraint_multiplier.end());
      // --- the load-driven frozen local response at this committed state -------
      //
      // The response is evaluated *here*, on the sample the integrator actually
      // committed, and never on a neighbouring state or on an interpolation the
      // run would have had to invent.  Three properties are deliberate:
      //
      //   * the increment vector carries the sample's own multipliers as the
      //     lambda base value, because `assemble_physical_tangent` reads
      //     `d(J^T lambda)` out of it and the frozen protocol forbids freezing
      //     `delta lambda` (`frozen-protocols.json` `lambda_rule`);
      //   * the working sample brings its own fixed inertia wrench, so the force
      //     bus's live gyroscopic derivative is removed and only the original
      //     snapshot remains -- the `frozen_inertial_load_v1` statement;
      //   * the active tire set is empty on purpose: the frozen sample carries
      //     its own contact state and therefore adds no compression unknowns
      //     (`physical_tangent_dimension`).
      //
      // A sample that cannot be answered is *reported*, not thrown: the design
      // requires one refused sample to leave its neighbours' answers intact, and
      // an exception here would abort the whole run and lose them.
      if (answers_local_response) {
        const std::size_t row = sample_offset + sample.accepted_step_id;
        if (row >= local_records) {
          throw std::runtime_error("local response row outside the request's samples");
        }
        WorkingSample local_working;
        const int local_dimension =
            physical_tangent_dimension(sample.model, local_working);
        std::vector<double> local_tangent;
        std::vector<double> local_x(static_cast<std::size_t>(local_dimension), 0.0);
        bool prepared = frozen_working_sample(sample.model, sample.state, {},
                                              local_working);
        // `local_dimension` was read through `local_working` before it was
        // filled, which is exactly what `physical_tangent_dimension` documents
        // it may be read for; the value does not depend on the contents.
        if (prepared &&
            sample.constraint_multiplier.size() !=
                static_cast<std::size_t>(sample.model.rows)) {
          prepared = false;
        }
        if (prepared) {
          for (std::size_t index = 0; index < sample.constraint_multiplier.size();
               ++index) {
            local_x[static_cast<std::size_t>(sample.model.ndof) + index] =
                sample.constraint_multiplier[index];
          }
          SampleInput local_input;
          interpolate_input(sample.model, input.axle, sample.time, local_input);
          prepared = assemble_physical_tangent(sample.model, local_input,
                                               local_working, local_x,
                                               local_tangent);
        }
        std::vector<double> local_response;
        std::vector<unsigned char> local_mode_ok;
        int local_factorizations = 0;
        FrozenResponseAudit local_sample_audit;
        if (prepared) {
          prepared = frozen_local_response(sample.model, local_working,
                                          local_tangent, local_modes,
                                          local_measurements, local_response,
                                          local_mode_ok, local_factorizations,
                                          local_sample_audit);
        }
        if (!prepared) {
          // Every value stays NaN and the reason is a number a caller can read:
          // a zero-filled refusal is numerically indistinguishable from a
          // mechanism whose response genuinely is zero.
          ++local_refused_samples;
          for (std::size_t cell = 0;
               cell < local_modes.size()*local_measurements.size(); ++cell) {
            local_status[row*local_modes.size()*local_measurements.size() + cell] =
                static_cast<double>(kLocalResponseRefused);
          }
        } else {
          const std::size_t cells = local_modes.size()*local_measurements.size();
          local_modes_per_sample[row] = static_cast<double>(local_modes.size());
          local_audit[row*3] = static_cast<double>(local_sample_audit.dimension);
          local_audit[row*3+1] = static_cast<double>(local_sample_audit.matrix_rank);
          local_audit[row*3+2] = static_cast<double>(local_factorizations);
          for (std::size_t mode = 0; mode < local_modes.size(); ++mode) {
            const bool mode_answered =
                local_mode_ok[mode] != 0;
            for (std::size_t measurement = 0;
                 measurement < local_measurements.size(); ++measurement) {
              const std::size_t cell = mode*local_measurements.size() + measurement;
              // The value block keeps NaN for anything that was refused; the
              // status block carries *why* so the two never have to be inferred
              // from each other.  `combined_mode_rule` lives here as well: an
              // incompatible mode leaves its own cells unanswered while a
              // compatible neighbour of the same sample still reports.
              if (!mode_answered) {
                local_status[row*cells + cell] =
                    static_cast<double>(kLocalResponseSingular);
                continue;
              }
              if (local_sample_audit.measurement_identifiable[measurement] == 0) {
                local_status[row*cells + cell] =
                    static_cast<double>(kLocalResponseUnidentifiable);
                continue;
              }
              const double value = local_response[cell];
              if (!std::isfinite(value)) {
                local_status[row*cells + cell] =
                    static_cast<double>(kLocalResponseRefused);
                continue;
              }
              local_values[row*cells + cell] = value;
              local_status[row*cells + cell] =
                  static_cast<double>(kLocalResponseValid);
            }
          }
        }
        local_identity[row*4] = static_cast<double>(case_spans.size());
        local_identity[row*4+1] = static_cast<double>(sample.accepted_step_id);
        local_identity[row*4+2] = sample.time;
        local_identity[row*4+3] =
            sample.kind == RunSampleKind::initial ? 0.0 : -1.0;
      }
      if (internal_counts.empty()) {
        for (const auto& field : internal_fields) {
          const std::size_t count = (sample.state.*field.second).size();
          internal_counts.push_back(count); internal_width += count;
        }
      }
      for (std::size_t index = 0; index < internal_fields.size(); ++index) {
        const auto& values = sample.state.*internal_fields[index].second;
        if (values.size() != internal_counts[index]) {
          throw std::runtime_error("accepted internal state layout changed within a run");
        }
        accepted_internal.insert(accepted_internal.end(), values.begin(), values.end());
      }
      accepted_internal.push_back(sample.state.step_size);
    };
    for (;;) {
      // A contact-event buffer retry replays the case. Retain only its final
      // execution, including partial accepted states if that execution fails.
      accepted_identity.resize(accepted_start*4);
      accepted_output_map.resize(output_map_start);
      accepted_states.resize(accepted_start*bodies*kStatePerBody);
      accepted_multipliers.resize(accepted_start*static_cast<std::size_t>(built.rows));
      accepted_internal.resize(accepted_start*internal_width);
      events.assign(event_capacity * 3, 0.0);
      axle_output.contact_event_output = event_capacity == 0 ? nullptr : events.data();
      axle_output.contact_event_output_capacity = events.size();
      contact_count = 0;
      axle_output.contact_event_count = &contact_count;
      status = run_model(&input.axle, &axle_output, error_buffer, error_capacity,
                         &built, accepted_state_enabled ? &accepted_observer : nullptr);
      if (status == 10) {
        event_capacity = contact_count == 0 ? event_capacity + 1 : contact_count;
        continue;
      }
      break;
    }
    if (status != 0) {
      // `run_model` writes what actually went wrong -- residual magnitudes,
      // iterations, which coordinate -- and replacing that with a status number
      // throws away the only useful part of the report.
      const std::size_t base = sample_offset + 2 * case_spans.size();
      bool found = false;
      for (std::size_t sample = 0; sample < run.sample_count; ++sample) {
        const double accepted = diagnostics[(base + sample) * kDiagnosticsWidth];
        if (accepted == 0.0) {
          failure.sample_index = sample;
          failure.time_s = sample < run.sample_times.size()
                               ? run.sample_times[sample] : 0.0;
          found = true;
          break;
        }
      }
      if (!found) {
        failure.sample_index = 0;
        failure.time_s = run.sample_times.empty() ? 0.0 : run.sample_times.front();
      }
      failure.failed = true;
      failure.status = status;
      failure.case_name = run.name;
      failure.message = "case " + run.name + " failed with status " +
                        std::to_string(status) +
                        (error_buffer == nullptr ? std::string()
                                                 : ": " + std::string(error_buffer));
      // The failing case is part of the manifest even though it did not finish:
      // its samples are the ones a caller can still read.
      case_spans.push_back(CaseSpan{run.name, sample_offset, run.sample_count,
                                    contact_count});
      break;
    }
    // The steering measurement is the same writer the vehicle entry point calls
    // after its own run: it reads the per-sample states back out of the axle
    // output, so it has to be given the case's slice of them.  Publishing it here
    // is what lets a vehicle caller stop reaching for the flat ABI to read a rack
    // travel.
    if (actuator_count != 0) {
      VehicleOutput steering_output{};
      steering_output.steering_output =
          block_slot(steering_block, actuator_count * kSteeringOutputWidth, sample_offset);
      steering_output.steering_output_capacity =
          run.sample_count * actuator_count * kSteeringOutputWidth;
      write_vehicle_steering_output(built, input.axle, axle_output, steering_output);
    }
    for (std::size_t index = 0; index < contact_count * 3; ++index) {
      contact_events.push_back(events[index]);
    }
    case_spans.push_back(
        CaseSpan{run.name, sample_offset, run.sample_count, contact_count});
    sample_offset += run.sample_count;
  }

  // --- result document ----------------------------------------------------
  // The identity of a run is the identity of the two documents that produced
  // it, so the hashes are taken over the canonical bytes the caller sent rather
  // than over the parsed tree, which would re-derive them.
  std::string model_canonical;
  std::string case_canonical;
  if (!contract_write_canonical(model_payload_parsed.document, model_canonical, error) ||
      !contract_write_canonical(case_payload_parsed.document, case_canonical, error)) {
    return fail(error_buffer, error_capacity, 4, "result identity: " + error);
  }
  const std::string model_sha256 = contract_sha256_hex(model_canonical);
  const std::string case_sha256 = contract_sha256_hex(case_canonical);

  std::string canonical;
  {
    std::vector<Json> entries;
    for (const auto& span : case_spans) {
      entries.push_back(json_object({
          {"name", json_string(span.name)},
          {"contact_events", json_integer(static_cast<long long>(span.contact_events))},
          {"sample_count", json_integer(static_cast<long long>(span.sample_count))},
          {"sample_offset", json_integer(static_cast<long long>(span.offset))},
      }));
    }
    // The body order is part of the result: a caller reads a block by index,
    // and an index is only meaningful next to the names it was assigned to.
    std::vector<Json> body_names;
    for (const std::string& body : model.body_order()) {
      body_names.push_back(json_string(body));
    }
    std::vector<std::string> constraint_ids;
    const Json* joints = model_payload_parsed.document.find("joints");
    if (joints != nullptr) {
      for (const Json& joint : joints->items) {
        const std::string& type = *joint.find_string("type");
        if (type != "driven_rotation" && type != "driven_translation") {
          constraint_ids.push_back(*joint.find_string("name"));
        }
      }
    }
    const Json* elements = model_payload_parsed.document.find("elements");
    if (elements != nullptr && model.needs_vehicle_stages()) {
      for (const Json& element : elements->items) {
        const std::string* type = element.find_string("type");
        if (type == nullptr || *type != "steering_actuator") continue;
        const Json* parameters = element.find("parameters");
        const std::string* mode = parameters == nullptr ? nullptr : parameters->find_string("type");
        if (mode != nullptr && (*mode == "prescribed_rotation" || *mode == "prescribed_translation")) {
          constraint_ids.push_back(*element.find_string("name")+":motion");
        }
      }
    }
    if (joints != nullptr) {
      for (const Json& joint : joints->items) {
        const std::string& type = *joint.find_string("type");
        if (type == "driven_rotation" || type == "driven_translation") {
          constraint_ids.push_back(*joint.find_string("name"));
        }
      }
    }
    std::vector<Json> constraint_channels;
    if (constraint_output_enabled || jacobian_output_enabled) {
      if (constraint_ids.size() != built.constraints.size()) {
        return fail(error_buffer, error_capacity, 4, "constraint output identity disagrees with model rows");
      }
      for (std::size_t index = 0; index < built.constraints.size(); ++index) {
        const Constraint& constraint = built.constraints[index];
        std::vector<Json> units;
        const auto& info = joint_type_info(constraint.type);
        for (int group = 0; group < info.class_count; ++group) {
          for (int row = 0; row < info.classes[group].count; ++row) {
            units.push_back(json_string(info.classes[group].angular ? "Nm" : "N"));
          }
        }
        constraint_channels.push_back(json_object({
            {"id", json_string(constraint_ids[index])},
            {"body_a", json_string(model.body_order()[constraint.a])},
            {"body_b", json_string(model.body_order()[constraint.b])},
            {"row_offset", json_integer(constraint.row)},
            {"row_count", json_integer(constraint_rows(constraint.type))},
            {"multiplier_units", json_array(std::move(units))}}));
      }
      const Json* couplers = model_payload_parsed.document.find("couplers");
      if (built.coordinate_couplers.size() != (couplers == nullptr ? 0 : couplers->items.size())) {
        return fail(error_buffer, error_capacity, 4, "coupler output identity disagrees with model rows");
      }
      for (std::size_t index = 0; index < built.coordinate_couplers.size(); ++index) {
        const auto& coupler = built.coordinate_couplers[index];
        const Constraint& first = built.constraints[static_cast<std::size_t>(coupler.joint_a)];
        const Constraint& second = built.constraints[static_cast<std::size_t>(coupler.joint_b)];
        const char* unit = coupler.coordinate_a == coupler.coordinate_b
            ? (coupler.coordinate_a == 0 ? "Nm" : "N") : "generalized";
        constraint_channels.push_back(json_object({
            {"id", json_string(*couplers->items[index].find_string("name"))},
            {"row_offset", json_integer(coupler.row)}, {"row_count", json_integer(1)},
            {"reaction_block", json_string("coupler_reaction")},
            {"reaction_index", json_integer(static_cast<long long>(index))},
            {"ends", json_array({json_string("a_a"), json_string("a_b"), json_string("b_a"), json_string("b_b")})},
            {"bodies", json_array({json_string(model.body_order()[first.a]), json_string(model.body_order()[first.b]),
                json_string(model.body_order()[second.a]), json_string(model.body_order()[second.b])})},
            {"multiplier_units", json_array({json_string(unit)})}}));
      }
    }
    std::vector<Json> blocks;
    std::size_t offset = 0;
    blocks.push_back(block_descriptor("body_state", offset, state_length * sizeof(double),
                                      {total_samples, bodies, kStatePerBody}));
    offset += state_length * sizeof(double);
    blocks.push_back(block_descriptor("constraint_wrench", offset,
                                      wrench_length * sizeof(double),
                                      {total_samples, wrench_rows, kConstraintOutputWidth}));
    offset += wrench_length * sizeof(double);
    blocks.push_back(block_descriptor("diagnostics", offset, diagnostics_length * sizeof(double),
                                      {diagnostics_rows, kDiagnosticsWidth}));
    offset += diagnostics_length * sizeof(double);
    // A block whose shape has a zero extent is not expressible: the descriptor
    // schema requires every extent to be at least one.  An empty event list is
    // therefore an absent block, which is also what the result model means by
    // an empty tuple.
    if (!contact_events.empty()) {
      blocks.push_back(block_descriptor("contact_events", offset,
                                        contact_events.size() * sizeof(double),
                                        {contact_events.size() / 3, 3}));
      offset += contact_events.size() * sizeof(double);
    }
    // The ledgers, in the order they are appended to the blob below.  A model
    // with no element of a kind omits that block; the shape carries the counts,
    // so a caller reads the element order it was given and never has to guess.
    const auto push_block = [&](const char* name, const std::vector<double>& block,
                                std::vector<std::size_t> shape) {
      if (block.empty()) return;
      blocks.push_back(block_descriptor(name, offset, block.size() * sizeof(double),
                                        std::move(shape)));
      offset += block.size() * sizeof(double);
    };
    push_block("energy", energy_block, {total_samples, kEnergyOutputWidth});
    push_block("steering_output", steering_block,
               {total_samples, actuator_count, kSteeringOutputWidth});
    push_block("tire_output", tire_block, {total_samples, tire_count, kTireOutputWidth});
    push_block("spring_output", spring_block, {total_samples, spring_count, kSpringOutputWidth});
    push_block("damper_output", damper_block,
               {total_samples, damper_count, kDamperOutputWidth});
    push_block("bump_stop_output", bump_stop_block,
               {total_samples, bump_stop_count, kBumpStopOutputWidth});
    push_block("bushing_output", bushing_block,
               {total_samples, bushing_count, kBushingOutputWidth});
    push_block("anti_roll_output", anti_roll_block,
               {total_samples, anti_roll_count, kAntiRollOutputWidth});
    push_block("element_wrench", element_wrench_block,
               {total_samples, element_wrench_records, kElementWrenchOutputWidth});
    // The closed-loop ledger goes last, in the same order the blob appends it:
    // a descriptor's offset is where the reader looks, and the two lists are
    // written from one pass so they cannot disagree.
    push_block("controller_output", controller_block,
               {total_samples, kControllerOutputWidth});
    push_block("constraint_multiplier", constraint_multipliers,
               {total_samples, static_cast<std::size_t>(built.rows)});
    push_block("constraint_reaction", constraint_reactions,
               {total_samples, wrench_rows, 2, kConstraintReactionWidth});
    push_block("constraint_jacobian", constraint_jacobians,
               {total_samples, static_cast<std::size_t>(built.rows), static_cast<std::size_t>(built.ndof)});
    push_block("coupler_reaction", coupler_reactions,
               {total_samples, built.coordinate_couplers.size(), 4, kConstraintReactionWidth});
    const std::size_t accepted_count = accepted_identity.size()/4;
    push_block("accepted_sample_identity", accepted_identity, {accepted_count, 4});
    push_block("accepted_output_map", accepted_output_map, {accepted_output_map.size()/4, 4});
    push_block("accepted_body_state", accepted_states, {accepted_count, bodies, kStatePerBody});
    push_block("accepted_constraint_multiplier", accepted_multipliers,
               {accepted_count, static_cast<std::size_t>(built.rows)});
    push_block("accepted_internal_state", accepted_internal, {accepted_count, internal_width});
    // The load-driven frozen response, when one was requested and answered.  The
    // three numeric blocks are pushed together with the strings that name their
    // axes: a value block whose rows and columns are not named is a matrix a
    // caller has to guess the meaning of, and the manifest below is the only
    // channel that can carry those names (`block_descriptor` is numeric-only).
    push_block("local_response_value", local_values,
               {local_records, local_modes.size(), local_measurements.size()});
    push_block("local_response_status", local_status,
               {local_records, local_modes.size(), local_measurements.size()});
    push_block("local_response_modes_per_sample", local_modes_per_sample,
               {local_records});
    push_block("local_response_audit", local_audit, {local_records, 3});
    push_block("local_response_identity", local_identity, {local_records, 4});
    std::vector<Json> accepted_internal_layout;
    std::size_t internal_offset = 0;
    for (std::size_t index = 0; index < internal_counts.size(); ++index) {
      accepted_internal_layout.push_back(json_object({
          {"name", json_string(internal_fields[index].first)},
          {"offset", json_integer(static_cast<long long>(internal_offset))},
          {"count", json_integer(static_cast<long long>(internal_counts[index]))}}));
      internal_offset += internal_counts[index];
    }
    if (accepted_state_enabled) accepted_internal_layout.push_back(json_object({
        {"name", json_string("step_size")},
        {"offset", json_integer(static_cast<long long>(internal_offset))},
        {"count", json_integer(1)}}));
    std::vector<Json> free_bodies;
    for (int body : built.free_body) free_bodies.push_back(json_string(model.body_order()[body]));

    Json document = json_object({
        {"blocks", json_array(std::move(blocks))},
        {"case_identity",
         json_object({
             {"case_sha256", json_string(case_sha256)},
             {"contract_version", json_integer(document_contract_version(element_wrench_enabled || constraint_output_enabled))},
             {"model_sha256", json_string(model_sha256)},
         })},
        {"contract", json_string("multibody-result")},
        {"contract_version", json_integer(document_contract_version(element_wrench_enabled || constraint_output_enabled))},
        {"kind", json_string("result")},
        {"manifest",
         json_object({
             {"abi_version", json_integer(kVehicleKernelAbiVersion)},
             {"case_count", json_integer(static_cast<long long>(cases))},
             {"bodies", json_array(std::move(body_names))},
             {"case_name", json_string(plan.name)},
             {"cases", json_array(std::move(entries))},
             {"family", json_string(plan.family)},
             {"model_name", json_string(model.name())},
             {"sample_count", json_integer(static_cast<long long>(total_samples))},
             {"failed_case", json_string(failure.case_name)},
             {"failed_sample_index",
              json_integer(static_cast<long long>(failure.sample_index))},
             {"failed_status", json_integer(static_cast<long long>(failure.status))},
             {"failed_time_s", json_number(failure.time_s)},
             {"failure_message", json_string(failure.message)},
             {"constraint_channel_version", json_integer(1)},
             {"constraints", json_array(std::move(constraint_channels))},
             {"free_bodies", json_array(std::move(free_bodies))},
             {"constraint_wrench_frame", json_string("world")},
             {"constraint_moment_reference", json_string("body_com")},
             {"multiplier_convention", json_string("generalized_wrench = J^T lambda")},
         })},
        {"status", json_string(failure.failed ? "failed" : "success")},
    });
    if (accepted_state_enabled) {
      for (auto& field : document.fields) if (field.first == "manifest") {
        field.second.fields.push_back({"accepted_state_version", json_integer(1)});
        field.second.fields.push_back({"accepted_constraint_rows", json_integer(built.rows)});
        field.second.fields.push_back({"accepted_internal_layout", json_array(std::move(accepted_internal_layout))});
        field.second.fields.push_back({"accepted_identity_columns", json_array({
            json_string("case_index"), json_string("accepted_step_id"),
            json_string("time_s"), json_string("first_output_sample_id")})});
        field.second.fields.push_back({"accepted_output_map_columns", json_array({
            json_string("case_index"), json_string("output_sample_id"),
            json_string("accepted_step_id"), json_string("time_s")})});
      }
    }
    // A decoded characteristics request is echoed back so "the request arrived"
    // is observable rather than assumed.  The kernel still does not compute a
    // characteristics response -- the capability document says so with an empty
    // `characteristics_response_kinds` -- but it must not silently drop what it
    // read.  Only the fields the reader decoded are echoed; the reader refuses
    // unknown fields, so there is nothing left over to lose.
    if (plan.characteristics.present) {
      const ContractCharacteristics& request = plan.characteristics;
      std::vector<Json> measurements;
      for (const ContractDeclaration& declared : request.measurements) {
        measurements.push_back(json_object({
            {"id", json_string(declared.id)},
            {"measurement_id", json_string(declared.measurement_id)},
            {"definition_version", json_integer(declared.definition_version)},
            {"units", json_string(declared.units)},
            {"response_kind", json_string(declared.response_kind)},
            {"response_version", json_integer(declared.response_version)},
            {"mode", json_string(declared.mode)}}));
      }
      std::vector<Json> samples_json;
      for (int index : request.samples) {
        samples_json.push_back(json_integer(static_cast<long long>(index)));
      }
      std::vector<Json> released_json;
      for (const std::string& name : request.boundary_released) {
        released_json.push_back(json_string(name));
      }
      std::vector<Json> held_json;
      for (const std::string& name : request.boundary_held) {
        held_json.push_back(json_string(name));
      }
      std::vector<std::pair<std::string, Json>> characteristics_fields = {
          {"present", Json{JsonKind::Bool, true, 0.0, false, 0, std::string(), {}, {}}},
          {"request_id", json_string(request.request_id)},
          {"domain", json_string(request.domain)},
          {"samples", json_array(std::move(samples_json))},
          {"measurements", json_array(std::move(measurements))},
          {"boundary_profile", json_string(request.boundary_profile)},
          {"boundary_version", json_integer(request.boundary_version)},
          {"boundary_released", json_array(std::move(released_json))},
          {"boundary_held", json_array(std::move(held_json))},
          {"definition_kind", json_string(request.definition_kind)},
          {"definition_version", json_integer(request.definition_version)},
          {"frozen_protocol", json_string(request.frozen_protocol)},
          {"frozen_version", json_integer(request.frozen_version)},
          {"operator_kind", json_string(request.operator_kind)},
          {"operator_version", json_integer(request.operator_version)}};
      if (request.has_wheel_end_response) {
        characteristics_fields.push_back(std::make_pair(
            std::string("wheel_end_response"), request.wheel_end_response));
      }
      for (auto& field : document.fields) if (field.first == "manifest") {
        field.second.fields.push_back(std::make_pair(
            std::string("characteristics"),
            json_object(std::move(characteristics_fields))));
      }
    }
    if (answers_local_response) {
      // The names of the two axes and the reason vocabulary.  A caller reading
      // `local_response_value` by index would otherwise have to reconstruct the
      // mode order from the request, and the status column would be four
      // unexplained integers.
      std::vector<Json> mode_ids;
      for (const FrozenLoadMode& mode : local_modes) {
        mode_ids.push_back(json_string(mode.id));
      }
      std::vector<Json> measurement_ids, measurement_units, measurement_components;
      for (const FrozenMeasurement& measurement : local_measurements) {
        measurement_ids.push_back(json_string(measurement.id));
        measurement_units.push_back(json_string(measurement.units));
        static const std::vector<std::string> components = {"x",  "y",  "z",
                                                           "rx", "ry", "rz"};
        measurement_components.push_back(json_string(
            components[static_cast<std::size_t>(measurement.component)]));
      }
      for (auto& field : document.fields) if (field.first == "manifest") {
        field.second.fields.push_back({"local_response",
          json_object({
              {"status_vocabulary", json_object({
                  {"0", json_string("valid")},
                  {"1", json_string("unidentifiable")},
                  {"2", json_string("singular")},
                  {"3", json_string("refused")}})},
              {"value_block", json_string("local_response_value")},
              {"status_block", json_string("local_response_status")},
              {"modes_per_sample_block", json_string("local_response_modes_per_sample")},
              {"audit_block", json_string("local_response_audit")},
              {"identity_block", json_string("local_response_identity")},
              {"audit_columns", json_array({json_string("dimension"),
                                            json_string("matrix_rank"),
                                            json_string("factorizations_used")})},
              {"identity_columns", json_array({json_string("case_index"),
                                               json_string("accepted_step_id"),
                                               json_string("time_s"),
                                               json_string("first_output_sample_id")})},
              {"load_modes", json_array(std::move(mode_ids))},
              {"measurements", json_array(std::move(measurement_ids))},
              {"measurement_units", json_array(std::move(measurement_units))},
              {"measurement_components", json_array(std::move(measurement_components))},
              {"refused_samples", json_integer(static_cast<long long>(local_refused_samples))},
              {"frozen_protocol", json_string(plan.characteristics.frozen_protocol)},
              {"frozen_version", json_integer(plan.characteristics.frozen_version)}})});
      }
    }
    if (!contract_write_canonical(document, canonical, error)) {
      return fail(error_buffer, error_capacity, 4, "result document: " + error);
    }
  }

  std::string blob;
  blob.reserve((state_length + wrench_length + diagnostics_length) * sizeof(double));
  append_doubles(states.data(), state_length, blob);
  append_doubles(wrenches.data(), wrench_length, blob);
  append_doubles(diagnostics.data(), diagnostics_length, blob);
  append_doubles(contact_events.data(), contact_events.size(), blob);
  append_doubles(energy_block.data(), energy_block.size(), blob);
  append_doubles(steering_block.data(), steering_block.size(), blob);
  append_doubles(tire_block.data(), tire_block.size(), blob);
  append_doubles(spring_block.data(), spring_block.size(), blob);
  append_doubles(damper_block.data(), damper_block.size(), blob);
  append_doubles(bump_stop_block.data(), bump_stop_block.size(), blob);
  append_doubles(bushing_block.data(), bushing_block.size(), blob);
  append_doubles(anti_roll_block.data(), anti_roll_block.size(), blob);
  append_doubles(element_wrench_block.data(), element_wrench_block.size(), blob);
  append_doubles(controller_block.data(), controller_block.size(), blob);
  append_doubles(constraint_multipliers.data(), constraint_multipliers.size(), blob);
  append_doubles(constraint_reactions.data(), constraint_reactions.size(), blob);
  append_doubles(constraint_jacobians.data(), constraint_jacobians.size(), blob);
  append_doubles(coupler_reactions.data(), coupler_reactions.size(), blob);
  append_doubles(accepted_identity.data(), accepted_identity.size(), blob);
  append_doubles(accepted_output_map.data(), accepted_output_map.size(), blob);
  append_doubles(accepted_states.data(), accepted_states.size(), blob);
  append_doubles(accepted_multipliers.data(), accepted_multipliers.size(), blob);
  append_doubles(accepted_internal.data(), accepted_internal.size(), blob);
  append_doubles(local_values.data(), local_values.size(), blob);
  append_doubles(local_status.data(), local_status.size(), blob);
  append_doubles(local_modes_per_sample.data(), local_modes_per_sample.size(), blob);
  append_doubles(local_audit.data(), local_audit.size(), blob);
  append_doubles(local_identity.data(), local_identity.size(), blob);

  const std::string payload = contract_build_container(canonical, blob);
  if (result_out == nullptr || *result_length_in_out < payload.size()) {
    *result_length_in_out = payload.size();
    return 11;
  }
  std::memcpy(result_out, payload.data(), payload.size());
  *result_length_in_out = payload.size();
  return 0;
} catch (const std::exception& error) {
  return axle_kernel::fail(error_buffer, error_capacity, 4,
                           "native evaluation: " + std::string(error.what()));
}
