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
#include "mb_cases/functions.hpp"
#include "mb_contract/functions.hpp"
#include "mb_model/enums.hpp"
#include "mb_tire/pac2002/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_output/functions.hpp"
#include "mb_solve_static/functions.hpp"
#include "mb_joint/functions.hpp"
#include "mb_assembly/functions.hpp"

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
  return json_object({
      {"contract", json_string("multibody-capabilities")},
      {"contract_version", json_integer(1)},
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
) {
  using namespace axle_kernel;
  if (model_payload == nullptr || case_payload == nullptr ||
      result_length_in_out == nullptr) {
    return fail(error_buffer, error_capacity, 1,
                "model payload, case payload and result length are required");
  }

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
  if (!error.empty()) {
    return fail(error_buffer, error_capacity, 2, "model build: " + error);
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

  const std::size_t bodies = built.bodies.size();
  const std::size_t wrench_rows = built.constraints.size();
  const std::size_t cases = plan.cases.size();
  std::size_t total_samples = 0;
  for (const ContractCase& run : plan.cases) total_samples += run.sample_count;
  const std::size_t diagnostics_rows = total_samples + 2 * cases;

  const std::size_t state_length = total_samples * bodies * kStatePerBody;
  const std::size_t wrench_length = total_samples * wrench_rows * kConstraintOutputWidth;
  const std::size_t diagnostics_length = diagnostics_rows * kDiagnosticsWidth;

  std::vector<double> states(state_length, std::numeric_limits<double>::quiet_NaN());
  std::vector<double> wrenches(wrench_length, std::numeric_limits<double>::quiet_NaN());
  std::vector<double> diagnostics(diagnostics_length,
                                  std::numeric_limits<double>::quiet_NaN());
  std::vector<double> contact_events;
  // The element, tire and energy ledgers are *result*, not scratch: the dynamic
  // families report tire forces, spring and bushing loads and the energy
  // balance, and a caller that had to reach the flat ABI for them would keep
  // that ABI alive.  Each is one block per sample in element order, sized for
  // the whole run so a multi-case plan can offset into it like `body_state`.
  const std::size_t spring_count = built.springs.size();
  const std::size_t bushing_count = built.bushings.size();
  const std::size_t anti_roll_count = built.anti_roll_bars.size();
  const double kNan = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> energy_block(total_samples * kEnergyOutputWidth, kNan);
  std::vector<double> steering_block(
      total_samples * actuator_count * kSteeringOutputWidth, kNan);
  std::vector<double> spring_block(total_samples * spring_count * kSpringOutputWidth, kNan);
  std::vector<double> bushing_block(total_samples * bushing_count * kBushingOutputWidth, kNan);
  std::vector<double> anti_roll_block(total_samples * anti_roll_count * kAntiRollOutputWidth, kNan);
  std::vector<double> tire_block(total_samples * tire_count * kTireOutputWidth, kNan);
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
    for (;;) {
      events.assign(event_capacity * 3, 0.0);
      axle_output.contact_event_output = event_capacity == 0 ? nullptr : events.data();
      axle_output.contact_event_output_capacity = events.size();
      contact_count = 0;
      axle_output.contact_event_count = &contact_count;
      status = run_model(&input.axle, &axle_output, error_buffer, error_capacity,
                         &built);
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
    push_block("bushing_output", bushing_block,
               {total_samples, bushing_count, kBushingOutputWidth});
    push_block("anti_roll_output", anti_roll_block,
               {total_samples, anti_roll_count, kAntiRollOutputWidth});

    Json document = json_object({
        {"blocks", json_array(std::move(blocks))},
        {"case_identity",
         json_object({
             {"case_sha256", json_string(case_sha256)},
             {"contract_version", json_integer(1)},
             {"model_sha256", json_string(model_sha256)},
         })},
        {"contract", json_string("multibody-result")},
        {"contract_version", json_integer(1)},
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
         })},
        {"status", json_string(failure.failed ? "failed" : "success")},
    });
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
  append_doubles(bushing_block.data(), bushing_block.size(), blob);
  append_doubles(anti_roll_block.data(), anti_roll_block.size(), blob);

  const std::string payload = contract_build_container(canonical, blob);
  if (result_out == nullptr || *result_length_in_out < payload.size()) {
    *result_length_in_out = payload.size();
    return 11;
  }
  std::memcpy(result_out, payload.data(), payload.size());
  *result_length_in_out = payload.size();
  return 0;
}
