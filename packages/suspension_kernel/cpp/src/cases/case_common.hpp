#pragma once

/// Shared internals of the case layer.
///
/// Every family needs the same things before it can expand anything: the
/// document's identity, its output grid and its solver scalars.  They live here
/// rather than being copied into each family, so a change to the time grid is
/// one edit and not one edit per family.
///
/// This is a private header.  It is deliberately outside `cpp/include`: it is
/// not part of the module's public interface, and keeping it here means the
/// module layering gate does not have to treat these includes as new edges.

#include "mb_config/prelude.hpp"
#include "mb_cases/functions.hpp"
#include "mb_contract/types.hpp"
#include "mb_input/types.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace axle_kernel {
namespace case_detail {

using Json = JsonValue;

/// One millimetre in metres.  A field whose name says `_mm` is millimetres
/// whatever length unit the document declares.
inline constexpr double kMillimetreScale = 1e-3;

inline bool fail(std::string& error, const std::string& message) {
  error = message;
  return false;
}

inline double number_or(const Json& object, const char* key, double fallback) {
  const Json* value = object.find(key);
  if (value == nullptr || value->kind != JsonKind::Number) return fallback;
  return value->number;
}

inline int integer_or(const Json& object, const char* key, int fallback) {
  const Json* value = object.find(key);
  if (value == nullptr || value->kind != JsonKind::Number) return fallback;
  return static_cast<int>(value->number_is_integer
                              ? value->integer
                              : static_cast<long long>(value->number));
}

inline bool read_numbers(const Json& value, std::vector<double>& out, const char* what,
                         std::string& error) {
  out.clear();
  if (!value.is_array()) return fail(error, std::string(what) + " must be an array");
  for (const Json& item : value.items) {
    if (item.kind != JsonKind::Number) {
      return fail(error, std::string(what) + " must contain only numbers");
    }
    out.push_back(item.number);
  }
  return true;
}

/// The standard load-path order.  A document may name a subset, but a name that
/// is not one of the six is an error, not a silently ignored column.
inline const char* const* load_axes() {
  static const char* const kAxes[] = {"fx", "fy", "fz", "mx", "my", "mz"};
  return kAxes;
}

inline int load_axis_index(const std::string& name) {
  const char* const* axes = load_axes();
  for (int index = 0; index < 6; ++index) {
    if (name == axes[index]) return index;
  }
  return -1;
}

inline std::string format(const char* pattern, double value) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), pattern, value);
  return std::string(buffer);
}

/// `np.linspace(start, stop, count)`, reproduced bit for bit.
///
/// The recipe matters: numpy computes one step and then multiplies, so
/// `start + index * ((stop - start) / (count - 1))` is the only form that
/// reproduces the array the Python workflow used.  Re-deriving the step per
/// index rounds differently and moves the targets by an ulp.
inline void linspace(double start, double stop, std::size_t count,
                     std::vector<double>& out) {
  out.assign(count, start);
  if (count == 0) return;
  if (count == 1) {
    out[0] = start;
    return;
  }
  const double step = (stop - start) / static_cast<double>(count - 1);
  for (std::size_t index = 0; index < count; ++index) {
    out[index] = start + static_cast<double>(index) * step;
  }
  out[count - 1] = stop;
}

/// The shared time grid of a case document, plus the sample count it implies.
///
/// Declared here and defined below `read_described_array`, because an irregular
/// history names a table in the payload and reading one is that function's job.
inline bool read_time(const Json& document, const std::string& blob,
                      std::vector<double>& times, std::string& error);

/// Read the solver block into `plan`, leaving anything absent at its default.
///
/// The defaults here are the ones the Python solver settings class declares, and
/// they are a fallback rather than the contract: a caller that wants a specific
/// run writes the scalars out, and the document is then the whole truth.
inline void read_solver(const Json& document, ContractPlan& plan) {
  const Json* solver = document.find("solver");
  if (solver == nullptr || !solver->is_object()) return;
  if (const std::string* integrator = solver->find_string("integrator")) {
    plan.solver_integrator = *integrator;
  }
  plan.rho_inf = number_or(*solver, "rho_inf", plan.rho_inf);
  plan.hht_alpha = number_or(*solver, "hht_alpha", plan.hht_alpha);
  if (const std::string* mode = solver->find_string("initialization_mode")) {
    plan.initialization_mode = (*mode == "provided_consistent_state") ? 1 : 0;
  }
  if (const Json* value = solver->find("adaptive_step")) {
    if (value->kind == JsonKind::Bool) plan.adaptive_step = value->boolean ? 1 : 0;
  }
  plan.internal_step = number_or(*solver, "internal_step_s", plan.internal_step);
  plan.min_step = number_or(*solver, "minimum_step_s", plan.min_step);
  plan.max_step = number_or(*solver, "maximum_step_s", plan.max_step);
  plan.local_relative_tolerance =
      number_or(*solver, "local_relative_tolerance", plan.local_relative_tolerance);
  plan.local_position_tolerance =
      number_or(*solver, "local_position_tolerance_m", plan.local_position_tolerance);
  plan.local_angle_tolerance =
      number_or(*solver, "local_angle_tolerance_rad", plan.local_angle_tolerance);
  plan.local_velocity_tolerance = number_or(
      *solver, "local_velocity_tolerance_m_per_s", plan.local_velocity_tolerance);
  plan.local_angular_velocity_tolerance =
      number_or(*solver, "local_angular_velocity_tolerance_rad_per_s",
                plan.local_angular_velocity_tolerance);
  plan.local_brush_tolerance =
      number_or(*solver, "local_brush_tolerance_m", plan.local_brush_tolerance);
  plan.contact_event_tolerance =
      number_or(*solver, "contact_event_tolerance_s", plan.contact_event_tolerance);
  plan.max_newton_iterations =
      integer_or(*solver, "max_newton_iterations", plan.max_newton_iterations);
  plan.max_line_search_iterations = integer_or(
      *solver, "max_line_search_iterations", plan.max_line_search_iterations);
  plan.position_tolerance =
      number_or(*solver, "position_tolerance_m", plan.position_tolerance);
  plan.velocity_tolerance = number_or(
      *solver, "velocity_tolerance_m_per_s", plan.velocity_tolerance);
  plan.dynamics_tolerance =
      number_or(*solver, "dynamics_tolerance", plan.dynamics_tolerance);
  plan.increment_tolerance =
      number_or(*solver, "increment_tolerance", plan.increment_tolerance);
}

/// Copy the plan's solver scalars into the input structure.
inline void apply_solver(const ContractPlan& plan, AxleInput& input) {
  input.rho_inf = plan.rho_inf;
  input.integrator_type = plan.solver_integrator == "hht" ? 1 : 0;
  input.hht_alpha = plan.hht_alpha;
  input.initialization_mode = plan.initialization_mode;
  input.adaptive_step = plan.adaptive_step;
  input.internal_step = plan.internal_step;
  input.min_step = plan.min_step;
  input.max_step = plan.max_step;
  input.local_relative_tolerance = plan.local_relative_tolerance;
  input.local_position_tolerance = plan.local_position_tolerance;
  input.local_angle_tolerance = plan.local_angle_tolerance;
  input.local_velocity_tolerance = plan.local_velocity_tolerance;
  input.local_angular_velocity_tolerance = plan.local_angular_velocity_tolerance;
  input.local_brush_tolerance = plan.local_brush_tolerance;
  input.contact_event_tolerance = plan.contact_event_tolerance;
  input.max_newton_iterations = plan.max_newton_iterations;
  input.max_line_search_iterations = plan.max_line_search_iterations;
  input.position_tolerance = plan.position_tolerance;
  input.velocity_tolerance = plan.velocity_tolerance;
  input.dynamics_tolerance = plan.dynamics_tolerance;
  input.increment_tolerance = plan.increment_tolerance;
}

/// Read the identity fields every family shares into `plan`.
inline bool read_identity(const Json& document, const char* family, ContractPlan& out,
                          std::string& error) {
  const std::string* contract = document.find_string("contract");
  const std::string* kind = document.find_string("kind");
  const std::string* declared = document.find_string("family");
  if (contract == nullptr || *contract != "multibody-case" || kind == nullptr ||
      *kind != "case" || declared == nullptr) {
    return fail(error, "expected a multibody-case document");
  }
  const std::string* name = document.find_string("name");
  if (name == nullptr || name->empty()) {
    return fail(error, "case document has no name");
  }
  if (*declared != family) {
    return fail(error, "the dispatcher offered a " + *declared + " document to the " +
                            family + " family");
  }
  out.family = *declared;
  out.name = *name;
  read_solver(document, out);
  return true;
}

/// Read a described `float64` array out of the model's own payload.
///
/// The descriptor is the same one the contract uses everywhere else: an offset
/// and a length in bytes, a dtype, and a name.  Everything is checked against
/// the payload before it is read.
inline bool read_described_array(const JsonValue& document, const std::string& blob,
                          const std::string& name, std::size_t expected_count,
                          std::vector<double>& out, std::string& error) {
  const Json* descriptors = document.find("blobs");
  if (descriptors == nullptr || !descriptors->is_array()) {
    return fail(error, "the model has no blobs, so " + name + " cannot be read");
  }
  const Json* found = nullptr;
  for (const Json& descriptor : descriptors->items) {
    if (const std::string* entry = descriptor.find_string("name");
        entry != nullptr && *entry == name) {
      found = &descriptor;
    }
  }
  if (found == nullptr) {
    return fail(error, "no descriptor named " + name + " in the model's blobs");
  }
  const std::string* dtype = found->find_string("dtype");
  const Json* offset_value = found->find("offset");
  const Json* length_value = found->find("length");
  if (dtype == nullptr || offset_value == nullptr || length_value == nullptr) {
    return fail(error, "descriptor " + name + " needs dtype, offset and length");
  }
  if (*dtype != "float64") {
    return fail(error, "descriptor " + name + " must be float64, got " + *dtype);
  }
  const long long offset = offset_value->number_is_integer
                               ? offset_value->integer
                               : static_cast<long long>(offset_value->number);
  const long long length = length_value->number_is_integer
                               ? length_value->integer
                               : static_cast<long long>(length_value->number);
  if (length != static_cast<long long>(expected_count * sizeof(double))) {
    return fail(error, "descriptor " + name + " has the wrong length");
  }
  if (offset < 0 ||
      static_cast<std::size_t>(offset) + static_cast<std::size_t>(length) >
          blob.size()) {
    return fail(error, "descriptor " + name + " falls outside the model payload");
  }
  const double* values = reinterpret_cast<const double*>(blob.data() + offset);
  out.assign(values, values + expected_count);
  return true;
}

/// Read an integer field of a blob descriptor.
inline bool blob_int(const Json& descriptor, const char* key, long long& out) {
  const Json* value = descriptor.find(key);
  if (value == nullptr || value->kind != JsonKind::Number) return false;
  out = value->number_is_integer ? value->integer
                                 : static_cast<long long>(value->number);
  return true;
}

/// The element count a described `float64` array declares.
inline bool described_array_count(const JsonValue& document, const std::string& name,
                                  std::size_t& count, std::string& error) {
  const Json* descriptors = document.find("blobs");
  if (descriptors == nullptr || !descriptors->is_array()) {
    return fail(error, "the payload has no blobs, so " + name + " cannot be read");
  }
  const Json* found = nullptr;
  for (const Json& descriptor : descriptors->items) {
    if (const std::string* entry = descriptor.find_string("name");
        entry != nullptr && *entry == name) {
      found = &descriptor;
    }
  }
  if (found == nullptr) {
    return fail(error, "no descriptor named " + name + " in the payload");
  }
  long long length = 0;
  if (!blob_int(*found, "length", length) || length <= 0 ||
      length % static_cast<long long>(sizeof(double)) != 0) {
    return fail(error, "descriptor " + name + " has a malformed length");
  }
  count = static_cast<std::size_t>(length / static_cast<long long>(sizeof(double)));
  return true;
}

/// The shared time grid of a case document, plus the sample count it implies.
///
/// Two spellings are allowed.  A generated case writes a start/end/step triple
/// and the grid is reproduced exactly as `np.linspace` builds it.  A measured or
/// otherwise irregular history names a `float64` table in the case payload
/// instead: those sample instants are data rather than a rule, and re-deriving
/// them from an average step would move every sample by a rounding error.
inline bool read_time(const Json& document, const std::string& blob,
                      std::vector<double>& times, std::string& error) {
  const Json* time = document.find("time");
  if (time == nullptr || !time->is_object()) {
    return fail(error, "case document has no time object");
  }
  if (const std::string* samples = time->find_string("samples");
      samples != nullptr && !samples->empty()) {
    std::size_t count = 0;
    if (!described_array_count(document, *samples, count, error)) return false;
    if (count < 2) return fail(error, "case time needs at least two samples");
    if (!read_described_array(document, blob, *samples, count, times, error)) {
      return false;
    }
    for (std::size_t index = 1; index < times.size(); ++index) {
      if (!(times[index] > times[index - 1])) {
        return fail(error, "the case time samples must be strictly increasing");
      }
    }
    return true;
  }
  const double start = number_or(*time, "start_s", 0.0);
  const Json* end = time->find("end_s");
  const Json* step = time->find("step_s");
  if (end == nullptr || end->kind != JsonKind::Number || step == nullptr ||
      step->kind != JsonKind::Number || step->number <= 0.0) {
    return fail(error, "case time needs a positive step_s and a finite end_s");
  }
  const double span = end->number - start;
  if (!(span > 0.0)) {
    return fail(error, "case time must span a positive interval");
  }
  const double divisions = span / step->number;
  const std::size_t count = static_cast<std::size_t>(std::llround(divisions)) + 1;
  if (count < 2) return fail(error, "case time needs at least two samples");
  linspace(start, end->number, count, times);
  return true;
}

/// Read one `[sample_count]` table and scatter it into a column of a
/// sample-major matrix, scaling as it goes.
///
/// The range is checked against the blob before it is read: a descriptor that
/// points past the end of its payload is an error, not a short read, because a
/// silently truncated history solves to a plausible wrong answer.
inline bool read_column_table(const Json& descriptor, const std::string& blob,
                              std::size_t sample_count, std::size_t column,
                              std::size_t column_count, double scale,
                              std::vector<double>& out, const std::string& who,
                              std::string& error) {
  long long offset = 0;
  long long length = 0;
  const std::string* dtype = descriptor.find_string("dtype");
  const Json* shape = descriptor.find("shape");
  if (!blob_int(descriptor, "offset", offset) ||
      !blob_int(descriptor, "length", length) || dtype == nullptr ||
      shape == nullptr || !shape->is_array()) {
    return fail(error, who + " descriptor needs offset, length, dtype and shape");
  }
  if (*dtype != "float64") return fail(error, who + " must be float64, got " + *dtype);
  if (shape->items.size() != 1 ||
      static_cast<std::size_t>(shape->items[0].number) != sample_count) {
    return fail(error, who + " must have shape [sample_count]");
  }
  if (length != static_cast<long long>(sample_count * sizeof(double))) {
    return fail(error, who + " length does not match its shape");
  }
  if (offset < 0 || length < 0 ||
      static_cast<std::size_t>(offset) + static_cast<std::size_t>(length) > blob.size()) {
    return fail(error, who + " range falls outside the payload blob");
  }
  const double* values = reinterpret_cast<const double*>(blob.data() + offset);
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    out[sample * column_count + column] = values[sample] * scale;
  }
  return true;
}

/// Read one `[sample_count, 6]` table into a body's slice of a wrench matrix.
inline bool read_body_table(const Json& descriptor, const std::string& blob,
                            std::size_t sample_count, std::size_t body,
                            std::size_t body_stride, double moment_scale,
                            std::vector<double>& out, const std::string& who,
                            std::string& error) {
  long long offset = 0;
  long long length = 0;
  const std::string* dtype = descriptor.find_string("dtype");
  const Json* shape = descriptor.find("shape");
  if (!blob_int(descriptor, "offset", offset) ||
      !blob_int(descriptor, "length", length) || dtype == nullptr ||
      shape == nullptr || !shape->is_array()) {
    return fail(error, who + " descriptor needs offset, length, dtype and shape");
  }
  if (*dtype != "float64") return fail(error, who + " must be float64, got " + *dtype);
  if (shape->items.size() != 2 ||
      static_cast<std::size_t>(shape->items[0].number) != sample_count ||
      static_cast<std::size_t>(shape->items[1].number) != 6) {
    return fail(error, who + " must have shape [sample_count, 6]");
  }
  if (length != static_cast<long long>(sample_count * 6 * sizeof(double))) {
    return fail(error, who + " length does not match its shape");
  }
  if (offset < 0 || length < 0 ||
      static_cast<std::size_t>(offset) + static_cast<std::size_t>(length) > blob.size()) {
    return fail(error, who + " range falls outside the payload blob");
  }
  const double* values = reinterpret_cast<const double*>(blob.data() + offset);
  for (std::size_t sample = 0; sample < sample_count; ++sample) {
    for (std::size_t component = 0; component < 6; ++component) {
      // A force is a force; a moment is a force times the document's length
      // unit, so it carries that unit's scale and a force does not.
      const double scale = component < 3 ? 1.0 : moment_scale;
      out[(sample * body_stride + body * 6) + component] =
          values[sample * 6 + component] * scale;
    }
  }
  return true;
}

/// Read the road object a case document may carry, either inline or under
/// `inputs.road`.  Returns false only on a malformed road.
inline bool read_road(const Json& document, ContractCase& run, std::string& error) {
  const Json* road = document.find("road");
  if (road == nullptr) {
    if (const Json* inputs = document.find("inputs");
        inputs != nullptr && inputs->is_object()) {
      road = inputs->find("road");
    }
  }
  if (road == nullptr) return true;
  if (!road->is_object()) return fail(error, "road must be an object");
  run.has_road = true;
  if (const std::string* kind = road->find_string("kind"); kind != nullptr) {
    static const std::pair<const char*, int> kRoads[] = {
        {"plane", 1}, {"sine", 2}, {"bump", 3},
        {"random_fourier", 4}, {"four_post", 5},
    };
    int value = 0;
    for (const auto& entry : kRoads) {
      if (*kind == entry.first) value = entry.second;
    }
    if (value == 0) return fail(error, "unknown road kind " + *kind);
    run.road_kind = value;
  }
  if (const Json* parameters = road->find("parameters");
      parameters != nullptr && parameters->is_object()) {
    run.road_origin_x = number_or(*parameters, "origin_x", 0.0);
    run.road_origin_z = number_or(*parameters, "origin_z", 0.0);
    run.road_amplitude = number_or(*parameters, "amplitude", 0.0);
    run.road_wavelength = number_or(*parameters, "wavelength", 0.0);
    run.road_phase = number_or(*parameters, "phase", 0.0);
    run.road_bump_start = number_or(*parameters, "bump_start", 0.0);
    run.road_bump_length = number_or(*parameters, "bump_length", 0.0);
    if (const Json* scales = parameters->find("corner_scale");
        scales != nullptr && scales->is_array() && scales->items.size() == 4) {
      for (std::size_t index = 0; index < 4; ++index) {
        run.road_corner_scale[index] = scales->items[index].number;
      }
    }
  }
  return true;
}

/// Read the optional case-level solver overrides a dynamic case may carry.
inline bool read_case_overrides(const Json& document, ContractCase& run,
                                std::string& error) {
  const Json* inputs = document.find("inputs");
  if (inputs == nullptr || !inputs->is_object()) return true;
  run.initial_state_angle_tolerance =
      number_or(*inputs, "initial_state_angle_tolerance", run.initial_state_angle_tolerance);
  const Json* gauge = inputs->find("static_gauge");
  if (gauge == nullptr) return true;
  if (!gauge->is_object()) return fail(error, "static_gauge must be an object");
  run.has_static_gauge = true;
  const Json* body = gauge->find("body");
  if (body == nullptr || body->kind != JsonKind::String) {
    return fail(error, "static_gauge needs a body name");
  }
  const Json* mask = gauge->find("dof_mask");
  if (mask == nullptr || mask->kind != JsonKind::Number) {
    return fail(error, "static_gauge needs a dof_mask");
  }
  run.static_gauge_dof_mask = static_cast<std::uint32_t>(mask->number);
  if (const Json* release = gauge->find("trim_then_release");
      release != nullptr && release->kind == JsonKind::Bool) {
    run.static_trim_then_release = release->boolean;
  }
  // The body has to be resolved against the model, which the caller owns.
  run.static_gauge_body_name = body->text;
  return true;
}

/// Expand a `handling` document.
bool expand_handling(const Json& document, const std::string& blob,
                     const ContractModel& model, ContractPlan& out,
                     std::string& error);

/// Expand a `ride_random_road` document.
bool expand_ride_random_road(const Json& document, const std::string& blob,
                             const ContractModel& model, ContractPlan& out,
                             std::string& error);

/// Expand a `ride_four_post` document.
bool expand_ride_four_post(const Json& document, const std::string& blob,
                           const ContractModel& model, ContractPlan& out,
                           std::string& error);

/// Expand a `vehicle_dynamic` document.
bool expand_vehicle_dynamic(const Json& document, const std::string& blob,
                            const ContractModel& model, ContractPlan& out,
                            std::string& error);

/// Expand the `k` and `c` sections of a document whose family the caller has
/// already read.  Shared by every family that sweeps driven coordinates.
bool expand_driven_grid(const Json& document, const ContractModel& model,
                        ContractPlan& out, std::string& error);

/// Expand a `vehicle_kc` document.
bool expand_vehicle_kc(const Json& document, const ContractModel& model,
                       ContractPlan& out, std::string& error);

/// Expand a `kc_quasi_static` document.
bool expand_kc_quasi_static(const Json& document, const ContractModel& model,
                            ContractPlan& out, std::string& error);

/// Expand an `axle_dynamic` document.  `blob` carries its sampled tables.
bool expand_axle_dynamic(const Json& document, const std::string& blob,
                         const ContractModel& model, ContractPlan& out,
                         std::string& error);

}  // namespace case_detail
}  // namespace axle_kernel
