/// Read a `multibody-model` contract document into the kernel's input tables.
///
/// This translation unit is the whole of the model half of the document
/// boundary.  Two decisions are worth stating because everything else follows
/// from them:
///
/// * **The document is body-local.**  Points, axes and bushing frames are
///   expressed in the frame of the body that owns them, so this reader never
///   has to know a body pose to place a joint.  The Python authoring layer does
///   the world-to-local conversion once, where the assembly geometry lives, and
///   the kernel gets the same numbers it would have received through the ctypes
///   mirror.
/// * **The document is millimetres.**  Lengths are converted here and nowhere
///   else, and the mixed-unit stiffness blocks of a bushing are scaled by the
///   unit of each block rather than by a single global factor.
///
/// What this reader deliberately does *not* do is guess.  A missing required
/// field, an unknown joint or element type, an unsupported unit system and a
/// duplicate body name are all errors, because a silently defaulted model that
/// still solves is the most expensive failure mode this boundary has.

#include "case_common.hpp"

#include "mb_model/enums.hpp"
// `install_tire_mass` below writes into `Model`, and the model type is complete
// here so the definition can index `Model::tires`.  The module edge this adds is
// one `mb_cases` already has in its translation units.
#include "mb_model/types.hpp"

#include <string>
#include <unordered_map>

namespace axle_kernel {
namespace {

using Json = JsonValue;

bool fail(std::string& error, const std::string& message) {
  error = message;
  return false;
}

bool is_number(const Json& value) {
  return value.kind == JsonKind::Number;
}

bool number_at(const Json& value, double& out) {
  if (!is_number(value)) return false;
  out = value.number;
  return true;
}

/// The ABI's fixed PAC2002 parameter count.  It is a property of the kernel's
/// parameter layout, not of any one tire, so the reader names it once.
constexpr std::size_t VehicleParameterCount = 226;

/// Read `key` as a number, or return `fallback` when it is absent.
double number_or_default(const Json& object, const char* key, double fallback) {
  const Json* value = object.find(key);
  if (value == nullptr || value->kind != JsonKind::Number) return fallback;
  return value->number;
}

/// Read a described `(abscissa, value)` table into an offset/count pair.
///
/// The ABI stores these as one flat list per axis with an offset and a count per
/// tire, so a tire with no table gets a count of zero and contributes nothing.
bool read_described_table(const JsonValue& document, const std::string& blob,
                          const std::string& name, std::vector<double>& x_out,
                          std::vector<double>& y_out, int& count_out,
                          std::string& error) {
  std::vector<double> flat;
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
  const Json* shape = found->find("shape");
  const Json* offset_value = found->find("offset");
  const Json* length_value = found->find("length");
  if (shape == nullptr || offset_value == nullptr || length_value == nullptr ||
      !shape->is_array() || shape->items.size() != 2 ||
      static_cast<std::size_t>(shape->items[1].number) != 2) {
    return fail(error, "descriptor " + name + " must have shape [rows, 2]");
  }
  const std::size_t rows =
      static_cast<std::size_t>(shape->items[0].number);
  const long long offset = offset_value->number_is_integer
                               ? offset_value->integer
                               : static_cast<long long>(offset_value->number);
  const long long length = length_value->number_is_integer
                               ? length_value->integer
                               : static_cast<long long>(length_value->number);
  if (length != static_cast<long long>(rows * 2 * sizeof(double))) {
    return fail(error, "descriptor " + name + " has the wrong length");
  }
  if (offset < 0 ||
      static_cast<std::size_t>(offset) + static_cast<std::size_t>(length) >
          blob.size()) {
    return fail(error, "descriptor " + name + " falls outside the model payload");
  }
  const double* values = reinterpret_cast<const double*>(blob.data() + offset);
  for (std::size_t row = 0; row < rows; ++row) {
    x_out.push_back(values[row * 2]);
    y_out.push_back(values[row * 2 + 1]);
  }
  count_out = static_cast<int>(rows);
  return true;
}

/// Read `key` as a number when it is present; absent leaves `out` untouched.
bool optional_number(const Json& object, const char* key, double& out) {
  const Json* value = object.find(key);
  if (value == nullptr) return true;
  return number_at(*value, out);
}

bool vec3_at(const Json& value, double* out) {
  if (!value.is_array() || value.items.size() != 3) return false;
  for (std::size_t index = 0; index < 3; ++index) {
    if (!number_at(value.items[index], out[index])) return false;
  }
  return true;
}

/// Read a 3-vector field, defaulting to `fallback` when it is absent.
bool optional_vec3(const Json& object, const char* key, const double* fallback, double* out) {
  const Json* value = object.find(key);
  if (value == nullptr) {
    for (std::size_t index = 0; index < 3; ++index) out[index] = fallback[index];
    return true;
  }
  return vec3_at(*value, out);
}

/// Read a 3x3 numeric array into nine row-major doubles.
///
/// The shape is checked here rather than trusted: the schema says `3x3`, but a
/// document can reach the kernel through a payload that never went past the
/// Python validator, and a short matrix would otherwise read past its own row.
bool mat3_at(const Json& value, double* out) {
  if (!value.is_array() || value.items.size() != 3) return false;
  for (std::size_t row = 0; row < 3; ++row) {
    const Json& line = value.items[row];
    if (!line.is_array() || line.items.size() != 3) return false;
    for (std::size_t column = 0; column < 3; ++column) {
      if (!number_at(line.items[column], out[row * 3 + column])) return false;
    }
  }
  return true;
}

/// Read an optional 3x3 field, leaving every entry zero when it is absent.
///
/// A zero tensor is the dense spelling of "the tire owns no inertia of its
/// own", so an absent field and an explicitly zero one mean the same thing.
bool optional_mat3(const Json& object, const char* key, double* out) {
  const Json* value = object.find(key);
  if (value == nullptr) {
    for (std::size_t index = 0; index < 9; ++index) out[index] = 0.0;
    return true;
  }
  return mat3_at(*value, out);
}
bool optional_quaternion(const Json& object, const char* key, double* out) {
  const Json* value = object.find(key);
  if (value == nullptr) {
    out[0] = 1.0;
    out[1] = 0.0;
    out[2] = 0.0;
    out[3] = 0.0;
    return true;
  }
  if (!value->is_array() || value->items.size() != 4) return false;
  for (std::size_t index = 0; index < 4; ++index) {
    if (!number_at(value->items[index], out[index])) return false;
  }
  return true;
}

/// A 3x3 matrix in row-major order into a 9-element destination.
bool matrix3_at(const Json& value, double* out) {
  if (!value.is_array() || value.items.size() != 3) return false;
  for (std::size_t row = 0; row < 3; ++row) {
    if (!vec3_at(value.items[row], out + row * 3)) return false;
  }
  return true;
}

/// A 6x6 matrix in row-major order into a 36-element destination.
bool matrix6_at(const Json& value, double* out) {
  if (!value.is_array() || value.items.size() != 6) return false;
  for (std::size_t row = 0; row < 6; ++row) {
    const Json& line = value.items[row];
    if (!line.is_array() || line.items.size() != 6) return false;
    for (std::size_t column = 0; column < 6; ++column) {
      if (!number_at(line.items[column], out[row * 6 + column])) return false;
    }
  }
  return true;
}

/// A `[x, y]` pair, the shape a two-column curve point is written in.
bool pair_at(const Json& value, double* out) {
  if (!value.is_array() || value.items.size() != 2) return false;
  return number_at(value.items[0], out[0]) && number_at(value.items[1], out[1]);
}

bool vec6_at(const Json& value, double* out) {
  if (!value.is_array() || value.items.size() != 6) return false;
  for (std::size_t index = 0; index < 6; ++index) {
    if (!number_at(value.items[index], out[index])) return false;
  }
  return true;
}

/// The contract's joint names are a closed set; an unknown one is an error
/// rather than a zero, because row counts drive every later allocation.
bool joint_type_at(const std::string& name, int& out, bool& driven) {
  driven = false;
  static const std::pair<const char*, int> kTable[] = {
      {"spherical", AXLE_SPHERICAL}, {"revolute", AXLE_REVOLUTE},
      {"fixed", AXLE_FIXED},         {"prismatic", AXLE_PRISMATIC},
      {"universal", AXLE_UNIVERSAL}, {"cylindrical", AXLE_CYLINDRICAL},
      {"inplane", AXLE_INPLANE},     {"convel", AXLE_CONVEL},
  };
  for (const auto& entry : kTable) {
    if (name == entry.first) {
      out = entry.second;
      return true;
    }
  }
  if (name == "driven_translation") {
    out = AXLE_DRIVEN_TRANSLATION;
    driven = true;
    return true;
  }
  if (name == "driven_rotation") {
    out = AXLE_DRIVEN_ROTATION;
    driven = true;
    return true;
  }
  return false;
}

std::string quote(const std::string& text) { return "\"" + text + "\""; }

/// Rotation matrix of a scalar-first quaternion, row-major into `out`.
void quaternion_to_rotation(const double* q, double* out) {
  const double w = q[0], x = q[1], y = q[2], z = q[3];
  out[0] = 1.0 - 2.0 * (y * y + z * z);
  out[1] = 2.0 * (x * y - w * z);
  out[2] = 2.0 * (x * z + w * y);
  out[3] = 2.0 * (x * y + w * z);
  out[4] = 1.0 - 2.0 * (x * x + z * z);
  out[5] = 2.0 * (y * z - w * x);
  out[6] = 2.0 * (x * z - w * y);
  out[7] = 2.0 * (y * z + w * x);
  out[8] = 1.0 - 2.0 * (x * x + y * y);
}

}  // namespace

static bool read_units(const Json& document, double& length_scale,
                       std::string& error) {
  const Json* units = document.find("units");
  if (units == nullptr || !units->is_object()) {
    return fail(error, "model document has no units object");
  }
  struct Unit {
    const char* key;
    const char* expected;
  };
  static const Unit kUnits[] = {
      {"mass", "kg"}, {"time", "s"}, {"angle", "rad"},
  };
  // The length unit is the one the reader scales by, so it is read first and
  // anything but the two forms this build understands is rejected.
  if (const std::string* length = units->find_string("length")) {
    if (*length == "mm") {
      length_scale = 1e-3;
    } else if (*length == "m") {
      length_scale = 1.0;
    } else {
      return fail(error, "units.length must be mm or m, got " + *length);
    }
  } else {
    return fail(error, "units.length is missing");
  }
  for (const Unit& unit : kUnits) {
    const std::string* value = units->find_string(unit.key);
    if (value == nullptr) {
      return fail(error, std::string("units.") + unit.key + " is missing");
    }
    if (*value != unit.expected) {
      return fail(error, std::string("units.") + unit.key + " must be " +
                              unit.expected + ", got " + *value);
    }
  }
  return true;
}

bool ContractModel::read(const JsonValue& document, const std::string& blob,
                         std::string& error) {
  error.clear();
  const std::string* contract = document.find_string("contract");
  const std::string* kind = document.find_string("kind");
  if (contract == nullptr || *contract != "multibody-model" ||
      kind == nullptr || *kind != "model") {
    return fail(error, "expected a multibody-model document");
  }
  const std::string* name = document.find_string("name");
  if (name == nullptr || name->empty()) {
    return fail(error, "model document has no name");
  }
  if (!read_units(document, length_scale_, error)) return false;

  name_ = *name;

  // A model says whether it needs the vehicle-level registration stages rather
  // than having the reader infer it from which elements happen to be present: a
  // model that is assembled through the vehicle path and one that is not can
  // differ even when they look identical, and guessing here would reintroduce
  // exactly the divergence the declaration removes.
  if (const Json* capabilities = document.find("capabilities");
      capabilities != nullptr && capabilities->is_array()) {
    for (const Json& entry : capabilities->items) {
      if (entry.kind == JsonKind::String && entry.text == "vehicle") {
        vehicle_stages_ = true;
      }
    }
  }

  {
    // Gravity follows the document's length unit, so it arrives in mm/s^2.
    double fallback_gravity[3] = {0.0, 0.0, 0.0};
    if (!optional_vec3(document, "gravity", fallback_gravity, gravity_)) {
      return fail(error, "gravity must be three numbers");
    }
    for (double& component : gravity_) component *= length_scale_;
  }

  const Json* body_items = document.find("bodies");
  if (body_items == nullptr || !body_items->is_array() || body_items->items.empty()) {
    return fail(error, "model document must declare at least one body");
  }

  std::unordered_map<std::string, int> body_lookup;
  std::vector<std::string> joint_names;
  double fallback_position[3] = {0.0, 0.0, 0.0};
  double fallback_velocity[3] = {0.0, 0.0, 0.0};
  for (const Json& body : body_items->items) {
    if (!body.is_object()) return fail(error, "a body entry is not an object");
    const std::string* body_name = body.find_string("name");
    if (body_name == nullptr || body_name->empty()) {
      return fail(error, "a body entry has no name");
    }
    if (body_lookup.count(*body_name) != 0) {
      return fail(error, "duplicate body name " + quote(*body_name));
    }

    double mass = 0.0;
    if (const Json* value = body.find("mass"); value == nullptr || !number_at(*value, mass)) {
      return fail(error, "body " + quote(*body_name) + " has no mass");
    }
    double inertia[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    if (const Json* value = body.find("inertia");
        value == nullptr || !matrix3_at(*value, inertia)) {
      return fail(error, "body " + quote(*body_name) + " has no 3x3 inertia");
    }
    bool fixed = false;
    if (const Json* value = body.find("fixed"); value != nullptr) {
      if (value->kind != JsonKind::Bool) {
        return fail(error, "body " + quote(*body_name) + " has a non-boolean fixed flag");
      }
      fixed = value->boolean;
    }
    double position[3];
    double quaternion[4];
    double velocity[3];
    double omega[3];
    if (!optional_vec3(body, "position", fallback_position, position) ||
        !optional_quaternion(body, "quaternion", quaternion) ||
        !optional_vec3(body, "velocity", fallback_velocity, velocity) ||
        !optional_vec3(body, "omega", fallback_velocity, omega)) {
      return fail(error, "body " + quote(*body_name) + " has a malformed pose");
    }

    const int index = static_cast<int>(body_lookup.size());
    body_lookup.emplace(*body_name, index);

    body_names_.push_back(*body_name);
    // A fixed body is inert and massless: the pose constraints carry it, and a
    // finite mass would only make the constrained system stiffer to factor.
    if (fixed) {
      body_mass_.push_back(0.0);
      for (std::size_t entry = 0; entry < 9; ++entry) {
        body_inertia_.push_back(entry % 4 == 0 ? 1.0 : 0.0);
      }
    } else {
      body_mass_.push_back(mass > 0.0 ? mass : 1.0);
      // Inertia is a mass times a length squared, so under a millimetre
      // document it arrives in kg*mm^2 and the kernel wants kg*m^2.
      const double inertia_factor = inertia_scale();
      double norm = 0.0;
      for (double entry : inertia) norm += entry * entry;
      for (double entry : inertia) {
        body_inertia_.push_back(norm > 1e-18 ? entry * inertia_factor : 0.0);
      }
    }
    body_pose_.push_back(position[0] * length_scale_);
    body_pose_.push_back(position[1] * length_scale_);
    body_pose_.push_back(position[2] * length_scale_);
    body_pose_.push_back(quaternion[0]);
    body_pose_.push_back(quaternion[1]);
    body_pose_.push_back(quaternion[2]);
    body_pose_.push_back(quaternion[3]);
    body_velocity_.insert(body_velocity_.end(), velocity, velocity + 3);
    body_velocity_.insert(body_velocity_.end(), omega, omega + 3);
    body_fixed_.push_back(fixed ? 1 : 0);
  }

  // --- joints -------------------------------------------------------------
  const Json* joint_items = document.find("joints");
  if (joint_items != nullptr) {
    if (!joint_items->is_array()) return fail(error, "joints must be an array");
    for (const Json& joint : joint_items->items) {
      if (!joint.is_object()) return fail(error, "a joint entry is not an object");
      const std::string* joint_name = joint.find_string("name");
      const std::string* type_name = joint.find_string("type");
      const std::string* body_a = joint.find_string("body_a");
      const std::string* body_b = joint.find_string("body_b");
      if (joint_name == nullptr || type_name == nullptr || body_a == nullptr ||
          body_b == nullptr) {
        return fail(error, "a joint entry is missing name/type/body_a/body_b");
      }
      const auto index_a = body_lookup.find(*body_a);
      const auto index_b = body_lookup.find(*body_b);
      if (index_a == body_lookup.end() || index_b == body_lookup.end()) {
        return fail(error, "joint " + quote(*joint_name) + " names an unknown body");
      }
      int type = 0;
      bool driven = false;
      if (!joint_type_at(*type_name, type, driven)) {
        return fail(error, "joint " + quote(*joint_name) + " has unknown type " +
                                quote(*type_name));
      }
      double point_a[3] = {0.0, 0.0, 0.0};
      double point_b[3] = {0.0, 0.0, 0.0};
      double axis_a[3] = {0.0, 0.0, 1.0};
      double axis_b[3] = {0.0, 0.0, 1.0};
      double axis_a_secondary[3] = {0.0, 1.0, 0.0};
      double axis_b_secondary[3] = {1.0, 0.0, 0.0};
      double convel_angle_target = 0.0;
      if (!optional_vec3(joint, "point_a", point_a, point_a) ||
          !optional_vec3(joint, "point_b", point_b, point_b) ||
          !optional_vec3(joint, "axis_a", axis_a, axis_a) ||
          !optional_vec3(joint, "axis_b", axis_b, axis_b) ||
          !optional_vec3(
              joint, "axis_a_secondary", axis_a_secondary, axis_a_secondary
          ) ||
          !optional_vec3(
              joint, "axis_b_secondary", axis_b_secondary, axis_b_secondary
          ) ||
          !optional_number(
              joint, "convel_angle_target", convel_angle_target
          )) {
        return fail(error, "joint " + quote(*joint_name) + " has a malformed point or axis");
      }

      if (!driven) {
        joint_names.push_back(*joint_name);
        constraint_type_.push_back(type);
        constraint_body_a_.push_back(index_a->second);
        constraint_body_b_.push_back(index_b->second);
        for (double value : point_a) constraint_point_a_.push_back(value * length_scale_);
        for (double value : point_b) constraint_point_b_.push_back(value * length_scale_);
        for (double value : axis_a) constraint_axis_a_.push_back(value);
        for (double value : axis_b) constraint_axis_b_.push_back(value);
        for (double value : axis_a_secondary) {
          constraint_axis_a_secondary_.push_back(value);
        }
        for (double value : axis_b_secondary) {
          constraint_axis_b_secondary_.push_back(value);
        }
        constraint_convel_angle_target_.push_back(convel_angle_target);
        continue;
      }

      // A driven coordinate is not a joint row in the document's sense: it
      // prescribes a single relative degree of freedom, and the target signal
      // that drives it is named by `target`.
      const std::string* target = joint.find_string("target");
      if (target == nullptr || target->empty()) {
        return fail(error, "driven coordinate " + quote(*joint_name) + " has no target");
      }
      double reference[4];
      if (!optional_quaternion(joint, "reference_quaternion", reference)) {
        return fail(error, "driven coordinate " + quote(*joint_name) +
                                " has a malformed reference_quaternion");
      }
      driven_type_.push_back(type);
      driven_body_.push_back(index_a->second);
      driven_reaction_body_.push_back(index_b->second);
      for (double value : point_a) driven_point_.push_back(value * length_scale_);
      for (double value : point_b) driven_reaction_point_.push_back(value * length_scale_);
      for (double value : axis_b) driven_axis_.push_back(value);
      for (double value : reference) driven_reference_.push_back(value);
      driven_names_.push_back(*target);
    }
  }

  // --- bushings -----------------------------------------------------------
  const Json* element_items = document.find("elements");
  if (element_items != nullptr) {
    if (!element_items->is_array()) return fail(error, "elements must be an array");
    for (const Json& element : element_items->items) {
      if (!element.is_object()) return fail(error, "an element entry is not an object");
      const std::string* element_name = element.find_string("name");
      const std::string* type_name = element.find_string("type");
      if (element_name == nullptr || type_name == nullptr) {
        return fail(error, "an element entry is missing name/type");
      }
      if (*type_name == "spring_damper") {
        double point_a[3];
        double point_b[3];
        const std::string* body_a = element.find_string("body_a");
        const std::string* body_b = element.find_string("body_b");
        const Json* parameters = element.find("parameters");
        if (body_a == nullptr || body_b == nullptr || parameters == nullptr ||
            !parameters->is_object()) {
          return fail(error, "spring " + quote(*element_name) + " has no bodies or parameters");
        }
        const auto index_a = body_lookup.find(*body_a);
        const auto index_b = body_lookup.find(*body_b);
        if (index_a == body_lookup.end() || index_b == body_lookup.end()) {
          return fail(error, "spring " + quote(*element_name) + " names an unknown body");
        }
        double zero[3] = {0.0, 0.0, 0.0};
        if (!optional_vec3(*parameters, "point_a", zero, point_a) ||
            !optional_vec3(*parameters, "point_b", zero, point_b)) {
          return fail(error, "spring " + quote(*element_name) + " has a malformed point");
        }
        // A spring's stiffness is a force per length, so it scales with the
        // document's length unit; the lengths themselves do not.
        const double force_per_length = force_per_length_scale();
        double stiffness = 0.0;
        double compression_damping = 0.0;
        double rebound_damping = 0.0;
        double free_length = 0.0;
        double minimum_length = std::numeric_limits<double>::quiet_NaN();
        double maximum_length = std::numeric_limits<double>::quiet_NaN();
        double compression_stop_stiffness = 0.0;
        double compression_stop_damping = 0.0;
        double rebound_stop_stiffness = 0.0;
        double rebound_stop_damping = 0.0;
        if (!optional_number(*parameters, "stiffness", stiffness) ||
            !optional_number(*parameters, "compression_damping", compression_damping) ||
            !optional_number(*parameters, "rebound_damping", rebound_damping) ||
            !optional_number(*parameters, "free_length", free_length) ||
            !optional_number(*parameters, "minimum_length", minimum_length) ||
            !optional_number(*parameters, "maximum_length", maximum_length) ||
            !optional_number(*parameters, "compression_stop_stiffness",
                             compression_stop_stiffness) ||
            !optional_number(*parameters, "compression_stop_damping",
                             compression_stop_damping) ||
            !optional_number(*parameters, "rebound_stop_stiffness",
                             rebound_stop_stiffness) ||
            !optional_number(*parameters, "rebound_stop_damping",
                             rebound_stop_damping)) {
          return fail(error, "spring " + quote(*element_name) + " has a malformed parameter");
        }
        spring_body_a_.push_back(index_a->second);
        spring_body_b_.push_back(index_b->second);
        for (double value : point_a) spring_point_a_.push_back(value * length_scale_);
        for (double value : point_b) spring_point_b_.push_back(value * length_scale_);
        spring_stiffness_.push_back(stiffness * force_per_length);
        spring_compression_damping_.push_back(compression_damping * force_per_length);
        spring_rebound_damping_.push_back(rebound_damping * force_per_length);
        spring_free_length_.push_back(free_length * length_scale_);
        spring_minimum_length_.push_back(minimum_length * length_scale_);
        spring_maximum_length_.push_back(maximum_length * length_scale_);
        spring_compression_stop_stiffness_.push_back(
            compression_stop_stiffness * force_per_length);
        spring_compression_stop_damping_.push_back(
            compression_stop_damping * force_per_length);
        spring_rebound_stop_stiffness_.push_back(rebound_stop_stiffness * force_per_length);
        spring_rebound_stop_damping_.push_back(rebound_stop_damping * force_per_length);

        // The damper curve is a force against a velocity, so its abscissa
        // scales with the length unit and its ordinate does not.
        const Json* curve = parameters->find("damper_curve");
        spring_damper_curve_offset_.push_back(
            static_cast<int>(spring_damper_curve_velocity_.size()));
        if (curve == nullptr) {
          spring_damper_curve_count_.push_back(0);
        } else {
          if (!curve->is_array()) {
            return fail(error, "spring " + quote(*element_name) +
                                    " has a malformed damper_curve");
          }
          spring_damper_curve_count_.push_back(static_cast<int>(curve->items.size()));
          for (const Json& point : curve->items) {
            double pair[2] = {0.0, 0.0};
            if (!pair_at(point, pair)) {
              return fail(error, "spring " + quote(*element_name) +
                                      " has a malformed damper_curve point");
            }
            spring_damper_curve_velocity_.push_back(pair[0] * length_scale_);
            spring_damper_curve_force_.push_back(pair[1]);
          }
        }

        const auto read_spring_curve = [this, &parameters, &element_name, &error](
            const char* key, std::vector<int>& offsets,
            std::vector<int>& counts, std::vector<double>& abscissa,
            std::vector<double>& force, const char* label) -> bool {
          offsets.push_back(static_cast<int>(abscissa.size()));
          const Json* table = parameters->find(key);
          if (table == nullptr) {
            counts.push_back(0);
            return true;
          }
          if (!table->is_array()) {
            return fail(error, "spring " + quote(*element_name) +
                                " has a malformed " + label);
          }
          counts.push_back(static_cast<int>(table->items.size()));
          for (const Json& point : table->items) {
            double pair[2] = {0.0, 0.0};
            if (!pair_at(point, pair)) {
              return fail(error, "spring " + quote(*element_name) +
                                  " has a malformed " + label + " point");
            }
            abscissa.push_back(pair[0] * length_scale_);
            force.push_back(pair[1]);
          }
          return true;
        };
        if (!read_spring_curve(
                "elastic_curve", spring_elastic_curve_offset_,
                spring_elastic_curve_count_, spring_elastic_curve_deflection_,
                spring_elastic_curve_force_, "elastic_curve") ||
            !read_spring_curve(
                "compression_stop_curve", spring_compression_stop_curve_offset_,
                spring_compression_stop_curve_count_,
                spring_compression_stop_curve_penetration_,
                spring_compression_stop_curve_force_,
                "compression_stop_curve") ||
            !read_spring_curve(
                "rebound_stop_curve", spring_rebound_stop_curve_offset_,
                spring_rebound_stop_curve_count_,
                spring_rebound_stop_curve_penetration_,
                spring_rebound_stop_curve_force_,
                "rebound_stop_curve")) {
          return false;
        }
        continue;
      }
      if (*type_name == "anti_roll_bar") {
        const std::string* body_a = element.find_string("body_a");
        const std::string* body_b = element.find_string("body_b");
        const Json* parameters = element.find("parameters");
        if (body_a == nullptr || body_b == nullptr || parameters == nullptr ||
            !parameters->is_object()) {
          return fail(error, "anti-roll bar " + quote(*element_name) +
                                  " has no bodies or parameters");
        }
        const auto index_a = body_lookup.find(*body_a);
        const auto index_b = body_lookup.find(*body_b);
        if (index_a == body_lookup.end() || index_b == body_lookup.end()) {
          return fail(error, "anti-roll bar " + quote(*element_name) +
                                  " names an unknown body");
        }
        double axis[3] = {1.0, 0.0, 0.0};
        double reference[4];
        if (!optional_vec3(*parameters, "axis_a", axis, axis) ||
            !optional_quaternion(*parameters, "reference_quaternion", reference)) {
          return fail(error, "anti-roll bar " + quote(*element_name) +
                                  " has a malformed axis or reference");
        }
        anti_roll_body_a_.push_back(index_a->second);
        anti_roll_body_b_.push_back(index_b->second);
        for (double value : axis) anti_roll_axis_a_.push_back(value);
        for (double value : reference) anti_roll_reference_.push_back(value);
        anti_roll_stiffness_.push_back(
            number_or_default(*parameters, "stiffness", 0.0) * length_scale_);
        anti_roll_damping_.push_back(
            number_or_default(*parameters, "damping", 0.0) * length_scale_);
        continue;
      }
      if (*type_name == "aerodynamic_drag") {
        const std::string* body = element.find_string("body_a");
        const Json* parameters = element.find("parameters");
        if (body == nullptr || parameters == nullptr || !parameters->is_object()) {
          return fail(error, "aerodynamic drag " + quote(*element_name) +
                                  " has no body or parameters");
        }
        const auto index = body_lookup.find(*body);
        if (index == body_lookup.end()) {
          return fail(error, "aerodynamic drag " + quote(*element_name) +
                                  " names an unknown body");
        }
        double zero[3] = {0.0, 0.0, 0.0};
        double point[3];
        double forward[3];
        double coefficient = 0.0;
        if (!optional_vec3(*parameters, "application_point", zero, point) ||
            !optional_vec3(*parameters, "forward_axis", zero, forward) ||
            !optional_number(*parameters, "coefficient", coefficient)) {
          return fail(error, "aerodynamic drag " + quote(*element_name) +
                                  " has malformed parameters");
        }
        aero_body_.push_back(index->second);
        for (double value : point) aero_point_.push_back(value * length_scale_);
        for (double value : forward) aero_axis_.push_back(value);
        aero_coefficient_.push_back(coefficient);
        continue;
      }
      if (*type_name == "steering_actuator") {
        const Json* parameters = element.find("parameters");
        const std::string* target = element.find_string("target");
        if (parameters == nullptr || !parameters->is_object() || target == nullptr ||
            target->empty()) {
          return fail(error, "steering actuator " + quote(*element_name) +
                                  " has no parameters or target");
        }
        static const std::pair<const char*, int> kActuators[] = {
            {"translation", VEHICLE_STEERING_TRANSLATION},
            {"rotation", VEHICLE_STEERING_ROTATION},
            {"prescribed_rotation", VEHICLE_STEERING_PRESCRIBED_ROTATION},
            {"prescribed_translation", VEHICLE_STEERING_PRESCRIBED_TRANSLATION},
        };
        const std::string* type = parameters->find_string("type");
        if (type == nullptr) {
          return fail(error, "steering actuator " + quote(*element_name) + " has no type");
        }
        int type_value = -1;
        for (const auto& entry : kActuators) {
          if (*type == entry.first) type_value = entry.second;
        }
        if (type_value < 0) {
          return fail(error, "steering actuator " + quote(*element_name) +
                                  " has unknown type " + quote(*type));
        }
        const std::string* body = parameters->find_string("body");
        if (body == nullptr) {
          return fail(error, "steering actuator " + quote(*element_name) + " has no body");
        }
        const auto index = body_lookup.find(*body);
        if (index == body_lookup.end()) {
          return fail(error, "steering actuator " + quote(*element_name) +
                                  " names an unknown body");
        }
        int reaction = -1;
        if (const std::string* name = parameters->find_string("reaction_body");
            name != nullptr) {
          const auto found = body_lookup.find(*name);
          if (found == body_lookup.end()) {
            return fail(error, "steering actuator " + quote(*element_name) +
                                    " names an unknown reaction body");
          }
          reaction = found->second;
        }
        double zero[3] = {0.0, 0.0, 0.0};
        double point[3];
        double reaction_point[3];
        double axis[3] = {0.0, 0.0, 1.0};
        double reference[4];
        if (!optional_vec3(*parameters, "point_local", zero, point) ||
            !optional_vec3(*parameters, "reaction_point_local", zero, reaction_point) ||
            !optional_vec3(*parameters, "axis_local", axis, axis) ||
            !optional_quaternion(*parameters, "reference_quaternion", reference)) {
          return fail(error, "steering actuator " + quote(*element_name) +
                                  " has malformed geometry");
        }
        steering_names_.push_back(*target);
        steering_type_.push_back(type_value);
        steering_body_.push_back(index->second);
        steering_reaction_body_.push_back(reaction);
        for (double value : point) steering_point_.push_back(value * length_scale_);
        for (double value : reaction_point) {
          steering_reaction_point_.push_back(value * length_scale_);
        }
        for (double value : axis) steering_axis_.push_back(value);
        for (double value : reference) steering_reference_.push_back(value);
        steering_stiffness_.push_back(
            number_or_default(*parameters, "stiffness", 0.0) * force_per_length_scale());
        steering_damping_.push_back(
            number_or_default(*parameters, "damping", 0.0) * force_per_length_scale());
        continue;
      }
      if (*type_name != "bushing") {
        return fail(error, "element " + quote(*element_name) + " has unsupported type " +
                                quote(*type_name));
      }
      const std::string* body_a = element.find_string("body_a");
      const std::string* body_b = element.find_string("body_b");
      const Json* parameters = element.find("parameters");
      if (body_a == nullptr || body_b == nullptr || parameters == nullptr ||
          !parameters->is_object()) {
        return fail(error, "bushing " + quote(*element_name) + " has no bodies or parameters");
      }
      const auto index_a = body_lookup.find(*body_a);
      const auto index_b = body_lookup.find(*body_b);
      if (index_a == body_lookup.end() || index_b == body_lookup.end()) {
        return fail(error, "bushing " + quote(*element_name) + " names an unknown body");
      }

      double point_a[3];
      double point_b[3];
      double frame_a[4];
      double frame_b[4];
      double stiffness[36];
      double damping[36];
      double preload[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      double reference_translation[3] = {0.0, 0.0, 0.0};
      double reference_quaternion[4] = {1.0, 0.0, 0.0, 0.0};
      const Json* stiffness_json = parameters->find("stiffness");
      const Json* damping_json = parameters->find("damping");
      if (stiffness_json == nullptr || damping_json == nullptr) {
        return fail(error, "bushing " + quote(*element_name) + " has no stiffness or damping");
      }
      if (!optional_vec3(*parameters, "point_a", point_a, point_a) ||
          !optional_vec3(*parameters, "point_b", point_b, point_b) ||
          !optional_quaternion(*parameters, "frame_a_quaternion", frame_a) ||
          !optional_quaternion(*parameters, "frame_b_quaternion", frame_b) ||
          !matrix6_at(*stiffness_json, stiffness) ||
          !matrix6_at(*damping_json, damping)) {
        return fail(error, "bushing " + quote(*element_name) + " has malformed parameters");
      }
      if (const Json* value = parameters->find("preload"); value != nullptr) {
        if (!vec6_at(*value, preload)) {
          return fail(error, "bushing " + quote(*element_name) + " has a malformed preload");
        }
      }
      if (!optional_vec3(*parameters, "reference_translation",
                        reference_translation, reference_translation) ||
          !optional_quaternion(*parameters, "reference_quaternion",
                               reference_quaternion)) {
        return fail(error, "bushing " + quote(*element_name) +
                                " has a malformed reference pose");
      }
      int rotation_coordinates = 0;
      if (const std::string* value =
              parameters->find_string("rotation_coordinates");
          value != nullptr) {
        if (*value == "rotation_vector") {
          rotation_coordinates = 0;
        } else if (*value == "cardan_xyz") {
          rotation_coordinates = 1;
        } else {
          return fail(error, "bushing " + quote(*element_name) +
                                  " has an unknown rotation_coordinates");
        }
      }

      bushing_body_a_.push_back(index_a->second);
      bushing_body_b_.push_back(index_b->second);
      for (double value : point_a) bushing_point_a_.push_back(value * length_scale_);
      for (double value : point_b) bushing_point_b_.push_back(value * length_scale_);
      for (double value : frame_a) bushing_frame_a_.push_back(value);
      for (double value : frame_b) bushing_frame_b_.push_back(value);
      for (double value : reference_translation) {
        bushing_reference_translation_.push_back(value * length_scale_);
      }
      for (double value : reference_quaternion) {
        bushing_reference_quaternion_.push_back(value);
      }
      bushing_rotation_coordinate_.push_back(rotation_coordinates);
      // Each 3x3 block carries the unit of its own rows and columns: a
      // translational force per metre, a rotational moment per radian, and the
      // two coupling blocks, which already read as force per radian and moment
      // per metre and therefore convert with a factor of one.
      for (std::size_t row = 0; row < 6; ++row) {
        for (std::size_t column = 0; column < 6; ++column) {
          const std::size_t entry = row * 6 + column;
          double scale = 1.0;
          if (row < 3 && column < 3) {
            scale = force_per_length_scale();
          } else if (row >= 3 && column >= 3) {
            scale = length_scale_;
          }
          bushing_stiffness_.push_back(stiffness[entry] * scale);
          bushing_damping_.push_back(damping[entry] * scale);
        }
      }
      for (std::size_t entry = 0; entry < 6; ++entry) {
        bushing_preload_.push_back(
            entry < 3 ? preload[entry] : preload[entry] * length_scale_);
      }

      // Bushing curves are optional and axis-indexed.  The array is emitted as
      // six curve arrays (possibly empty) so the ABI's flat offset/count table
      // has a stable shape even for bushings that use only linear stiffness.
      int interpolation = 0;
      if (const Json* value = parameters->find("force_curve_interpolation");
          value != nullptr) {
        if (value->kind != JsonKind::String) {
          return fail(error, "bushing " + quote(*element_name) +
                                  " has a malformed force_curve_interpolation");
        }
        const std::string& text = value->text;
        if (text == "piecewise_linear") {
          interpolation = 0;
        } else if (text == "akima") {
          interpolation = 1;
        } else {
          return fail(error, "bushing " + quote(*element_name) +
                                  " has an unknown force_curve_interpolation");
        }
      }
      bushing_force_curve_interpolation_.push_back(interpolation);
      const Json* curves = parameters->find("force_curves");
      if (curves != nullptr && (!curves->is_array() || curves->items.size() != 6)) {
        return fail(error, "bushing " + quote(*element_name) +
                                " force_curves must contain six axis curves");
      }
      for (int axis = 0; axis < 6; ++axis) {
        bushing_force_curve_offset_.push_back(
            static_cast<int>(bushing_force_curve_coordinate_.size()));
        int count = 0;
        if (curves != nullptr) {
          const Json& curve = curves->items[static_cast<std::size_t>(axis)];
          if (!curve.is_array()) {
            return fail(error, "bushing " + quote(*element_name) +
                                    " has a malformed force curve");
          }
          if (curve.items.size() == 1) {
            return fail(error, "bushing " + quote(*element_name) +
                                    " force curve needs at least two points");
          }
          double previous = 0.0;
          for (std::size_t point = 0; point < curve.items.size(); ++point) {
            double pair[2];
            if (!pair_at(curve.items[point], pair)) {
              return fail(error, "bushing " + quote(*element_name) +
                                      " has a malformed force curve point");
            }
            const double coordinate =
                pair[0] * (axis < 3 ? length_scale_ : 1.0);
            const double force = pair[1] * (axis < 3 ? 1.0 : length_scale_);
            if (!std::isfinite(coordinate) || !std::isfinite(force) ||
                (point > 0 && coordinate <= previous)) {
              return fail(error, "bushing " + quote(*element_name) +
                                      " force curve must be finite and increasing");
            }
            bushing_force_curve_coordinate_.push_back(coordinate);
            bushing_force_curve_force_.push_back(force);
            previous = coordinate;
          }
          count = static_cast<int>(curve.items.size());
        }
        bushing_force_curve_count_.push_back(count);
      }
    }
  }

  // --- tires --------------------------------------------------------------
  const Json* tire_items = document.find("tires");
  if (tire_items != nullptr) {
    if (!tire_items->is_array()) return fail(error, "tires must be an array");
    for (const Json& tire : tire_items->items) {
      if (!tire.is_object()) return fail(error, "a tire entry is not an object");
      const std::string* tire_name = tire.find_string("name");
      const std::string* model = tire.find_string("model");
      const std::string* body = tire.find_string("body");
      if (tire_name == nullptr || model == nullptr || body == nullptr) {
        return fail(error, "a tire entry is missing name/model/body");
      }
      static const std::pair<const char*, int> kTireModels[] = {
          {"native_brush", 0}, {"pac2002", 1}, {"fiala", 3},
      };
      int model_kind = -1;
      for (const auto& entry : kTireModels) {
        if (*model == entry.first) model_kind = entry.second;
      }
      if (model_kind < 0) {
        return fail(error, "tire " + quote(*tire_name) + " has unknown model " +
                                quote(*model));
      }
      const auto body_index = body_lookup.find(*body);
      if (body_index == body_lookup.end()) {
        return fail(error, "tire " + quote(*tire_name) + " names an unknown body");
      }
      const Json* parameters = tire.find("parameters");
      if (parameters == nullptr || !parameters->is_object()) {
        return fail(error, "tire " + quote(*tire_name) + " has no parameters");
      }
      double center[3] = {0.0, 0.0, 0.0};
      double spin[3] = {0.0, 1.0, 0.0};
      double forward[3] = {1.0, 0.0, 0.0};
      if (!optional_vec3(*parameters, "center_local", center, center) ||
          !optional_vec3(*parameters, "spin_axis_local", spin, spin) ||
          !optional_vec3(*parameters, "forward_axis_local", forward, forward)) {
        return fail(error, "tire " + quote(*tire_name) + " has malformed geometry");
      }
      double radius = 0.0;
      double maximum_compression = 0.0;
      double vertical_stiffness = 0.0;
      double vertical_damping = 0.0;
      double mu_longitudinal = 0.0;
      double mu_lateral = 0.0;
      double brush_longitudinal = 0.0;
      double brush_lateral = 0.0;
      double relaxation_longitudinal = 0.0;
      double relaxation_lateral = 0.0;
      double detached_relaxation = 0.0;
      if (!optional_number(*parameters, "unloaded_radius", radius) ||
          !optional_number(*parameters, "maximum_compression", maximum_compression) ||
          !optional_number(*parameters, "vertical_stiffness", vertical_stiffness) ||
          !optional_number(*parameters, "vertical_damping", vertical_damping) ||
          !optional_number(*parameters, "longitudinal_friction_coefficient", mu_longitudinal) ||
          !optional_number(*parameters, "lateral_friction_coefficient", mu_lateral) ||
          !optional_number(*parameters, "longitudinal_brush_stiffness", brush_longitudinal) ||
          !optional_number(*parameters, "lateral_brush_stiffness", brush_lateral) ||
          !optional_number(*parameters, "longitudinal_relaxation_length",
                           relaxation_longitudinal) ||
          !optional_number(*parameters, "lateral_relaxation_length", relaxation_lateral) ||
          !optional_number(*parameters, "detached_relaxation_s", detached_relaxation)) {
        return fail(error, "tire " + quote(*tire_name) + " has a malformed parameter");
      }
      // The tire's own mass and inertia are optional top-level fields of the
      // entry, not parameters of the force law: they say who owns the inertia,
      // not how the tire pushes on the road.  An entry that declares neither is
      // exactly the entry it would have been before these fields existed --
      // zero mass, zero tensor, and the wheel-end body keeps carrying the
      // inertia.  The values do not travel through `AxleInput` (that would be
      // an ABI freeze release); `ContractModel` keeps them and the contract
      // entry point installs them on the built model.
      double tire_mass = 0.0;
      if (!optional_number(tire, "mass", tire_mass)) {
        return fail(error, "tire " + quote(*tire_name) + " has a malformed mass");
      }
      if (tire_mass < 0.0) {
        return fail(error, "tire " + quote(*tire_name) +
                                " declares a negative mass");
      }
      double tire_inertia[9];
      if (!optional_mat3(tire, "inertia", tire_inertia)) {
        return fail(error, "tire " + quote(*tire_name) + " has a malformed inertia");
      }
      // An inertia is a mass times a length squared, so under a millimetre
      // document it arrives in kg*mm^2 and the kernel wants kg*m^2.  The mass
      // needs no conversion: the contract fixes the mass unit to kilograms.
      const double tire_inertia_factor = inertia_scale();
      for (double& entry : tire_inertia) {
        if (entry < 0.0) {
          return fail(error, "tire " + quote(*tire_name) +
                                  " declares a negative inertia");
        }
        entry *= tire_inertia_factor;
      }
      // The frame a tire is measured against is not always the body its force
      // acts on: a wheel spins on a carrier that does not.  The authoring layer
      // resolves that pair, so the reader only has to honour it.
      int frame_body = body_index->second;
      if (const std::string* frame = parameters->find_string("frame_body");
          frame != nullptr) {
        const auto found = body_lookup.find(*frame);
        if (found == body_lookup.end()) {
          return fail(error, "tire " + quote(*tire_name) +
                                  " names unknown frame body " + *frame);
        }
        frame_body = found->second;
      }
      double frame_center[3];
      if (!optional_vec3(*parameters, "frame_center_local", center, frame_center)) {
        return fail(error, "tire " + quote(*tire_name) + " has a malformed frame centre");
      }
      int drive_body = -1;
      int drive_reaction = -1;
      double drive_axis[3] = {0.0, 0.0, 0.0};
      if (const std::string* drive = parameters->find_string("drive_torque_body");
          drive != nullptr) {
        const auto found = body_lookup.find(*drive);
        if (found == body_lookup.end()) {
          return fail(error, "tire " + quote(*tire_name) +
                                  " names unknown drive body " + *drive);
        }
        drive_body = found->second;
        if (const std::string* reaction =
                parameters->find_string("drive_torque_reaction_body");
            reaction != nullptr) {
          const auto other = body_lookup.find(*reaction);
          if (other == body_lookup.end()) {
            return fail(error, "tire " + quote(*tire_name) +
                                    " names unknown drive reaction body " + *reaction);
          }
          drive_reaction = other->second;
        }
        if (!optional_vec3(*parameters, "drive_torque_axis_local", drive_axis,
                           drive_axis)) {
          return fail(error, "tire " + quote(*tire_name) +
                                  " has a malformed drive torque axis");
        }
      }

      // A PAC2002 tire that came from an Adams property file is a distinct
      // kind: the kernel treats its coefficients as source data and mirrors the
      // physical side.  The document says which, because the choice is the
      // author's, not the reader's.
      if (model_kind == 1) {
        const std::string* source = parameters->find_string("parameter_source");
        if (source != nullptr && *source == "adams_builtin") model_kind = 2;
      }
      int mirror = 0;
      if (const Json* value = parameters->find("mirror");
          value != nullptr && value->kind == JsonKind::Bool) {
        mirror = value->boolean ? 1 : 0;
      }
      // The parameter vector is big and has no useful name per slot, so it
      // travels in the payload rather than in the JSON: the tire names a
      // descriptor and the reader checks it against that payload.
      if (model_kind != 0) {
        std::vector<double> values;
        const std::string* descriptor = parameters->find_string("blob");
        if (descriptor == nullptr || descriptor->empty()) {
          return fail(error, "tire " + quote(*tire_name) +
                                  " needs a parameter block but names no blob");
        }
        if (!case_detail::read_described_array(document, blob, *descriptor,
                                  VehicleParameterCount, values, error)) {
          return false;
        }
        if (tire_parameters_.empty()) {
          tire_parameters_.assign(tire_names_.size() * VehicleParameterCount, 0.0);
        }
        tire_parameters_.insert(tire_parameters_.end(), values.begin(), values.end());
      }

      // A measured vertical table replaces the stiffness polynomial, so a tire
      // that has one says so by naming its descriptor.
      // The ABI indexes these tables per tire, so every tire must contribute an
      // offset/count pair even when it has no measured curve.  Omitting a pair
      // makes a mixed model look like it has no curves at all, because the
      // registration stage only activates the table when the count vector has
      // exactly tire_count entries.
      tire_deflection_offset_.push_back(
          static_cast<int>(tire_deflection_x_.size()));
      {
        int rows = 0;
        if (const std::string* descriptor =
                parameters->find_string("deflection_curve");
            descriptor != nullptr &&
            !read_described_table(document, blob, *descriptor,
                                  tire_deflection_x_, tire_deflection_y_, rows,
                                  error)) {
          return false;
        }
        tire_deflection_count_.push_back(rows);
      }
      tire_bottoming_offset_.push_back(
          static_cast<int>(tire_bottoming_x_.size()));
      {
        int rows = 0;
        if (const std::string* descriptor =
                parameters->find_string("bottoming_curve");
            descriptor != nullptr &&
            !read_described_table(document, blob, *descriptor,
                                  tire_bottoming_x_, tire_bottoming_y_, rows,
                                  error)) {
          return false;
        }
        tire_bottoming_count_.push_back(rows);
      }

      tire_names_.push_back(*tire_name);
      tire_model_kind_.push_back(model_kind);
      tire_mirror_.push_back(mirror);
      tire_frame_body_.push_back(frame_body);
      for (double value : frame_center) tire_frame_center_.push_back(value * length_scale_);
      tire_drive_body_.push_back(drive_body);
      tire_drive_reaction_.push_back(drive_reaction);
      for (double value : drive_axis) tire_drive_axis_.push_back(value);
      tire_body_.push_back(body_index->second);
      for (double value : center) tire_center_.push_back(value * length_scale_);
      for (double value : spin) tire_spin_axis_.push_back(value);
      for (double value : forward) tire_forward_axis_.push_back(value);
      tire_radius_.push_back(radius * length_scale_);
      tire_maximum_compression_.push_back(maximum_compression * length_scale_);
      tire_stiffness_.push_back(vertical_stiffness * force_per_length_scale());
      tire_damping_.push_back(vertical_damping * force_per_length_scale());
      tire_mu_longitudinal_.push_back(mu_longitudinal);
      tire_mu_lateral_.push_back(mu_lateral);
      tire_brush_longitudinal_.push_back(
          brush_longitudinal * force_per_length_scale());
      tire_brush_lateral_.push_back(brush_lateral * force_per_length_scale());
      tire_relaxation_longitudinal_.push_back(relaxation_longitudinal * length_scale_);
      tire_relaxation_lateral_.push_back(relaxation_lateral * length_scale_);
      tire_detached_relaxation_.push_back(detached_relaxation);
      tire_mass_.push_back(tire_mass);
      for (double entry : tire_inertia) tire_inertia_.push_back(entry);
    }
  }

  // --- coordinate couplers -------------------------------------------------
  // A coupler adds one row per coupler to the constraint system, so it needs the
  // joint indices rather than the joint names the document carries.
  if (const Json* couplers = document.find("couplers");
      couplers != nullptr && couplers->is_array()) {
    std::unordered_map<std::string, int> joint_lookup;
    for (std::size_t index = 0; index < joint_names.size(); ++index) {
      joint_lookup.emplace(joint_names[index], static_cast<int>(index));
    }
    for (const Json& coupler : couplers->items) {
      if (!coupler.is_object()) return fail(error, "a coupler entry is not an object");
      const std::string* name = coupler.find_string("name");
      const std::string* joint_a = coupler.find_string("joint_a");
      const std::string* joint_b = coupler.find_string("joint_b");
      if (name == nullptr || joint_a == nullptr || joint_b == nullptr) {
        return fail(error, "a coupler entry is missing name/joint_a/joint_b");
      }
      const auto index_a = joint_lookup.find(*joint_a);
      const auto index_b = joint_lookup.find(*joint_b);
      if (index_a == joint_lookup.end() || index_b == joint_lookup.end()) {
        return fail(error, "coupler " + quote(*name) + " names an unknown joint");
      }
      const std::string* coordinate_a = coupler.find_string("coordinate_a");
      const std::string* coordinate_b = coupler.find_string("coordinate_b");
      coupler_joint_a_.push_back(index_a->second);
      coupler_joint_b_.push_back(index_b->second);
      coupler_coordinate_a_.push_back(
          (coordinate_a != nullptr && *coordinate_a == "rotation") ? 0 : 1);
      coupler_coordinate_b_.push_back(
          (coordinate_b != nullptr && *coordinate_b == "rotation") ? 0 : 1);
      coupler_scale_a_.push_back(number_or_default(coupler, "scale_a", 0.0));
      coupler_scale_b_.push_back(number_or_default(coupler, "scale_b", 0.0));
    }
  }

  // --- static rotation gauges ----------------------------------------------
  if (const Json* gauges = document.find("gauges");
      gauges != nullptr && gauges->is_array()) {
    for (const Json& gauge : gauges->items) {
      if (!gauge.is_object()) return fail(error, "a gauge entry is not an object");
      const std::string* body = gauge.find_string("body");
      if (body == nullptr) return fail(error, "a gauge entry has no body");
      const auto index = body_lookup.find(*body);
      if (index == body_lookup.end()) {
        return fail(error, "gauge names an unknown body " + quote(*body));
      }
      double axis[3] = {0.0, 0.0, 1.0};
      if (!optional_vec3(gauge, "axis_local", axis, axis)) {
        return fail(error, "gauge on " + quote(*body) + " has a malformed axis");
      }
      gauge_body_.push_back(index->second);
      for (double value : axis) gauge_axis_.push_back(value);
    }
  }

  // --- road ---------------------------------------------------------------
  if (const Json* road = document.find("road"); road != nullptr && road->is_object()) {
    static const std::pair<const char*, int> kRoads[] = {
        {"plane", 1},   {"sine", 2},           {"bump", 3},
        {"random_fourier", 4}, {"four_post", 5},
    };
    if (const std::string* kind = road->find_string("kind"); kind != nullptr) {
      int value = 0;
      for (const auto& entry : kRoads) {
        if (*kind == entry.first) value = entry.second;
      }
      road_kind_ = value;
    }
    if (const Json* parameters = road->find("parameters");
        parameters != nullptr && parameters->is_object()) {
      road_origin_x_ = number_or_default(*parameters, "origin_x", 0.0) * length_scale_;
      road_origin_z_ = number_or_default(*parameters, "origin_z", 0.0) * length_scale_;
      road_amplitude_ = number_or_default(*parameters, "amplitude", 0.0) * length_scale_;
      road_wavelength_ = number_or_default(*parameters, "wavelength", 0.0) * length_scale_;
      road_phase_ = number_or_default(*parameters, "phase", 0.0);
      road_bump_start_ = number_or_default(*parameters, "bump_start", 0.0) * length_scale_;
      road_bump_length_ = number_or_default(*parameters, "bump_length", 0.0) * length_scale_;
      if (const Json* scales = parameters->find("corner_scale");
          scales != nullptr && scales->is_array() && scales->items.size() == 4) {
        for (std::size_t index = 0; index < 4; ++index) {
          road_corner_scale_[index] = scales->items[index].number;
        }
      }
    }
  }

  // --- markers ------------------------------------------------------------
  const Json* marker_items = document.find("markers");
  if (marker_items != nullptr) {
    if (!marker_items->is_array()) return fail(error, "markers must be an array");
    for (const Json& marker : marker_items->items) {
      if (!marker.is_object()) return fail(error, "a marker entry is not an object");
      const std::string* marker_name = marker.find_string("name");
      const std::string* body = marker.find_string("body");
      if (marker_name == nullptr || body == nullptr) {
        return fail(error, "a marker entry is missing name/body");
      }
      const auto index = body_lookup.find(*body);
      if (index == body_lookup.end()) {
        return fail(error, "marker " + quote(*marker_name) + " names an unknown body");
      }
      ContractMarker entry;
      entry.name = *marker_name;
      entry.body = index->second;
      double point[3] = {0.0, 0.0, 0.0};
      if (!optional_vec3(marker, "point", point, point)) {
        return fail(error, "marker " + quote(*marker_name) + " has a malformed point");
      }
      entry.point = Vec3{point[0] * length_scale_, point[1] * length_scale_,
                         point[2] * length_scale_};
      markers_.push_back(entry);
    }
  }

  // --- external load application points -----------------------------------
  // A family whose load is a force at a body-fixed marker says so here, and
  // the kernel then resolves the lever arm at the pose it is solving for.
  // Saying nothing keeps the historical convention -- the wrench acts at the
  // body origin -- which is what every dynamics family and the legacy entry
  // point want, so this table stays empty for them and nothing changes.
  if (const Json* application = document.find("body_wrench_markers");
      application != nullptr) {
    if (!application->is_array()) {
      return fail(error, "body_wrench_markers must be an array of marker names");
    }
    body_wrench_point_.assign(body_names_.size(), Vec3{});
    body_wrench_declared_.assign(body_names_.size(), 0);
    for (const Json& value : application->items) {
      if (value.kind != JsonKind::String) {
        return fail(error, "body_wrench_markers must contain marker names");
      }
      bool found = false;
      for (const ContractMarker& marker : markers_) {
        if (marker.name != value.text) continue;
        body_wrench_point_[static_cast<std::size_t>(marker.body)] = marker.point;
        body_wrench_declared_[static_cast<std::size_t>(marker.body)] = 1;
        found = true;
      }
      if (!found) {
        return fail(error,
                    "body_wrench_markers names unknown marker " + quote(value.text));
      }
    }
  }

  // The separation of each driven coordinate *at the assembling pose*.  A case
  // document says "wheel travel +10 mm", not "an absolute separation of
  // 0.311 m": turning the former into the latter is geometry, and the geometry
  // is in this document, so it is this side's job.  It is also what makes the
  // neutral case a zero-residual state, which is what the C family measures its
  // deformations against.
  const std::size_t driven_total = driven_names_.size();
  driven_separation_.assign(driven_total, 0.0);
  for (std::size_t index = 0; index < driven_total; ++index) {
    const std::size_t a = static_cast<std::size_t>(driven_body_[index]);
    const std::size_t b = static_cast<std::size_t>(driven_reaction_body_[index]);
    double rotation_a[9];
    double rotation_b[9];
    quaternion_to_rotation(&body_pose_[a * 7 + 3], rotation_a);
    quaternion_to_rotation(&body_pose_[b * 7 + 3], rotation_b);
    double separation = 0.0;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      double point_a = body_pose_[a * 7 + axis];
      double point_b = body_pose_[b * 7 + axis];
      for (std::size_t column = 0; column < 3; ++column) {
        point_a += rotation_a[axis * 3 + column] * driven_point_[index * 3 + column];
        point_b += rotation_b[axis * 3 + column] *
                   driven_reaction_point_[index * 3 + column];
      }
      double axis_world = 0.0;
      for (std::size_t column = 0; column < 3; ++column) {
        axis_world += rotation_b[axis * 3 + column] * driven_axis_[index * 3 + column];
      }
      separation += (point_a - point_b) * axis_world;
    }
    driven_separation_[index] = separation;
  }

  // A zero-length vector may legally have a null `data()`, and the spring
  // registration path tests pointers rather than counts.  The fallbacks are
  // built once, here, so `fill` only ever publishes addresses.
  const std::size_t spring_slots = spring_body_a_.empty() ? 1 : spring_body_a_.size();
  spring_fallback_minimum_.assign(spring_slots, std::numeric_limits<double>::quiet_NaN());
  spring_fallback_maximum_.assign(spring_slots, std::numeric_limits<double>::quiet_NaN());
  spring_fallback_offset_.assign(spring_slots, 0);
  spring_fallback_count_.assign(spring_slots, 0);

  if (body_names_.size() != body_fixed_.size()) {
    return fail(error, "internal error: body tables disagree");
  }
  return true;
}

double ContractModel::driven_separation(std::size_t index) const {
  return index < driven_separation_.size() ? driven_separation_[index] : 0.0;
}

bool ContractModel::driven_is_rotation(std::size_t index) const {
  return index < driven_type_.size() &&
         driven_type_[index] == static_cast<int>(AXLE_DRIVEN_ROTATION);
}

int ContractModel::body_index(const std::string& name) const {
  for (std::size_t index = 0; index < body_names_.size(); ++index) {
    if (body_names_[index] == name) return static_cast<int>(index);
  }
  return -1;
}

int ContractModel::tire_index(const std::string& name) const {
  for (std::size_t index = 0; index < tire_names_.size(); ++index) {
    if (tire_names_[index] == name) return static_cast<int>(index);
  }
  return -1;
}

int ContractModel::steering_index(const std::string& name) const {
  for (std::size_t index = 0; index < steering_names_.size(); ++index) {
    if (steering_names_[index] == name) return static_cast<int>(index);
  }
  return -1;
}

int ContractModel::driven_index(const std::string& name) const {
  for (std::size_t index = 0; index < driven_names_.size(); ++index) {
    if (driven_names_[index] == name) return static_cast<int>(index);
  }
  return -1;
}

bool ContractModel::find_marker(const std::string& name, ContractMarker& out) const {
  for (const ContractMarker& marker : markers_) {
    if (marker.name == name) {
      out = marker;
      return true;
    }
  }
  return false;
}

/// Install the tire-own mass and inertia a model document declared.
///
/// Lives in this translation unit because `ContractModel` is the object that
/// holds the parsed declaration; `mb_cases/functions.hpp` forward declares
/// `Model` so the declaration costs no new header edge.
bool install_tire_mass(const ContractModel& model, Model& built, std::string& error) {
  const std::vector<double>& masses = model.tire_masses();
  const std::vector<double>& inertias = model.tire_inertias();
  if (masses.empty() && inertias.empty()) {
    // A document that declares neither field.  The reader still produces dense
    // tables (one zero per tire), so this branch only fires for a model with no
    // tires at all; either way there is nothing to install and the wheel-end
    // body keeps owning the inertia, which is the historical answer.
    return true;
  }
  if (masses.size() != built.tires.size() ||
      inertias.size() != built.tires.size() * 9) {
    // Truncating here would leave some tires owning their inertia and others
    // not, and the resulting mass error would not be attributable to this line.
    error = "tire mass declaration does not match the built tires: " +
            std::to_string(masses.size()) + " masses and " +
            std::to_string(inertias.size() / 9) + " inertias for " +
            std::to_string(built.tires.size()) + " tires";
    return false;
  }
  for (std::size_t index = 0; index < built.tires.size(); ++index) {
    built.tires[index].mass = masses[index];
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        built.tires[index].inertia.a[row][column] =
            inertias[index * 9 + row * 3 + column];
      }
    }
  }
  return true;
}


void ContractModel::fill(VehicleInput& input) const {
  AxleInput& axle = input.axle;
  axle.body_count = body_names_.size();
  axle.body_mass = body_mass_.data();
  axle.body_inertia_body_3x3 = body_inertia_.data();
  axle.body_pose_position_quaternion = body_pose_.data();
  axle.body_velocity_omega = body_velocity_.data();
  axle.body_fixed = body_fixed_.data();

  axle.constraint_count = constraint_type_.size();
  axle.constraint_type = constraint_type_.data();
  axle.constraint_body_a = constraint_body_a_.data();
  axle.constraint_body_b = constraint_body_b_.data();
  axle.constraint_point_a = constraint_point_a_.data();
  axle.constraint_point_b = constraint_point_b_.data();
  axle.constraint_axis_a = constraint_axis_a_.data();
  axle.constraint_axis_b = constraint_axis_b_.data();
  input.constraint_axis_a_secondary = constraint_axis_a_secondary_.data();
  input.constraint_axis_b_secondary = constraint_axis_b_secondary_.data();
  input.constraint_convel_angle_target = constraint_convel_angle_target_.data();

  axle.spring_count = spring_body_a_.size();
  // A spring's optional length limits and its damper curve are indexed per
  // spring, so those tables are always at least one element long even when the
  // model has no springs: the registration path tests the pointer, not the
  // count, and a zero-length vector is allowed to have a null one.
  axle.spring_body_a = spring_body_a_.data();
  axle.spring_body_b = spring_body_b_.data();
  axle.spring_point_a = spring_point_a_.data();
  axle.spring_point_b = spring_point_b_.data();
  axle.spring_stiffness = spring_stiffness_.data();
  axle.spring_compression_damping = spring_compression_damping_.data();
  axle.spring_rebound_damping = spring_rebound_damping_.data();
  axle.spring_free_length = spring_free_length_.data();
  axle.spring_minimum_length =
      spring_body_a_.empty() ? spring_fallback_minimum_.data() : spring_minimum_length_.data();
  axle.spring_maximum_length =
      spring_body_a_.empty() ? spring_fallback_maximum_.data() : spring_maximum_length_.data();
  axle.spring_compression_stop_stiffness = spring_compression_stop_stiffness_.data();
  axle.spring_compression_stop_damping = spring_compression_stop_damping_.data();
  axle.spring_rebound_stop_stiffness = spring_rebound_stop_stiffness_.data();
  axle.spring_rebound_stop_damping = spring_rebound_stop_damping_.data();
  axle.spring_damper_curve_offset =
      spring_body_a_.empty() ? spring_fallback_offset_.data()
                             : spring_damper_curve_offset_.data();
  axle.spring_damper_curve_count =
      spring_body_a_.empty() ? spring_fallback_count_.data()
                             : spring_damper_curve_count_.data();
  axle.spring_damper_curve_velocity =
      spring_damper_curve_velocity_.empty() ? spring_fallback_minimum_.data()
                                            : spring_damper_curve_velocity_.data();
  axle.spring_damper_curve_force =
      spring_damper_curve_force_.empty() ? spring_fallback_minimum_.data()
                                         : spring_damper_curve_force_.data();

  input.vehicle_spring_elastic_curve_offset =
      spring_elastic_curve_offset_.empty()
          ? spring_fallback_offset_.data()
          : spring_elastic_curve_offset_.data();
  input.vehicle_spring_elastic_curve_count =
      spring_elastic_curve_count_.empty()
          ? spring_fallback_count_.data()
          : spring_elastic_curve_count_.data();
  input.vehicle_spring_elastic_curve_deflection =
      spring_elastic_curve_deflection_.empty()
          ? spring_fallback_minimum_.data()
          : spring_elastic_curve_deflection_.data();
  input.vehicle_spring_elastic_curve_force =
      spring_elastic_curve_force_.empty()
          ? spring_fallback_minimum_.data()
          : spring_elastic_curve_force_.data();
  input.vehicle_spring_compression_stop_curve_offset =
      spring_compression_stop_curve_offset_.empty()
          ? spring_fallback_offset_.data()
          : spring_compression_stop_curve_offset_.data();
  input.vehicle_spring_compression_stop_curve_count =
      spring_compression_stop_curve_count_.empty()
          ? spring_fallback_count_.data()
          : spring_compression_stop_curve_count_.data();
  input.vehicle_spring_compression_stop_curve_penetration =
      spring_compression_stop_curve_penetration_.empty()
          ? spring_fallback_minimum_.data()
          : spring_compression_stop_curve_penetration_.data();
  input.vehicle_spring_compression_stop_curve_force =
      spring_compression_stop_curve_force_.empty()
          ? spring_fallback_minimum_.data()
          : spring_compression_stop_curve_force_.data();
  input.vehicle_spring_rebound_stop_curve_offset =
      spring_rebound_stop_curve_offset_.empty()
          ? spring_fallback_offset_.data()
          : spring_rebound_stop_curve_offset_.data();
  input.vehicle_spring_rebound_stop_curve_count =
      spring_rebound_stop_curve_count_.empty()
          ? spring_fallback_count_.data()
          : spring_rebound_stop_curve_count_.data();
  input.vehicle_spring_rebound_stop_curve_penetration =
      spring_rebound_stop_curve_penetration_.empty()
          ? spring_fallback_minimum_.data()
          : spring_rebound_stop_curve_penetration_.data();
  input.vehicle_spring_rebound_stop_curve_force =
      spring_rebound_stop_curve_force_.empty()
          ? spring_fallback_minimum_.data()
          : spring_rebound_stop_curve_force_.data();

  axle.tire_count = tire_names_.size();
  axle.tire_body = tire_body_.data();
  axle.tire_center_local = tire_center_.data();
  axle.tire_spin_axis_local = tire_spin_axis_.data();
  axle.tire_forward_axis_local = tire_forward_axis_.data();
  axle.tire_radius = tire_radius_.data();
  axle.tire_maximum_compression = tire_maximum_compression_.data();
  axle.tire_stiffness = tire_stiffness_.data();
  axle.tire_damping = tire_damping_.data();
  axle.tire_mu_longitudinal = tire_mu_longitudinal_.data();
  axle.tire_mu_lateral = tire_mu_lateral_.data();
  axle.tire_brush_stiffness_longitudinal = tire_brush_longitudinal_.data();
  axle.tire_brush_stiffness_lateral = tire_brush_lateral_.data();
  axle.tire_relaxation_length_longitudinal = tire_relaxation_longitudinal_.data();
  axle.tire_relaxation_length_lateral = tire_relaxation_lateral_.data();
  axle.tire_detached_relaxation = tire_detached_relaxation_.data();

  axle.bushing_count = bushing_body_a_.size();
  axle.bushing_body_a = bushing_body_a_.data();
  axle.bushing_body_b = bushing_body_b_.data();
  axle.bushing_point_a = bushing_point_a_.data();
  axle.bushing_point_b = bushing_point_b_.data();
  axle.bushing_frame_a_quaternion = bushing_frame_a_.data();
  axle.bushing_frame_b_quaternion = bushing_frame_b_.data();
  axle.bushing_reference_translation = bushing_reference_translation_.data();
  axle.bushing_reference_quaternion = bushing_reference_quaternion_.data();
  axle.bushing_stiffness_6x6 = bushing_stiffness_.data();
  axle.bushing_damping_6x6 = bushing_damping_.data();
  axle.bushing_preload_6 = bushing_preload_.data();

  axle.gravity_x = gravity_[0];
  axle.gravity_y = gravity_[1];
  axle.gravity_z = gravity_[2];
  axle.rho_inf = 0.8;

  axle.anti_roll_bar_count = anti_roll_body_a_.size();
  if (!anti_roll_body_a_.empty()) {
    axle.anti_roll_body_a = anti_roll_body_a_.data();
    axle.anti_roll_body_b = anti_roll_body_b_.data();
    axle.anti_roll_axis_a = anti_roll_axis_a_.data();
    axle.anti_roll_reference_quaternion = anti_roll_reference_.data();
    axle.anti_roll_stiffness = anti_roll_stiffness_.data();
    axle.anti_roll_damping = anti_roll_damping_.data();
  }

  input.aerodynamic_drag_count = aero_body_.size();
  if (!aero_body_.empty()) {
    input.aerodynamic_drag_body = aero_body_.data();
    input.aerodynamic_drag_application_point = aero_point_.data();
    input.aerodynamic_drag_forward_axis = aero_axis_.data();
    input.aerodynamic_drag_coefficient = aero_coefficient_.data();
  }

  input.steering_count = steering_names_.size();
  if (!steering_names_.empty()) {
    input.steering_type = steering_type_.data();
    input.steering_body = steering_body_.data();
    input.steering_reaction_body = steering_reaction_body_.data();
    input.steering_point_local = steering_point_.data();
    input.steering_reaction_point_local = steering_reaction_point_.data();
    input.steering_axis_local = steering_axis_.data();
    input.steering_reference_quaternion = steering_reference_.data();
    input.steering_stiffness = steering_stiffness_.data();
    input.steering_damping = steering_damping_.data();
  }

  input.road_kind = road_kind_;
  input.road_origin_x = road_origin_x_;
  input.road_origin_z = road_origin_z_;
  input.road_amplitude = road_amplitude_;
  input.road_wavelength = road_wavelength_;
  input.road_phase = road_phase_;
  input.road_bump_start = road_bump_start_;
  input.road_bump_length = road_bump_length_;
  input.road_corner_scale = road_corner_scale_;

  if (!tire_frame_body_.empty()) {
    // The vertical curves are optional per tire, so the caller's zero-filled
    // offset/count tables stay in place unless the model carries real ones.
    if (tire_deflection_count_.size() == tire_frame_body_.size()) {
      input.tire_deflection_curve_offset = tire_deflection_offset_.data();
      input.tire_deflection_curve_count = tire_deflection_count_.data();
      input.tire_deflection_curve_deflection = tire_deflection_x_.data();
      input.tire_deflection_curve_force = tire_deflection_y_.data();
    }
    if (tire_bottoming_count_.size() == tire_frame_body_.size()) {
      input.tire_bottoming_curve_offset = tire_bottoming_offset_.data();
      input.tire_bottoming_curve_count = tire_bottoming_count_.data();
      input.tire_bottoming_curve_penetration = tire_bottoming_x_.data();
      input.tire_bottoming_curve_force = tire_bottoming_y_.data();
    }
    input.tire_model_kind = tire_model_kind_.data();
    input.tire_pac2002_mirror = tire_mirror_.data();
    if (!tire_parameters_.empty()) {
      input.tire_pac2002_parameters = tire_parameters_.data();
    }
    input.tire_frame_body = tire_frame_body_.data();
    input.tire_frame_center_local = tire_frame_center_.data();
    input.tire_drive_torque_body = tire_drive_body_.data();
    input.tire_drive_torque_reaction_body = tire_drive_reaction_.data();
    input.tire_drive_torque_axis_local = tire_drive_axis_.data();
  }

  input.coordinate_coupler_count = coupler_joint_a_.size();
  if (!coupler_joint_a_.empty()) {
    input.coordinate_coupler_joint_a = coupler_joint_a_.data();
    input.coordinate_coupler_coordinate_a = coupler_coordinate_a_.data();
    input.coordinate_coupler_scale_a = coupler_scale_a_.data();
    input.coordinate_coupler_joint_b = coupler_joint_b_.data();
    input.coordinate_coupler_coordinate_b = coupler_coordinate_b_.data();
    input.coordinate_coupler_scale_b = coupler_scale_b_.data();
  }

  input.static_rotation_gauge_count = gauge_body_.size();
  if (!gauge_body_.empty()) {
    input.static_rotation_gauge_body = gauge_body_.data();
    input.static_rotation_gauge_axis_local = gauge_axis_.data();
  }

  input.driven_count = driven_names_.size();
  input.driven_type = driven_type_.data();
  input.driven_body = driven_body_.data();
  input.driven_reaction_body = driven_reaction_body_.data();
  input.driven_point_local = driven_point_.data();
  input.driven_reaction_point_local = driven_reaction_point_.data();
  input.driven_axis_local = driven_axis_.data();
  input.driven_reference_quaternion = driven_reference_.data();
}

void ContractModel::fill_case(const ContractCase& run, VehicleInput& input) const {
  // Only the per-sample tables the solver reads *through the input structure*
  // are set here.  The driven targets are deliberately absent: the registration
  // stage stores the pointer it was given, so every case has to write into one
  // buffer whose address never changes.  The caller owns that buffer.
  input.axle.sample_count = run.sample_count;
  input.axle.body_wrench = run.body_wrench.empty() ? nullptr : run.body_wrench.data();
  input.axle.road_z = run.road_z.empty() ? nullptr : run.road_z.data();
  input.axle.road_z_velocity =
      run.road_velocity.empty() ? nullptr : run.road_velocity.data();
  input.axle.wheel_torque = run.wheel_torque.empty() ? nullptr : run.wheel_torque.data();
  input.steering_target_angle =
      run.steering_target.empty() ? nullptr : run.steering_target.data();
  input.steering_target_rate =
      run.steering_rate.empty() ? nullptr : run.steering_rate.data();
  input.brake_torque = run.brake_torque.empty() ? nullptr : run.brake_torque.data();
  input.initial_state_angle_tolerance = run.initial_state_angle_tolerance;
  if (run.has_static_gauge) {
    input.static_gauge_body = run.static_gauge_body;
    input.static_gauge_dof_mask = run.static_gauge_dof_mask;
    input.static_trim_then_release = run.static_trim_then_release ? 1 : 0;
  }
}

}  // namespace axle_kernel
