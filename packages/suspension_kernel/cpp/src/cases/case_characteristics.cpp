/// The characteristics request reader.
///
/// A characteristics request asks the one physical path for extra derived
/// numbers.  It is not a study, not a case family and not a second entry point,
/// so the reader's whole job is to decode inert data without inventing any: it
/// never defaults a missing field and never ignores one it does not know.
///
/// That strictness is the point.  The contract reader used to look fields up by
/// name and drop everything else, which turns a misspelled field into a
/// *different* physical question that still runs.  A request names a domain, a
/// unit and a response kind, so silently dropping one of them would answer
/// something the caller did not ask for and report success.

#include "case_common.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace axle_kernel {
namespace {

using case_detail::fail;

constexpr int kRequestVersion = 1;

/// Every field this reader knows at each level, so the unknown-field check is
/// derived from the same list the reader consumes rather than kept beside it.
const std::vector<std::string>& request_fields() {
  static const std::vector<std::string> fields = {"schema_version",
                                                 "characteristics"};
  return fields;
}

const std::vector<std::string>& body_fields() {
  static const std::vector<std::string> fields = {
      "request_id",        "domain",      "samples",        "measurements",
      "perturbations",     "boundary",    "definition",     "frozen_conditions",
      "response_operator", "wheel_end_response"};
  return fields;
}

const std::vector<std::string>& declaration_fields() {
  static const std::vector<std::string> fields = {
      "id", "measurement_id", "definition_version", "units",
      "response_kind", "response_version", "mode"};
  return fields;
}

const std::vector<std::string>& boundary_fields() {
  static const std::vector<std::string> fields = {"profile", "version",
                                                  "released", "held"};
  return fields;
}

const std::vector<std::string>& definition_fields() {
  static const std::vector<std::string> fields = {"kind", "version"};
  return fields;
}

const std::vector<std::string>& frozen_fields() {
  static const std::vector<std::string> fields = {"protocol", "version"};
  return fields;
}

const std::vector<std::string>& operator_fields() {
  static const std::vector<std::string> fields = {"kind", "version"};
  return fields;
}

/// Units are opaque tokens checked against an explicit list.  A free-form
/// string would let two incompatible requests look compatible.
bool known_unit(const std::string& value) {
  static const std::vector<std::string> units = {
      "1",     "rad",   "deg",   "m",      "mm",     "s",      "kg",
      "N",     "N*m",   "N/m",   "N*m/rad", "N*m/m",  "N*m*s",  "rad/m",
      "rad/s", "rad/N", "rad/(N*m)", "m/m", "m/s",   "m/N",    "m/(N*m)",
      "N*m*s/rad", "percent"};
  return std::find(units.begin(), units.end(), value) != units.end();
}

/// Named response kinds, kept apart from their versions: `roll_center` exists
/// at version 1 (legacy) and version 2 (force-equivalent).
bool known_response_kind(const std::string& value) {
  static const std::vector<std::string> kinds = {
      "measurement_value",     "measurement_derivative", "time_derivative",
      "frozen_force_tangent",  "frozen_load_compliance", "local_response",
      "matrix_view",           "load_compliance",        "wheel_rate",
      "ride_rate",             "roll_rate",              "component_rate",
      "grouped_rate",          "roll_center",            "instant_center",
      "swing_arm",             "pitch_center",           "screw_axis",
      "ackermann"};
  return std::find(kinds.begin(), kinds.end(), value) != kinds.end();
}

bool known_domain(const std::string& value) {
  return value == "kinematic" || value == "equilibrium" ||
         value == "trajectory" || value == "dynamic";
}

bool known_dynamic_kind(const std::string& value) {
  return value == "frozen_force_tangent" ||
         value == "frozen_load_compliance" || value == "continuous_descriptor";
}

bool known_request_tokens(const std::string& value) {
  static const std::string allowed =
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-*{};";
  if (value.empty()) return false;
  return value.find_first_not_of(allowed) == std::string::npos;
}

/// Reject any field the reader does not consume, naming all of them at once so
/// a typo is reported as a typo instead of being silently dropped.
bool check_fields(const JsonValue& object, const std::vector<std::string>& known,
                  const std::string& path, std::string& error) {
  std::vector<std::string> unknown;
  for (const auto& field : object.fields) {
    if (std::find(known.begin(), known.end(), field.first) == known.end()) {
      unknown.push_back(field.first);
    }
  }
  if (unknown.empty()) return true;
  std::string joined;
  for (std::size_t index = 0; index < unknown.size(); ++index) {
    if (index != 0) joined += ", ";
    joined += "\"" + unknown[index] + "\"";
  }
  return fail(error, path + " has unknown field(s) " + joined);
}

bool require_positive_int(const JsonValue& object, const std::string& key,
                          const std::string& path, int& out, std::string& error) {
  const JsonValue* value = object.find(key);
  if (value == nullptr || value->kind != JsonKind::Number ||
      !value->number_is_integer || value->integer < 1 ||
      value->integer > 2147483647LL) {
    return fail(error, path + " requires a positive integer " + key);
  }
  out = static_cast<int>(value->integer);
  return true;
}

bool require_text(const JsonValue& object, const std::string& key,
                  const std::string& path, std::string& out, std::string& error) {
  const std::string* value = object.find_string(key);
  if (value == nullptr || value->empty()) {
    return fail(error, path + " requires a non-empty " + key);
  }
  out = *value;
  return true;
}

bool read_declaration(const JsonValue& entry, const std::string& path,
                      ContractDeclaration& out, std::string& error) {
  if (!entry.is_object()) return fail(error, path + " must be an object");
  if (!check_fields(entry, declaration_fields(), path, error)) return false;
  if (!require_text(entry, "measurement_id", path, out.measurement_id, error)) {
    return false;
  }
  if (!known_request_tokens(out.measurement_id)) {
    return fail(error, path + " id \"" + out.measurement_id +
                            "\" has an unsupported character");
  }
  // A wildcard declares a family; it is never a runtime pattern match, because
  // recovering a metric from an entity name is exactly what the contract bans.
  const std::size_t wildcard = out.measurement_id.find('*');
  if (wildcard != std::string::npos &&
      wildcard + 1 != out.measurement_id.size() &&
      out.measurement_id.back() != '}') {
    return fail(error, path + " id \"" + out.measurement_id +
                            "\" uses a wildcard outside a final segment");
  }
  if (!require_positive_int(entry, "definition_version", path,
                            out.definition_version, error)) {
    return false;
  }
  if (!require_text(entry, "units", path, out.units, error)) return false;
  if (!known_unit(out.units)) {
    return fail(error, path + " declares unsupported unit \"" + out.units + "\"");
  }
  if (!require_text(entry, "response_kind", path, out.response_kind, error)) {
    return false;
  }
  if (!known_response_kind(out.response_kind)) {
    return fail(error, path + " declares unknown response_kind \"" +
                            out.response_kind + "\"");
  }
  if (!require_positive_int(entry, "response_version", path,
                            out.response_version, error)) {
    return false;
  }
  const JsonValue* identifier = entry.find("id");
  if (identifier != nullptr) {
    if (identifier->kind != JsonKind::String || identifier->text.empty()) {
      return fail(error, path + " id must be a non-empty string");
    }
    out.id = identifier->text;
  }
  const std::string* mode = entry.find_string("mode");
  if (mode != nullptr) out.mode = *mode;
  return true;
}

}  // namespace

bool contract_read_characteristics(const JsonValue& document,
                                   ContractCharacteristics& out,
                                   std::string& error) {
  out = ContractCharacteristics{};
  const JsonValue* request = document.find("characteristics");
  if (request == nullptr) return true;
  if (!request->is_object()) {
    return fail(error, "case document characteristics must be an object");
  }
  if (!check_fields(*request, request_fields(), "characteristics", error)) {
    return false;
  }
  const JsonValue* version = request->find("schema_version");
  if (version == nullptr || version->kind != JsonKind::Number ||
      !version->number_is_integer || version->integer != kRequestVersion) {
    return fail(error, "characteristics schema_version must be 1");
  }
  const JsonValue* body = request->find("characteristics");
  if (body == nullptr || !body->is_object()) {
    return fail(error, "characteristics request must be an object");
  }
  if (!check_fields(*body, body_fields(), "characteristics request", error)) {
    return false;
  }
  out.present = true;
  if (!require_text(*body, "request_id", "characteristics request",
                    out.request_id, error)) {
    return false;
  }
  if (!require_text(*body, "domain", "characteristics request", out.domain,
                    error)) {
    return false;
  }
  if (!known_domain(out.domain)) {
    return fail(error, "characteristics request has unknown domain \"" +
                           out.domain + "\"");
  }

  const JsonValue* samples = body->find("samples");
  if (samples == nullptr || !samples->is_array() || samples->items.empty()) {
    return fail(error, "characteristics request samples must be a non-empty array");
  }
  for (const JsonValue& item : samples->items) {
    if (item.kind != JsonKind::Number || !item.number_is_integer ||
        item.integer < 0 || item.integer > 2147483647LL) {
      return fail(error,
                  "characteristics request samples must be non-negative integers");
    }
    const int index = static_cast<int>(item.integer);
    if (std::find(out.samples.begin(), out.samples.end(), index) !=
        out.samples.end()) {
      return fail(error, "characteristics request has duplicate sample index " +
                             std::to_string(index));
    }
    out.samples.push_back(index);
  }

  const JsonValue* measurements = body->find("measurements");
  if (measurements == nullptr || !measurements->is_array() ||
      measurements->items.empty()) {
    return fail(error,
                "characteristics request needs at least one measurement");
  }
  for (std::size_t index = 0; index < measurements->items.size(); ++index) {
    ContractDeclaration declaration;
    if (!read_declaration(measurements->items[index],
                          "characteristics measurement[" +
                              std::to_string(index) + "]",
                          declaration, error)) {
      return false;
    }
    out.measurements.push_back(std::move(declaration));
  }

  if (const JsonValue* perturbations = body->find("perturbations");
      perturbations != nullptr) {
    if (!perturbations->is_array() || perturbations->items.empty()) {
      return fail(error,
                  "characteristics request perturbations must be a non-empty array");
    }
    for (std::size_t index = 0; index < perturbations->items.size(); ++index) {
      ContractDeclaration declaration;
      if (!read_declaration(perturbations->items[index],
                            "characteristics perturbation[" +
                                std::to_string(index) + "]",
                            declaration, error)) {
        return false;
      }
      out.perturbations.push_back(std::move(declaration));
    }
  }

  const JsonValue* boundary = body->find("boundary");
  if (boundary == nullptr || !boundary->is_object()) {
    return fail(error, "characteristics request boundary must be an object");
  }
  if (!check_fields(*boundary, boundary_fields(),
                    "characteristics request boundary", error)) {
    return false;
  }
  if (!require_text(*boundary, "profile", "characteristics request boundary",
                    out.boundary_profile, error) ||
      !require_positive_int(*boundary, "version",
                            "characteristics request boundary",
                            out.boundary_version, error)) {
    return false;
  }

  const JsonValue* definition = body->find("definition");
  if (definition == nullptr || !definition->is_object()) {
    return fail(error, "characteristics request definition must be an object");
  }
  if (!check_fields(*definition, definition_fields(),
                    "characteristics request definition", error)) {
    return false;
  }
  if (!require_text(*definition, "kind", "characteristics request definition",
                    out.definition_kind, error) ||
      !require_positive_int(*definition, "version",
                            "characteristics request definition",
                            out.definition_version, error)) {
    return false;
  }
  // Dynamic definitions are named separately and accepted separately: a frozen
  // tangent is not a continuous descriptor and neither replaces the other.
  if (out.domain == "dynamic" && !known_dynamic_kind(out.definition_kind)) {
    return fail(error, "characteristics request dynamic definition kind \"" +
                           out.definition_kind + "\" is not a dynamic definition");
  }
  if (known_dynamic_kind(out.definition_kind) && out.domain != "dynamic") {
    return fail(error, "characteristics request definition kind \"" +
                           out.definition_kind + "\" is dynamic-only");
  }

  if (const JsonValue* frozen = body->find("frozen_conditions");
      frozen != nullptr) {
    if (!frozen->is_object()) {
      return fail(error,
                  "characteristics request frozen_conditions must be an object");
    }
    if (!check_fields(*frozen, frozen_fields(),
                      "characteristics request frozen_conditions", error)) {
      return false;
    }
    if (!require_text(*frozen, "protocol",
                      "characteristics request frozen_conditions",
                      out.frozen_protocol, error) ||
        !require_positive_int(*frozen, "version",
                              "characteristics request frozen_conditions",
                              out.frozen_version, error)) {
      return false;
    }
  } else if (out.definition_kind == "frozen_force_tangent" ||
             out.definition_kind == "frozen_load_compliance") {
    // A frozen definition without its protocol would let the kernel pick one,
    // which is how a frozen response stops meaning a known frozen response.
    return fail(error, "characteristics request definition \"" +
                           out.definition_kind +
                           "\" requires frozen_conditions");
  }

  const JsonValue* response_operator = body->find("response_operator");
  if (response_operator == nullptr || !response_operator->is_object()) {
    return fail(error,
                "characteristics request response_operator must be an object");
  }
  if (!check_fields(*response_operator, operator_fields(),
                    "characteristics request response_operator", error)) {
    return false;
  }
  if (!require_text(*response_operator, "kind",
                    "characteristics request response_operator",
                    out.operator_kind, error) ||
      !require_positive_int(*response_operator, "version",
                            "characteristics request response_operator",
                            out.operator_version, error)) {
    return false;
  }

  if (const JsonValue* wheel_end = body->find("wheel_end_response");
      wheel_end != nullptr && !wheel_end->is_object()) {
    return fail(error,
                "characteristics request wheel_end_response must be an object");
  }
  return true;
}

}  // namespace axle_kernel
