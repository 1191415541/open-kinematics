/// The case-family dispatcher.
///
/// One table decides everything asked about a family: whether the protocol knows
/// the name, whether this build implements it, and which function expands it.
/// Before the table those three facts lived in three places -- `mb_contract`'s
/// registry, a hand-written `||` chain, and an `if` ladder -- and nothing checked
/// that they agreed.  A family declared and unimplemented was refused only by the
/// accident of two lists differing; one implemented but left off a list was
/// silently reported as missing.
///
/// The two failure modes are kept distinct on purpose:
///
///   * a name the protocol does not define is an **unknown family**;
///   * a name the protocol defines but this build cannot run is **not implemented
///     by this build**.
///
/// Collapsing them would tell a user their typo and their missing feature are the
/// same problem.  `comparison` is the live example of the second kind: the
/// contract names it, and it is a per-target gate rather than a solve, so no
/// expander exists for it.

#include "case_common.hpp"

#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace axle_kernel {
namespace {

using axle_kernel::ContractPlan;
using case_detail::fail;

/// How well a family name is known to this build.
enum class Support {
  kSupported,
  kKnownUnimplemented,
  kNotInProtocol,
};
/// One expander's signature, matching the per-family functions.
using Expander = bool (*)(const JsonValue& document, const std::string& blob,
                          const ContractModel& model, ContractPlan& out,
                          std::string& error);
/// One family: its name, which failure applies, how it expands, and whether a
/// document of that family may declare its cases as a `runs` list.
///
/// `expander` is non-null exactly when the family is supported, so "can we run
/// it" and "which function runs it" cannot disagree.  `in_protocol` separates the
/// two failure modes above.  `reads_runs` is the fourth fact a caller needs: a
/// grid family names its cases from its own sweep, so a `runs` list on one is
/// refused instead of being expanded into colliding names.
struct FamilyRow {
  const char* name;
  bool in_protocol;
  Expander expander;
  bool reads_runs;
};

/// Every name the contract defines, with the handler when there is one.
///
/// This table is the single source of truth for the case layer.  The contract
/// module keeps its own list of names -- it must, because the protocol half is
/// allowed to know a family that no build implements -- and the self-test asserts
/// the two agree on the `in_protocol` column, so a protocol name that no row
/// mentions is a test failure rather than a silent omission.
/// One expander's signature, uniform across the table.
///
/// Two families ignore the blob (`kc_quasi_static`, `vehicle_kc`: a grid has no
/// sampled tables), so they are reached through thin adapters below.  The table
/// stays uniform on purpose -- a table whose rows had different types could not be
/// searched, and the adapters are where the difference is visible instead of
/// spread across the dispatch.
bool expand_kc_without_blob(const JsonValue& document, const std::string& blob,
                            const ContractModel& model, ContractPlan& out,
                            std::string& error) {
  (void)blob;
  return case_detail::expand_kc_quasi_static(document, model, out, error);
}

bool expand_vehicle_kc_without_blob(const JsonValue& document,
                                    const std::string& blob,
                                    const ContractModel& model, ContractPlan& out,
                                    std::string& error) {
  (void)blob;
  return case_detail::expand_vehicle_kc(document, model, out, error);
}
const FamilyRow kFamilies[] = {
    /// The grid families take no blob, so they arrive through the adapters above.
    /// They also expand their own case grid, so no `runs` list reaches them.
    {"kc_quasi_static", true, &expand_kc_without_blob, false},
    {"vehicle_kc", true, &expand_vehicle_kc_without_blob, false},
    /// A time-history family runs one excitation per case, so it may be handed
    /// one document per run through `runs`.
    {"axle_dynamic", true, &case_detail::expand_axle_dynamic, true},
    {"vehicle_dynamic", true, &case_detail::expand_vehicle_dynamic, true},
    {"handling", true, &case_detail::expand_handling, true},
    {"ride_four_post", true, &case_detail::expand_ride_four_post, true},
    {"ride_random_road", true, &case_detail::expand_ride_random_road, true},
    // Declared by the protocol and deliberately without a handler: `comparison`
    // is a per-target gate, not a solve.  The row keeps the name visible so the
    // refusal can say *why* it is refused.
    {"comparison", true, nullptr, false},
};

const FamilyRow* find_row(const std::string& family) {
  for (const FamilyRow& row : kFamilies) {
    if (family == row.name) {
      return &row;
    }
  }
  return nullptr;
}

Support support_of(const std::string& family) {
  const FamilyRow* row = find_row(family);
  if (row == nullptr) {
    return Support::kNotInProtocol;
  }
  return row->expander != nullptr ? Support::kSupported
                                  : Support::kKnownUnimplemented;
}

JsonValue json_text(const std::string& value) {
  JsonValue out;
  out.kind = JsonKind::String;
  out.text = value;
  return out;
}

/// The field named `key`, or null when the object has none.
JsonValue* field_of(JsonValue& object, const std::string& key) {
  for (auto& field : object.fields) {
    if (field.first == key) return &field.second;
  }
  return nullptr;
}

/// Insert or replace one field of `object`.
void set_field(JsonValue& object, const std::string& key, const JsonValue& value) {
  if (JsonValue* existing = field_of(object, key); existing != nullptr) {
    *existing = value;
    return;
  }
  object.fields.push_back({key, value});
}

/// The keys a `runs` entry of `family` may override.
///
/// A whitelist, not a blacklist: the list is exactly what the family's expander
/// reads per run, plus the run's own name.  An override the family never reads
/// would otherwise be accepted and silently dropped, which is the failure this
/// table exists to prevent -- and it is also what keeps a run from restating the
/// time grid, the solver, the outputs or the identity, all of which the kernel
/// requires to be one per document.
bool run_key_allowed(const std::string& family, const std::string& key) {
  if (key == "name") return true;
  if (family == "axle_dynamic") return key == "blobs" || key == "inputs";
  if (family == "vehicle_dynamic") {
    return key == "blobs" || key == "road" || key == "inputs";
  }
  if (family == "handling") {
    return key == "blobs" || key == "road" || key == "handling" || key == "inputs";
  }
  if (family == "ride_four_post") {
    return key == "blobs" || key == "road" || key == "four_post" || key == "inputs";
  }
  if (family == "ride_random_road") {
    return key == "blobs" || key == "road" || key == "ride_random_road" ||
           key == "inputs";
  }
  return false;
}

/// Merge one `runs` entry over the shared document.
///
/// The shared document keeps everything the entry does not name, so every run of
/// one document is solved on the document's grid with the document's solver --
/// which is what the kernel requires.  `blobs` merges by name, because the shared
/// `time.samples` descriptor has to stay visible to every run while a run's own
/// tables are replaced; `inputs` merges one level deep for the same reason --
/// a run's own override must not drop the fields it did not mention; every other
/// key the entry is allowed to name replaces wholesale.
bool merge_run(const JsonValue& document, const JsonValue& entry,
               const std::string& family, JsonValue& out, std::string& error) {
  if (!entry.is_object()) return fail(error, "a runs entry must be an object");
  const std::string* name = entry.find_string("name");
  if (name == nullptr || name->empty()) {
    return fail(error, "a runs entry needs a name");
  }
  for (const auto& field : entry.fields) {
    if (!run_key_allowed(family, field.first)) {
      return fail(error, "runs entry \"" + *name + "\" may not override \"" +
                             field.first + "\" for family " + family);
    }
  }

  out = JsonValue{};
  out.kind = JsonKind::Object;
  for (const auto& field : document.fields) {
    if (field.first == "runs") continue;
    out.fields.push_back({field.first, field.second});
  }
  bool named = false;
  for (auto& field : out.fields) {
    if (field.first == "name") {
      field.second = json_text(*name);
      named = true;
    }
  }
  if (!named) out.fields.push_back({"name", json_text(*name)});

  for (const auto& field : entry.fields) {
    if (field.first == "name" || field.first == "blobs" || field.first == "inputs") {
      continue;
    }
    set_field(out, field.first, field.second);
  }

  if (const JsonValue* entry_blobs = entry.find("blobs"); entry_blobs != nullptr) {
    if (!entry_blobs->is_array()) {
      return fail(error, "runs entry \"" + *name + "\" blobs must be an array");
    }
    // The descriptor names are checked before either branch: a nameless
    // descriptor is meaningless whether it is merged by name or, when the
    // document carries no table of its own, becomes the run's whole list --
    // and a nameless one there would be silently unusable rather than refused.
    for (const JsonValue& descriptor : entry_blobs->items) {
      if (descriptor.find_string("name") == nullptr) {
        return fail(error, "a runs entry blob descriptor needs a name");
      }
    }
    JsonValue* shared = field_of(out, "blobs");
    if (shared == nullptr || !shared->is_array()) {
      set_field(out, "blobs", *entry_blobs);
    } else {
      for (const JsonValue& descriptor : entry_blobs->items) {
        const std::string* entry_name = descriptor.find_string("name");
        bool replaced = false;
        for (JsonValue& existing : shared->items) {
          const std::string* shared_name = existing.find_string("name");
          if (shared_name != nullptr && *shared_name == *entry_name) {
            existing = descriptor;
            replaced = true;
            break;
          }
        }
        if (!replaced) shared->items.push_back(descriptor);
      }
    }
  }

  if (const JsonValue* entry_inputs = entry.find("inputs"); entry_inputs != nullptr) {
    if (!entry_inputs->is_object()) {
      return fail(error, "runs entry \"" + *name + "\" inputs must be an object");
    }
    // Motion is document-level: it names an excitation every run shares, and the
    // dispatcher applies it to the whole plan after expansion.  A run that
    // declared its own would either be ignored or fight the shared one.
    if (entry_inputs->find("motion") != nullptr) {
      return fail(error, "runs entry \"" + *name +
                             "\" may not declare motion: the document's motion "
                             "applies to every run");
    }
    JsonValue* shared = field_of(out, "inputs");
    if (shared == nullptr || !shared->is_object()) {
      set_field(out, "inputs", *entry_inputs);
    } else {
      for (const auto& field : entry_inputs->fields) {
        set_field(*shared, field.first, field.second);
      }
    }
  }
  return true;
}

/// Expand a document that declares its cases as a `runs` list.
///
/// Each entry becomes one merged document, so the family expanders stay
/// untouched: they already describe how one excitation is read, and a run is
/// exactly one of those.  What is checked here is what only the whole list can
/// show -- one case per entry, names that stay distinguishable, and a single
/// shared time grid.
bool expand_runs(const JsonValue& document, const JsonValue& runs,
                 const std::string& family, Expander expander, const std::string& blob,
                 const ContractModel& model, ContractPlan& out, std::string& error) {
  if (!runs.is_array()) return fail(error, "runs must be an array");
  if (runs.items.empty()) return fail(error, "runs must not be empty");
  const std::string* declared = document.find_string("name");
  if (declared == nullptr || declared->empty()) {
    return fail(error, "case document has no name");
  }
  const std::string document_name = *declared;
  for (const JsonValue& entry : runs.items) {
    JsonValue merged;
    if (!merge_run(document, entry, family, merged, error)) return false;
    const std::size_t before = out.cases.size();
    if (!expander(merged, blob, model, out, error)) return false;
    if (out.cases.size() != before + 1) {
      return fail(error, "a runs entry must describe exactly one case");
    }
  }
  // Every case of one document shares its time grid.  The kernel sizes the
  // driven, steering and brake buffers from the first case and copies each
  // case's tables into them, so a run on a shorter grid would be a heap overrun
  // rather than a wrong number.
  for (const ContractCase& run : out.cases) {
    if (run.sample_count != out.cases.front().sample_count ||
        !(run.sample_times == out.cases.front().sample_times)) {
      return fail(error, "every run must be solved on the document's time grid");
    }
  }
  // A manifest entry is read by name, so two runs with one name would be two
  // indistinguishable cases.
  for (std::size_t first = 0; first < out.cases.size(); ++first) {
    for (std::size_t second = first + 1; second < out.cases.size(); ++second) {
      if (out.cases[first].name == out.cases[second].name) {
        return fail(error, "two runs share the name \"" + out.cases[first].name +
                               "\"");
      }
    }
  }
  // The plan's own name is the document's, not the last run's: the manifest
  // reports it beside the per-case names.
  out.name = document_name;
  return true;
}

}  // namespace
bool contract_case_supported(const std::string& family) {
  return support_of(family) == Support::kSupported;
}

/// Whether the protocol defines a family, whether or not this build runs it.
///
/// Separate from `contract_case_supported` because they answer different
/// questions, and the dispatcher's error message depends on telling them apart.
bool contract_case_family_in_protocol(const std::string& family) {
  return support_of(family) != Support::kNotInProtocol;
}

int contract_case_family_count(bool supported_only) {
  int count = 0;
  for (const FamilyRow& row : kFamilies) {
    if (!supported_only || row.expander != nullptr) {
      ++count;
    }
  }
  return count;
}

const char* contract_case_family_name(int index) {
  const int size = static_cast<int>(sizeof(kFamilies) / sizeof(kFamilies[0]));
  if (index < 0 || index >= size) {
    return nullptr;
  }
  return kFamilies[index].name;
}

bool contract_expand_case(const JsonValue& document, const std::string& blob,
                          const ContractModel& model, ContractPlan& out,
                          std::string& error) {
  using case_detail::fail;
  error.clear();
  out = ContractPlan{};

  const std::string* contract = document.find_string("contract");
  const std::string* kind = document.find_string("kind");
  const std::string* family = document.find_string("family");
  if (contract == nullptr || *contract != "multibody-case" || kind == nullptr ||
      *kind != "case" || family == nullptr) {
    return fail(error, "expected a multibody-case document");
  }
  // The classification and the dispatch read the *same* row, so a family cannot
  // be reported runnable and then fall through to the wrong expander.
  switch (support_of(*family)) {
    case Support::kNotInProtocol:
      return fail(error, "unknown case family \"" + *family + "\"");
    case Support::kKnownUnimplemented:
      return fail(error, "case family \"" + *family +
                             "\" is defined by the contract but not implemented "
                             "by this build");
    case Support::kSupported:
      break;
  }
  // A document that lists its cases in `runs` is expanded once per entry, so a
  // time-history family states one excitation per case; a family that builds its
  // own grid from a sweep refuses the list rather than producing colliding names.
  // The characteristics request and `inputs.motion` stay document-level: one grid,
  // one solver and one boundary set reach every case of one document.
  const FamilyRow* row = find_row(*family);
  if (const JsonValue* runs = document.find("runs"); runs != nullptr) {
    if (!row->reads_runs) {
      return fail(error, "case family \"" + *family +
                             "\" expands its own case grid and does not read a "
                             "runs list");
    }
    if (!expand_runs(document, *runs, *family, row->expander, blob, model, out, error)) {
      return false;
    }
  } else if (!row->expander(document, blob, model, out, error)) {
    return false;
  }
  if (!contract_read_characteristics(document, out.characteristics, error)) return false;
  const JsonValue* inputs = document.find("inputs");
  const JsonValue* motions = inputs == nullptr ? nullptr : inputs->find("motion");
  if (motions == nullptr) return true;
  if (!motions->is_array()) return fail(error, "inputs.motion must be an array");
  std::vector<bool> controlled(model.driven_count(), false);
  for (const JsonValue& motion : motions->items) {
    const std::string* coordinate = motion.find_string("coordinate");
    const std::string* values = motion.find_string("blob");
    const std::string* rates = motion.find_string("rate_blob");
    if (coordinate == nullptr || values == nullptr || rates == nullptr) {
      return fail(error, "a motion boundary needs coordinate, blob and rate_blob");
    }
    const int index = model.driven_index(*coordinate);
    if (index < 0) return fail(error, "motion names unknown coordinate " + *coordinate);
    const std::size_t signal = static_cast<std::size_t>(index);
    if (controlled[signal]) return fail(error, "duplicate motion boundary " + *coordinate);
    controlled[signal] = true;
    for (ContractCase& run : out.cases) {
      std::vector<double> target, rate;
      if (!case_detail::read_described_array(document, blob, *values, run.sample_count, target, error) ||
          !case_detail::read_described_array(document, blob, *rates, run.sample_count, rate, error)) return false;
      const double offset = model.driven_is_rotation(signal) ? 0.0 : model.driven_separation(signal);
      run.driven_target.resize(run.sample_count * model.driven_count(), 0.0);
      run.driven_target_rate.resize(run.sample_count * model.driven_count(), 0.0);
      for (std::size_t sample = 0; sample < run.sample_count; ++sample) {
        if (!std::isfinite(target[sample]) || !std::isfinite(rate[sample])) {
          return fail(error, "motion history must be finite");
        }
        run.driven_target[sample * model.driven_count() + signal] = target[sample] + offset;
        run.driven_target_rate[sample * model.driven_count() + signal] = rate[sample];
      }
    }
  }
  return true;
}

void contract_apply_solver(const ContractPlan& plan, AxleInput& input) {
  case_detail::apply_solver(plan, input);
}

}  // namespace axle_kernel
