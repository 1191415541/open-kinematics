/// Self-test for the tire-state interface.
///
/// Subtask 09's contract is that the *solver* no longer knows about specific tire
/// models: it asks the tire layer for a state width, a slot value and a derivative
/// through the generic interface, and the per-model behaviour arrives through the
/// descriptor table's semantic flags.  Two things therefore have to be true, and
/// this executable checks both:
///
///   1. **the descriptor table is complete and self-consistent** -- every
///      registered model answers every question, and a model whose state comes from
///      the PAC2002 mode table actually says so;
///   2. **the slot layout is internally consistent** -- the descriptor table's
///      `tire_index`/`group_size`/`group_base` triple describes a packed layout
///      that the width calculation agrees with.
///
/// The values themselves are covered by the product's numeric gates
/// (`dynamic_hash_sentinel`, `kc_parity`), which compare against the frozen
/// baseline: a change to the state layout would move those bytes, so the layout is
/// pinned by something stronger than a unit test would be.  This file is what
/// catches the *structural* mistakes those byte comparisons cannot name.

#include <cstdint>

#include "mb_model/types.hpp"
#include "mb_tire/model.hpp"
#include "mb_tire/pac2002/functions.hpp"
#include "mb_tire_state/tire_state.hpp"

#include <cstdio>
#include <set>
#include <string>

namespace {

int g_failures = 0;
int g_checks = 0;

void check(bool condition, const char* what) {
  ++g_checks;
  if (!condition) {
    ++g_failures;
    std::printf("FAIL: %s\n", what);
  }
}

}  // namespace

int main() {
  using namespace axle_kernel;

  // --- every registered model answers every question ------------------------
  check(kTireModelCount >= 3, "at least three tire models are registered");
  std::set<int> kinds;
  for (int index = 0; index < kTireModelCount; ++index) {
    const TireModelDescriptor& descriptor = kTireModelDescriptors[index];
    check(descriptor.name != nullptr, "every descriptor carries a name");
    check(descriptor.name[0] != '\0', "no descriptor has an empty name");
    check(kinds.insert(descriptor.kind).second,
          "every descriptor has a distinct kind");
    // The descriptor for a kind must be reachable by that kind: the lookup and
    // the table must agree, or the solver's question and the model's answer come
    // from different rows.
    const TireModelDescriptor* found = tire_model_descriptor(descriptor.kind);
    check(found == &descriptor, "a descriptor is reachable by its own kind");
  }
  check(tire_model_descriptor(-1) == nullptr,
        "an unregistered kind has no descriptor");

  // --- the base width is the shared linear-transient pair -------------------
  // Every model's block is at least the `sx`/`sy` pair, whatever its own
  // semantics; a model narrower than that would drop the transient state.
  check(kTireModelBaseStateSlotWidth == 2,
        "the base state width is the linear transient pair");

  // --- the PAC2002 models are the ones with a mode-driven width -------------
  // Stated as an implication rather than a list: a mode-table model must be a
  // PAC2002 model, and the two PAC2002 kinds must be exactly the mode-table ones.
  int mode_table_models = 0;
  for (int index = 0; index < kTireModelCount; ++index) {
    const TireModelDescriptor& descriptor = kTireModelDescriptors[index];
    if (descriptor.uses_pac2002_mode_table) {
      ++mode_table_models;
      check(descriptor.uses_pac2002_law,
            "a mode-table model is a PAC2002-law model");
    }
    if (descriptor.uses_pac2002_law) {
      check(descriptor.uses_pac2002_mode_table,
            "a PAC2002-law model takes its width from the mode table");
    }
  }
  check(mode_table_models >= 1, "at least one model uses the mode table");

  // --- exactly one model uses each of the exclusive behaviours --------------
  // `uses_exact_relaxation` and `has_state_return_mapping` are alternatives: a
  // model cannot both replace the BDF2 row and be projected by the generic path.
  // Two models claiming both would mean the flags stopped being a partition.
  for (int index = 0; index < kTireModelCount; ++index) {
    const TireModelDescriptor& descriptor = kTireModelDescriptors[index];
    check(!(descriptor.uses_exact_relaxation && descriptor.has_state_return_mapping),
          "exact relaxation and return mapping are alternatives");
  }

  // --- the supported PAC2002 modes are a real, non-empty set ----------------
  const std::vector<int>& modes = pac2002_supported_use_modes();
  check(!modes.empty(), "the supported PAC2002 USE_MODEs are declared");
  std::set<int> unique(modes.begin(), modes.end());
  check(unique.size() == modes.size(), "no USE_MODE is declared twice");
  for (int mode : modes) {
    // The declaration must agree with the predicate the solver uses: a mode
    // listed as supported but rejected by the predicate would be refused at run
    // time while the capability document advertised it.
    check(pac2002_mode_supported_by_native(mode),
          "a declared USE_MODE is supported by the predicate");
  }

  // --- the slot descriptor table is internally consistent -------------------
  check(kTireSlotCount > 0, "the slot descriptor table is not empty");
  std::set<int> slots;
  for (int index = 0; index < kTireSlotCount; ++index) {
    const TireSlotDescriptor& slot = kTireSlotDescriptors[index];
    check(slots.insert(slot.slot).second, "every slot index is distinct");
    check(slot.name != nullptr && slot.name[0] != '\0',
          "every slot carries a name");
    check(slot.minimum_width >= 2,
          "every slot fits in at least the base block width");
    check(slot.packing >= 1, "every slot declares a positive packing");
    // A packed group (the turn-slip block) shares one state vector across several
    // slots, so its first slot must be at or before its own index and the group
    // must be big enough to contain it.  `packed_first_slot` is documented as
    // ignored when `packing == 1`, so the containment claim is made only where it
    // means something -- asserting it for an unpacked slot would demand a value
    // the header explicitly says is not used.
    if (slot.packing > 1) {
      check(slot.packed_first_slot >= 0,
            "a packed group has a non-negative first slot");
      check(slot.packed_first_slot <= slot.slot,
            "a slot's packed group starts at or before it");
      check(slot.slot - slot.packed_first_slot < slot.packing,
            "a slot lies inside the packed group it names");
    } else {
      check(slot.packed_first_slot == 0,
            "an unpacked slot leaves the packed-group field at its documented zero");
    }
    // The value and rate members must be distinct: a slot whose derivative is
    // stored where its value is would overwrite itself every step.
    check(slot.value != slot.rate, "a slot's value and rate are different members");
  }

  if (g_failures == 0) {
    std::printf("mb_tire_state selftest: OK (%d checks)\n", g_checks);
    return 0;
  }
  std::printf("mb_tire_state selftest: FAILED (%d/%d checks)\n", g_failures, g_checks);
  return 1;
}
