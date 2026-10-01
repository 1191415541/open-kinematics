/// Compile-time registries for the contract's extensible dimensions.
///
/// The solver talks to these tables instead of to concrete implementations, so
/// adding a joint / element / tire model / case family means adding an entry
/// here plus its implementation file -- not editing the solver.

#include "mb_contract/functions.hpp"

#include <string>
#include <utility>
#include <vector>

namespace axle_kernel {
namespace {

struct JointEntry {
  const char* name;
  int rows;
};

// Row counts mirror `mb_joint/types.hpp`; the self-test asserts they agree
// with the constraint registry.
const JointEntry kJoints[] = {
    {"spherical", 3},      {"revolute", 5},          {"fixed", 6},
    {"prismatic", 5},      {"universal", 4},         {"cylindrical", 4},
    {"inplane", 1},        {"convel", 4},            {"driven_translation", 1},
    {"driven_rotation", 1},
};

// One name per element family.  The axial three are the structures the fused
// `spring_damper` record was split into (2026-09-26): a model that wants an
// elastic member, a dissipative one and a stop declares `spring`, `damper` and
// `bump_stop`, and the fused name is gone rather than aliased, so a document
// that still uses it is refused by name instead of being read as one of the
// three.
const char* const kElements[] = {
    "spring",           "damper",          "bump_stop",        "bushing",
    "anti_roll_bar",    "aerodynamic_drag", "steering_actuator", "rotational_torque",
    "wheel_torque",     "point_wrench",     "gravity",
};

const char* const kTires[] = {"vertical_linear", "fiala", "pac2002", "native_brush"};

const char* const kCaseFamilies[] = {
    "kc_quasi_static", "axle_dynamic",  "vehicle_kc",       "vehicle_dynamic",
    "handling",        "ride_four_post", "ride_random_road", "comparison",
};

template <std::size_t N>
bool contains(const char* const (&table)[N], const std::string& name) {
  for (const char* entry : table) {
    if (name == entry) {
      return true;
    }
  }
  return false;
}

}  // namespace

int contract_joint_rows(const std::string& name) {
  for (const JointEntry& entry : kJoints) {
    if (name == entry.name) {
      return entry.rows;
    }
  }
  return -1;
}

bool contract_element_known(const std::string& name) {
  return contains(kElements, name);
}

bool contract_tire_known(const std::string& name) {
  return contains(kTires, name);
}

bool contract_case_family_known(const std::string& name) {
  return contains(kCaseFamilies, name);
}

int contract_registry_size(int table) {
  switch (table) {
    case 0: return static_cast<int>(sizeof(kJoints) / sizeof(kJoints[0]));
    case 1: return static_cast<int>(sizeof(kElements) / sizeof(kElements[0]));
    case 2: return static_cast<int>(sizeof(kTires) / sizeof(kTires[0]));
    case 3: return static_cast<int>(sizeof(kCaseFamilies) / sizeof(kCaseFamilies[0]));
    default: return -1;
  }
}

const char* contract_registry_name(int table, int index) {
  // Only the name-only tables are enumerable this way.  `kJoints` carries row
  // counts as well, so it is queried through `contract_joint_rows`; exposing its
  // names here would invite a caller to read the count from the wrong place.
  switch (table) {
    case 1:
      if (index < 0 ||
          index >= static_cast<int>(sizeof(kElements) / sizeof(kElements[0]))) {
        return nullptr;
      }
      return kElements[index];
    case 2:
      if (index < 0 ||
          index >= static_cast<int>(sizeof(kTires) / sizeof(kTires[0]))) {
        return nullptr;
      }
      return kTires[index];
    case 3:
      if (index < 0 ||
          index >=
              static_cast<int>(sizeof(kCaseFamilies) / sizeof(kCaseFamilies[0]))) {
        return nullptr;
      }
      return kCaseFamilies[index];
    default:
      return nullptr;
  }
}

}  // namespace axle_kernel