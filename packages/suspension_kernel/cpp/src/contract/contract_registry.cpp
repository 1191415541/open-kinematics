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

// Row counts mirror `mb_constraint/types.hpp`; the self-test asserts they agree
// with the constraint registry.
const JointEntry kJoints[] = {
    {"spherical", 3},      {"revolute", 5},          {"fixed", 6},
    {"prismatic", 5},      {"universal", 4},         {"cylindrical", 4},
    {"inplane", 1},        {"convel", 4},            {"driven_translation", 1},
    {"driven_rotation", 1},
};

const char* const kElements[] = {
    "spring_damper", "bushing",        "anti_roll_bar", "bump_stop",
    "aerodynamic_drag", "steering_actuator", "wheel_torque", "point_wrench",
    "gravity",
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

}  // namespace axle_kernel