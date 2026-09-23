/// Read back what the contract reader and the installer did with a tire's mass
/// and inertia.
///
/// This is a standalone probe rather than a unit test because the values under
/// test live in `ContractModel::tire_masses()` / `tire_inertias()` and on the
/// built `Model`'s `Tire`s -- neither of which the product C ABI exposes, and
/// neither of which this step may add to the frozen ABI just to look at them.
/// The probe is compiled at test time from the kernel sources and linked against
/// the static libraries the kernel build already produced, so it reads the same
/// translation units the shipped library was built from.
///
/// It prints, in a stable order, one line per fact the test asserts:
///
/// * `offset <field> <n>` and `sizeof <n>` for every `Tire` field, which is how
///   the test proves the two appended fields left every earlier offset alone;
/// * `declared ...` for a document that declares a mass and a tensor, after
///   building the model and installing the declaration the way the contract
///   entry point does;
/// * `absent ...` for the historical document that declares neither, which must
///   come back as zeros;
/// * `mismatch error <text>` for a declaration whose length cannot be installed,
///   which must be named rather than silently truncated.

#include "mb_assembly/functions.hpp"
#include "mb_cases/functions.hpp"
#include "mb_contract/functions.hpp"
#include "mb_model/types.hpp"

#include <cstddef>
#include <cstdio>
#include <string>

namespace {

using axle_kernel::ContractModel;
using axle_kernel::JsonValue;
using axle_kernel::Model;
using axle_kernel::Tire;

bool read_model(const std::string& text, ContractModel& model, std::string& error) {
  JsonValue document;
  if (!axle_kernel::contract_parse_json(text, document, error)) return false;
  return model.read(document, "", error);
}
/// The parameter block a brush tire must carry for `build_model` to accept it.
/// The fields the reader defaults to zero would otherwise trip the unrelated
/// "invalid tire parameters" check, so the probe supplies real numbers here.
const char* const kTireParameters =
    "\"detached_relaxation_s\":1.0,\"forward_axis_local\":[1.0,0.0,0.0],"
    "\"lateral_brush_stiffness\":80000.0,"
    "\"lateral_friction_coefficient\":0.9,\"lateral_relaxation_length\":0.15,"
    "\"longitudinal_brush_stiffness\":100000.0,"
    "\"longitudinal_friction_coefficient\":0.9,"
    "\"longitudinal_relaxation_length\":0.05,\"maximum_compression\":0.29,"
    "\"spin_axis_local\":[0.0,1.0,0.0],\"unloaded_radius\":0.322,"
    "\"vertical_damping\":3.1,\"vertical_stiffness\":310000.0";


/// One body and one tire, in metres so the reader's unit conversions are the
/// identity and the printed numbers are the numbers the document wrote.
///
/// `parameters` supplies the force-law parameters, which is where the reader
/// looks for the geometry; `declaration` supplies the tire-level fields, which
/// is where `mass` and `inertia` live.  Keeping the two separate is the point:
/// the pair belongs to the entry, not to the force law.
std::string document(const std::string& parameters, const std::string& declaration) {
  return std::string(
             "{\"bodies\":[{\"inertia\":[[1.0,0.0,0.0],[0.0,1.0,0.0],[0.0,0.0,1.0]],"
             "\"mass\":1000.0,\"name\":\"chassis\"}],\"contract\":\"multibody-model\","
             "\"contract_version\":1,\"kind\":\"model\",\"name\":\"tire-mass-probe\","
             "\"tires\":[{\"body\":\"chassis\",\"model\":\"native_brush\","
             "\"name\":\"front_left\",\"parameters\":{") +
         parameters + "}" + declaration +
         "}],\"units\":{\"angle\":\"rad\",\"length\":\"m\",\"mass\":\"kg\","
         "\"time\":\"s\"}}";
}

void dump_tire_field_offsets() {
#define AXLE_KERNEL_PROBE_OFFSET(field)                        \
  std::printf("offset %s %zu\n", #field, offsetof(Tire, field))
  AXLE_KERNEL_PROBE_OFFSET(body);
  AXLE_KERNEL_PROBE_OFFSET(frame_body);
  AXLE_KERNEL_PROBE_OFFSET(drive_torque_body);
  AXLE_KERNEL_PROBE_OFFSET(drive_torque_reaction_body);
  AXLE_KERNEL_PROBE_OFFSET(center);
  AXLE_KERNEL_PROBE_OFFSET(frame_center);
  AXLE_KERNEL_PROBE_OFFSET(drive_torque_axis);
  AXLE_KERNEL_PROBE_OFFSET(spin_axis);
  AXLE_KERNEL_PROBE_OFFSET(forward_axis);
  AXLE_KERNEL_PROBE_OFFSET(radius);
  AXLE_KERNEL_PROBE_OFFSET(maximum_compression);
  AXLE_KERNEL_PROBE_OFFSET(k);
  AXLE_KERNEL_PROBE_OFFSET(c);
  AXLE_KERNEL_PROBE_OFFSET(mu_longitudinal);
  AXLE_KERNEL_PROBE_OFFSET(mu_lateral);
  AXLE_KERNEL_PROBE_OFFSET(brush_k_longitudinal);
  AXLE_KERNEL_PROBE_OFFSET(brush_k_lateral);
  AXLE_KERNEL_PROBE_OFFSET(relaxation_length_longitudinal);
  AXLE_KERNEL_PROBE_OFFSET(relaxation_length_lateral);
  AXLE_KERNEL_PROBE_OFFSET(detached_relaxation);
  AXLE_KERNEL_PROBE_OFFSET(model_kind);
  AXLE_KERNEL_PROBE_OFFSET(state_slot_width);
  AXLE_KERNEL_PROBE_OFFSET(contact_mass);
  AXLE_KERNEL_PROBE_OFFSET(maxwell_enabled);
  AXLE_KERNEL_PROBE_OFFSET(uses_exact_relaxation);
  AXLE_KERNEL_PROBE_OFFSET(has_state_return_mapping);
  AXLE_KERNEL_PROBE_OFFSET(uses_pac2002_law);
  AXLE_KERNEL_PROBE_OFFSET(evaluates_at_wheel_center);
  AXLE_KERNEL_PROBE_OFFSET(projects_compression_on_spin);
  AXLE_KERNEL_PROBE_OFFSET(pac2002_parameters);
  AXLE_KERNEL_PROBE_OFFSET(deflection_curve);
  AXLE_KERNEL_PROBE_OFFSET(bottoming_curve);
  AXLE_KERNEL_PROBE_OFFSET(mass);
  AXLE_KERNEL_PROBE_OFFSET(inertia);
#undef AXLE_KERNEL_PROBE_OFFSET
  std::printf("sizeof %zu\n", sizeof(Tire));
}

/// Build the model the contract entry point would build, install the parsed
/// declaration on it exactly as that entry point does, and print what the
/// resulting `Tire` carries.  Going through `build_model` and
/// `install_tire_mass` rather than reading `ContractModel` directly is what
/// makes the printed numbers evidence about what the solver would see.
void dump_installed(const char* label, const std::string& declaration) {
  ContractModel model;
  std::string error;
  if (!read_model(document(kTireParameters, declaration), model, error)) {
    std::printf("%s error %s\n", label, error.c_str());
    return;
  }
  VehicleInput input{};
  model.fill(input);
  Model built = axle_kernel::build_model(input.axle, error);
  if (!error.empty()) {
    std::printf("%s build_error %s\n", label, error.c_str());
    return;
  }
  std::printf("%s tires %zu\n", label, built.tires.size());
  std::string install_error;
  if (!axle_kernel::install_tire_mass(model, built, install_error)) {
    std::printf("%s install_error %s\n", label, install_error.c_str());
    return;
  }
  for (const Tire& tire : built.tires) {
    std::printf("%s mass %.17g\n", label, tire.mass);
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        std::printf("%s inertia %.17g\n", label, tire.inertia.a[row][column]);
      }
    }
  }
  for (double value : model.tire_masses()) {
    std::printf("%s parsed_mass %.17g\n", label, value);
  }
  for (double value : model.tire_inertias()) {
    std::printf("%s parsed_inertia %.17g\n", label, value);
  }
}

/// A model whose tires cannot take the declaration: the document carries one
/// tire, the built model carries none.  This is the shape of every way the
/// counts can disagree, and the install must name it rather than truncate.
void dump_mismatch() {
  ContractModel model;
  std::string error;
  if (!read_model(
          document(kTireParameters,
                   ",\"inertia\":[[0.4,0.0,0.0],[0.0,0.7,0.0],[0.0,0.0,0.7]],"
                   "\"mass\":12.5"),
          model, error)) {
    std::printf("mismatch error %s\n", error.c_str());
    return;
  }
  Model empty;
  std::string install_error;
  if (axle_kernel::install_tire_mass(model, empty, install_error)) {
    std::printf("mismatch accepted\n");
    return;
  }
  std::printf("mismatch error %s\n", install_error.c_str());
}



}  // namespace

int main() {
  dump_tire_field_offsets();
  // A tire that owns its inertia: mass 12.5 kg, a diagonal tensor in kg*m^2.
  // The declarative fields sit next to the force-law parameters, which is where
  // the schema puts them.
  dump_installed(
      "declared",
      ",\"inertia\":[[0.4,0.0,0.0],[0.0,0.7,0.0],[0.0,0.0,0.7]],\"mass\":12.5");
  // The historical document: no mass and no inertia at all.
  dump_installed("absent", "");
  dump_mismatch();
  return 0;
}
