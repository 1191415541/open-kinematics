#pragma once

/// The free functions of the `mb_model` module.
///
/// K6 moved these declarations here out of the transitional aggregate
/// `kernel_internal.hpp`, which was deleted once every translation unit
/// included the header of its own module (`MODULES.md` section 4).  The
/// declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_config/prelude.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_numeric/monotone_cubic.hpp"
#include "mb_config/constants.hpp"
#include "mb_config/diagnostics.hpp"
#include "mb_config/env.hpp"
#include "mb_numeric/util.hpp"
#include "mb_dual/dual.hpp"
#include "mb_dual/dual_geometry.hpp"
#include "mb_config/prelude.hpp"
#include "mb_model/enums.hpp"

namespace axle_kernel {
double road_profile_height( const Model& model, const State& state, std::size_t tire_index );

double road_profile_slope( const Model& model, const State& state, std::size_t tire_index );

void set_error(char* buffer, std::size_t capacity, const std::string& text);


Vec3 state_point(const State& state, int body, const Vec3& local);

Vec3 state_point_velocity(const State& state, int body, const Vec3& local);

int tire_frame_body(const Tire& tire);

Vec3 tire_frame_center(const Tire& tire);

int tire_center_body(const Tire& tire);

Vec3 tire_center_local(const Tire& tire);

Vec3 tire_relative_spin_omega(const State& state, const Tire& tire);

DVec3 d_state_point( const State& state, const std::vector<Vec3>& dr, const std::vector<Vec3>& dtheta, int body, const Vec3& local );

DVec3 d_state_point_velocity( const State& state, const std::vector<Vec3>& dr, const std::vector<Vec3>& dtheta, const std::vector<Vec3>& dv, const std::vector<Vec3>& domega, int body, const Vec3& local );

DirectionalRoadProfile road_profile_directional( const Model& model, const State& state, const DirectionalState& direction, std::size_t tire_index, bool& smooth );

bool directional_vec_active(const Vec3& value);

bool directional_body_active(const DirectionalState& direction, int body);

bool directional_orientation_active( const DirectionalState& direction, int body );

bool directional_inertial_active( const DirectionalState& direction, int body );

void reset_directional_state( const Model& model, DirectionalState& direction );

void ensure_directional_state( const Model& model, DirectionalState& direction );

void add_directional_force_at_arm( std::vector<Vec3>& force, std::vector<Vec3>& torque, const Model& model, int body, const DVec3& arm, const DVec3& f );

void add_directional_torque( std::vector<Vec3>& torque, const Model& model, int body, const DVec3& value );

// A pure Model accessor moved here from `mb_solve_static` at subtask 04:
// `static_rotation_gauge_for_pivot` reads the assembled gauge list; the
// solver-side `static_rotation_gauge_value` stays with the static solve.
const StaticRotationGauge* static_rotation_gauge_for_pivot( const Model& model, int coordinate );

// Effective body inertia, after any tire that declares its own mass.  These are
// the accessors every consumer reads, so the fact "a tire owns part of the wheel"
// is stated once instead of at each `body.mass` use.
//
// A model whose effective tables were never computed (or a body index outside
// them) returns the body's own values: the historical answer, and the only
// possible answer for a document that declares no tire mass.
double body_effective_mass(const Model& model, int body);

const Mat3& body_effective_inertia_body(const Model& model, int body);

// The effective body inertia rotated into the world frame: `R * I_body * R^T`.
//
// Extracted at P3 step 6 (R8) because the identical expression appeared seven times
// across six translation units -- `force/layout.cpp`, `joint/kernel_directional_constraint.cpp`
// (twice), `output/kernel_output.cpp`, `solve_dynamic/kernel_integrator.cpp`,
// `solve_dynamic/kernel_integrator_newton.cpp` (twice) and
// `solve_dynamic/kernel_integrator_residual.cpp`.  Rotation into the world frame is a
// property of the model and the pose, not of any solver's discretisation, so it belongs
// beside the accessor it is built on rather than at each call site.
//
// Callers that already hold the rotation matrix pass it in, so the shared unit never
// re-evaluates `qmat` on a value the caller computed.  Every one of the seven migrated
// call sites already had the matrix in hand, so there is deliberately no quaternion
// overload: an unused one would be speculative API, and the rule here is to extract on
// the second real use rather than to guess at a second use.
Mat3 body_world_inertia(const Model& model, int body, const Mat3& rotation);

// The pose coordinates the model's declared static gauge holds.
//
// A gauge row is a *declared* coordinate rather than a physical constraint: the
// static problem is rank-deficient by the linkage's rigid-body modes, and the gauge
// removes them by naming coordinates to hold (see `WorkingSample`).  The declaration
// lives on the model -- the contract installs it before the first run and every case
// in a document shares it -- so deriving the coordinate list from it is a model
// question, not a solver one.
//
// Moved here from `mb_solve_static` at P4a step 1: the frozen-response path needs the
// same list but sits *below* the static solver, so reaching up for it would invert the
// layering.  `static_gauge_coordinates` stays as a delegating wrapper so the static
// path's own callers are unchanged.
//
// Returns an empty list when the model declares no gauge (`static_gauge_body < 0` or
// a zero mask) or names a body outside the free set.  An empty gauge is the honest
// answer for such a model, not a default.
std::vector<int> gauge_coordinates(const Model& model);

} // namespace axle_kernel
