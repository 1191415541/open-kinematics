#pragma once

/// The free functions of the `mb_model` module.
///
/// K6 moved these declarations here out of the transitional aggregate
/// `kernel_internal.hpp`, which was deleted once every translation unit
/// included the header of its own module (`MODULES.md` section 4).  The
/// declarations are grouped by the module that defines them, not by the
/// module that calls them, so the layering the project checks with
/// `check_module_layering.py` is also the layering of these headers.

#include "mb_base/prelude.hpp"
#include "mb_base/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_base/monotone_cubic.hpp"
#include "mb_base/constants.hpp"
#include "mb_base/diagnostics.hpp"
#include "mb_base/env.hpp"
#include "mb_base/util.hpp"
#include "mb_base/dual.hpp"
#include "mb_base/dual_geometry.hpp"
#include "mb_base/prelude.hpp"
#include "mb_model/enums.hpp"

namespace axle_kernel {
double road_profile_height( const Model& model, const State& state, std::size_t tire_index );

double road_profile_slope( const Model& model, const State& state, std::size_t tire_index );

void set_error(char* buffer, std::size_t capacity, const std::string& text);

int constraint_rows(int type);

Vec3 state_point(const State& state, int body, const Vec3& local);

Vec3 state_point_velocity(const State& state, int body, const Vec3& local);

int tire_frame_body(const Tire& tire);

Vec3 tire_frame_center(const Tire& tire);

int tire_center_body(const Tire& tire);

Vec3 tire_center_local(const Tire& tire);

Vec3 tire_relative_spin_omega(const State& state, const Tire& tire);

Vec3 add_force_on_body( std::vector<Vec3>& force, std::vector<Vec3>& torque, const Model& model, const State& state, int body, const Vec3& point_local, const Vec3& f_world);

void add_torque_on_body( std::vector<Vec3>& torque, const Model& model, int body, const Vec3& tau_world );

std::array<double, 6> mat6_mul( const std::array<double, 36>& matrix, const std::array<double, 6>& vector );

std::array<double, 6> bushing_deformation( const Bushing& bushing, const Model& model, const State& state, std::array<double, 6>& rate );

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
} // namespace axle_kernel
