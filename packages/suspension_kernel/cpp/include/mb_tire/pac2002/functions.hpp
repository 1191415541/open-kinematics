#pragma once

/// The free functions of the `mb_tire/pac2002` module.
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
#include "mb_tire/pac2002/parameters.hpp"
#include "mb_numeric/monotone_cubic.hpp"
#include "mb_config/constants.hpp"
#include "mb_config/diagnostics.hpp"
#include "mb_tire_state/tire_state.hpp"
#include "mb_config/env.hpp"
#include "mb_numeric/util.hpp"
#include "mb_dual/dual.hpp"
#include "mb_dual/dual_geometry.hpp"
#include "mb_tire/pac2002/turn_slip.hpp"
#include "mb_tire/pac2002/spin.hpp"
#include "mb_tire/common/kinematics.hpp"
#include "mb_config/prelude.hpp"
#include "mb_model/enums.hpp"

namespace axle_kernel {
double pac2002_parameter(const Tire& tire, int index, double fallback);

double pac2002_positive_scale( const Tire& tire, int index, double fallback );

double pac2002_pressure_difference(const Tire& tire);

double pac2002_effective_rolling_radius( const Tire& tire, double penetration, double spin_rate );

double pac2002_bottoming_rim_limit(const Tire& tire);

double pac2002_bottoming_force(const Tire& tire, double penetration);

double pac2002_vertical_force( const Tire& tire, double penetration, double penetration_rate, double camber, double spin_rate, double longitudinal_force, double lateral_force, double maxwell_displacement );

double pac2002_maxwell_decay(const Tire& tire, double step_size);

double pac2002_maxwell_end_displacement( const Tire& tire, double previous, double deflection, double step_size );

double pac2002_pure_force( const Tire& tire, double slip, double normal_force, bool lateral, double camber, const Pac2002TurnSlip& turn_slip );

double pac2002_force_limit( const Tire& tire, double normal_force, bool lateral, double camber );

double pac2002_relaxation_length( const Tire& tire, double normal_force, bool lateral, double camber );

double pac2002_gyroscopic_moment( const Tire& tire, double normal_force, double camber, double effective_lateral_slip, double effective_lateral_slip_rate, double rolling_radius, double spin_rate );

double pac2002_low_speed_threshold(const Tire& tire);

double pac2002_slip_reference_speed(double rolling_speed, double low_speed_threshold);

double pac2002_low_speed_force_scale(const Tire& tire, double rolling_speed);

bool pac2002_has_vertical_force_coupling_terms(const Tire& tire);

int pac2002_use_mode(const Tire& tire);

/// One PAC2002 coefficient family this kernel does not implement.
///
/// A tire that requests any of its coefficients has to be refused rather than
/// solved with the family silently dropped.
struct Pac2002RefusedFamily {
    const char* name;
    const char* reason;
    std::vector<const char*> coefficients;
};

/// The USE_MODEs this kernel's PAC2002 mode tables implement.
///
/// `pac2002_mode_supported_by_native` reads this table rather than repeating it,
/// so the declaration and the behaviour cannot disagree.
const std::vector<int>& pac2002_supported_use_modes();

/// Scalar coefficient names that must be zero for a tire to run exactly.
const std::vector<const char*>& pac2002_refused_parameters();

/// Importer-synthesised feature flags that must be zero for the same reason.
const std::vector<const char*>& pac2002_refused_feature_flags();

/// Whole coefficient families this kernel does not implement, with the reason
/// each one is refused.
const std::vector<Pac2002RefusedFamily>& pac2002_refused_families();


bool pac2002_mode_allows_longitudinal(int use_mode);

bool pac2002_mode_allows_lateral(int use_mode);

bool pac2002_mode_allows_combined(int use_mode);

bool pac2002_mode_uses_transient_state(int use_mode);

bool pac2002_mode_supported_by_native(int use_mode);

bool pac2002_mode_is_vertical_only(int use_mode);

bool pac2002_mode_uses_contact_mass(int use_mode);

bool pac2002_mode_uses_turn_slip(int use_mode);

int pac2002_mode_state_width(int use_mode);

double pac2002_clamp_load(const Tire& tire, double normal_force);

double pac2002_clamp_camber(const Tire& tire, double camber);

double pac2002_clamp_lateral_slip(const Tire& tire, double lateral_slip);

double pac2002_clamp_longitudinal_slip(const Tire& tire, double slip);

double pac2002_contact_stiffness( const Tire& tire, double normal_force, double longitudinal_slip, double lateral_slip, bool lateral );

double pac2002_load_difference(const Tire& tire, double normal_force);

double pac2002_slip_stiffness( const Tire& tire, double normal_force, double camber, bool lateral );

double pac2002_half_contact_length( const Tire& tire, double vertical_deflection, double unloaded_radius );

double pac2002_lateral_stiffness( const Tire& tire, double normal_force, double camber );

double pac2002_turn_slip_equivalent_slip( const Tire& tire, double half_contact_length, double normal_force, double kappa, double alpha, double phi_c, double phi_1, double phi_2 );

double pac2002_turn_slip_contact_relaxation( const Tire& tire, double half_contact_length, double normal_force, double kappa, double alpha, double phi_c, double phi_1, double phi_2 );

double pac2002_turn_slip_switch(const char* name, double fallback);

Pac2002SpinFactors pac2002_spin_factors( const Tire& tire, double spin, double normal_force, double camber, double longitudinal_slip, double lateral_slip, double travel_sign );

bool pac2002_has_combined_slip_terms(const Tire& tire);

double pac2002_combined_longitudinal_force( const Tire& tire, double kappa, double alpha, double normal_force, double camber, double pure_force );

double pac2002_combined_lateral_force( const Tire& tire, double kappa, double alpha, double normal_force, double camber, double pure_force );

bool pac2002_has_aligning_moment_terms(const Tire& tire);

double pac2002_aligning_moment( const Tire& tire, double kappa, double alpha, double normal_force, double camber, double fx, double fy, const Pac2002TurnSlip& turn_slip );

bool pac2002_has_overturning_moment_terms(const Tire& tire);

bool pac2002_has_rolling_resistance_terms(const Tire& tire);

double pac2002_overturning_moment( const Tire& tire, double fy_source, double normal_force, double camber );

double pac2002_rolling_resistance_moment( const Tire& tire, double fx_source, double normal_force, double camber, double longitudinal_speed );

void write_pac2002_turn_slip_derivatives( const Tire& tire, const State& state, std::size_t index, int stride, double yaw_rate, double speed, double half_contact_length, double normal_force, double longitudinal_slip, double lateral_slip, double lateral_force, std::vector<double>& derivatives );

void write_pac2002_contact_mass_derivatives( const Tire& tire, const State& state, std::size_t index, int stride, double normal_force, double longitudinal_slip, double lateral_slip, double fx, double fy, std::vector<double>& derivatives );

void write_pac2002_contact_yaw_derivatives( const Tire& tire, const State& state, std::size_t index, int stride, double aligning_moment, std::vector<double>& derivatives );

Pac2002TurnSlip pac2002_state_turn_slip( const Tire& tire, const State& state, std::size_t index, double travel_sign );

double pac2002_parameter( const Tire& tire, int index, double fallback );

double pac2002_sign(double value);

double pac2002_reference_load(const Tire& tire);

double pac2002_slip_reference_speed( double rolling_speed, double low_speed_threshold );

double pac2002_clamp_to_range( const Tire& tire, int minimum_index, int maximum_index, double value, double fallback_minimum, double fallback_maximum );

double pac2002_peak_force( const Tire& tire, double normal_force, bool lateral, double camber );

double pac2002_safe_combined_ratio( double numerator, double denominator );

double pac2002_longitudinal_stiffness( const Tire& tire, double normal_force );

double spin_value(double value);

double spin_value(const DirectionalScalar& value);

double spin_derivative(double);

double spin_derivative(const DirectionalScalar& value);

double spin_abs(double value, bool&);

DirectionalScalar spin_abs(const DirectionalScalar& value, bool& smooth);

double spin_sin(double value);

DirectionalScalar spin_sin(const DirectionalScalar& value);

double spin_cos(double value);

DirectionalScalar spin_cos(const DirectionalScalar& value);

double spin_tan(double value);

DirectionalScalar spin_tan(const DirectionalScalar& value);

double spin_sqrt(double value);

DirectionalScalar spin_sqrt(const DirectionalScalar& value);

double spin_atan(double value);

DirectionalScalar spin_atan(const DirectionalScalar& value);

double pac2002_spin_camber_reduction(const Tire& tire, double normal_force);

Pac2002SpinFactorsDirectional pac2002_spin_factors_directional( const Tire& tire, const DirectionalScalar& spin, const DirectionalScalar& normal_force, const DirectionalScalar& camber, const DirectionalScalar& longitudinal_slip, const DirectionalScalar& lateral_slip, double travel_sign, bool& smooth );

DirectionalScalar pac2002_effective_rolling_radius_directional( const Tire& tire, const DirectionalScalar& penetration, const DirectionalScalar& spin_rate, bool& smooth );

DirectionalScalar pac2002_vertical_force_directional( const Tire& tire, const DirectionalScalar& penetration, const DirectionalScalar& penetration_rate, const DirectionalScalar& camber, const DirectionalScalar& spin_rate, const DirectionalScalar& longitudinal_force, const DirectionalScalar& lateral_force, const DirectionalScalar& maxwell_displacement, bool& smooth );

DirectionalScalar pac2002_relaxation_length_directional( const Tire& tire, const DirectionalScalar& normal_force, bool lateral, const DirectionalScalar& camber, bool& smooth );

DirectionalScalar pac2002_gyroscopic_moment_directional( const Tire& tire, const DirectionalScalar& normal_force, const DirectionalScalar& camber, const DirectionalScalar& effective_lateral_slip, const DirectionalScalar& effective_lateral_slip_rate, const DirectionalScalar& rolling_radius, const DirectionalScalar& spin_rate, bool& smooth );

DirectionalScalar pac2002_peak_force_directional( const Tire& tire, const DirectionalScalar& normal_force, bool lateral, const DirectionalScalar& camber, bool& smooth );

DirectionalScalar pac2002_pure_force_directional( const Tire& tire, const DirectionalScalar& slip, const DirectionalScalar& normal_force, bool lateral, const DirectionalScalar& camber, const Pac2002TurnSlipDirectional& turn_slip, bool& smooth );

DirectionalScalar pac2002_force_limit_directional( const Tire& tire, const DirectionalScalar& normal_force, bool lateral, const DirectionalScalar& camber, bool& smooth );

DirectionalScalar pac2002_combined_ratio_directional( const DirectionalScalar& numerator, const DirectionalScalar& denominator, bool& smooth );

DirectionalScalar pac2002_combined_longitudinal_force_directional( const Tire& tire, const DirectionalScalar& kappa, const DirectionalScalar& alpha, const DirectionalScalar& normal_force, const DirectionalScalar& camber, const DirectionalScalar& pure_force, bool& smooth );

DirectionalScalar pac2002_combined_lateral_force_directional( const Tire& tire, const DirectionalScalar& kappa, const DirectionalScalar& alpha, const DirectionalScalar& normal_force, const DirectionalScalar& camber, const DirectionalScalar& pure_force, bool& smooth );

DirectionalScalar pac2002_aligning_moment_directional( const Tire& tire, const DirectionalScalar& kappa, const DirectionalScalar& alpha, const DirectionalScalar& normal_force, const DirectionalScalar& camber, const DirectionalScalar& fx, const DirectionalScalar& fy, const Pac2002TurnSlipDirectional& turn_slip, bool& smooth );

DirectionalScalar pac2002_overturning_moment_directional( const Tire& tire, const DirectionalScalar& fy_source, const DirectionalScalar& normal_force, const DirectionalScalar& camber, bool& smooth );

DirectionalScalar pac2002_rolling_resistance_moment_directional( const Tire& tire, const DirectionalScalar& fx_source, const DirectionalScalar& normal_force, const DirectionalScalar& camber, const DirectionalScalar& longitudinal_speed, bool& smooth );
} // namespace axle_kernel
