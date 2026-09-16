// K6 (epic MODULES.md section 2.5): one tire model's law.
//
// Split out of `kernel_fiala_directional.cpp` by top-level function; the bodies are verbatim.  The
// three tire models are separate libraries so the link graph enforces that they
// do not call each other.

#include "mb_tire/pac2002/functions.hpp"

namespace axle_kernel {

double spin_value(double value) { return value; }

double spin_value(const DirectionalScalar& value) { return value.value; }

double spin_derivative(double) { return 0.0; }

double spin_derivative(const DirectionalScalar& value) {
    return value.derivative;
}

double spin_abs(double value, bool&) { return std::abs(value); }

DirectionalScalar spin_abs(const DirectionalScalar& value, bool& smooth) {
    return d_abs(value, smooth);
}

double spin_sin(double value) { return std::sin(value); }

DirectionalScalar spin_sin(const DirectionalScalar& value) {
    return d_sin(value);
}

double spin_cos(double value) { return std::cos(value); }

DirectionalScalar spin_cos(const DirectionalScalar& value) {
    return d_cos(value);
}

double spin_tan(double value) { return std::tan(value); }

DirectionalScalar spin_tan(const DirectionalScalar& value) {
    return d_tan(value);
}

double spin_sqrt(double value) { return std::sqrt(std::max(value, 0.0)); }

DirectionalScalar spin_sqrt(const DirectionalScalar& value) {
    return d_sqrt(value);
}

double spin_atan(double value) { return std::atan(value); }

DirectionalScalar spin_atan(const DirectionalScalar& value) {
    return d_atan2(value, DirectionalScalar{1.0});
}

double pac2002_spin_camber_reduction(const Tire& tire, double normal_force) {
    const double dfz = pac2002_load_difference(tire, normal_force);
    return pac2002_parameter(tire, PAC_PEY1, 0.0)
        *(1.0+pac2002_parameter(tire, PAC_PEY2, 0.0)*dfz);
}

Pac2002SpinFactors pac2002_spin_factors(
    const Tire& tire, double spin, double normal_force, double camber,
    double longitudinal_slip, double lateral_slip, double travel_sign
) {
    bool smooth = true;
    return pac2002_spin_factors_t<double>(
        tire, spin, normal_force, camber, longitudinal_slip, lateral_slip,
        travel_sign, smooth
    );
}

} // namespace axle_kernel
