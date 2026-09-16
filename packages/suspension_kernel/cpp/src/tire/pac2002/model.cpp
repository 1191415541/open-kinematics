// K6 (epic MODULES.md section 2.5): one tire model's law.
//
// Split out of `kernel_tire_model.cpp` by top-level function; the bodies are verbatim.  The
// three tire models are separate libraries so the link graph enforces that they
// do not call each other.

#include "mb_tire/pac2002/functions.hpp"

namespace axle_kernel {

double pac2002_parameter(
    const Tire& tire, int index, double fallback
) {
    const double value = tire.pac2002_parameters[
        static_cast<std::size_t>(index)
    ];
    return std::isfinite(value) ? value : fallback;
}

double pac2002_sign(double value) {
    return value < 0.0 ? -1.0 : 1.0;
}

double pac2002_positive_scale(
    const Tire& tire, int index, double fallback
) {
    const double value = pac2002_parameter(tire, index, fallback);
    return value > 0.0 ? value : fallback;
}

double pac2002_reference_load(const Tire& tire) {
    return std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0)
            *pac2002_positive_scale(tire, PAC_LFZO, 1.0),
        1e-9
    );
}

double pac2002_load_difference(const Tire& tire, double normal_force) {
    const double reference_load = pac2002_reference_load(tire);
    return (normal_force-reference_load)/reference_load;
}

double pac2002_relaxation_length(
    const Tire& tire, double normal_force, bool lateral, double camber
) {
    if (normal_force <= 0.0) {
        return lateral
            ? tire.relaxation_length_lateral
            : tire.relaxation_length_longitudinal;
    }
    const double reference_load = pac2002_reference_load(tire);
    const double dfz = pac2002_load_difference(tire, normal_force);
    double length = 0.0;
    if (lateral) {
        const double pty2 = pac2002_parameter(tire, PAC_PTY2, 1.0);
        const double denominator = std::max(
            std::abs(pty2*reference_load), 1e-9
        );
        length = pac2002_parameter(tire, PAC_PTY1, 1.0)
            * std::sin(2.0*std::atan(normal_force/denominator))
            *(1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)
                *std::abs(camber))
            *tire.radius*pac2002_positive_scale(tire, PAC_LFZO, 1.0)
            *pac2002_positive_scale(tire, PAC_LSGAL, 1.0);
    } else {
        length = normal_force * (
            pac2002_parameter(tire, PAC_PTX1, 1.0)
                + pac2002_parameter(tire, PAC_PTX2, 0.0)*dfz
        ) * std::exp(pac2002_parameter(tire, PAC_PTX3, 0.0)*dfz)
            * tire.radius/reference_load
            *pac2002_positive_scale(tire, PAC_LSGKP, 1.0);
    }
    if (tire.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
        return std::max(length, 1e-9);
    }
    return std::max(
        length,
        lateral
            ? tire.relaxation_length_lateral
            : tire.relaxation_length_longitudinal
    );
}

double pac2002_gyroscopic_moment(
    const Tire& tire, double normal_force, double camber,
    double effective_lateral_slip, double effective_lateral_slip_rate,
    double rolling_radius, double spin_rate
) {
    const double coefficient = pac2002_parameter(tire, PAC_QTZ1, 0.0)
        *pac2002_parameter(tire, PAC_LGYR, 1.0)
        *pac2002_parameter(tire, PAC_MBELT, 0.0);
    if (std::abs(coefficient) <= 1e-12 || normal_force <= 0.0 ||
        rolling_radius <= 0.0) {
        return 0.0;
    }
    const double sigma_alpha = pac2002_relaxation_length(
        tire, normal_force, true, camber
    );
    const double tangent = std::tan(effective_lateral_slip);
    const double lateral_deflection_rate = sigma_alpha
        *(1.0+tangent*tangent)*effective_lateral_slip_rate;
    const double result = coefficient*(rolling_radius*spin_rate)
        *lateral_deflection_rate;
    return std::isfinite(result) ? result : 0.0;
}

} // namespace axle_kernel
