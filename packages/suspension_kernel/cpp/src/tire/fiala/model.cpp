// K6 (epic MODULES.md section 2.5): one tire model's law.
//
// Split out of `kernel_tire_model.cpp` by top-level function; the bodies are verbatim.  The
// three tire models are separate libraries so the link graph enforces that they
// do not call each other.

#include "mb_tire/fiala/functions.hpp"

namespace axle_kernel {

double fiala_parameter(const Tire& tire, int index, double fallback) {
    const double value = tire.pac2002_parameters[static_cast<std::size_t>(index)];
    return std::isfinite(value) ? value : fallback;
}

void fiala_forces(
    const Tire& tire, double kappa, double alpha, double normal_force,
    double& fx, double& fy, double& mz
) {
    fx = fy = mz = 0.0;
    if (normal_force <= 0.0) return;
    const double cslip = std::max(fiala_parameter(tire, FIALA_CSLIP, 1000.0), 1e-9);
    const double calpha = std::max(fiala_parameter(tire, FIALA_CALPHA, 800.0), 1e-9);
    const double umin = std::max(fiala_parameter(tire, FIALA_UMIN, 0.9), 0.0);
    const double umax = std::max(fiala_parameter(tire, FIALA_UMAX, 1.0), umin);
    const double tan_alpha = std::tan(alpha);
    // Adams' comprehensive slip S_sa = sqrt(kappa^2 + tan(alpha)^2) carries no
    // upper clamp (Fiala_Tire_Force_Evaluat.html, Eq3168/Eq3169): past S_sa = 1
    // the friction coefficient keeps falling linearly towards UMIN instead of
    // flattening.  The clamp that used to sit here held the friction at or above
    // the Adams value for every S_sa > 1, i.e. in every combined-slip maneuver.
    const double ss = std::sqrt(kappa*kappa + tan_alpha*tan_alpha);
    const double uf = (umax-(umax-umin)*ss)*normal_force;
    if (uf <= 0.0) return;
    const double sign_k = kappa < 0.0 ? -1.0 : (kappa > 0.0 ? 1.0 : 0.0);
    const double abs_k = std::abs(kappa);
    if (abs_k <= uf/(2.0*cslip)) {
        fx = cslip*kappa;
    } else if (sign_k != 0.0) {
        fx = sign_k*(uf-(uf*uf)/(4.0*abs_k*cslip));
    }
    const double critical_alpha = std::atan(3.0*uf/calpha);
    const double sign_alpha = alpha < 0.0 ? -1.0 : (alpha > 0.0 ? 1.0 : 0.0);
    if (std::abs(alpha) <= critical_alpha) {
        const double h = 1.0-calpha*std::abs(tan_alpha)/(3.0*uf);
        fy = -uf*(1.0-h*h*h)*sign_alpha;
        mz = uf*fiala_parameter(tire, FIALA_WIDTH, 0.235)*(1.0-h)*h*h*h*sign_alpha;
    } else {
        fy = -uf*sign_alpha;
    }
}

} // namespace axle_kernel
