// K6 (epic MODULES.md section 2.5): one tire model's law.
//
// Split out of `kernel_directional_tire.cpp` by top-level function; the bodies are verbatim.  The
// three tire models are separate libraries so the link graph enforces that they
// do not call each other.

#include "mb_tire/fiala/functions.hpp"

// Direct dependencies of this translation unit.  The module headers no
// longer aggregate each other's declarations, so each unit includes the
// modules whose functions it actually calls.
#include "mb_base/functions.hpp"

namespace axle_kernel {

void fiala_forces_directional(
    const Tire& tire,
    const DirectionalScalar& kappa,
    const DirectionalScalar& alpha,
    const DirectionalScalar& normal_force,
    DirectionalScalar& fx,
    DirectionalScalar& fy,
    DirectionalScalar& mz,
    bool& smooth
) {
    fx = fy = mz = {};
    if (normal_force.value <= 0.0) {
        smooth = false;
        return;
    }
    const double cslip = std::max(
        fiala_parameter(tire, FIALA_CSLIP, 1000.0), 1e-9
    );
    const double calpha = std::max(
        fiala_parameter(tire, FIALA_CALPHA, 800.0), 1e-9
    );
    const double umin = std::max(
        fiala_parameter(tire, FIALA_UMIN, 0.9), 0.0
    );
    const double umax = std::max(
        fiala_parameter(tire, FIALA_UMAX, 1.0), umin
    );
    const DirectionalScalar tan_alpha = d_tan(alpha);
    const DirectionalScalar combined_argument =
        kappa*kappa+tan_alpha*tan_alpha;
    DirectionalScalar combined_slip;
    if (combined_argument.value <= 1e-24) {
        smooth = false;
    } else {
        combined_slip = d_sqrt(combined_argument);
    }
    constexpr double kJacobianStep = 1e-7;
    // No upper clamp on the comprehensive slip, for the reason given in the
    // scalar fiala_forces().  Removing the clamp also removes the kink it made
    // at S_sa = 1, so that boundary no longer needs a non-smoothness mark.
    const DirectionalScalar ss = combined_slip;
    const DirectionalScalar uf = (
        DirectionalScalar{umax}-(umax-umin)*ss
    )*normal_force;
    if (uf.value <= 0.0) {
        smooth = false;
        return;
    }

    const DirectionalScalar longitudinal_limit =
        uf/(2.0*cslip);
    const double abs_kappa = std::abs(kappa.value);
    const double trial_abs_kappa = abs_kappa +
        kJacobianStep*(kappa.value < 0.0 ? -kappa.derivative : kappa.derivative);
    if (
        std::abs(abs_kappa-longitudinal_limit.value) <= 1e-12 ||
        (abs_kappa-longitudinal_limit.value)
            *(trial_abs_kappa-longitudinal_limit.value) <= 0.0
    ) {
        smooth = false;
    }
    if (abs_kappa <= longitudinal_limit.value) {
        fx = cslip*kappa;
    } else if (abs_kappa > 1e-12) {
        const double sign_kappa = kappa.value < 0.0 ? -1.0 : 1.0;
        const DirectionalScalar abs_kappa_directional{
            abs_kappa,
            sign_kappa*kappa.derivative
        };
        fx = DirectionalScalar{sign_kappa}*(
            uf-uf*uf/(4.0*abs_kappa_directional*cslip)
        );
    } else {
        smooth = false;
    }

    const DirectionalScalar critical_alpha = d_atan2(
        3.0*uf/calpha, DirectionalScalar{1.0}
    );
    const DirectionalScalar tan_alpha_abs = d_abs(tan_alpha, smooth);
    const double abs_alpha = std::abs(alpha.value);
    const double trial_abs_alpha = abs_alpha+
        kJacobianStep*(alpha.value < 0.0 ? -alpha.derivative : alpha.derivative);
    if (
        std::abs(abs_alpha-critical_alpha.value) <= 1e-12 ||
        (abs_alpha-critical_alpha.value)
            *(trial_abs_alpha-critical_alpha.value) <= 0.0
    ) {
        smooth = false;
    }
    const double sign_alpha = d_sign(alpha, smooth);
    if (abs_alpha <= critical_alpha.value) {
        const DirectionalScalar h = DirectionalScalar{1.0}
            -calpha*tan_alpha_abs/(3.0*uf);
        const DirectionalScalar h3 = h*h*h;
        fy = -uf*(DirectionalScalar{1.0}-h3)*sign_alpha;
        mz = uf*fiala_parameter(tire, FIALA_WIDTH, 0.235)
            *(DirectionalScalar{1.0}-h)*h*h*h*sign_alpha;
    } else {
        fy = -uf*sign_alpha;
    }
}

DirectionalScalar fiala_vertical_force_directional(
    const Tire& tire, const DirectionalScalar& penetration,
    const DirectionalScalar& penetration_rate
) {
    DirectionalScalar elastic;
    if (tire.deflection_curve.size() >= 4) {
        const CurveValue curve = monotone_cubic(
            tire.deflection_curve, penetration.value
        );
        // d(load)/d(penetration) is the table's slope; the clamp's derivative is
        // zero wherever it bites.
        elastic = DirectionalScalar{
            std::max(0.0, curve.value),
            (curve.value > 0.0 ? curve.slope : 0.0)*penetration.derivative
        };
    } else if (penetration.value > 0.0) {
        elastic = DirectionalScalar{
            tire.k*penetration.value, tire.k*penetration.derivative
        };
    }
    return elastic+tire.c*penetration_rate;
}

} // namespace axle_kernel
