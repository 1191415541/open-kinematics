// K6 (epic MODULES.md section 2.5): one tire model's law.
//
// Split out of `kernel_directional_tire.cpp` by top-level function; the bodies are verbatim.  The
// three tire models are separate libraries so the link graph enforces that they
// do not call each other.

#include "mb_tire/pac2002/functions.hpp"

// Direct dependencies of this translation unit.  The module headers no
// longer aggregate each other's declarations, so each unit includes the
// modules whose functions it actually calls.
#include "mb_config/functions.hpp"
#include "mb_dual/functions.hpp"
#include "mb_numeric/functions.hpp"

namespace axle_kernel {

Pac2002SpinFactorsDirectional pac2002_spin_factors_directional(
    const Tire& tire, const DirectionalScalar& spin,
    const DirectionalScalar& normal_force, const DirectionalScalar& camber,
    const DirectionalScalar& longitudinal_slip,
    const DirectionalScalar& lateral_slip, double travel_sign, bool& smooth
) {
    return pac2002_spin_factors_t<DirectionalScalar>(
        tire, spin, normal_force, camber, longitudinal_slip, lateral_slip,
        travel_sign, smooth
    );
}

DirectionalScalar pac2002_effective_rolling_radius_directional(
    const Tire& tire, const DirectionalScalar& penetration,
    const DirectionalScalar& spin_rate, bool& smooth
) {
    const double radius = std::max(tire.radius, 1e-9);
    const double nominal_load = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double vertical_scale = pac2002_positive_scale(tire, PAC_LCZ, 1.0);
    const double vertical_stiffness = std::max(tire.k*vertical_scale, 1e-9);
    const double nominal_deflection = nominal_load/vertical_stiffness;
    DirectionalScalar normalized_deflection;
    if (penetration.value <= 0.0) {
        if (std::abs(penetration.value) <= 1e-12) smooth = false;
        normalized_deflection = {};
    } else {
        normalized_deflection = penetration/
            std::max(nominal_deflection, 1e-9);
    }
    const double speed_reference = std::max(
        pac2002_parameter(tire, PAC_LONGVL, 16.6), 1e-9
    );
    const DirectionalScalar speed_ratio = spin_rate*radius/speed_reference;
    const DirectionalScalar speed_growth =
        pac2002_parameter(tire, PAC_QV1, 0.0)*radius
            *speed_ratio*speed_ratio;
    const DirectionalScalar radius_correction = nominal_deflection* (
        pac2002_parameter(tire, PAC_DREFF, 0.27)
            *d_atan2(
                pac2002_parameter(tire, PAC_BREFF, 8.4)
                    *normalized_deflection,
                {1.0, 0.0}
            )
        +pac2002_parameter(tire, PAC_FREFF, 0.07)
            *normalized_deflection
    );
    DirectionalScalar result = radius*
        pac2002_parameter(tire, PAC_QREO, 1.0)
        +speed_growth-radius_correction;
    if (result.value <= 1e-9) {
        smooth = false;
        result = {1e-9, 0.0};
    }
    return result;
}

DirectionalScalar pac2002_vertical_force_directional(
    const Tire& tire, const DirectionalScalar& penetration,
    const DirectionalScalar& penetration_rate,
    const DirectionalScalar& camber, const DirectionalScalar& spin_rate,
    const DirectionalScalar& longitudinal_force,
    const DirectionalScalar& lateral_force,
    const DirectionalScalar& maxwell_displacement, bool& smooth
) {
    if (penetration.value <= 0.0) {
        if (std::abs(penetration.value) <= 1e-12) smooth = false;
        return {};
    }
    const double radius = std::max(tire.radius, 1e-9);
    const double nominal_load = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double load_scale = pac2002_positive_scale(tire, PAC_LCZ, 1.0);
    double qfz1 = pac2002_parameter(tire, PAC_QFZ1, 0.0);
    if (std::abs(qfz1) <= 1e-12) {
        qfz1 = tire.k*radius/(nominal_load*load_scale);
    }
    const DirectionalScalar normalized_penetration = penetration/radius;
    const DirectionalScalar camber_scale = camber*camber
        *pac2002_parameter(tire, PAC_QFZ3, 0.0);
    const double pressure_scale = 1.0
        +pac2002_parameter(tire, PAC_QPFZ1, 0.0)
            *pac2002_pressure_difference(tire);
    const double speed_reference = std::max(
        pac2002_parameter(tire, PAC_LONGVL, 16.6), 1e-9
    );
    const DirectionalScalar speed_ratio = spin_rate*radius/speed_reference;
    DirectionalScalar force_scale = DirectionalScalar{1.0}
        +pac2002_parameter(tire, PAC_QV2, 0.0)*speed_ratio*speed_ratio
        -pac2002_parameter(tire, PAC_QFCX1, 0.0)
            *d_abs(longitudinal_force/nominal_load, smooth)
        -pac2002_parameter(tire, PAC_QFCY1, 0.0)
            *d_abs(lateral_force/nominal_load, smooth)
        -pac2002_parameter(tire, PAC_QFCG1, 0.0)*camber*camber;
    if (force_scale.value <= 0.0) {
        smooth = false;
        force_scale = {};
    }
    const DirectionalScalar elastic_force = nominal_load*load_scale
        *pressure_scale*force_scale* (
            qfz1*normalized_penetration
            +pac2002_parameter(tire, PAC_QFZ2, 0.0)
                *normalized_penetration*normalized_penetration
            +camber_scale*normalized_penetration
        );
    // Maxwell branch, the same expression the double path uses; z_m is a per-step
    // constant, so the dual captures d(F_m)/d(delta) = K_dyn exactly.
    const double dynamic_stiffness =
        pac2002_parameter(tire, PAC_DYNAMIC_STIFFNESS, 0.0);
    const DirectionalScalar maxwell_force = tire.maxwell_enabled
        ? dynamic_stiffness*(penetration-maxwell_displacement)
        : DirectionalScalar{};
    // [DEFLECTION_LOAD_CURVE] replaces the stiffness polynomial outright, exactly as
    // in the plain path; the table's own slope carries the elastic derivative so the
    // analytic Jacobian stays exact.
    DirectionalScalar result;
    if (tire.deflection_curve.size() >= 4) {
        const CurveValue curve = monotone_cubic(
            tire.deflection_curve, penetration.value
        );
        const DirectionalScalar tabulated{
            std::max(0.0, curve.value),
            curve.value > 0.0 ? curve.slope : 0.0
        };
        result = tabulated*pressure_scale*force_scale
            +tire.c*penetration_rate+maxwell_force;
    } else {
        result = elastic_force+tire.c*penetration_rate+maxwell_force;
    }
    if (result.value <= 0.0) {
        smooth = false;
        // The rim force is added on its own; a light tire still gets it.
        const double rim = pac2002_bottoming_force(tire, penetration.value);
        if (rim <= 0.0) return {};
        return DirectionalScalar{rim, 0.0};
    }
    const double rim_force = pac2002_bottoming_force(tire, penetration.value);
    if (rim_force > 0.0) {
        // The rim curve is a function of the penetration, so its derivative enters
        // like the tire's own elastic term.
        const double limit = pac2002_bottoming_rim_limit(tire);
        const CurveValue rim_curve = monotone_cubic(
            tire.bottoming_curve, std::max(penetration.value-limit, 0.0)
        );
        const double rim_slope = rim_curve.value > 0.0 ? rim_curve.slope : 0.0;
        const DirectionalScalar rim{
            rim_force, rim_slope*penetration.derivative
        };
        return result+rim;
    }
    return result;
}

DirectionalScalar pac2002_relaxation_length_directional(
    const Tire& tire, const DirectionalScalar& normal_force, bool lateral,
    const DirectionalScalar& camber, bool& smooth
) {
    const double reference_load = pac2002_reference_load(tire);
    const DirectionalScalar dfz =
        (normal_force-reference_load)/reference_load;
    DirectionalScalar length;
    if (lateral) {
        const double pty2 = pac2002_parameter(tire, PAC_PTY2, 1.0);
        const double denominator = std::max(
            std::abs(pty2*reference_load), 1e-9
        );
        length = pac2002_parameter(tire, PAC_PTY1, 1.0)
            *d_sin(2.0*d_atan2(
                normal_force, DirectionalScalar{denominator}
            ))*tire.radius
            *(
                1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)
                    *d_abs(
                        camber*pac2002_parameter(tire, PAC_LGAY, 1.0),
                        smooth
                    )
            )*pac2002_positive_scale(tire, PAC_LFZO, 1.0)
                *pac2002_positive_scale(tire, PAC_LSGAL, 1.0);
    } else {
        length = normal_force * (
            pac2002_parameter(tire, PAC_PTX1, 1.0)
            +pac2002_parameter(tire, PAC_PTX2, 0.0)*dfz
        ) * d_exp(pac2002_parameter(tire, PAC_PTX3, 0.0)*dfz)
            *tire.radius/reference_load
            *pac2002_positive_scale(tire, PAC_LSGKP, 1.0);
    }
    const double fallback = lateral
        ? tire.relaxation_length_lateral
        : tire.relaxation_length_longitudinal;
    if (tire.model_kind != VEHICLE_TIRE_PAC2002_ADAMS_SOURCE
        && length.value <= fallback) {
        smooth = false;
        return {fallback, 0.0};
    }
    if (length.value <= 0.0) {
        smooth = false;
        return {1e-9, 0.0};
    }
    return length;
}

DirectionalScalar pac2002_gyroscopic_moment_directional(
    const Tire& tire, const DirectionalScalar& normal_force,
    const DirectionalScalar& camber,
    const DirectionalScalar& effective_lateral_slip,
    const DirectionalScalar& effective_lateral_slip_rate,
    const DirectionalScalar& rolling_radius,
    const DirectionalScalar& spin_rate, bool& smooth
) {
    const double coefficient = pac2002_parameter(tire, PAC_QTZ1, 0.0)
        *pac2002_parameter(tire, PAC_LGYR, 1.0)
        *pac2002_parameter(tire, PAC_MBELT, 0.0);
    if (std::abs(coefficient) <= 1e-12 || normal_force.value <= 0.0 ||
        rolling_radius.value <= 0.0) {
        if (normal_force.value <= 0.0 || rolling_radius.value <= 0.0) {
            smooth = false;
        }
        return {};
    }
    const DirectionalScalar sigma_alpha =
        pac2002_relaxation_length_directional(
            tire, normal_force, true, camber, smooth
        );
    const DirectionalScalar sine = d_sin(effective_lateral_slip);
    const DirectionalScalar cosine = d_cos(effective_lateral_slip);
    if (std::abs(cosine.value) <= 1e-12) {
        smooth = false;
        return {};
    }
    const DirectionalScalar tangent = sine/cosine;
    const DirectionalScalar lateral_deflection_rate = sigma_alpha
        *(1.0+tangent*tangent)*effective_lateral_slip_rate;
    const DirectionalScalar result = coefficient
        *(rolling_radius*spin_rate)*lateral_deflection_rate;
    if (!std::isfinite(result.value) || !std::isfinite(result.derivative)) {
        smooth = false;
        return {};
    }
    return result;
}

DirectionalScalar pac2002_peak_force_directional(
    const Tire& tire, const DirectionalScalar& normal_force,
    bool lateral, const DirectionalScalar& camber, bool& smooth
) {
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double reference_load = pac2002_reference_load(tire);
    const DirectionalScalar dfz = (normal_force-reference_load)/reference_load;
    const double dpi = pac2002_pressure_difference(tire);
    DirectionalScalar mu;
    if (lateral) {
        const DirectionalScalar gamma_y = camber
            *pac2002_parameter(tire, PAC_LGAY, 1.0);
        mu = (
            pac2002_parameter(tire, PAC_PDY1, 1.0)
            +pac2002_parameter(tire, PAC_PDY2, 0.0)*dfz
        ) * (
            1.0+pac2002_parameter(tire, PAC_PDY3, 0.0)
                *gamma_y*gamma_y
        ) * (
            1.0+pac2002_parameter(tire, PAC_PPY3, 0.0)*dpi
                +pac2002_parameter(tire, PAC_PPY4, 0.0)*dpi*dpi
        )*pac2002_parameter(tire, PAC_LMUY, 1.0);
    } else {
        const DirectionalScalar gamma_x = camber
            *pac2002_parameter(tire, PAC_LGAX, 1.0);
        mu = (
            pac2002_parameter(tire, PAC_PDX1, 1.0)
            +pac2002_parameter(tire, PAC_PDX2, 0.0)*dfz
        ) * (
            1.0-pac2002_parameter(tire, PAC_PDX3, 0.0)
                *gamma_x*gamma_x
        ) * (
            1.0+pac2002_parameter(tire, PAC_PPX3, 0.0)*dpi
                +pac2002_parameter(tire, PAC_PPX4, 0.0)*dpi*dpi
        )*pac2002_parameter(tire, PAC_LMUX, 1.0);
    }
    return d_abs(mu*normal_force, smooth);
}

DirectionalScalar pac2002_pure_force_directional(
    const Tire& tire, const DirectionalScalar& slip,
    const DirectionalScalar& normal_force, bool lateral,
    const DirectionalScalar& camber,
    const Pac2002TurnSlipDirectional& turn_slip, bool& smooth
) {
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double reference_load = pac2002_reference_load(tire);
    const DirectionalScalar dfz = (normal_force-reference_load)/reference_load;
    const double dpi = pac2002_pressure_difference(tire);
    const double c = lateral
        ? pac2002_parameter(tire, PAC_PCY1, 1.3)
            *pac2002_parameter(tire, PAC_LCY, 1.0)
        : pac2002_parameter(tire, PAC_PCX1, 1.65)
            *pac2002_parameter(tire, PAC_LCX, 1.0);
    // Steady-state spin factors, the same template the double path uses, so the
    // force value the analytic Jacobian differentiates is the one the residual
    // evaluates.
    const Pac2002SpinFactorsDirectional factors = pac2002_spin_factors_directional(
        tire, turn_slip.force, normal_force, camber,
        lateral ? DirectionalScalar{} : slip,
        lateral ? slip : DirectionalScalar{}, turn_slip.travel_sign, smooth
    );
    const DirectionalScalar d = pac2002_peak_force_directional(
        tire, normal_force, lateral, camber, smooth
    )*(lateral ? factors.zeta2 : factors.zeta1);
    if (d.value <= 0.0 || c <= 0.0) {
        smooth = false;
        return {};
    }
    DirectionalScalar stiffness;
    DirectionalScalar e;
    DirectionalScalar sh;
    DirectionalScalar sv;
    if (lateral) {
        const DirectionalScalar gamma_y = camber
            *pac2002_parameter(tire, PAC_LGAY, 1.0);
        const double pky2 = pac2002_parameter(tire, PAC_PKY2, 0.0);
        const DirectionalScalar ky0 = std::abs(pky2) > 1e-12
            ? pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
                *nominal
                *(1.0+pac2002_parameter(tire, PAC_PPY1, 0.0)*dpi)
                *d_sin(2.0*d_atan2(
                    normal_force, DirectionalScalar{
                        pky2*reference_load
                        *(1.0+pac2002_parameter(tire, PAC_PPY2, 0.0)*dpi)
                    }
                ))
                *pac2002_parameter(tire, PAC_LFZO, 1.0)
                *pac2002_parameter(tire, PAC_LMUY, 1.0)
            : normal_force
                *pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
                *pac2002_parameter(tire, PAC_LFZO, 1.0)
                *pac2002_parameter(tire, PAC_LMUY, 1.0);
        stiffness = ky0 * (
            1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)
                *d_abs(gamma_y, smooth)
        )*factors.zeta3;
        sh = (
            pac2002_parameter(tire, PAC_PHY1, 0.0)
            +pac2002_parameter(tire, PAC_PHY2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LHY, 1.0)
            +pac2002_parameter(tire, PAC_PHY3, 0.0)*gamma_y
                *pac2002_parameter(tire, PAC_LKYG, 1.0)*factors.zeta0
            +(factors.zeta4-1.0);
        const double sign = d_sign(slip+sh, smooth);
        e = (
            pac2002_parameter(tire, PAC_PEY1, 0.0)
            +pac2002_parameter(tire, PAC_PEY2, 0.0)*dfz
        ) * (
            1.0-(pac2002_parameter(tire, PAC_PEY3, 0.0)
                +pac2002_parameter(tire, PAC_PEY4, 0.0)*gamma_y)*sign
        )*pac2002_parameter(tire, PAC_LEY, 1.0);
        sv = normal_force * (
            (
                pac2002_parameter(tire, PAC_PVY1, 0.0)
                +pac2002_parameter(tire, PAC_PVY2, 0.0)*dfz
            )*pac2002_parameter(tire, PAC_LVY, 1.0)
            +(
                pac2002_parameter(tire, PAC_PVY3, 0.0)
                +pac2002_parameter(tire, PAC_PVY4, 0.0)*dfz
            )*gamma_y*pac2002_parameter(tire, PAC_LKYG, 1.0)
        )*pac2002_parameter(tire, PAC_LMUY, 1.0)*factors.zeta2;
    } else {
        stiffness = normal_force * (
            pac2002_parameter(tire, PAC_PKX1, 120000.0/4850.0)
            +pac2002_parameter(tire, PAC_PKX2, 0.0)*dfz
        ) * d_exp(pac2002_parameter(tire, PAC_PKX3, 0.0)*dfz)
            *(1.0+pac2002_parameter(tire, PAC_PPX1, 0.0)*dpi
                +pac2002_parameter(tire, PAC_PPX2, 0.0)*dpi*dpi)
            *pac2002_parameter(tire, PAC_LKX, 1.0);
        sh = (
            pac2002_parameter(tire, PAC_PHX1, 0.0)
            +pac2002_parameter(tire, PAC_PHX2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LHX, 1.0);
        const double sign = d_sign(slip+sh, smooth);
        e = (
            pac2002_parameter(tire, PAC_PEX1, 0.0)
            +pac2002_parameter(tire, PAC_PEX2, 0.0)*dfz
            +pac2002_parameter(tire, PAC_PEX3, 0.0)*dfz*dfz
        ) * (
            1.0-pac2002_parameter(tire, PAC_PEX4, 0.0)*sign
        )*pac2002_parameter(tire, PAC_LEX, 1.0);
        sv = normal_force * (
            pac2002_parameter(tire, PAC_PVX1, 0.0)
                +pac2002_parameter(tire, PAC_PVX2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LVX, 1.0)
            *pac2002_parameter(tire, PAC_LMUX, 1.0)*factors.zeta1;
    }
    if (e.value > 1.0) {
        smooth = false;
        e = {1.0, 0.0};
    }
    const DirectionalScalar b = stiffness/(c*d+0.1);
    const auto raw = [&](const DirectionalScalar& value) {
        const DirectionalScalar argument = d_clamp(
            b*(value+sh), -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
        );
        const DirectionalScalar arctangent = d_atan2(
            argument, DirectionalScalar{1.0}
        );
        return d* d_sin(c*(arctangent-e*(argument-arctangent)))+sv;
    };
    if (tire.model_kind == VEHICLE_TIRE_PAC2002_ADAMS_SOURCE) {
        return raw(slip);
    }
    return raw(slip)-raw(DirectionalScalar{});
}

DirectionalScalar pac2002_force_limit_directional(
    const Tire& tire, const DirectionalScalar& normal_force, bool lateral,
    const DirectionalScalar& camber, bool& smooth
) {
    return pac2002_peak_force_directional(
        tire, normal_force, lateral, camber, smooth
    );
}

DirectionalScalar pac2002_combined_ratio_directional(
    const DirectionalScalar& numerator, const DirectionalScalar& denominator,
    bool& smooth
) {
    if (!std::isfinite(denominator.value)
        || std::abs(denominator.value) <= 1e-12) {
        smooth = false;
        return {1.0, 0.0};
    }
    return numerator/denominator;
}

DirectionalScalar pac2002_combined_longitudinal_force_directional(
    const Tire& tire, const DirectionalScalar& kappa,
    const DirectionalScalar& alpha, const DirectionalScalar& normal_force,
    const DirectionalScalar& camber, const DirectionalScalar& pure_force,
    bool& smooth
) {
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const DirectionalScalar dfz =
        (normal_force-pac2002_reference_load(tire))
        /pac2002_reference_load(tire);
    const double sh = pac2002_parameter(tire, PAC_RHX1, 0.0);
    const double c = pac2002_parameter(tire, PAC_RCX1, 1.0);
    DirectionalScalar e =
        pac2002_parameter(tire, PAC_REX1, 0.0)
            +pac2002_parameter(tire, PAC_REX2, 0.0)*dfz
        ;
    if (e.value > 1.0) {
        smooth = false;
        e = {1.0, 0.0};
    }
    const DirectionalScalar b = d_abs(
        pac2002_parameter(tire, PAC_RBX1, 0.0)
            *d_cos(d_atan2(
                pac2002_parameter(tire, PAC_RBX2, 0.0)*kappa,
                {1.0, 0.0}
            ))*pac2002_parameter(tire, PAC_LXAL, 1.0),
        smooth
    );
    const auto combined_shape = [&](const DirectionalScalar& value) {
        const DirectionalScalar argument = d_clamp(
            b*value, -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
        );
        const DirectionalScalar arctangent = d_atan2(
            argument, {1.0, 0.0}
        );
        return d_cos(c* (arctangent-e*(argument-arctangent)));
    };
    const DirectionalScalar scale = pac2002_combined_ratio_directional(
        combined_shape(alpha+sh), combined_shape(DirectionalScalar{sh}),
        smooth
    );
    (void)camber;
    return pure_force*scale;
}

DirectionalScalar pac2002_combined_lateral_force_directional(
    const Tire& tire, const DirectionalScalar& kappa,
    const DirectionalScalar& alpha, const DirectionalScalar& normal_force,
    const DirectionalScalar& camber, const DirectionalScalar& pure_force,
    bool& smooth
) {
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const DirectionalScalar dfz =
        (normal_force-pac2002_reference_load(tire))
        /pac2002_reference_load(tire);
    const DirectionalScalar sh = pac2002_parameter(tire, PAC_RHY1, 0.0)
        +pac2002_parameter(tire, PAC_RHY2, 0.0)*dfz;
    const double c = pac2002_parameter(tire, PAC_RCY1, 1.0);
    DirectionalScalar e =
        pac2002_parameter(tire, PAC_REY1, 0.0)
            +pac2002_parameter(tire, PAC_REY2, 0.0)*dfz
        ;
    if (e.value > 1.0) {
        smooth = false;
        e = {1.0, 0.0};
    }
    const DirectionalScalar b =
        pac2002_parameter(tire, PAC_RBY1, 0.0)
        *d_cos(d_atan2(
            pac2002_parameter(tire, PAC_RBY2, 0.0)
                *(alpha-pac2002_parameter(tire, PAC_RBY3, 0.0)),
            {1.0, 0.0}
        ))*pac2002_parameter(tire, PAC_LYKA, 1.0);
    const auto combined_shape = [&](const DirectionalScalar& value) {
        const DirectionalScalar argument = d_clamp(
            b*value, -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
        );
        const DirectionalScalar arctangent = d_atan2(
            argument, {1.0, 0.0}
        );
        return d_cos(c* (arctangent-e*(argument-arctangent)));
    };
    const DirectionalScalar scale = pac2002_combined_ratio_directional(
        combined_shape(kappa+sh), combined_shape(sh),
        smooth
    );
    const DirectionalScalar lateral_peak =
        pac2002_force_limit_directional(
            tire, normal_force, true, camber, smooth
        );
    const DirectionalScalar velocity_offset = lateral_peak* (
        pac2002_parameter(tire, PAC_RVY1, 0.0)
        +pac2002_parameter(tire, PAC_RVY2, 0.0)*dfz
        +pac2002_parameter(tire, PAC_RVY3, 0.0)*camber
    ) *d_cos(d_atan2(
        pac2002_parameter(tire, PAC_RVY4, 0.0)*alpha,
        {1.0, 0.0}
    ))*pac2002_parameter(tire, PAC_LVYKA, 1.0);
    const DirectionalScalar combined_offset = velocity_offset*d_sin(
        pac2002_parameter(tire, PAC_RVY5, 0.0)*d_atan2(
            pac2002_parameter(tire, PAC_RVY6, 0.0)*kappa,
            {1.0, 0.0}
        )
    );
    return pure_force*scale+combined_offset;
}

DirectionalScalar pac2002_aligning_moment_directional(
    const Tire& tire, const DirectionalScalar& kappa,
    const DirectionalScalar& alpha, const DirectionalScalar& normal_force,
    const DirectionalScalar& camber, const DirectionalScalar& fx,
    const DirectionalScalar& fy, const Pac2002TurnSlipDirectional& turn_slip,
    bool& smooth
) {
    if (!pac2002_has_aligning_moment_terms(tire)) return {};
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    // The moment channel takes the filtered phi'_M (Eq3972); the same factor
    // template the double path uses keeps the value the analytic Jacobian
    // differentiates equal to the one the residual evaluates.
    const Pac2002SpinFactorsDirectional factors =
        pac2002_spin_factors_directional(
            tire, turn_slip.moment, normal_force, camber, kappa, alpha,
            turn_slip.travel_sign, smooth
        );
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double reference_load = pac2002_reference_load(tire);
    const DirectionalScalar dfz = (normal_force-reference_load)/reference_load;
    const double dpi = pac2002_pressure_difference(tire);
    const double pky2 = pac2002_parameter(tire, PAC_PKY2, 0.0);
    const DirectionalScalar ky0 = std::abs(pky2) > 1e-12
        ? pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
            *nominal*(1.0+pac2002_parameter(tire, PAC_PPY1, 0.0)*dpi)
            *d_sin(2.0*d_atan2(
                normal_force, DirectionalScalar{
                    pky2*reference_load
                    *(1.0+pac2002_parameter(tire, PAC_PPY2, 0.0)*dpi)
                }
            ))
            *pac2002_parameter(tire, PAC_LFZO, 1.0)
            *pac2002_parameter(tire, PAC_LMUY, 1.0)
        : normal_force
            *pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0)
            *pac2002_parameter(tire, PAC_LFZO, 1.0)
            *pac2002_parameter(tire, PAC_LMUY, 1.0);
    const DirectionalScalar ky = ky0* (
        1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)
            *d_abs(camber*pac2002_parameter(tire, PAC_LGAY, 1.0), smooth)
    )*factors.zeta3;
    if (std::abs(ky.value) <= 1e-12) {
        smooth = false;
        return {};
    }
    const DirectionalScalar kx = normal_force* (
        pac2002_parameter(tire, PAC_PKX1, 120000.0/4850.0)
        +pac2002_parameter(tire, PAC_PKX2, 0.0)*dfz
    )*d_exp(pac2002_parameter(tire, PAC_PKX3, 0.0)*dfz)
        *(1.0+pac2002_parameter(tire, PAC_PPX1, 0.0)*dpi
            +pac2002_parameter(tire, PAC_PPX2, 0.0)*dpi*dpi)
        *pac2002_parameter(tire, PAC_LKX, 1.0);
    const DirectionalScalar gamma_y = camber
        *pac2002_parameter(tire, PAC_LGAY, 1.0);
    const DirectionalScalar gamma_z = camber
        *pac2002_parameter(tire, PAC_LGAZ, 1.0);
    const DirectionalScalar shy = (
        pac2002_parameter(tire, PAC_PHY1, 0.0)
        +pac2002_parameter(tire, PAC_PHY2, 0.0)*dfz
    )*pac2002_parameter(tire, PAC_LHY, 1.0)
        +pac2002_parameter(tire, PAC_PHY3, 0.0)*gamma_y
            *pac2002_parameter(tire, PAC_LKYG, 1.0)*factors.zeta0
        +(factors.zeta4-1.0);
    const DirectionalScalar svy = normal_force* (
        (
            pac2002_parameter(tire, PAC_PVY1, 0.0)
            +pac2002_parameter(tire, PAC_PVY2, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LVY, 1.0)
        +(
            pac2002_parameter(tire, PAC_PVY3, 0.0)
            +pac2002_parameter(tire, PAC_PVY4, 0.0)*dfz
        )*gamma_y*pac2002_parameter(tire, PAC_LKYG, 1.0)
    )*pac2002_parameter(tire, PAC_LMUY, 1.0)*factors.zeta2;
    const DirectionalScalar sht = pac2002_parameter(tire, PAC_QHZ1, 0.0)
        +pac2002_parameter(tire, PAC_QHZ2, 0.0)*dfz
        +(
            pac2002_parameter(tire, PAC_QHZ3, 0.0)
        +pac2002_parameter(tire, PAC_QHZ4, 0.0)*dfz
        )*gamma_z;
    const DirectionalScalar alpha_r = alpha+shy+svy/ky;
    const DirectionalScalar alpha_t = alpha+sht;
    const double ct = pac2002_parameter(tire, PAC_QCZ1, 1.0);
    if (ct <= 0.0) {
        smooth = false;
        return {};
    }
    const DirectionalScalar bt = d_abs(
        pac2002_parameter(tire, PAC_QBZ1, 0.0)
        +pac2002_parameter(tire, PAC_QBZ2, 0.0)*dfz
        +pac2002_parameter(tire, PAC_QBZ3, 0.0)*dfz*dfz,
        smooth
    )*d_abs(
        1.0
        +pac2002_parameter(tire, PAC_QBZ4, 0.0)*gamma_z
        +pac2002_parameter(tire, PAC_QBZ5, 0.0)*d_abs(gamma_z, smooth),
        smooth
    )*pac2002_parameter(tire, PAC_LKY, 1.0)
        /std::max(pac2002_parameter(tire, PAC_LMUY, 1.0), 1e-9);
    DirectionalScalar et = (
        pac2002_parameter(tire, PAC_QEZ1, 0.0)
        +pac2002_parameter(tire, PAC_QEZ2, 0.0)*dfz
        +pac2002_parameter(tire, PAC_QEZ3, 0.0)*dfz*dfz
    )*(
        1.0+(
            pac2002_parameter(tire, PAC_QEZ4, 0.0)
            +pac2002_parameter(tire, PAC_QEZ5, 0.0)*gamma_z
        )*(2.0/kPi)*d_atan2(
                bt*pac2002_parameter(tire, PAC_QCZ1, 1.0)*alpha_t,
                {1.0, 0.0}
            )
    );
    if (et.value > 1.0) {
        smooth = false;
        et = {1.0, 0.0};
    }
    const DirectionalScalar dt = normal_force* (
        pac2002_parameter(tire, PAC_QDZ1, 0.0)
        +pac2002_parameter(tire, PAC_QDZ2, 0.0)*dfz
    )* (
        1.0
        +pac2002_parameter(tire, PAC_QDZ3, 0.0)*gamma_z
        +pac2002_parameter(tire, PAC_QDZ4, 0.0)*gamma_z*gamma_z
    )*tire.radius/reference_load
        *(1.0-pac2002_parameter(tire, PAC_QPZ1, 0.0)*dpi)
        *pac2002_parameter(tire, PAC_LTR, 1.0)*factors.zeta5;
    const auto trail = [&](const DirectionalScalar& value) {
        const DirectionalScalar argument = d_clamp(
            bt*value, -0.5*kPi+0.01, 0.5*kPi-0.01, smooth
        );
        return dt*d_cos(
            ct*d_atan2(
                argument-et*(argument-d_atan2(argument, {1.0, 0.0})),
                {1.0, 0.0}
            )
        )*d_cos(alpha);
    };
    const DirectionalScalar dy = pac2002_force_limit_directional(
        tire, normal_force, true, camber, smooth
    );
    const DirectionalScalar by = ky/(ct*dy+0.1);
    const DirectionalScalar br = (pac2002_parameter(tire, PAC_QBZ9, 0.0)
        *pac2002_parameter(tire, PAC_LKY, 1.0)
        /std::max(pac2002_parameter(tire, PAC_LMUY, 1.0), 1e-9)
        +pac2002_parameter(tire, PAC_QBZ10, 0.0)*by*ct)*factors.zeta6;
    const DirectionalScalar dr = normal_force* (
        (
            pac2002_parameter(tire, PAC_QDZ6, 0.0)
            +pac2002_parameter(tire, PAC_QDZ7, 0.0)*dfz
        )*pac2002_parameter(tire, PAC_LRES, 1.0)
        +(
            pac2002_parameter(tire, PAC_QDZ8, 0.0)
            +pac2002_parameter(tire, PAC_QDZ9, 0.0)*dfz
        )*(1.0+pac2002_parameter(tire, PAC_QPZ2, 0.0)*dpi)*gamma_z
    )*tire.radius*pac2002_parameter(tire, PAC_LMUY, 1.0)
        +(factors.zeta8-1.0);
    DirectionalScalar mz = trail(alpha_r)*fy
        -dr*d_cos(factors.zeta7*d_atan2(br*alpha_r, {1.0, 0.0}))*d_cos(alpha);
    if (pac2002_has_combined_slip_terms(tire)) {
        const DirectionalScalar lateral_peak = dy;
        const DirectionalScalar velocity_offset = lateral_peak* (
            pac2002_parameter(tire, PAC_RVY1, 0.0)
            +pac2002_parameter(tire, PAC_RVY2, 0.0)*dfz
            +pac2002_parameter(tire, PAC_RVY3, 0.0)*camber
        )*d_cos(d_atan2(
            pac2002_parameter(tire, PAC_RVY4, 0.0)*alpha, {1.0, 0.0}
        ));
        const DirectionalScalar svyk = -velocity_offset*d_sin(
            pac2002_parameter(tire, PAC_RVY5, 0.0)*d_atan2(
                pac2002_parameter(tire, PAC_RVY6, 0.0)*kappa,
                {1.0, 0.0}
            )
        );
        const DirectionalScalar tan_t = d_sin(alpha_t)/d_cos(alpha_t);
        const DirectionalScalar tan_r = d_sin(alpha_r)/d_cos(alpha_r);
        const DirectionalScalar combined_tangent = d_sqrt(
            tan_t*tan_t+(kx/ky)*(kx/ky)*kappa*kappa
        );
        const DirectionalScalar combined_radial = d_sqrt(
            tan_r*tan_r+(kx/ky)*(kx/ky)*kappa*kappa
        );
        const double sign_kappa = d_sign(kappa, smooth);
        const double sign_alpha_r = d_sign(alpha_r, smooth);
        const DirectionalScalar alpha_teq = d_atan2(
            combined_tangent, {1.0, 0.0}
        )*sign_kappa;
        const DirectionalScalar alpha_req = d_atan2(
            combined_radial, {1.0, 0.0}
        )*sign_alpha_r;
        const DirectionalScalar s = (
            pac2002_parameter(tire, PAC_SSZ1, 0.0)
            -pac2002_parameter(tire, PAC_SSZ2, 0.0)*fy/reference_load
            +(
                pac2002_parameter(tire, PAC_SSZ3, 0.0)
                +pac2002_parameter(tire, PAC_SSZ4, 0.0)*dfz
            )*gamma_z
        )*tire.radius*pac2002_parameter(tire, PAC_LS, 1.0);
        mz = trail(alpha_teq)*(fy-svyk)
            -dr*d_cos(factors.zeta7*d_atan2(br*alpha_req, {1.0, 0.0}))
                *d_cos(alpha)
            -s*fx;
    }
    if (!std::isfinite(mz.value) || !std::isfinite(mz.derivative)) {
        smooth = false;
        return {};
    }
    return mz*(-1.0);
}

DirectionalScalar pac2002_overturning_moment_directional(
    const Tire& tire, const DirectionalScalar& fy_source,
    const DirectionalScalar& normal_force, const DirectionalScalar& camber,
    bool& smooth
) {
    if (!pac2002_has_overturning_moment_terms(tire)) return {};
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double reference_load = pac2002_reference_load(tire);
    const double pressure_difference = pac2002_pressure_difference(tire);
    const DirectionalScalar fy_ratio = fy_source/reference_load;
    const DirectionalScalar load_ratio = normal_force/reference_load;
    const DirectionalScalar bracket =
        pac2002_parameter(tire, PAC_QSX3, 0.0)*fy_ratio
        +pac2002_parameter(tire, PAC_QSX4, 0.0)
            *d_cos(pac2002_parameter(tire, PAC_QSX5, 0.0)
                *d_atan2(load_ratio*load_ratio, {1.0, 0.0}))
            *d_sin(
                pac2002_parameter(tire, PAC_QSX7, 0.0)*camber
                +pac2002_parameter(tire, PAC_QSX8, 0.0)
                    *d_atan2(
                        pac2002_parameter(tire, PAC_QSX9, 0.0)*fy_ratio,
                        {1.0, 0.0}
                    )
            )
        +(
            pac2002_parameter(tire, PAC_QSX10, 0.0)
                *d_atan2(
                    pac2002_parameter(tire, PAC_QSX11, 0.0)*load_ratio,
                    {1.0, 0.0}
                )
            -pac2002_parameter(tire, PAC_QSX2, 0.0)
                *(1.0+pac2002_parameter(tire, PAC_QPX1, 0.0)
                    *pressure_difference)
        )*camber
        +pac2002_parameter(tire, PAC_QSX1, 0.0)
            *pac2002_parameter(tire, PAC_LVMX, 1.0);
    const DirectionalScalar moment = normal_force*tire.radius*bracket
        *pac2002_parameter(tire, PAC_LMX, 1.0);
    if (!std::isfinite(moment.value) || !std::isfinite(moment.derivative)) {
        smooth = false;
        return {};
    }
    return moment;
}

DirectionalScalar pac2002_rolling_resistance_moment_directional(
    const Tire& tire, const DirectionalScalar& fx_source,
    const DirectionalScalar& normal_force, const DirectionalScalar& camber,
    const DirectionalScalar& longitudinal_speed, bool& smooth
) {
    if (!pac2002_has_rolling_resistance_terms(tire)) return {};
    if (normal_force.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double pressure_ratio = std::max(
        pac2002_parameter(tire, PAC_IP, 200000.0)
            /std::max(pac2002_parameter(tire, PAC_IP_NOM, 200000.0), 1e-9),
        1e-12
    );
    const double reference_speed = std::sqrt(9.81*tire.radius);
    const DirectionalScalar speed_ratio = longitudinal_speed/
        std::max(reference_speed, 1e-9);
    const DirectionalScalar load_ratio = normal_force/nominal;
    const DirectionalScalar bracket =
        pac2002_parameter(tire, PAC_QSY1, 0.0)
        +pac2002_parameter(tire, PAC_QSY2, 0.0)*fx_source/nominal
        +pac2002_parameter(tire, PAC_QSY3, 0.0)
            *d_abs(speed_ratio, smooth)
        +pac2002_parameter(tire, PAC_QSY4, 0.0)
            *speed_ratio*speed_ratio*speed_ratio*speed_ratio
        +pac2002_parameter(tire, PAC_QSY5, 0.0)*camber*camber
        +pac2002_parameter(tire, PAC_QSY6, 0.0)*camber*camber*load_ratio;
    const DirectionalScalar load_scale = d_pow_positive(
        load_ratio, pac2002_parameter(tire, PAC_QSY7, 0.0), smooth
    );
    const double pressure_scale = std::pow(
        pressure_ratio, pac2002_parameter(tire, PAC_QSY8, 0.0)
    );
    const DirectionalScalar moment = normal_force*tire.radius*bracket
        *load_scale*pressure_scale*pac2002_parameter(tire, PAC_LMY, 1.0);
    if (!std::isfinite(moment.value) || !std::isfinite(moment.derivative)) {
        smooth = false;
        return {};
    }
    // PAC2002 的 My 为 SAE 轮胎坐标约定；native 侧向轴与 SAE y 轴相反。
    return -moment;
}

} // namespace axle_kernel
