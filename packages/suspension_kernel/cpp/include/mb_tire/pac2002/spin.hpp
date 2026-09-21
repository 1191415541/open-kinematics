#pragma once

/// The PAC2002 spin (camber) factors (MB_TIRE_PAC2002).
///
/// The factor set is templated on the scalar type: the scalar pass instantiates
/// it with `double` and the directional pass with `DirectionalScalar`, so one
/// body of equations serves both and the two cannot drift.  The `spin_*` helpers
/// are the same idea one level down -- `spin_sin` of a `double` is `std::sin`, of
/// a `DirectionalScalar` it is the derivative-carrying sine.

#include "mb_model/types.hpp"
#include "mb_dual/dual.hpp"
#include "mb_tire/pac2002/parameters.hpp"
#include "mb_tire/pac2002/turn_slip.hpp"
#include <cmath>

namespace axle_kernel {
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

template <typename S>
S spin_ratio(const S& numerator, const S& denominator) {
    if (std::abs(spin_value(denominator)) < 1e-9) return S{};
    return numerator/denominator;
}

template <typename S>
S pac2002_nominal_lateral_stiffness_t(const Tire& tire, const S& normal_force) {
    const double nominal = std::max(
        pac2002_parameter(tire, PAC_FNOMIN, 4850.0), 1e-9
    );
    const double reference_load = pac2002_reference_load(tire);
    const double dpi = pac2002_pressure_difference(tire);
    const double pky2 = pac2002_parameter(tire, PAC_PKY2, 0.0);
    const double kappa_y1 = pac2002_parameter(tire, PAC_PKY1, -80000.0/4850.0);
    const double load_scale = pac2002_parameter(tire, PAC_LFZO, 1.0)
        *pac2002_parameter(tire, PAC_LMUY, 1.0);
    if (std::abs(pky2) <= 1e-12) {
        return normal_force*kappa_y1*load_scale;
    }
    const S denominator = pky2*reference_load
        *(1.0+pac2002_parameter(tire, PAC_PPY2, 0.0)*dpi);
    return kappa_y1*nominal
        *(1.0+pac2002_parameter(tire, PAC_PPY1, 0.0)*dpi)
        *spin_sin(2.0*spin_atan(spin_ratio(normal_force, denominator)))
        *load_scale;
}

template <typename S>
S pac2002_lateral_friction_t(
    const Tire& tire, const S& normal_force, const S& camber
) {
    const double reference_load = pac2002_reference_load(tire);
    const double dpi = pac2002_pressure_difference(tire);
    const S dfz = spin_ratio(
        normal_force-reference_load, S{reference_load}
    );
    const S gamma_y = camber*pac2002_parameter(tire, PAC_LGAY, 1.0);
    return (pac2002_parameter(tire, PAC_PDY1, 1.0)
            +pac2002_parameter(tire, PAC_PDY2, 0.0)*dfz)
        *(1.0+pac2002_parameter(tire, PAC_PPY3, 0.0)*dpi
            +pac2002_parameter(tire, PAC_PPY4, 0.0)*dpi*dpi)
        *(1.0+pac2002_parameter(tire, PAC_PDY3, 0.0)*gamma_y*gamma_y)
        *pac2002_parameter(tire, PAC_LMUY, 1.0);
}

template <typename S>
S pac2002_combined_lateral_weight_t(
    const Tire& tire, const S& kappa, const S& alpha, const S& normal_force
) {
    const double reference_load = pac2002_reference_load(tire);
    const S dfz = spin_ratio(
        normal_force-reference_load, S{reference_load}
    );
    const S sh = pac2002_parameter(tire, PAC_RHY1, 0.0)
        +pac2002_parameter(tire, PAC_RHY2, 0.0)*dfz;
    const S b = pac2002_parameter(tire, PAC_RBY1, 0.0)
        *spin_cos(spin_atan(pac2002_parameter(tire, PAC_RBY2, 0.0)
            *(alpha-pac2002_parameter(tire, PAC_RBY3, 0.0))))
        *pac2002_parameter(tire, PAC_LYKA, 1.0);
    const double c = pac2002_parameter(tire, PAC_RCY1, 1.0);
    const double e = std::min(
        1.0,
        pac2002_parameter(tire, PAC_REY1, 0.0)
            +pac2002_parameter(tire, PAC_REY2, 0.0)*spin_value(dfz)
    );
    const S argument = b*(kappa+sh);
    return spin_cos(c*spin_atan(argument-e*(argument-spin_atan(argument))));
}


struct Pac2002SpinFactors {
    double zeta1{1.0};  // longitudinal peak force and its vertical shift
    double zeta2{1.0};  // lateral peak force and its vertical shift
    double zeta3{1.0};  // cornering stiffness
    double zeta0{1.0};  // camber part of the lateral horizontal shift
    double zeta4{1.0};  // lateral horizontal shift, entering as (zeta4 - 1)
    double zeta5{1.0};  // pneumatic trail
    double zeta6{1.0};  // residual torque slope
    double zeta7{1.0};  // residual torque shape (Cr)
    double zeta8{1.0};  // residual torque amplitude, entering as (zeta8 - 1)
};

struct Pac2002SpinFactorsDirectional {
    DirectionalScalar zeta1{1.0};
    DirectionalScalar zeta2{1.0};
    DirectionalScalar zeta3{1.0};
    DirectionalScalar zeta0{1.0};
    DirectionalScalar zeta4{1.0};
    DirectionalScalar zeta5{1.0};
    DirectionalScalar zeta6{1.0};
    DirectionalScalar zeta7{1.0};
    DirectionalScalar zeta8{1.0};
};

template <typename S>
struct SpinFactorsOf;

template <>
struct SpinFactorsOf<double> {
    using type = Pac2002SpinFactors;
};

template <>
struct SpinFactorsOf<DirectionalScalar> {
    using type = Pac2002SpinFactorsDirectional;
};

template <typename S>
typename SpinFactorsOf<S>::type pac2002_spin_factors_t(
    const Tire& tire, const S& spin, const S& normal_force, const S& camber,
    const S& longitudinal_slip, const S& lateral_slip, double travel_sign,
    bool& smooth
) {
    typename SpinFactorsOf<S>::type factors;
    if (spin_value(normal_force) <= 0.0) return factors;
    if (spin_value(spin) == 0.0) {
        // Eq3327: with the spin neglected every factor is one and both additive
        // terms vanish, i.e. exactly the pre-turn-slip law.  If the spin merely
        // *evaluates* to zero while carrying a derivative, the degenerate branch is
        // a kink of the factor, so say so and let the caller keep a numeric column.
        if (spin_derivative(spin) != 0.0) smooth = false;
        return factors;
    }
    const double reference_load = std::max(
        pac2002_reference_load(tire), 1e-9
    );
    const S dfz = spin_ratio(
        normal_force-reference_load, S{reference_load}
    );
    const S gamma_y = camber*pac2002_parameter(tire, PAC_LGAY, 1.0);
    const double radius = std::max(tire.radius, 1e-9);
    const S radius_spin = radius*spin;
    const S abs_spin = spin_abs(spin, smooth);

    // zeta1: longitudinal peak force (Eq3329/Eq3330).
    const S bx_phi = pac2002_parameter(tire, PAC_PDXP1, 0.0)
        *(1.0+pac2002_parameter(tire, PAC_PDXP2, 0.0)*dfz)
        *spin_cos(spin_atan(
            pac2002_parameter(tire, PAC_PDXP3, 0.0)*longitudinal_slip
        ));
    factors.zeta1 = spin_cos(spin_atan(bx_phi*radius_spin));

    // zeta2: lateral peak force (Eq3332/Eq3333).
    const S by_phi = pac2002_parameter(tire, PAC_PDYP1, 0.0)
        *(1.0+pac2002_parameter(tire, PAC_PDYP2, 0.0)*dfz)
        *spin_cos(spin_atan(
            pac2002_parameter(tire, PAC_PDYP3, 0.0)*spin_tan(lateral_slip)
        ));
    const S abs_radius_spin = radius*abs_spin;
    factors.zeta2 = spin_cos(spin_atan(by_phi*(
        abs_radius_spin
        +pac2002_parameter(tire, PAC_PDYP4, 0.0)*spin_sqrt(abs_radius_spin)
    )));

    // zeta3: cornering stiffness (Eq3335).
    factors.zeta3 = spin_cos(spin_atan(
        pac2002_parameter(tire, PAC_PKYP1, 0.0)*radius*radius*spin*spin
    ));

    // SHy_phi and zeta4 (Eq3336-Eq3339, Eq3306/Eq3307, Eq3342).
    const S ky0 = pac2002_nominal_lateral_stiffness_t(tire, normal_force);
    const S ky = ky0*(
        1.0-pac2002_parameter(tire, PAC_PKY3, 0.0)*spin_abs(gamma_y, smooth)
    );
    const S ky_gamma0 = (
        pac2002_parameter(tire, PAC_PHY3, 0.0)*ky0
        +normal_force*(pac2002_parameter(tire, PAC_PVY3, 0.0)
            +pac2002_parameter(tire, PAC_PVY4, 0.0)*dfz)
    )*pac2002_parameter(tire, PAC_LKYG, 1.0);
    const double epsilon_gamma = pac2002_parameter(tire, PAC_PECP1, 0.0)
        *(1.0+pac2002_parameter(tire, PAC_PECP2, 0.0)*spin_value(dfz));
    const S ky_r_phi0 = spin_ratio(
        ky_gamma0, S{std::max(1.0-epsilon_gamma, 1e-9)}
    );
    const double c_hy_phi = pac2002_parameter(tire, PAC_PHYP1, 0.0);
    // Eq3337 renders this factor ``sin(Vx)``; a sine of a velocity in m/s has no
    // meaning, and the Magic Formula literature has the direction of travel there
    // (sgn(Vx)), which is what the shift needs to flip with.  Documented gap, to be
    // adjudicated by the um25 reference.
    const double d_hy_phi = (
        pac2002_parameter(tire, PAC_PHYP2, 0.0)
        +pac2002_parameter(tire, PAC_PHYP3, 0.0)*spin_value(dfz)
    )*travel_sign;
    const double e_hy_phi = pac2002_parameter(tire, PAC_PHYP4, 0.0);
    const S b_hy_phi = spin_ratio(ky_r_phi0, ky0*(c_hy_phi*d_hy_phi));
    const S hy_argument = b_hy_phi*radius_spin;
    const S sh_y_phi = d_hy_phi*spin_sin(c_hy_phi*spin_atan(
        hy_argument-e_hy_phi*(hy_argument-spin_atan(hy_argument))
    ));
    const S sv_y_gamma = normal_force*(
        (pac2002_parameter(tire, PAC_PVY3, 0.0)
            +pac2002_parameter(tire, PAC_PVY4, 0.0)*dfz)*gamma_y
        *pac2002_parameter(tire, PAC_LKYG, 1.0)
    )*pac2002_parameter(tire, PAC_LMUY, 1.0)*factors.zeta2;
    // Eq3342: with the spin modelled the camber part of SHy is dropped (zeta0 = 0)
    // and its effect reappears inside zeta4 - 1.
    factors.zeta0 = S{0.0};
    S svy_shift = spin_ratio(sv_y_gamma, ky);
    // Isolation switch for the second half of zeta4-1 on its own.  PAC2002_TURN_SLIP_
    // DISABLE_SHY zeroes both halves, so it cannot say which one is wrong; the two have
    // different structures (sh_y_phi comes from the camber-shift stiffness Ky_gamma0
    // through a saturating sin(atan(.)), the SVy shift is the camber thrust divided by
    // the cornering stiffness) and the parking tire has every coefficient either one
    // needs, so separating them is the only way to size them.
    if (pac2002_turn_slip_switch("PAC2002_TURN_SLIP_DISABLE_SVY", 0.0) > 0.5) {
        svy_shift = S{0.0};
    }
    factors.zeta4 = 1.0+sh_y_phi-svy_shift;
    // Sizes the two halves of zeta4-1 and the moment chain next to them.  Without this
    // the comparison "force side too strong, moment side right" is an inference from
    // two switches, not a measurement.
    if (std::is_same<S, double>::value
        && std::getenv("SUSPENSION_AXLE_SPIN_TRACE") != nullptr) {
        static int spin_trace_count = 0;
        const bool print_now = spin_trace_count % 4000 == 0;
        const int print_index = spin_trace_count / 4000;
        ++spin_trace_count;
        if (print_now && print_index < 64) {
            std::fprintf(
                stderr,
                "[spin] n=%d spin=%.6g Fz=%.6g R0=%.6g ky0=%.6g ky=%.6g "
                "ky_gamma0=%.6g b_hy_phi=%.6g hy_arg=%.6g sh_y_phi=%.6g "
                "sv_y_gamma=%.6g svy/ky=%.6g zeta1=%.6g zeta2=%.6g zeta3=%.6g "
                "zeta4=%.6g\n",
                print_index, spin_value(spin), spin_value(normal_force), radius,
                spin_value(ky0), spin_value(ky), spin_value(ky_gamma0),
                spin_value(b_hy_phi), spin_value(hy_argument),
                spin_value(sh_y_phi), spin_value(sv_y_gamma),
                spin_value(svy_shift), factors.zeta1, factors.zeta2,
                factors.zeta3, factors.zeta4
            );
        }
    }
    // Isolation switch for the *force*-side spin effect.  Measured on the parking
    // maneuver: with the turn-slip family applied the front aligning moment matches
    // Adams well (r = +0.98, 3.61 against 4.13 N*m) while the vehicle-level channels
    // degrade sharply, so the additive lateral shift (zeta4 - 1, the spin-induced
    // SHy) is the suspect -- it is the only force-side term that is first order in the
    // spin rather than a saturating cosine.
    if (pac2002_turn_slip_switch("PAC2002_TURN_SLIP_DISABLE_SHY", 0.0) > 0.5) {
        factors.zeta4 = 1.0;
    }

    // zeta5 (Eq3347) and zeta6 (Eq3352): trail and residual torque slope.
    factors.zeta5 = spin_cos(spin_atan(
        pac2002_parameter(tire, PAC_QDTP1, 0.0)*radius_spin
    ));
    factors.zeta6 = spin_cos(spin_atan(
        pac2002_parameter(tire, PAC_QBRP1, 0.0)*radius_spin
    ));

    // The spin torque chain: Mz_phi_inf (Eq3348), DDr_phi (Eq3346), the spin moment
    // at 90 degrees of slip (Eq3353), the residual torque increment Dr_phi (Eq3345)
    // that enters as (zeta8 - 1), and the residual torque shape Cr = zeta7
    // (Eq3356).
    const double c_dr_phi = pac2002_parameter(tire, PAC_QDRP1, 0.0);
    if (std::abs(c_dr_phi) < 1e-12) return factors;
    const S mu_y = pac2002_lateral_friction_t(tire, normal_force, camber);
    const S mz_phi_inf = pac2002_parameter(tire, PAC_QCRP1, 0.0)
        *mu_y*radius*normal_force
        *spin_sqrt(spin_ratio(normal_force, S{reference_load}));
    const double e_dr_phi = pac2002_parameter(tire, PAC_QDRP2, 0.0);
    const S d_dr_phi = mz_phi_inf*spin_sin(0.5*kPi*c_dr_phi);
    const S kz_gamma_r0 = normal_force*radius*(
        pac2002_parameter(tire, PAC_QDZ8, 0.0)
        +pac2002_parameter(tire, PAC_QDZ9, 0.0)*dfz
    );
    const S b_dr_phi = spin_ratio(
        kz_gamma_r0,
        S{c_dr_phi}*d_dr_phi
            *(1.0-pac2002_spin_camber_reduction(tire, spin_value(normal_force)))
    );
    const S dr_argument = b_dr_phi*radius_spin;
    const S dr_phi = d_dr_phi*spin_sin(c_dr_phi*spin_atan(
        dr_argument-e_dr_phi*(dr_argument-spin_atan(dr_argument))
    ));
    factors.zeta8 = 1.0+dr_phi;
    const S mz_phi90 = mz_phi_inf*(2.0/kPi)
        *spin_atan(pac2002_parameter(tire, PAC_QCRP2, 0.0)*radius*abs_spin)
        *pac2002_combined_lateral_weight_t(
            tire, longitudinal_slip, lateral_slip, normal_force
        );
    // arccos is only defined while the spin moment stays inside the peak spin
    // torque; outside it the ratio is clamped and the point is not smooth.
    const double dd = std::abs(spin_value(d_dr_phi));
    const double ratio = dd > 1e-12 ? spin_value(mz_phi90)/dd : 0.0;
    if (ratio < 0.0 || ratio > 1.0) smooth = false;
    factors.zeta7 = (2.0/kPi)*std::acos(std::clamp(ratio, 0.0, 1.0));
    return factors;
}

} // namespace axle_kernel
