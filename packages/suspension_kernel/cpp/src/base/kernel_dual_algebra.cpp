// The dual-number algebra the directional laws are written in.
//
// A `DirectionalScalar` carries a value and its derivative together, and these are
// its arithmetic operators and the `d_*` scalar helpers: the whole numeric surface
// `external_force_directional` and the directional tire laws are expressed in.
//
// The file was called `kernel_fiala_directional.cpp` and also held the PAC2002 spin
// factors and the Fiala vertical force law.  K6 moved those to `tire/pac2002/` and
// `tire/fiala/`, which is what makes "the three tire models do not call each other"
// checkable: the one Fiala piece that needed this algebra had been the reason the
// file looked model-specific, and it is no longer here.  The bodies below are
// unchanged.
//
// `d_sign` inlines `pac2002_sign` on purpose (epic D1); calling through would make
// this generic helper depend on the PAC2002 model, which the layering forbids.

#include "mb_base/functions.hpp"

namespace axle_kernel {
DirectionalScalar operator+(
    const DirectionalScalar& a, const DirectionalScalar& b
) {
    return {a.value+b.value, a.derivative+b.derivative};
}

DirectionalScalar operator-(
    const DirectionalScalar& a, const DirectionalScalar& b
) {
    return {a.value-b.value, a.derivative-b.derivative};
}

DirectionalScalar operator-(const DirectionalScalar& a) {
    return {-a.value, -a.derivative};
}

DirectionalScalar operator*(
    const DirectionalScalar& a, const DirectionalScalar& b
) {
    return {
        a.value*b.value,
        a.derivative*b.value+a.value*b.derivative
    };
}

DirectionalScalar operator/(
    const DirectionalScalar& a, const DirectionalScalar& b
) {
    const double denominator = b.value*b.value;
    return {
        a.value/b.value,
        (a.derivative*b.value-a.value*b.derivative)/denominator
    };
}

DirectionalScalar d_sqrt(const DirectionalScalar& a) {
    const double root = std::sqrt(a.value);
    return {root, a.derivative/(2.0*root)};
}

DirectionalScalar d_sin(const DirectionalScalar& a) {
    return {std::sin(a.value), std::cos(a.value)*a.derivative};
}

DirectionalScalar d_cos(const DirectionalScalar& a) {
    return {std::cos(a.value), -std::sin(a.value)*a.derivative};
}

DirectionalScalar d_tan(const DirectionalScalar& a) {
    const double cosine = std::cos(a.value);
    return {
        std::tan(a.value),
        a.derivative/(cosine*cosine)
    };
}

DirectionalScalar d_exp(const DirectionalScalar& a) {
    const double value = std::exp(a.value);
    return {value, value*a.derivative};
}

DirectionalScalar d_pow_positive(
    const DirectionalScalar& a, double exponent, bool& smooth
) {
    if (a.value <= 0.0) {
        smooth = false;
        return {};
    }
    const double value = std::pow(a.value, exponent);
    return {value, value*exponent*a.derivative/a.value};
}

DirectionalScalar d_atan2(
    const DirectionalScalar& y, const DirectionalScalar& x
) {
    const double denominator = x.value*x.value+y.value*y.value;
    return {
        std::atan2(y.value, x.value),
        (x.value*y.derivative-y.value*x.derivative)/denominator
    };
}

DirectionalScalar d_abs(const DirectionalScalar& a, bool& smooth) {
    constexpr double kJacobianStep = 1e-7;
    const double trial_value = a.value + kJacobianStep*a.derivative;
    if (
        std::abs(a.value) <= 1e-12 ||
        a.value*trial_value <= 0.0
    ) smooth = false;
    const double sign = a.value < 0.0 ? -1.0 : 1.0;
    return {std::abs(a.value), sign*a.derivative};
}

double d_sign(const DirectionalScalar& a, bool& smooth) {
    if (std::abs(a.value) <= 1e-12) smooth = false;
    // Inlined from `pac2002_sign` on purpose (epic D1).  Calling through would
    // make this generic dual-number helper depend on the PAC2002 tire module,
    // which the layering forbids.  The expression is bit-identical to
    // `pac2002_sign`: both return 1.0 for +0.0, -0.0 and NaN, and -1.0
    // otherwise.
    return a.value < 0.0 ? -1.0 : 1.0;
}

DirectionalScalar d_clamp(
    const DirectionalScalar& a, double lower, double upper, bool& smooth
) {
    constexpr double kJacobianStep = 1e-7;
    const double trial_value = a.value+kJacobianStep*a.derivative;
    if (
        a.value <= lower || a.value >= upper ||
        trial_value <= lower || trial_value >= upper
    ) {
        smooth = false;
    }
    if (a.value <= lower) return {lower, 0.0};
    if (a.value >= upper) return {upper, 0.0};
    return a;
}






















} // namespace axle_kernel
