#pragma once

/// The dual number the directional (derivative-carrying) pass is written in
/// (MB_BASE).
///
/// A `DirectionalScalar` carries a value and its derivative together, so one
/// expression computes both the force and its Jacobian column.  The operators
/// and the `d_*` helpers are defined in the base translation unit.


namespace axle_kernel {

struct DirectionalScalar {
    double value{0.0};
    double derivative{0.0};

    DirectionalScalar() = default;
    DirectionalScalar(double value_, double derivative_ = 0.0)
        : value(value_), derivative(derivative_) {}

    DirectionalScalar& operator+=(const DirectionalScalar& other) {
        value += other.value;
        derivative += other.derivative;
        return *this;
    }
    DirectionalScalar& operator-=(const DirectionalScalar& other) {
        value -= other.value;
        derivative -= other.derivative;
        return *this;
    }
};

DirectionalScalar operator+( const DirectionalScalar& a, const DirectionalScalar& b );
DirectionalScalar operator-( const DirectionalScalar& a, const DirectionalScalar& b );
DirectionalScalar operator-(const DirectionalScalar& a);
DirectionalScalar operator*( const DirectionalScalar& a, const DirectionalScalar& b );
DirectionalScalar operator/( const DirectionalScalar& a, const DirectionalScalar& b );
DirectionalScalar d_sqrt(const DirectionalScalar& a);
DirectionalScalar d_sin(const DirectionalScalar& a);
DirectionalScalar d_cos(const DirectionalScalar& a);
DirectionalScalar d_tan(const DirectionalScalar& a);
DirectionalScalar d_exp(const DirectionalScalar& a);
DirectionalScalar d_pow_positive( const DirectionalScalar& a, double exponent, bool& smooth );
DirectionalScalar d_atan2( const DirectionalScalar& y, const DirectionalScalar& x );
DirectionalScalar d_abs(const DirectionalScalar& a, bool& smooth);
double d_sign(const DirectionalScalar& a, bool& smooth);
DirectionalScalar d_clamp( const DirectionalScalar& a, double lower, double upper, bool& smooth );

} // namespace axle_kernel
