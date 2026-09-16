// The bushing curve helpers, moved here from `src/base/kernel_base.cpp` when the
// transitions to per-module headers made the cost of their old home visible.
//
// `MODULES.md` section 2.6 puts them in the suspension layer, and they belong
// there: both take a `Bushing`, so leaving them in `mb_base` made the base layer
// name a model type, and their only callers are in `src/suspension/bushing.cpp`.
// The bodies are unchanged.

#include "mb_suspension/functions.hpp"

namespace axle_kernel {

std::pair<double, double> bushing_curve_value_slope(
    const Bushing& bushing, std::size_t axis, double value
) {
    const auto& x = bushing.elastic_coordinate[axis];
    const auto& y = bushing.elastic_force[axis];
    if (bushing.force_curve_interpolation == 1) {
        return interpolate_akima_curve(x, y, value);
    }
    if (x.empty() || y.empty()) return {0.0, 0.0};
    const double result = interpolate_curve(x, y, value);
    double slope = 0.0;
    if (x.size() > 1) {
        std::size_t high = 1;
        if (value > x.front() && value < x.back()) {
            while (high < x.size() && x[high] < value) ++high;
        } else if (value >= x.back()) {
            high = x.size()-1;
        }
        slope = (y[high]-y[high-1])/(x[high]-x[high-1]);
    }
    return {result, slope};
}

double integrate_bushing_curve_from_zero(
    const Bushing& bushing, std::size_t axis, double value
) {
    if (bushing.force_curve_interpolation == 1) {
        return integrate_akima_curve_from_zero(
            bushing.elastic_coordinate[axis],
            bushing.elastic_force[axis], value
        );
    }
    return integrate_curve_from_zero(
        bushing.elastic_coordinate[axis],
        bushing.elastic_force[axis], value
    );
}

} // namespace axle_kernel
