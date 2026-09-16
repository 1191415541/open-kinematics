// K6 (epic MODULES.md section 2.5): one tire model's law.
//
// Split out of `kernel_model.cpp` by top-level function; the bodies are verbatim.  The
// three tire models are separate libraries so the link graph enforces that they
// do not call each other.

#include "mb_tire/brush/functions.hpp"

namespace axle_kernel {

void project_brush_state(
    const Tire& tire, double normal_force, double sx, double sy,
    double& projected_sx, double& projected_sy, double& trial_utilization
) {
    projected_sx = sx;
    projected_sy = sy;
    trial_utilization = 0.0;
    if (normal_force <= 0.0) return;
    const double longitudinal_limit =
        tire.mu_longitudinal * normal_force;
    const double lateral_limit = tire.mu_lateral * normal_force;
    const double normalized_x =
        tire.brush_k_longitudinal * sx / longitudinal_limit;
    const double normalized_y =
        tire.brush_k_lateral * sy / lateral_limit;
    trial_utilization = std::sqrt(
        normalized_x*normalized_x + normalized_y*normalized_y
    );
    if (trial_utilization >= 1.0) {
        projected_sx /= trial_utilization;
        projected_sy /= trial_utilization;
    }
}

} // namespace axle_kernel
