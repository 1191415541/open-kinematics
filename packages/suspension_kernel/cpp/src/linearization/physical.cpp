// The physical tangent assembly of `mb_linearization`.
//
// This unit was extracted from `mb_solve_static`'s `static_jacobian`, which had
// grown to hold two different things: the *physics* of a working sample (the
// force-law derivative, the constraint derivative, the `d(J^T lambda)` preload
// term) and the *static solver's policy* (which rows it treats as gauges, which
// tires are active, the load scaling).  The physics is shared; the policy is not.
// The extraction keeps the arithmetic verbatim -- the loop structure, the
// `so3_left_jacobian` rotation handling and the block layout are unchanged -- so
// the static path's numbers are bit-identical, which `gate-numeric` verifies.
//
// The one deliberate change is that the gauge and the active tire set arrive as
// data (`WorkingSample`) instead of being computed on the spot.  That is what
// lets a second caller assemble the same physics with a different policy, and it
// is why `static_jacobian` now delegates rather than owning the arithmetic.

#include "mb_linearization/types.hpp"

#include <cmath>

#include "mb_config/functions.hpp"
#include "mb_dual/functions.hpp"
#include "mb_numeric/functions.hpp"
#include "mb_joint/functions.hpp"
#include "mb_force/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_tire/common/functions.hpp"
namespace axle_kernel {

int physical_tangent_dimension(const Model& model, const WorkingSample& working) {
    return model.ndof + model.rows + static_cast<int>(working.active_tires.size());
}

namespace {

/// The rotation-parameter map for one coordinate block, shared by the pose and
/// constraint blocks so the two cannot disagree about it.
Vec3 rotation_parameter_direction(
    const std::vector<double>& x, int free_body, int component
) {
    const Vec3 increment{
        x[6*free_body+3], x[6*free_body+4], x[6*free_body+5]
    };
    const Vec3 parameter_direction{
        component == 3 ? 1.0 : 0.0,
        component == 4 ? 1.0 : 0.0,
        component == 5 ? 1.0 : 0.0
    };
    // The increment is a left-SO(3) parameter, so its Jacobian maps a parameter
    // direction into the tangent space; using the raw direction here would be
    // correct only to first order in the increment.
    return so3_left_jacobian(increment) * parameter_direction;
}

/// Set one body's directional state for a coordinate perturbation.
void set_direction_for_coordinate(
    DirectionalState& direction, int body, int component,
    const std::vector<double>& x, int free_body
) {
    if (component < 3) {
        direction.dr[static_cast<std::size_t>(body)] = {
            component == 0 ? 1.0 : 0.0,
            component == 1 ? 1.0 : 0.0,
            component == 2 ? 1.0 : 0.0
        };
    } else {
        direction.dtheta[static_cast<std::size_t>(body)] =
            rotation_parameter_direction(x, free_body, component);
    }
}

} // namespace

bool assemble_physical_tangent(
    const Model& model, const SampleInput& sample,
    const WorkingSample& working, const std::vector<double>& x,
    std::vector<double>& jacobian
) {
    const int n = model.ndof;
    const int m = model.rows;
    const int dimension = physical_tangent_dimension(model, working);
    if (static_cast<int>(x.size()) != dimension) return false;

    // The candidate pose and the position-constraint Jacobian arrive in
    // `working`: deriving them here would mean calling *up* into the solvers,
    // which own the pose parameterisation and the position-row policy.  See
    // `WorkingSample` for why that is a layering inversion and not a nicety.
    const State& candidate = working.pose;
    const auto& position_constraint = working.position_constraint;
    if (candidate.r.size() != model.bodies.size() ||
        position_constraint.size() !=
            static_cast<std::size_t>(m)*static_cast<std::size_t>(n)) {
        return false;
    }
    const auto constraint = constraint_jacobian(model, candidate);
    std::vector<int> active_mask(model.tires.size(), 0);
    std::vector<double> compression(model.tires.size(), 0.0);
    std::vector<double> compression_derivative(model.tires.size(), 0.0);
    for (std::size_t index = 0; index < working.active_tires.size(); ++index) {
        const int tire_index = working.active_tires[index];
        active_mask[static_cast<std::size_t>(tire_index)] = 1;
        compression[static_cast<std::size_t>(tire_index)] =
            x[static_cast<std::size_t>(n+m)+index];
    }
    StaticContactOverride static_contact{
        &active_mask, &compression, &compression_derivative,
        working.load_scale, working.load_scale
    };
    const std::vector<double> lambda(x.begin()+n, x.begin()+n+m);
    const auto& gauge = working.gauge;
    jacobian.assign(
        static_cast<std::size_t>(dimension)*static_cast<std::size_t>(dimension),
        0.0
    );
    const auto at = [dimension](int row, int column) {
        return static_cast<std::size_t>(row)*static_cast<std::size_t>(dimension)
            + static_cast<std::size_t>(column);
    };
    DirectionalState direction;
    ensure_directional_state(model, direction);
    DirectionalForceScratch force_scratch;
    force_scratch.force.resize(model.bodies.size());
    force_scratch.torque.resize(model.bodies.size());
    std::vector<double> force_derivative;
    std::vector<double> brush_derivative;
    std::vector<double> jacobian_derivative;
    bool smooth = true;

    for (int column = 0; column < n; ++column) {
        reset_directional_state(model, direction);
        const int free_body = column/6;
        const int component = column%6;
        const int body = model.free_body[static_cast<std::size_t>(free_body)];
        set_direction_for_coordinate(direction, body, component, x, free_body);
        const bool force_smooth = external_force_directional(
            model, candidate, sample, direction, force_derivative,
            brush_derivative, false, &static_contact, &force_scratch
        );
        const bool constraint_smooth = constraint_jacobian_directional(
            model, candidate, direction, jacobian_derivative
        );
        smooth = smooth && force_smooth && constraint_smooth;
        for (int row = 0; row < n; ++row) {
            if (gauge[static_cast<std::size_t>(row)] != 0) {
                jacobian[at(row, column)] = row == column ? 1.0 : 0.0;
                continue;
            }
            // Material tangent, then the preload geometry term.  The second sum
            // is non-zero only when a constraint carries load *and* its direction
            // changes with pose -- the case a "stiffness" that omits it gets
            // wrong, and the reason this loop cannot be simplified away.
            double value = -force_derivative[static_cast<std::size_t>(row)];
            for (int constraint_row = 0; constraint_row < m; ++constraint_row) {
                value -= jacobian_derivative[
                    static_cast<std::size_t>(constraint_row*n+row)
                ] * lambda[static_cast<std::size_t>(constraint_row)];
            }
            jacobian[at(row, column)] = value;
        }
        for (int row = 0; row < m; ++row) {
            if (column % 6 < 3) {
                jacobian[at(n+row, column)] = position_constraint[
                    static_cast<std::size_t>(row*n+column)
                ];
            } else {
                const int free_body = column/6;
                const int first_rotation = 6*free_body+3;
                const Vec3 increment{
                    x[6*free_body+3], x[6*free_body+4], x[6*free_body+5]
                };
                const Mat3 rotation_map = so3_left_jacobian(increment);
                double value = 0.0;
                for (int tangent = 0; tangent < 3; ++tangent) {
                    value += position_constraint[
                        static_cast<std::size_t>(row*n+first_rotation+tangent)
                    ] * rotation_map.a[tangent][column % 6 - 3];
                }
                jacobian[at(n+row, column)] = value;
            }
        }
        for (std::size_t index = 0; index < working.active_tires.size(); ++index) {
            const int tire_index = working.active_tires[index];
            bool road_smooth = true;
            smooth = smooth && road_smooth;
            const DirectionalScalar compression_d =
                tire_contact_compression_directional(
                    model, candidate, sample, direction,
                    static_cast<std::size_t>(tire_index), road_smooth
            );
            jacobian[at(n+m+static_cast<int>(index), column)] =
                -compression_d.derivative;
            smooth = smooth && road_smooth;
        }
    }

    // The multiplier block: d(residual)/d(lambda) is minus the constraint
    // Jacobian, except on gauge rows and rotation-gauge pivots, which the model
    // already owns as declared directions.
    for (int row = 0; row < m; ++row) {
        for (int coordinate = 0; coordinate < n; ++coordinate) {
            if (gauge[static_cast<std::size_t>(coordinate)] == 0 &&
                static_rotation_gauge_for_pivot(model, coordinate) == nullptr) {
                jacobian[at(coordinate, n+row)] = -constraint[
                    static_cast<std::size_t>(row*n+coordinate)
                ];
            }
        }
    }

    for (std::size_t active_index = 0;
         active_index < working.active_tires.size(); ++active_index) {
        std::fill(compression_derivative.begin(), compression_derivative.end(), 0.0);
        const int tire_index = working.active_tires[active_index];
        compression_derivative[static_cast<std::size_t>(tire_index)] = 1.0;
        reset_directional_state(model, direction);
        const bool force_smooth = external_force_directional(
            model, candidate, sample, direction, force_derivative,
            brush_derivative, false, &static_contact, &force_scratch
        );
        smooth = smooth && force_smooth;
        const int column = n+m+static_cast<int>(active_index);
        for (int row = 0; row < n; ++row) {
            if (gauge[static_cast<std::size_t>(row)] == 0 &&
                static_rotation_gauge_for_pivot(model, row) == nullptr) {
                jacobian[at(row, column)] =
                    -force_derivative[static_cast<std::size_t>(row)];
            }
        }
        jacobian[at(n+m+static_cast<int>(active_index), column)] = 1.0;
    }
    for (const auto& rotation_gauge : model.static_rotation_gauges) {
        const int free_body = model.body_to_free[
            static_cast<std::size_t>(rotation_gauge.body)
        ];
        if (free_body < 0 || rotation_gauge.pivot < 0 ||
            rotation_gauge.pivot >= n) {
            continue;
        }
        const int first = 6*free_body+3;
        for (int column = 0; column < dimension; ++column) {
            jacobian[at(rotation_gauge.pivot, column)] = 0.0;
        }
        jacobian[at(rotation_gauge.pivot, first)] = rotation_gauge.axis_world.x;
        jacobian[at(rotation_gauge.pivot, first+1)] = rotation_gauge.axis_world.y;
        jacobian[at(rotation_gauge.pivot, first+2)] = rotation_gauge.axis_world.z;
    }
    // The caller's own gate: a non-finite matrix must be reported even when both
    // directional evaluations claimed to be smooth.
    return smooth && finite_vec(jacobian);
}

bool assemble_pose_tangent(
    const Model& model, const SampleInput& sample,
    const WorkingSample& working, const std::vector<double>& x,
    std::vector<double>& pose_block
) {
    // Assembled through the full path and then sliced, deliberately: writing a
    // second, shorter loop here is how the preload term gets forgotten in one
    // place and not the other.
    std::vector<double> full;
    const bool smooth = assemble_physical_tangent(
        model, sample, working, x, full
    );
    const int n = model.ndof;
    const int dimension = physical_tangent_dimension(model, working);
    pose_block.assign(static_cast<std::size_t>(n)*static_cast<std::size_t>(n), 0.0);
    for (int row = 0; row < n; ++row) {
        for (int column = 0; column < n; ++column) {
            pose_block[static_cast<std::size_t>(row)*static_cast<std::size_t>(n)
                + static_cast<std::size_t>(column)] =
                full[static_cast<std::size_t>(row)*static_cast<std::size_t>(dimension)
                    + static_cast<std::size_t>(column)];
        }
    }
    return smooth;
}

bool frozen_working_sample(
    const Model& model, const State& pose,
    const std::vector<int>& active_tires, WorkingSample& working
) {
    // Every refusal test runs BEFORE `working` is touched, so that a `false` return
    // really does leave the caller's sample untouched.  That ordering is the whole
    // contract of the return value: a partially-filled refusal would leave the
    // caller holding the *previous* state's `position_constraint` with a size that
    // still matches the model, and `assemble_physical_tangent` would accept it and
    // assemble a tangent from stale data -- exactly the "plausible wrong answer"
    // this function exists to make impossible.
    //
    // A frozen state must describe the same bodies the model has.  A short or long
    // `r` is not a physics verdict, it is a caller that handed over a different
    // model's state.
    if (pose.r.size() != model.bodies.size() ||
        pose.q.size() != model.bodies.size() ||
        pose.v.size() != model.bodies.size() ||
        pose.omega.size() != model.bodies.size()) {
        return false;
    }
    for (std::size_t body = 0; body < pose.r.size(); ++body) {
        const Vec3& r = pose.r[body];
        const Quat& q = pose.q[body];
        if (!std::isfinite(r.x) || !std::isfinite(r.y) || !std::isfinite(r.z) ||
            !std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
            !std::isfinite(q.z)) {
            return false;
        }
    }
    // `constraint_jacobian` sizes its result as `model.rows * model.ndof` in `int`
    // arithmetic before converting to `size_t`.  A negative dimension, or a product
    // that overflows, would wrap into an enormous allocation, so the dimensions are
    // checked here rather than left to surface as a thrown `length_error` from a call
    // the caller cannot see.
    if (model.ndof < 0 || model.rows < 0) return false;
    const long long entries =
        static_cast<long long>(model.rows)*static_cast<long long>(model.ndof);
    if (entries > static_cast<long long>(model.bodies.size())*6LL*64LL) return false;
    for (std::size_t body = 0; body < pose.v.size(); ++body) {
        const Vec3& v = pose.v[body];
        const Vec3& omega = pose.omega[body];
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
            !std::isfinite(omega.x) || !std::isfinite(omega.y) ||
            !std::isfinite(omega.z)) {
            return false;
        }
    }

    const int n = model.ndof;
    // The gauge list is a model question (see `mb_model::gauge_coordinates`), so it
    // is read from the model rather than invented here.  A model with no declared
    // gauge yields all zeros, which is the honest description of that model.
    const auto gauges = gauge_coordinates(model);
    std::vector<unsigned char> gauge(static_cast<std::size_t>(n), 0);
    for (const int coordinate : gauges) {
        if (coordinate >= 0 && coordinate < n) {
            gauge[static_cast<std::size_t>(coordinate)] = 1;
        }
    }
    // The constraint Jacobian is computed into a local first, so the size check
    // below happens before anything is committed to `working`.
    std::vector<double> position_constraint = constraint_jacobian(model, pose);
    if (position_constraint.size() !=
        static_cast<std::size_t>(model.rows)*static_cast<std::size_t>(n)) {
        return false;
    }

    // Past this point every value has been validated, so the assignments cannot
    // fail and the sample is complete when the function returns `true`.
    working.gauge = std::move(gauge);
    working.active_tires = active_tires;
    // A frozen response is asked at the unperturbed load: the request asks what the
    // mechanism does when the *load direction* is varied about this state, so
    // scaling the load here would answer a different question.
    working.load_scale = 1.0;
    working.pose = pose;
    // `position_constraint` is the plain constraint Jacobian, unmasked.  The static
    // path passes it through `static_position_constraint_jacobian`, which zeroes the
    // rows that `static_position_constraint_mask` clears; that mask is constructed
    // all-ones and its only write re-sets a row to the 1 it already holds, so no row
    // is ever cleared and the masked matrix *is* the plain one.  Reproducing the
    // mask here would be a second copy of an identity -- if that mask ever gains the
    // ability to clear a row, this line is what has to change with it.
    working.position_constraint = std::move(position_constraint);
    return true;
}

} // namespace axle_kernel
