#include "mb_joint/functions.hpp"
#include "mb_output/functions.hpp"
#include "mb_numeric/functions.hpp"
#include <algorithm>
#include <iomanip>
#include <iostream>

using namespace axle_kernel;

int main() {
    Model model;
    model.bodies.resize(2);
    model.free_body = {0, 1}; model.body_to_free = {0, 1}; model.ndof = 12;
    model.rows = 1; model.has_driven_coordinates = true;
    Constraint coordinate;
    coordinate.type = AXLE_DRIVEN_ROTATION; coordinate.a = 0; coordinate.b = 1;
    coordinate.axis_a = normalized(Vec3{.3, .8, -.2});
    coordinate.reference = qexp(Vec3{.4, -.2, .3}); coordinate.signal = 0;
    model.constraints.push_back(coordinate);
    State state;
    state.r.resize(2); state.q.resize(2); state.v.resize(2); state.omega.resize(2);
    state.q[1] = qexp(Vec3{-.3, .4, .2});
    DirectionalState direction;
    direction.dr.resize(2); direction.dv.resize(2); direction.domega.resize(2);
    direction.dtheta = {Vec3{.13, -.08, .17}, Vec3{-.11, .21, -.09}};
    double residual_error = 0.0, jacobian_error = 0.0, directional_error = 0.0;
    const double h = 1e-6;
    for (int i = -80; i <= 80; ++i) {
        const double phase = i*.2;
        state.q[1] = qexp(Vec3{-.3+i*.01, .4, .2-i*.003});
        state.r[0] = state.r[1] = Vec3{.2*i, -.3, .4};
        state.q[0] = qmul(state.q[1], qmul(qexp(coordinate.axis_a*phase), coordinate.reference));
        SampleInput sample; sample.driven_target = {phase}; sample.driven_target_rate = {2.5};
        residual_error = std::max(residual_error, std::abs(constraint_residual(model, state, &sample)[0]));
        const auto scalar = constraint_jacobian(model, state);
        const auto fd = constraint_jacobian_central_difference(model, state);
        std::vector<double> dual;
        if (!constraint_jacobian_directional(model, state, direction, dual)) return 2;
        State plus = state, minus = state;
        for (int b = 0; b < 2; ++b) {
            plus.q[b] = qmul(qexp(direction.dtheta[b]*h), state.q[b]);
            minus.q[b] = qmul(qexp(direction.dtheta[b]*(-h)), state.q[b]);
        }
        const auto p = constraint_jacobian(model, plus), m = constraint_jacobian(model, minus);
        for (std::size_t k = 0; k < scalar.size(); ++k) {
            jacobian_error = std::max(jacobian_error, std::abs(scalar[k]-fd[k]));
            directional_error = std::max(directional_error, std::abs(dual[k]-(p[k]-m[k])/(2*h)));
        }
    }
    state.q[0] = qmul(state.q[1], coordinate.reference);
    double wrench[6]{};
    write_constraint_wrenches(model, state, {7.0}, wrench, 0);
    const Vec3 axis = rotate(state.q[1], coordinate.axis_a);
    const Vec3 reaction{wrench[3], wrench[4], wrench[5]};
    SampleInput sample; sample.driven_target_rate = {2.5};
    std::cout << std::setprecision(17) << "errors " << residual_error << ' ' << jacobian_error << ' ' << directional_error << '\n';
    std::cout << "reaction " << dot(reaction, axis) << ' ' << norm(reaction-axis*dot(reaction, axis)) << '\n';
    std::cout << "rate " << driven_row_target_rate(model, 0, sample) << '\n';
}
