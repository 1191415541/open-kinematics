#include "mb_force/functions.hpp"
#include "mb_numeric/functions.hpp"
#include "mb_solve_dynamic/functions.hpp"
#include "mb_tire/model.hpp"
#include "mb_tire_state/functions.hpp"
#include <iomanip>
#include <iostream>

using namespace axle_kernel;

int main() {
    Model model;
    model.bodies.resize(2); model.free_body = {0, 1}; model.body_to_free = {0, 1}; model.ndof = 12;
    for (int i = 0; i < 2; ++i) {
        Tire tire; tire.body = i; tire.frame_body = i;
        tire.radius = .5; tire.maximum_compression = .1;
        tire.k = 10000; tire.mu_longitudinal = tire.mu_lateral = .8;
        tire.brush_k_longitudinal = tire.brush_k_lateral = 1000;
        tire.relaxation_length_longitudinal = tire.relaxation_length_lateral = .1;
        tire.detached_relaxation = .1; apply_tire_state_semantics(tire);
        model.tires.push_back(tire);
    }
    State accepted;
    accepted.r = {Vec3{0, 0, .49}, Vec3{2, 0, .49}};
    accepted.q.resize(2); accepted.v.resize(2); accepted.omega.resize(2);
    resize_tire_states(model, accepted);
    accepted.tire_sx_dot.resize(2); accepted.tire_sy_dot.resize(2);
    accepted.tire_sx = {1, .001}; accepted.tire_sy = {.5, -.001};
    const State snapshot = accepted;
    AxleInput input{}; const double times[] = {0, .01};
    input.sample_count = 2; input.sample_times = times; input.body_count = 2; input.tire_count = 2;
    SampleInput sample;
    std::vector<double> normal, derivative, output, force;
    double potential = 0, power = 0, dissipation = 0;
    external_force_vector(model, accepted, sample, 0, 0, 0, normal, derivative,
        output, potential, power, dissipation, force);
    const auto first_force = force;
    external_force_vector(model, accepted, sample, 0, 0, 0, normal, derivative,
        output, potential, power, dissipation, force);
    const bool pure = accepted.tire_sx == snapshot.tire_sx && accepted.tire_sy == snapshot.tire_sy && force == first_force;
    State rejected = accepted;
    if (!apply_brush_return_mapping(model, input, accepted, 0, .002, 1, rejected)) return 2;
    // Discard the candidate, then replay from the same accepted snapshot.
    State replay = accepted, reference = snapshot;
    if (!apply_brush_return_mapping(model, input, accepted, 0, .001, 1, replay)
        || !apply_brush_return_mapping(model, input, snapshot, 0, .001, 1, reference)) return 3;
    const bool unchanged = accepted.tire_sx == snapshot.tire_sx && accepted.tire_sy == snapshot.tire_sy;
    const bool repeat = replay.tire_sx == reference.tire_sx && replay.tire_sy == reference.tire_sy
        && replay.tire_sx_dot == reference.tire_sx_dot && replay.tire_sy_dot == reference.tire_sy_dot;
    const bool projected = replay.tire_sx[0] < snapshot.tire_sx[0];
    const bool isolated = replay.tire_sx[1] == snapshot.tire_sx[1] && replay.tire_sy[1] == snapshot.tire_sy[1];
    accepted = replay;
    std::cout << std::setprecision(17) << "lifecycle " << pure << ' ' << unchanged << ' ' << repeat << ' ' << projected << ' ' << isolated << '\n';
    std::cout << "commit " << (accepted.tire_sx == replay.tire_sx && snapshot.tire_sx[0] == 1) << '\n';
}
