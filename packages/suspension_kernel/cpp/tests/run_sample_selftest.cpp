#include "abi/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_solve_static/functions.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

using namespace axle_kernel;

namespace {
int failures = 0;
void check(bool condition, const char* message) {
    if (!condition) { ++failures; std::fprintf(stderr, "FAIL: %s\n", message); }
}
bool same(const std::vector<double>& a, const std::vector<double>& b) {
    return a.size() == b.size() &&
        (a.empty() || std::memcmp(a.data(), b.data(), a.size()*sizeof(double)) == 0);
}

Model oscillator(bool offset = false) {
    Model model;
    Body ground; ground.fixed = true;
    Body body; body.mass = 10.0; body.r = {0, 0, .20};
    for (int i = 0; i < 3; ++i) body.inertia_body.a[i][i] = 1.0;
    model.bodies = {ground, body};
    model.free_body = {1}; model.body_to_free = {-1, 0}; model.ndof = 6;
    Spring spring; spring.a = 0; spring.b = 1; spring.k = 10000;
    spring.free_length = .25; spring.pb = offset ? Vec3{.1, 0, 0} : Vec3{};
    model.springs = {spring};
    Damper damper; damper.a = 0; damper.b = 1;
    damper.c_compression = 100; damper.c_rebound = 100;
    model.dampers = {damper};
    return model;
}

struct Checkpoint {
    State state;
    std::vector<double> lambda;
    double time;
    RunSampleKind kind;
    std::size_t accepted_id, output_id;
};
struct Run {
    std::vector<double> states, diagnostics, energy, spring, damper, tire, wrench, events;
    std::vector<Checkpoint> checkpoints;
    std::size_t event_count = 0;
    int status = -1;
};

Run execute(const Model& model, bool adaptive, bool observed, bool fail = false,
            bool pull = false) {
    const std::vector<double> times = model.tires.empty()
        ? std::vector<double>{0, .01, .02}
        : std::vector<double>{0, .008, .012};
    const std::size_t count = times.size();
    std::vector<double> loads(count*model.bodies.size()*6, 0.0);
    if (pull) for (std::size_t i = 0; i < count; ++i) loads[i*12+6] = 100;
    std::vector<double> road(count*model.tires.size(), 0.0);
    AxleInput input{};
    input.struct_size = sizeof(input); input.abi_version = kAxleKernelAbiVersion;
    input.body_count = model.bodies.size(); input.tire_count = model.tires.size();
    input.sample_count = count; input.sample_times = times.data();
    input.body_wrench = loads.data(); input.road_z = road.data();
    input.road_z_velocity = road.data(); input.wheel_torque = road.data();
    input.rho_inf = .8; input.initialization_mode = 1;
    input.adaptive_step = adaptive ? 1 : 0;
    input.internal_step = fail ? .01 : (adaptive ? .004 : .0005);
    if (!model.tires.empty()) input.internal_step = .004;
    input.min_step = fail ? .01 : 1e-9;
    input.max_step = fail ? .01 : .004;
    input.local_relative_tolerance = 1e-5;
    input.local_position_tolerance = input.local_angle_tolerance = 1e-8;
    input.local_velocity_tolerance = input.local_angular_velocity_tolerance = 1e-7;
    input.local_brush_tolerance = 1e-8;
    input.contact_event_tolerance = 1e-9;
    input.max_newton_iterations = fail ? 1 : 30;
    input.max_line_search_iterations = 12;
    input.position_tolerance = input.velocity_tolerance = 1e-9;
    input.dynamics_tolerance = input.increment_tolerance = 1e-9;
    Run result;
    const auto allocate = [count](std::vector<double>& values, std::size_t width) {
        values.assign(count*width, std::numeric_limits<double>::quiet_NaN());
    };
    allocate(result.states, model.bodies.size()*kStatePerBody);
    allocate(result.diagnostics, kDiagnosticsWidth);
    allocate(result.energy, kEnergyOutputWidth);
    allocate(result.spring, model.springs.size()*kSpringOutputWidth);
    allocate(result.damper, model.dampers.size()*kDamperOutputWidth);
    allocate(result.tire, model.tires.size()*kTireOutputWidth);
    allocate(result.wrench, model.constraints.size()*kConstraintOutputWidth);
    result.events.assign(30, 0.0);
    AxleOutput output{};
    output.struct_size = sizeof(output); output.abi_version = kAxleKernelAbiVersion;
    output.body_state = result.states.data(); output.body_state_capacity = result.states.size();
    output.diagnostics = result.diagnostics.data(); output.diagnostics_capacity = result.diagnostics.size();
    output.energy_output = result.energy.data(); output.energy_output_capacity = result.energy.size();
    output.spring_output = result.spring.data(); output.spring_output_capacity = result.spring.size();
    output.damper_output = result.damper.data(); output.damper_output_capacity = result.damper.size();
    output.tire_output = result.tire.data(); output.tire_output_capacity = result.tire.size();
    output.constraint_wrench = result.wrench.data(); output.constraint_wrench_capacity = result.wrench.size();
    output.contact_event_output = result.events.data(); output.contact_event_output_capacity = result.events.size();
    output.contact_event_count = &result.event_count;
    const RunSampleObserver observer = [&](const RunSample& sample) {
        check(&sample.model == &model, "observer reads the real running model");
        result.checkpoints.push_back({sample.state, sample.constraint_multiplier,
            sample.time, sample.kind, sample.accepted_step_id, sample.output_sample_id});
    };
    char error[512]{};
    result.status = run_model(&input, &output, error, sizeof(error), &model,
                             observed ? &observer : nullptr);
    if (result.status != 0 && !fail) std::fprintf(stderr, "run failed: %s\n", error);
    return result;
}

void compare(const Run& plain, const Run& observed) {
    check(plain.status == observed.status, "observer does not change status");
    check(same(plain.states, observed.states), "body state is bit-identical");
    check(same(plain.diagnostics, observed.diagnostics), "step/rejection/energy diagnostics are bit-identical");
    check(same(plain.energy, observed.energy), "energy ledger is bit-identical");
    check(same(plain.spring, observed.spring) && same(plain.damper, observed.damper), "force ledgers are bit-identical");
    check(same(plain.tire, observed.tire) && same(plain.wrench, observed.wrench), "tire and constraint ledgers are bit-identical");
    check(plain.event_count == observed.event_count && same(plain.events, observed.events), "contact events are bit-identical");
}

void test_run(const Model& model, bool adaptive, bool pull = false) {
    const Run plain = execute(model, adaptive, false, false, pull);
    const Run result = execute(model, adaptive, true, false, pull);
    compare(plain, result);
    check(result.status == 0, "real dynamic run succeeds");
    check(!result.checkpoints.empty(), "initial checkpoint exists");
    if (result.checkpoints.empty()) return;
    check(result.checkpoints.front().kind == RunSampleKind::initial &&
          result.checkpoints.front().accepted_id == 0 &&
          result.checkpoints.front().output_id == 0, "consistent initial identity is explicit");
    std::size_t accepted = 0, output = 0;
    double last_time = 0;
    bool nonzero_lambda = false, nonzero_acceleration = false;
    for (const auto& sample : result.checkpoints) {
        check(sample.state.a.size() == model.bodies.size() &&
              sample.state.alpha.size() == model.bodies.size(), "checkpoint contains accelerations");
        check(sample.lambda.size() == static_cast<std::size_t>(model.rows), "checkpoint contains all physical multipliers");
        for (double value : sample.lambda) nonzero_lambda |= std::abs(value) > 1;
        nonzero_acceleration |= std::abs(sample.state.a[1].z) > 1;
        if (!model.tires.empty()) {
            check(sample.state.tire_sx.size() == model.tires.size() &&
                  sample.state.tire_sy_dot.size() == model.tires.size(), "tire internal state and derivative are complete");
        }
        if (sample.kind == RunSampleKind::accepted) {
            ++accepted;
            check(sample.accepted_id == accepted, "accepted IDs are contiguous across output intervals");
            check(sample.output_id == kNoOutputSample, "accepted checkpoint does not invent an output ID");
            check(sample.time > last_time, "each accepted state has its actual increasing time");
            last_time = sample.time;
        } else if (sample.kind == RunSampleKind::output) {
            ++output;
            check(sample.output_id == output && sample.accepted_id == accepted, "output identity links to the real accepted checkpoint");
            const std::size_t base = (output*model.bodies.size()+1)*kStatePerBody;
            check(sample.state.r[1].z == result.states[base+2] &&
                  sample.state.a[1].z == result.states[base+15], "observed output agrees with the original stored full state");
        }
    }
    std::size_t reported = 0;
    for (std::size_t i = 0; i < 3; ++i)
        reported += static_cast<std::size_t>(result.diagnostics[i*kDiagnosticsWidth+1]);
    check(accepted == reported && accepted > output, "every actual accepted step is captured, including non-output steps");
    check(output == 2, "original output grid is preserved");
    if (pull) check(nonzero_lambda, "loaded constrained fixture has nonzero physical multipliers");
    else if (model.tires.empty()) check(nonzero_acceleration, "dynamic checkpoint acceleration is physically nonzero");
    if (adaptive) {
        check(accepted >= 2, "adaptive run has both half steps");
        const auto& first = result.checkpoints[1];
        const auto& second = result.checkpoints[2];
        check(first.kind == RunSampleKind::accepted && second.kind == RunSampleKind::accepted &&
              first.state.r[1].z != second.state.r[1].z, "adaptive first half stores its own state, not the second half twice");
    }
    if (!model.tires.empty()) check(result.event_count == 1, "contact fixture executes a real event commit");
    if (model.rows == 0 && model.tires.empty()) {
        double worst = 0.0, initial = -1.0;
        for (const auto& point : result.checkpoints) {
            if (point.kind == RunSampleKind::output) continue;
            WorkingSample working;
            check(frozen_working_sample(model, point.state, {}, working),
                  "real original accepted sample creates frozen working point");
            SampleInput input;
            std::vector<double> residual;
            double force = 0.0, position = 0.0;
            check(frozen_inertial_residual(model, input, working,
                  std::vector<double>(model.ndof, 0.0), 0.0, 0.0, 0.0,
                  residual, force, position), "original frozen mechanical residual is observable");
            if (point.kind == RunSampleKind::initial) initial = force;
            worst = std::max(worst, force);
        }
        check(initial < 1e-10, "consistent instantaneous initial point balances original inertia");
        std::printf("original frozen residual (%s): initial=%.9g max=%.9g\n",
                    adaptive ? "adaptive" : "fixed", initial, worst);
    }
}
} // namespace

int main() {
    test_run(oscillator(), false);
    test_run(oscillator(), true);
    Model slider = oscillator(); slider.springs.clear(); slider.dampers.clear();
    slider.bodies[1].r = {};
    Constraint guide; guide.type = AXLE_PRISMATIC; guide.a = 0; guide.b = 1;
    // A z-axis prismatic row constrains the applied x load, so its physical
    // multiplier is nonzero and the observation cannot pass on an empty row.
    guide.axis_a = guide.axis_b = {0, 0, 1};
    slider.constraints = {guide}; slider.rows = 5;
    test_run(slider, false, true);
    Model drop = oscillator(); drop.springs.clear(); drop.dampers.clear();
    drop.bodies[1].r = {0, 0, .3073}; drop.bodies[1].v = {0, 0, -1};
    Tire tire; tire.body = tire.frame_body = 1; tire.radius = .3;
    tire.maximum_compression = .05; tire.k = 10000; tire.c = 100;
    tire.mu_longitudinal = .2; tire.mu_lateral = .3;
    tire.brush_k_longitudinal = 100000; tire.brush_k_lateral = 80000;
    tire.relaxation_length_longitudinal = .2; tire.relaxation_length_lateral = .25;
    tire.detached_relaxation = .02; tire.has_state_return_mapping = true;
    drop.tires = {tire}; test_run(drop, false);
    const Run failed_plain = execute(oscillator(true), false, false, true);
    const Run failed = execute(oscillator(true), false, true, true);
    compare(failed_plain, failed);
    check(failed.status == 5 && failed.checkpoints.size() == 1 &&
          failed.checkpoints[0].kind == RunSampleKind::initial,
          "failed trial is absent while the consistent initial checkpoint is retained");
    if (failures != 0) return 1;
    std::puts("OK: real dynamic sample observation self-test passed");
    return 0;
}
