#include "mb_cases/functions.hpp"
#include "mb_contract/functions.hpp"
#include "mb_element/functions.hpp"
#include "mb_numeric/functions.hpp"

#include <iomanip>
#include <chrono>
#include <iostream>
#include <iterator>

using namespace axle_kernel;

int main(int argc, char** argv) {
    JsonValue document;
    ContractModel parsed;
    std::string error;
    const std::string text{std::istreambuf_iterator<char>(std::cin), {}};
    if (!contract_parse_json(text, document, error) || !parsed.read(document, "", error)) {
        std::cout << "error " << error << '\n';
        return 0;
    }
    VehicleInput input{};
    parsed.fill(input);
    const std::size_t count = input.axle.body_count;
    State state;
    DirectionalState direction;
    state.r.resize(count); state.q.resize(count); state.v.resize(count); state.omega.resize(count);
    direction.dr.resize(count); direction.dtheta.resize(count);
    direction.dv.resize(count); direction.domega.resize(count);
    for (std::size_t i = 0; i < count; ++i) {
        const double* p = input.axle.body_pose_position_quaternion + 7*i;
        const double* v = input.axle.body_velocity_omega + 6*i;
        state.r[i] = Vec3{p[0], p[1], p[2]}; state.q[i] = Quat{p[3], p[4], p[5], p[6]};
        state.v[i] = Vec3{v[0], v[1], v[2]}; state.omega[i] = Vec3{v[3], v[4], v[5]};
        const double scale = static_cast<double>(i+1);
        direction.dr[i] = Vec3{.13*scale, -.07*scale, .09*scale};
        direction.dtheta[i] = Vec3{-.11*scale, .03*scale, .17*scale};
        direction.dv[i] = Vec3{.21*scale, -.19*scale, .04*scale};
        direction.domega[i] = Vec3{-.15*scale, .08*scale, -.05*scale};
    }
    SampleInput sample;
    sample.time = argc > 1 ? std::stod(argv[1]) : .35;
    std::cout << std::setprecision(17);
    try {
        if (argc > 2) {
            const std::size_t iterations = std::stoull(argv[2]);
            volatile double checksum = 0.0;
            const auto start = std::chrono::steady_clock::now();
            for (std::size_t i = 0; i < iterations; ++i) {
                for (const FunctionProgram& program : parsed.function_programs())
                    checksum += evaluate_function(program, state, sample);
            }
            const auto middle = std::chrono::steady_clock::now();
            for (std::size_t i = 0; i < iterations; ++i) {
                bool smooth = true;
                for (const FunctionProgram& program : parsed.function_programs())
                    checksum += evaluate_function_directional(program, state, sample, direction, smooth).derivative;
            }
            const auto end = std::chrono::steady_clock::now();
            const double count = static_cast<double>(iterations * parsed.function_programs().size());
            std::cout << "benchmark " << parsed.function_programs().size() << ' ' << iterations << ' '
                << std::chrono::duration<double, std::nano>(middle-start).count()/count << ' '
                << std::chrono::duration<double, std::nano>(end-middle).count()/count << ' ' << checksum << '\n';
            return 0;
        }
        for (const FunctionProgram& program : parsed.function_programs()) {
            const double scalar = evaluate_function(program, state, sample);
            bool smooth = true;
            const DirectionalScalar dual = evaluate_function_directional(program, state, sample, direction, smooth);
            const double h = 1e-6;
            State plus = state, minus = state;
            for (std::size_t i = 0; i < count; ++i) {
                plus.r[i] = state.r[i] + direction.dr[i]*h;
                minus.r[i] = state.r[i] - direction.dr[i]*h;
                plus.q[i] = qmul(qexp(direction.dtheta[i]*h), state.q[i]);
                minus.q[i] = qmul(qexp(direction.dtheta[i]*(-h)), state.q[i]);
                plus.v[i] = state.v[i] + direction.dv[i]*h;
                minus.v[i] = state.v[i] - direction.dv[i]*h;
                plus.omega[i] = state.omega[i] + direction.domega[i]*h;
                minus.omega[i] = state.omega[i] - direction.domega[i]*h;
            }
            const double fd = (evaluate_function(program, plus, sample)-evaluate_function(program, minus, sample))/(2*h);
            std::cout << "value " << scalar << ' ' << dual.value << ' ' << dual.derivative << ' ' << fd << ' ' << smooth << '\n';
            std::cout << "repeat " << evaluate_function(program, state, sample) << '\n';
        }
        if (!parsed.function_elements().empty()) {
            Model model;
            model.bodies.resize(count);
            model.function_programs = parsed.function_programs();
            model.function_elements = parsed.function_elements();
            std::vector<Vec3> force(count), torque(count), df(count), dm(count);
            double power = 0.0;
            EnergyRates rates;
            assemble_function_forces(model, state, sample, force, torque, &rates, true, false, 1.0, power);
            bool smooth = true;
            assemble_function_directional(model, state, sample, direction, df, dm, 1.0, smooth);
            const double h = 1e-6;
            State plus = state, minus = state;
            for (std::size_t i = 0; i < count; ++i) {
                plus.r[i] = state.r[i] + direction.dr[i]*h;
                minus.r[i] = state.r[i] - direction.dr[i]*h;
                plus.q[i] = qmul(qexp(direction.dtheta[i]*h), state.q[i]);
                minus.q[i] = qmul(qexp(direction.dtheta[i]*(-h)), state.q[i]);
                plus.v[i] = state.v[i] + direction.dv[i]*h;
                minus.v[i] = state.v[i] - direction.dv[i]*h;
                plus.omega[i] = state.omega[i] + direction.domega[i]*h;
                minus.omega[i] = state.omega[i] - direction.domega[i]*h;
            }
            std::vector<Vec3> pf(count), pm(count), mf(count), mm(count);
            double ignored = 0.0;
            assemble_function_forces(model, plus, sample, pf, pm, nullptr, false, false, 1.0, ignored);
            assemble_function_forces(model, minus, sample, mf, mm, nullptr, false, false, 1.0, ignored);
            Vec3 total_force{}, total_moment{};
            for (std::size_t i = 0; i < count; ++i) {
                total_force += force[i];
                total_moment += torque[i]+cross(state.r[i], force[i]);
                const Vec3 fd = (pf[i]-mf[i])*(.5/h), md = (pm[i]-mm[i])*(.5/h);
                std::cout << "force_dual " << df[i].x << ' ' << df[i].y << ' ' << df[i].z << ' '
                    << dm[i].x << ' ' << dm[i].y << ' ' << dm[i].z << '\n';
                std::cout << "force_fd " << fd.x << ' ' << fd.y << ' ' << fd.z << ' '
                    << md.x << ' ' << md.y << ' ' << md.z << '\n';
            }
            std::cout << "balance " << total_force.x << ' ' << total_force.y << ' ' << total_force.z << ' '
                << total_moment.x << ' ' << total_moment.y << ' ' << total_moment.z << '\n';
            std::cout << "power " << power << ' ' << rates.external_power << '\n';
        }
    } catch (const std::exception& e) {
        std::cout << "error " << e.what() << '\n';
    }
}
