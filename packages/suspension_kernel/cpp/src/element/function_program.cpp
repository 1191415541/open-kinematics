#include "mb_element/functions.hpp"
#include "mb_dual/functions.hpp"
#include "mb_model/functions.hpp"
#include "mb_numeric/functions.hpp"

#include <algorithm>
#include <stdexcept>

namespace axle_kernel {
namespace {

using Scalar = DirectionalScalar;

DirectionalState zero_direction(std::size_t size) {
    DirectionalState direction;
    direction.dr.resize(size); direction.dtheta.resize(size);
    direction.dv.resize(size); direction.domega.resize(size);
    return direction;
}

DVec3 marker_axis(const FunctionMarker& marker, const Vec3& axis,
                 const State& state, const DirectionalState& direction) {
    return d_rotate(state.q[marker.body], rotate(marker.quaternion, axis), direction.dtheta[marker.body]);
}

Scalar linear_table(const std::vector<double>& x, const std::vector<double>& y,
                    Scalar value, const std::string& policy, bool& smooth) {
    if (value.value < x.front() || value.value > x.back()) {
        if (policy == "error") throw std::runtime_error("function table input is out of range");
        if (policy == "clamp") return Scalar{value.value < x.front() ? y.front() : y.back()};
    }
    std::size_t index = value.value <= x.front() ? 0 : value.value >= x.back() ? x.size()-2 :
        static_cast<std::size_t>(std::upper_bound(x.begin(), x.end(), value.value)-x.begin()-1);
    if (value.derivative != 0 && std::abs(value.value-x[index]) < 1e-12) smooth = false;
    return y[index] + (value-x[index])*((y[index+1]-y[index])/(x[index+1]-x[index]));
}

Scalar binding_value(const FunctionBinding& binding, const State& state,
                     const SampleInput& input, const DirectionalState& direction, bool& smooth) {
    if (binding.source == "time") return Scalar{input.time};
    if (binding.source == "channel" || binding.source == "signal")
        return binding.scale*linear_table(binding.times, binding.values, Scalar{input.time}, "clamp", smooth);
    if (binding.source != "measurement") return Scalar{binding.value*binding.scale};
    const FunctionMarker& a = binding.action;
    const FunctionMarker& b = binding.reaction;
    const DVec3 axis = marker_axis(binding.reference, binding.axis, state, direction);
    const DVec3 pa = d_state_point(state, direction.dr, direction.dtheta, a.body, a.point);
    const DVec3 pb = d_state_point(state, direction.dr, direction.dtheta, b.body, b.point);
    if (binding.measurement == "position") return d_dot(pa, axis);
    if (binding.measurement == "relative_position") return d_dot(pa-pb, axis);
    if (binding.measurement == "relative_velocity") {
        const DVec3 va = d_state_point_velocity(state, direction.dr, direction.dtheta, direction.dv, direction.domega, a.body, a.point);
        const DVec3 vb = d_state_point_velocity(state, direction.dr, direction.dtheta, direction.dv, direction.domega, b.body, b.point);
        const int r = binding.reference.body;
        const DVec3 omega{{state.omega[r].x,direction.domega[r].x}, {state.omega[r].y,direction.domega[r].y}, {state.omega[r].z,direction.domega[r].z}};
        return d_dot(va-vb-d_cross(omega, pa-pb), axis);
    }
    const DVec3 wa{{state.omega[a.body].x,direction.domega[a.body].x}, {state.omega[a.body].y,direction.domega[a.body].y}, {state.omega[a.body].z,direction.domega[a.body].z}};
    const DVec3 wb{{state.omega[b.body].x,direction.domega[b.body].x}, {state.omega[b.body].y,direction.domega[b.body].y}, {state.omega[b.body].z,direction.domega[b.body].z}};
    return d_dot(wa-wb, axis);
}

} // namespace

DirectionalScalar evaluate_function_directional(const FunctionProgram& program,
    const State& state, const SampleInput& input, const DirectionalState& direction, bool& smooth) {
    std::vector<Scalar> values;
    values.reserve(program.nodes.size());
    for (const FunctionNode& node : program.nodes) {
        const auto arg = [&](std::size_t i) { return values.at(static_cast<std::size_t>(node.args.at(i))); };
        Scalar value;
        if (node.op == "constant") value = Scalar{node.value};
        else if (node.op == "binding") value = binding_value(program.bindings.at(static_cast<std::size_t>(node.binding)), state, input, direction, smooth);
        else if (node.op == "identity") value = arg(0);
        else if (node.op == "neg") value = -arg(0);
        else if (node.op == "add") value = arg(0)+arg(1);
        else if (node.op == "sub") value = arg(0)-arg(1);
        else if (node.op == "mul") value = arg(0)*arg(1);
        else if (node.op == "div") value = arg(0)/arg(1);
        else if (node.op == "sin") value = d_sin(arg(0));
        else if (node.op == "cos") value = d_cos(arg(0));
        else if (node.op == "exp") value = d_exp(arg(0));
        else if (node.op == "sqrt") value = d_sqrt(arg(0));
        else if (node.op == "tanh") { const Scalar a = arg(0); const double t = std::tanh(a.value); value = Scalar{t, (1-t*t)*a.derivative}; }
        else if (node.op == "pow") { const Scalar a = arg(0); const double exponent = arg(1).value; const double p = std::pow(a.value, exponent); value = Scalar{p, exponent == 0 ? 0 : exponent*std::pow(a.value, exponent-1)*a.derivative}; }
        else if (node.op == "step") {
            const Scalar x = arg(0), x0 = arg(1), y0 = arg(2), x1 = arg(3), y1 = arg(4);
            if (!(x1.value > x0.value)) throw std::runtime_error("function step requires x1 > x0");
            if (x.value <= x0.value) value = y0;
            else if (x.value >= x1.value) value = y1;
            else { const Scalar u = (x-x0)/(x1-x0); value = y0+(y1-y0)*u*u*(3-2*u); }
        } else if (node.op == "curve" || node.op == "surface") {
            const FunctionTable& table = program.tables.at(static_cast<std::size_t>(node.table));
            if (node.op == "curve") {
                const Scalar x = arg(0);
                if (table.interpolation == "akima" && x.value >= table.x.front() && x.value <= table.x.back()) value = interpolated_curve_directional(table.x, table.values, x, smooth, true);
                else value = linear_table(table.x, table.values, x, table.extrapolation, smooth);
            } else {
                std::vector<double> row(table.y.size());
                std::vector<Scalar> along_x;
                for (std::size_t j = 0; j < table.y.size(); ++j) {
                    std::vector<double> column(table.x.size());
                    for (std::size_t i = 0; i < table.x.size(); ++i) column[i] = table.values.at(i*table.y.size()+j);
                    along_x.push_back(linear_table(table.x, column, arg(0), table.extrapolation, smooth));
                    row[j] = along_x.back().value;
                }
                const Scalar y = arg(1);
                value = linear_table(table.y, row, y, table.extrapolation, smooth);
                std::vector<double> dx;
                for (const Scalar& v : along_x) dx.push_back(v.derivative);
                value.derivative += linear_table(table.y, dx, Scalar{y.value}, table.extrapolation, smooth).value;
            }
        } else throw std::runtime_error("unsupported function operation " + node.op);
        if (!std::isfinite(value.value) || !std::isfinite(value.derivative)) throw std::runtime_error("function evaluation produced a non-finite value");
        values.push_back(value);
    }
    return values.at(static_cast<std::size_t>(program.output));
}

double evaluate_function(const FunctionProgram& program, const State& state, const SampleInput& input) {
    bool smooth = true;
    return evaluate_function_directional(program, state, input, zero_direction(state.r.size()), smooth).value;
}

namespace {
struct FunctionWrench { DVec3 force, moment, a, b; };

FunctionWrench function_wrench(const Model& model, const FunctionElement& element,
    const State& state, const SampleInput& input, const DirectionalState& direction, bool& smooth) {
    FunctionWrench out;
    const auto evaluate = [&](int index) { return evaluate_function_directional(model.function_programs.at(static_cast<std::size_t>(element.programs.at(static_cast<std::size_t>(index)))), state, input, direction, smooth); };
    if (element.kind == 0) out.force = marker_axis(element.reference, element.axis, state, direction)*evaluate(0);
    else if (element.kind == 1) out.moment = marker_axis(element.reference, element.axis, state, direction)*evaluate(0);
    else {
        for (int i = 0; i < 3; ++i) {
            Vec3 axis{}; if (i == 0) axis.x = 1; else if (i == 1) axis.y = 1; else axis.z = 1;
            const DVec3 world = marker_axis(element.reference, axis, state, direction);
            out.force = out.force + world*evaluate(i); out.moment = out.moment + world*evaluate(i+3);
        }
    }
    out.a = d_state_point(state, direction.dr, direction.dtheta, element.action.body, element.action.point);
    out.b = d_state_point(state, direction.dr, direction.dtheta, element.reaction.body, element.reaction.point);
    return out;
}
}

void assemble_function_forces(const Model& model, const State& state, const SampleInput& input,
    std::vector<Vec3>& force, std::vector<Vec3>& torque, EnergyRates* energy_rates, bool record_energy, bool brush_only, double force_scale, double& external_power) {
    if (brush_only || model.function_elements.empty()) return;
    const DirectionalState direction = zero_direction(state.r.size());
    ElementWrenchSink* const sink = active_element_wrench_sink();
    for (std::size_t index = 0; index < model.function_elements.size(); ++index) {
        const FunctionElement& element = model.function_elements[index];
        bool smooth = true;
        const FunctionWrench wrench = function_wrench(model, element, state, input, direction, smooth);
        const Vec3 f = wrench.force.value()*force_scale, m = wrench.moment.value()*force_scale, a = wrench.a.value(), b = wrench.b.value();
        const Vec3 mb = Vec3{-m.x, -m.y, -m.z}-cross(a-b, f);
        if (sink != nullptr) sink->open(kElementWrenchFunction, index, 0, element.action.body, element.reaction.body, element.action.body, a.x, a.y, a.z);
        add_force_on_body(force, torque, model, state, element.action.body, element.action.point, f, sink);
        add_torque_on_body(torque, model, element.action.body, m, sink);
        if (sink != nullptr) sink->open(kElementWrenchFunction, index, 1, element.action.body, element.reaction.body, element.reaction.body, b.x, b.y, b.z);
        add_force_on_body(force, torque, model, state, element.reaction.body, element.reaction.point, Vec3{-f.x, -f.y, -f.z}, sink);
        add_torque_on_body(torque, model, element.reaction.body, mb, sink);
        if (record_energy) {
            const double power = dot(f, state_point_velocity(state, element.action.body, element.action.point)) + dot(m, state.omega[element.action.body]) - dot(f, state_point_velocity(state, element.reaction.body, element.reaction.point)) + dot(mb, state.omega[element.reaction.body]);
            external_power += power;
            if (energy_rates != nullptr) energy_rates->external_power += power;
        }
    }
}

void assemble_function_directional(const Model& model, const State& state, const SampleInput& input,
    const DirectionalState& direction, std::vector<Vec3>& force, std::vector<Vec3>& torque, double force_scale, bool& smooth) {
    for (const FunctionElement& element : model.function_elements) {
        FunctionWrench wrench = function_wrench(model, element, state, input, direction, smooth);
        wrench.force = wrench.force*force_scale;
        wrench.moment = wrench.moment*force_scale;
        const DVec3 mb = -wrench.moment-d_cross(wrench.a-wrench.b, wrench.force);
        add_directional_force_at_arm(force, torque, model, element.action.body, d_rotate(state.q[element.action.body], element.action.point, direction.dtheta[element.action.body]), wrench.force);
        add_directional_torque(torque, model, element.action.body, wrench.moment);
        add_directional_force_at_arm(force, torque, model, element.reaction.body, d_rotate(state.q[element.reaction.body], element.reaction.point, direction.dtheta[element.reaction.body]), -wrench.force);
        add_directional_torque(torque, model, element.reaction.body, mb);
    }
}
} // namespace axle_kernel
