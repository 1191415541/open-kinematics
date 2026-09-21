// The shared input-sampling functions, split from the integrator's input
// translation unit at subtask 03 step 6 into `mb_input`.

#include "mb_input/functions.hpp"

namespace axle_kernel {

void interpolate_input(const AxleInput& in, double t, SampleInput& out) {
    const std::size_t n = in.sample_count;
    const std::size_t nb = in.body_count;
    const std::size_t nt = in.tire_count;
    // Set before the early return so a zero-sample input still carries its time.
    out.time = t;
    out.body_wrench.assign(nb * 6, 0.0);
    out.road_z.assign(nt, 0.0);
    out.road_v.assign(nt, 0.0);
    out.torque.assign(nt, 0.0);
    out.brake_torque.assign(nt, 0.0);
    if (n == 0) return;
    std::size_t k = 0;
    while (k+1 < n && in.sample_times[k+1] < t) ++k;
    const std::size_t k1 = std::min(k+1, n-1);
    const double t0 = in.sample_times[k], t1 = in.sample_times[k1];
    const double raw_u = std::abs(t1-t0) > kEps ? (t-t0)/(t1-t0) : 0.0;
    const double u = std::max(0.0, std::min(1.0, raw_u));
    auto interp_tire = [&](const double* data, std::size_t j) {
        if (!data) return 0.0;
        const double a = data[k*nt+j], b = data[k1*nt+j];
        return (1.0-u)*a+u*b;
    };
    auto interp_body = [&](const double* data, std::size_t j) {
        if (!data) return 0.0;
        const std::size_t stride = nb * 6;
        const double a = data[k*stride+j], b = data[k1*stride+j];
        return (1.0-u)*a+u*b;
    };
    for (std::size_t j = 0; j < nb * 6; ++j) {
        out.body_wrench[j] = interp_body(in.body_wrench, j);
    }
    for (std::size_t j = 0; j < nt; ++j) {
        out.road_z[j] = interp_tire(in.road_z, j);
        out.road_v[j] = interp_tire(in.road_z_velocity, j);
        out.torque[j] = interp_tire(in.wheel_torque, j);
    }
}


void interpolate_input(
    const Model& model, const AxleInput& in, double t, SampleInput& out
) {
    interpolate_input(in, t, out);
    const std::size_t n = in.sample_count;
    const std::size_t count = model.steering_actuators.size();
    if (n > 0 && model.vehicle_brake_torque != nullptr) {
        const std::size_t k = [&]() {
            std::size_t index = 0;
            while (index + 1 < n && in.sample_times[index + 1] < t) {
                ++index;
            }
            return index;
        }();
        const std::size_t k1 = std::min(k + 1, n - 1);
        const double t0 = in.sample_times[k];
        const double t1 = in.sample_times[k1];
        const double raw_u = std::abs(t1 - t0) > kEps
            ? (t - t0) / (t1 - t0) : 0.0;
        const double u = std::max(0.0, std::min(1.0, raw_u));
        const std::size_t tire_count = in.tire_count;
        for (std::size_t j = 0; j < tire_count; ++j) {
            const double a = model.vehicle_brake_torque[k*tire_count+j];
            const double b = model.vehicle_brake_torque[k1*tire_count+j];
            out.brake_torque[j] = (1.0 - u)*a + u*b;
        }
    }
    out.steering_target.assign(count, 0.0);
    out.steering_target_rate.assign(count, 0.0);
    out.steering_target_acceleration.assign(count, 0.0);
    const std::size_t driven = model.driven_signals.size();
    out.driven_target.assign(driven, 0.0);
    out.driven_target_rate.assign(driven, 0.0);
    out.driven_target_acceleration.assign(driven, 0.0);
    if (n == 0 || (count == 0 && driven == 0)) return;
    std::size_t k = 0;
    while (k + 1 < n && in.sample_times[k + 1] < t) ++k;
    const std::size_t k1 = std::min(k + 1, n - 1);
    const double t0 = in.sample_times[k];
    const double t1 = in.sample_times[k1];
    const double raw_u = std::abs(t1 - t0) > kEps ? (t - t0) / (t1 - t0) : 0.0;
    const double u = std::max(0.0, std::min(1.0, raw_u));
    for (std::size_t j = 0; j < driven; ++j) {
        const DrivenSignal& signal = model.driven_signals[j];
        const std::size_t stride = static_cast<std::size_t>(
            std::max(signal.stride, 0)
        );
        const std::size_t column = static_cast<std::size_t>(
            std::max(signal.column, 0)
        );
        if (signal.values != nullptr) {
            const double a = signal.values[k * stride + column];
            const double b = signal.values[k1 * stride + column];
            out.driven_target[j] = (1.0 - u) * a + u * b;
        }
        if (signal.rates != nullptr) {
            const double a = signal.rates[k * stride + column];
            const double b = signal.rates[k1 * stride + column];
            out.driven_target_rate[j] = (1.0 - u) * a + u * b;
            // Second derivative of the target, used by the acceleration-level row.
            // Same construction the steering targets use: the slope of the rate
            // table across the bracketing knots.
            if (k1 != k && std::abs(t1 - t0) > kEps) {
                out.driven_target_acceleration[j] = (b-a)/(t1-t0);
            }
        }
    }
    for (std::size_t j = 0; j < count; ++j) {
        const SteeringActuator& actuator = model.steering_actuators[j];
        if (actuator.target_angle != nullptr) {
            const double a = actuator.target_angle[k * count + j];
            const double b = actuator.target_angle[k1 * count + j];
            out.steering_target[j] = (1.0 - u) * a + u * b;
        }
        if (actuator.target_rate != nullptr) {
            const double a = actuator.target_rate[k * count + j];
            const double b = actuator.target_rate[k1 * count + j];
            out.steering_target_rate[j] = (1.0 - u) * a + u * b;
            if (k1 != k && std::abs(t1 - t0) > kEps) {
                out.steering_target_acceleration[j] = (b-a)/(t1-t0);
            }
        }
    }
}


double next_prescribed_input_breakpoint(
    const AxleInput& input, double time, double target
) {
    if (input.sample_times == nullptr || input.sample_count < 2) {
        return target;
    }
    const double tolerance =
        1e-14 * std::max({1.0, std::abs(time), std::abs(target)});
    for (std::size_t index = 0; index < input.sample_count; ++index) {
        const double knot = input.sample_times[index];
        if (knot > time + tolerance && knot < target - tolerance) {
            return knot;
        }
    }
    return target;
}


} // namespace axle_kernel
