/// Expand a `handling` case document into one solver run.
///
/// A handling manoeuvre is a steering input as a function of time, so this
/// family's job is the same shape of work as the four-post one: turn a
/// declaration -- a shape per actuator -- into the per-sample target and rate
/// the kernel interpolates.  The rate is the analytic derivative, because the
/// kernel interpolates the two independently and a finite difference of the
/// target is a different manoeuvre.
///
/// Only open-loop manoeuvres are expressed here.  A closed-loop manoeuvre (an
/// ISO lane change, say) needs a driver following a path, which is a different
/// model rather than a different shape, so it is refused by name instead of
/// being approximated with a steering history that happens to look similar.

#include "case_common.hpp"

#include "mb_base/constants.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace axle_kernel {
namespace case_detail {
namespace {

using Json = JsonValue;

struct Shape {
  int actuator = -1;
  std::string kind;
  double amplitude = 0.0;
  double start = 0.0;
  double rise = 0.0;
  double frequency = 0.0;
  double phase = 0.0;
  /// A recorded steering history, when the entry carries one instead of a shape.
  std::string target_blob;
  std::string rate_blob;
};

/// Clamp a ramp's progress to its own window.
double ramp_fraction(double t, double start, double rise) {
  if (rise <= 0.0) return t >= start ? 1.0 : 0.0;
  const double raw = (t - start) / rise;
  return raw <= 0.0 ? 0.0 : (raw >= 1.0 ? 1.0 : raw);
}

void sample_shape(const Shape& shape, double t, double& value, double& rate) {
  if (shape.kind == "constant") {
    // A manoeuvre that begins with the wheel already turned asks the static
    // trim for a state it cannot reach from the assembling pose, so a constant
    // shape is a step *at* its start time rather than a value that exists from
    // the beginning of the run.
    value = t >= shape.start ? shape.amplitude : 0.0;
    rate = 0.0;
    return;
  }
  if (shape.kind == "ramp" || shape.kind == "step") {
    const double fraction = ramp_fraction(t, shape.start, shape.rise);
    value = shape.amplitude * fraction;
    // A step with no rise is a discontinuity, whose derivative is not a
    // function any solver can sample; reporting a zero rate is the honest
    // answer for a point that is only reached instantaneously.
    const bool inside = shape.rise > 0.0 && t > shape.start &&
                        t < shape.start + shape.rise;
    rate = inside ? shape.amplitude / shape.rise : 0.0;
    return;
  }
  // sine
  const double angular = 2.0 * kPi * shape.frequency;
  const double angle = angular * (t - shape.start) + shape.phase;
  value = shape.amplitude * std::sin(angle);
  rate = shape.amplitude * angular * std::cos(angle);
}

}  // namespace

bool expand_handling(const Json& document, const std::string& blob,
                     const ContractModel& model, ContractPlan& out,
                     std::string& error) {
  if (!read_identity(document, "handling", out, error)) return false;

  std::vector<double> times;
  if (!read_time(document, blob, times, error)) return false;
  const std::size_t sample_count = times.size();
  const std::size_t actuator_count = model.steering_order().size();
  const std::size_t tire_count = model.tire_order().size();

  const Json* handling = document.find("handling");
  if (handling == nullptr || !handling->is_object()) {
    return fail(error, "a handling case needs a handling section");
  }
  const Json* shapes = handling->find("steering");
  if (shapes == nullptr || !shapes->is_array() || shapes->items.empty()) {
    return fail(error, "handling.steering must be a non-empty array");
  }

  std::vector<Shape> plan;
  for (const Json& entry : shapes->items) {
    if (!entry.is_object()) {
      return fail(error, "a handling steering entry is not an object");
    }
    const std::string* actuator = entry.find_string("actuator");
    if (actuator == nullptr) {
      return fail(error, "a handling steering entry needs an actuator name");
    }
    const int index = model.steering_index(*actuator);
    if (index < 0) {
      return fail(error, "handling names unknown actuator " + *actuator);
    }
    const std::string* kind = entry.find_string("shape");
    if (kind == nullptr) {
      return fail(error, "handling steering entry " + *actuator + " has no shape");
    }
    if (*kind == "recorded") {
      // A recorded history is data, so it comes through the payload rather
      // than being shaped here.
      Shape recorded;
      recorded.actuator = index;
      recorded.kind = "recorded";
      const std::string* targets = entry.find_string("blob");
      if (targets == nullptr || targets->empty()) {
        return fail(error, "recorded handling entry " + *actuator + " names no blob");
      }
      recorded.target_blob = *targets;
      if (const std::string* rates = entry.find_string("rate_blob");
          rates != nullptr) {
        recorded.rate_blob = *rates;
      }
      plan.push_back(recorded);
      continue;
    }
    if (*kind != "constant" && *kind != "ramp" && *kind != "step" &&
        *kind != "sine") {
      // A closed-loop manoeuvre is a driver model, not a shape.
      return fail(error, "handling manoeuvre \"" + *kind + "\" is"
                              " not open-loop and is not implemented");
    }
    Shape shape;
    shape.actuator = index;
    shape.kind = *kind;
    shape.amplitude = number_or(entry, "amplitude", 0.0);
    shape.start = number_or(entry, "start_s", 0.0);
    shape.rise = number_or(entry, "rise_s", 0.0);
    shape.frequency = number_or(entry, "frequency_hz", 0.0);
    shape.phase = number_or(entry, "phase_rad", 0.0);
    if (shape.rise < 0.0 || shape.frequency < 0.0) {
      return fail(error, "handling shape for " + *actuator +
                              " has a negative rise or frequency");
    }
    plan.push_back(shape);
  }

  ContractCase run;
  run.name = out.name;
  run.sample_count = sample_count;
  run.sample_times = times;
  run.steering_target.assign(sample_count * actuator_count, 0.0);
  run.steering_rate.assign(sample_count * actuator_count, 0.0);
  run.road_z.assign(sample_count * tire_count, 0.0);
  run.road_velocity.assign(sample_count * tire_count, 0.0);
  run.wheel_torque.assign(sample_count * tire_count, 0.0);
  run.brake_torque.assign(sample_count * tire_count, 0.0);
  run.body_wrench.assign(sample_count * model.body_count() * 6, 0.0);
  if (!read_road(document, run, error)) return false;
  if (!read_case_overrides(document, run, error)) return false;
  if (run.has_static_gauge) {
    const int index = model.body_index(run.static_gauge_body_name);
    if (index < 0) {
      return fail(error, "static_gauge names unknown body " + run.static_gauge_body_name);
    }
    run.static_gauge_body = static_cast<std::size_t>(index);
  }

  for (const Shape& shape : plan) {
    const std::size_t column = static_cast<std::size_t>(shape.actuator);
    std::vector<double> recorded;
    std::vector<double> recorded_rate;
    if (!shape.target_blob.empty()) {
      if (!read_described_array(document, blob, shape.target_blob, sample_count,
                                recorded, error)) {
        return false;
      }
      if (!shape.rate_blob.empty() &&
          !read_described_array(document, blob, shape.rate_blob, sample_count,
                                recorded_rate, error)) {
        return false;
      }
    }
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      double value = 0.0;
      double rate = 0.0;
      if (!shape.target_blob.empty()) {
        value = recorded[sample];
        // A recorded history with no recorded rate falls back to its own
        // difference quotient rather than to zero, which would misreport a
        // moving manoeuvre as a still one.
        if (!recorded_rate.empty()) {
          rate = recorded_rate[sample];
        } else if (sample_count > 1) {
          const std::size_t next = sample + 1 < sample_count ? sample + 1 : sample;
          const std::size_t previous = sample == 0 ? 0 : sample - 1;
          const double span = times[next] - times[previous];
          rate = span > 0.0 ? (recorded[next] - recorded[previous]) / span : 0.0;
        }
      } else {
        sample_shape(shape, times[sample], value, rate);
      }
      run.steering_target[sample * actuator_count + column] = value;
      run.steering_rate[sample * actuator_count + column] = rate;
    }
  }

  out.cases.push_back(std::move(run));
  return true;
}

}  // namespace case_detail
}  // namespace axle_kernel
