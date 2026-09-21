/// Expand a `ride_four_post` case document into one solver run.
///
/// A four-post rig prescribes the height of each wheel pad, so this family's
/// whole job is turning a declarative excitation -- per-corner offset,
/// amplitude, frequency and phase -- into the per-tire road height and road
/// velocity the kernel interpolates.  Nothing else about the run differs from a
/// vehicle case: the model decides what the excitation does to the car.
///
/// The velocity is the analytic derivative rather than a difference quotient.
/// A sampled height whose velocity is a finite difference of itself is a
/// slightly different excitation from the one that was asked for, and the
/// kernel interpolates both independently.
///
/// The document may also carry a measured height table per corner instead of a
/// sinusoid; the two forms are exclusive and a corner that declares neither is
/// an error, because a silently flat pad looks exactly like a rig that is
/// working and measuring nothing.

#include "case_common.hpp"

#include "mb_config/constants.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace axle_kernel {
namespace case_detail {
namespace {

using Json = JsonValue;

struct Corner {
  int tire = -1;
  double offset = 0.0;
  double amplitude = 0.0;
  double frequency = 0.0;
  double phase = 0.0;
  /// A measured pad history, when the corner carries one instead of a shape.
  std::string height_blob;
  std::string velocity_blob;
};

}  // namespace

bool expand_ride_four_post(const Json& document, const std::string& blob,
                           const ContractModel& model, ContractPlan& out,
                           std::string& error) {
  if (!read_identity(document, "ride_four_post", out, error)) return false;

  std::vector<double> times;
  if (!read_time(document, blob, times, error)) return false;
  const std::size_t sample_count = times.size();
  const std::size_t tire_count = model.tire_order().size();

  const Json* four_post = document.find("four_post");
  if (four_post == nullptr || !four_post->is_object()) {
    return fail(error, "a ride_four_post case needs a four_post section");
  }
  const Json* corners = four_post->find("corners");
  if (corners == nullptr || !corners->is_array() || corners->items.empty()) {
    return fail(error, "four_post.corners must be a non-empty array");
  }

  std::vector<Corner> plan;
  for (const Json& corner : corners->items) {
    if (!corner.is_object()) {
      return fail(error, "a four_post corner is not an object");
    }
    const std::string* tire = corner.find_string("tire");
    if (tire == nullptr) return fail(error, "a four_post corner needs a tire name");
    const int index = model.tire_index(*tire);
    if (index < 0) {
      return fail(error, "four_post names unknown tire " + *tire);
    }
    Corner entry;
    entry.tire = index;
    entry.offset = number_or(corner, "offset_m", 0.0);
    entry.amplitude = number_or(corner, "amplitude_m", 0.0);
    entry.frequency = number_or(corner, "frequency_hz", 0.0);
    entry.phase = number_or(corner, "phase_rad", 0.0);
    if (entry.amplitude < 0.0 || entry.frequency < 0.0) {
      return fail(error, "four_post corner " + *tire +
                              " has a negative amplitude or frequency");
    }
    if (const std::string* name = corner.find_string("blob"); name != nullptr) {
      entry.height_blob = *name;
    }
    if (const std::string* name = corner.find_string("velocity_blob");
        name != nullptr) {
      entry.velocity_blob = *name;
    }
    plan.push_back(entry);
  }

  ContractCase run;
  run.name = out.name;
  run.sample_count = sample_count;
  run.sample_times = times;
  run.road_z.assign(sample_count * tire_count, 0.0);
  run.road_velocity.assign(sample_count * tire_count, 0.0);
  run.body_wrench.assign(sample_count * model.body_count() * 6, 0.0);
  run.wheel_torque.assign(sample_count * tire_count, 0.0);
  run.brake_torque.assign(sample_count * tire_count, 0.0);
  run.steering_target.assign(sample_count * model.steering_order().size(), 0.0);
  run.steering_rate.assign(sample_count * model.steering_order().size(), 0.0);
  if (!read_road(document, run, error)) return false;
  if (!read_case_overrides(document, run, error)) return false;
  if (run.has_static_gauge) {
    const int index = model.body_index(run.static_gauge_body_name);
    if (index < 0) {
      return fail(error, "static_gauge names unknown body " + run.static_gauge_body_name);
    }
    run.static_gauge_body = static_cast<std::size_t>(index);
  }

  for (const Corner& corner : plan) {
    const std::size_t column = static_cast<std::size_t>(corner.tire);
    const double angular = 2.0 * kPi * corner.frequency;
    // A measured history is read through the same described-array path every
    // other payload table uses, so a truncated one is an error.
    std::vector<double> measured;
    std::vector<double> measured_velocity;
    if (!corner.height_blob.empty()) {
      if (!read_described_array(document, blob, corner.height_blob, sample_count,
                                measured, error)) {
        return false;
      }
      if (!corner.velocity_blob.empty() &&
          !read_described_array(document, blob, corner.velocity_blob, sample_count,
                                measured_velocity, error)) {
        return false;
      }
    }
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      const std::size_t slot = sample * tire_count + column;
      if (!corner.height_blob.empty()) {
        run.road_z[slot] = measured[sample];
        // A measured height with no measured velocity falls back to the
        // analytic derivative of the shape it was declared with, which is
        // exactly zero when the corner declared no shape.
        run.road_velocity[slot] = measured_velocity.empty()
                                      ? corner.amplitude * angular *
                                            std::cos(angular * times[sample] + corner.phase)
                                      : measured_velocity[sample];
        continue;
      }
      const double angle = angular * times[sample] + corner.phase;
      run.road_z[slot] = corner.offset + corner.amplitude * std::sin(angle);
      run.road_velocity[slot] = corner.amplitude * angular * std::cos(angle);
    }
  }

  out.cases.push_back(std::move(run));
  return true;
}

}  // namespace case_detail
}  // namespace axle_kernel
