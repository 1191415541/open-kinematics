/// Expand a `ride_random_road` case document into one solver run.
///
/// A random road is a superposition of spatial harmonics, and a vehicle driving
/// along it at speed turns each harmonic into a temporal one: a component of
/// wavelength L traversed at v has angular frequency 2*pi*v/L.  That conversion
/// is this family's whole job -- the wheels follow the profile, and everything
/// after that is the model's business.
///
/// Each wheel carries its own components rather than one profile plus a
/// correlation factor.  That is the more general form and it costs nothing: a
/// correlated pair is the same components on both wheels with an added
/// independent set, and front-to-rear delay is a phase.  A family that took a
/// correlation coefficient instead would only be able to express the special
/// case, and would have to grow a second mechanism the first time someone
/// wanted an uncorrelated profile.
///
/// The velocity is the analytic derivative of that sum, because the kernel
/// interpolates height and velocity independently.

#include "case_common.hpp"

#include "mb_config/constants.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace axle_kernel {
namespace case_detail {
namespace {

using Json = JsonValue;

struct Component {
  double amplitude = 0.0;
  double wavelength = 0.0;
  double phase = 0.0;
};

struct Wheel {
  int tire = -1;
  std::vector<Component> components;
};

}  // namespace

bool expand_ride_random_road(const Json& document, const std::string& blob,
                             const ContractModel& model, ContractPlan& out,
                             std::string& error) {
  if (!read_identity(document, "ride_random_road", out, error)) return false;
  (void)blob;

  std::vector<double> times;
  // A random road is declared as spatial harmonics, so this family carries no
  // payload at all: an irregular history would need one to name its samples.
  if (!read_time(document, "", times, error)) return false;
  const std::size_t sample_count = times.size();
  const std::size_t tire_count = model.tire_order().size();

  const Json* section = document.find("ride_random_road");
  if (section == nullptr || !section->is_object()) {
    return fail(error, "a ride_random_road case needs a ride_random_road section");
  }
  const double speed = number_or(*section, "speed_mps", 0.0);
  if (!(speed >= 0.0)) {
    return fail(error, "ride_random_road speed must not be negative");
  }
  const Json* wheels = section->find("wheels");
  if (wheels == nullptr || !wheels->is_array() || wheels->items.empty()) {
    return fail(error, "ride_random_road.wheels must be a non-empty array");
  }

  std::vector<Wheel> plan;
  for (const Json& entry : wheels->items) {
    if (!entry.is_object()) {
      return fail(error, "a ride_random_road wheel is not an object");
    }
    const std::string* tire = entry.find_string("tire");
    if (tire == nullptr) {
      return fail(error, "a ride_random_road wheel needs a tire name");
    }
    const int index = model.tire_index(*tire);
    if (index < 0) {
      return fail(error, "ride_random_road names unknown tire " + *tire);
    }
    const Json* components = entry.find("components");
    if (components == nullptr || !components->is_array() ||
        components->items.empty()) {
      return fail(error, "ride_random_road wheel " + *tire +
                              " needs a non-empty components array");
    }
    Wheel wheel;
    wheel.tire = index;
    for (const Json& item : components->items) {
      if (!item.is_object()) {
        return fail(error, "a ride_random_road component is not an object");
      }
      Component component;
      component.amplitude = number_or(item, "amplitude_m", 0.0);
      component.wavelength = number_or(item, "wavelength_m", 0.0);
      component.phase = number_or(item, "phase_rad", 0.0);
      if (component.wavelength <= 0.0) {
        return fail(error, "ride_random_road component on " + *tire +
                                " needs a positive wavelength");
      }
      if (component.amplitude < 0.0) {
        return fail(error, "ride_random_road component on " + *tire +
                                " has a negative amplitude");
      }
      wheel.components.push_back(component);
    }
    plan.push_back(std::move(wheel));
  }

  ContractCase run;
  run.name = out.name;
  run.sample_count = sample_count;
  run.sample_times = times;
  run.road_z.assign(sample_count * tire_count, 0.0);
  run.road_velocity.assign(sample_count * tire_count, 0.0);
  run.wheel_torque.assign(sample_count * tire_count, 0.0);
  run.brake_torque.assign(sample_count * tire_count, 0.0);
  run.body_wrench.assign(sample_count * model.body_count() * 6, 0.0);
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

  for (const Wheel& wheel : plan) {
    const std::size_t column = static_cast<std::size_t>(wheel.tire);
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      const double t = times[sample];
      double height = 0.0;
      double vertical = 0.0;
      for (const Component& component : wheel.components) {
        // Spatial frequency over wavelength, carried by the speed.
        const double angular = 2.0 * kPi * speed / component.wavelength;
        const double angle = angular * t + component.phase;
        height += component.amplitude * std::sin(angle);
        vertical += component.amplitude * angular * std::cos(angle);
      }
      const std::size_t slot = sample * tire_count + column;
      run.road_z[slot] = height;
      run.road_velocity[slot] = vertical;
    }
  }

  out.cases.push_back(std::move(run));
  return true;
}

}  // namespace case_detail
}  // namespace axle_kernel
