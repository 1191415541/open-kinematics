/// Expand a `vehicle_dynamic` case document into one solver run.
///
/// A full vehicle case differs from an axle one in what a case *is*: the model
/// already carries the steering actuator, the tires and their drive mapping, so
/// what the case adds is the excitation -- a steering target, a road the wheels
/// follow, brake torque, wheel torque and body wrenches -- plus the road profile
/// itself, which is a case input because the same vehicle is driven over
/// different surfaces.
///
/// The sample tables travel in the container's blob and the document describes
/// them by role, exactly as the axle family does.  The roles are the same set
/// plus `brake_torque` and `steering_target`, which is why the readers for them
/// live in the shared header rather than here.

#include "case_common.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace axle_kernel {
namespace case_detail {
namespace {

using Json = JsonValue;

/// The per-tire roles a sample table can play.
///
/// The first two are the old N*m tables the drive and brake path applies as
/// torque; the last two are the *normalized* driver demands the rotational
/// actuator's demand channel reads (subtask p2-10).  They are distinct roles
/// because they carry different units, and a table that names one may not be
/// read by the other's consumer.
enum class TireRole {
  RoadHeight,
  RoadVelocity,
  WheelTorque,
  BrakeTorque,
  ThrottleDemand,
  BrakePressure,
  None
};

TireRole tire_role_of(const std::string& role) {
  if (role == "road_height") return TireRole::RoadHeight;
  if (role == "road_velocity") return TireRole::RoadVelocity;
  if (role == "wheel_torque") return TireRole::WheelTorque;
  if (role == "brake_torque") return TireRole::BrakeTorque;
  if (role == "throttle_demand") return TireRole::ThrottleDemand;
  if (role == "brake_pressure") return TireRole::BrakePressure;
  return TireRole::None;
}

/// Whether a role carries a normalized driver demand.
bool role_is_demand(TireRole role) {
  return role == TireRole::ThrottleDemand || role == TireRole::BrakePressure;
}

}  // namespace

bool expand_vehicle_dynamic(const Json& document, const std::string& blob,
                            const ContractModel& model, ContractPlan& out,
                            std::string& error) {
  if (!read_identity(document, "vehicle_dynamic", out, error)) return false;

  std::vector<double> times;
  if (!read_time(document, blob, times, error)) return false;
  const std::size_t sample_count = times.size();
  const std::size_t tire_count = model.tire_order().size();
  const std::size_t body_count = model.body_count();
  const std::size_t actuator_count = model.steering_order().size();

  ContractCase run;
  run.name = out.name;
  run.sample_count = sample_count;
  run.sample_times = times;
  run.body_wrench.assign(sample_count * body_count * 6, 0.0);
  run.road_z.assign(sample_count * tire_count, 0.0);
  run.road_velocity.assign(sample_count * tire_count, 0.0);
  run.wheel_torque.assign(sample_count * tire_count, 0.0);
  run.brake_torque.assign(sample_count * tire_count, 0.0);
  run.steering_target.assign(sample_count * actuator_count, 0.0);
  run.steering_rate.assign(sample_count * actuator_count, 0.0);
  // The driven tables are read per coordinate by the `driven_offset`
  // role, so they are sized once here for the whole history.
  run.driven_target.assign(sample_count * model.driven_count(), 0.0);
  run.driven_target_rate.assign(sample_count * model.driven_count(), 0.0);

  if (!read_road(document, run, error)) return false;
  if (!read_case_overrides(document, run, error)) return false;
  if (run.has_static_gauge) {
    const int index = model.body_index(run.static_gauge_body_name);
    if (index < 0) {
      return fail(error, "static_gauge names unknown body " + run.static_gauge_body_name);
    }
    run.static_gauge_body = static_cast<std::size_t>(index);
  }

  // Which unit system each tire has *already* been stated in, per channel, by
  // the roles read so far.  A channel is the wheel or the brake -- a car may
  // legitimately have a driven wheel *and* a braked one, so the two are
  // tracked apart.  The unit tables are zero-filled before the loop, so "is
  // the table non-empty" cannot answer this; only the roles the document
  // actually named can, and that is what this records.
  std::vector<int> wheel_units(static_cast<std::size_t>(tire_count), 0);
  std::vector<int> brake_units(static_cast<std::size_t>(tire_count), 0);
  const Json* blobs = document.find("blobs");
  if (blobs != nullptr) {
    if (!blobs->is_array()) return fail(error, "blobs must be an array");
    for (const Json& descriptor : blobs->items) {
      if (!descriptor.is_object()) {
        return fail(error, "a blob descriptor is not an object");
      }
      const std::string* role = descriptor.find_string("role");
      if (role == nullptr) return fail(error, "a blob descriptor has no role");
      if (*role == "sample_times") {
        // An irregular history puts its sample instants in this same payload;
        // `read_time` has already consumed them by name.
        continue;
      }

      const TireRole tire_role = tire_role_of(*role);
      if (tire_role != TireRole::None) {
        const std::string* tire = descriptor.find_string("tire");
        if (tire == nullptr) {
          return fail(error, "role " + *role + " needs a tire name");
        }
        const int index = model.tire_index(*tire);
        if (index < 0) {
          return fail(error, "role " + *role + " names unknown tire " + *tire);
        }
        std::vector<double>* target = nullptr;
        double scale = 1.0;
        switch (tire_role) {
          case TireRole::RoadHeight:
            target = &run.road_z;
            scale = model.length_scale();
            break;
          case TireRole::RoadVelocity:
            target = &run.road_velocity;
            scale = model.length_scale();
            break;
          case TireRole::WheelTorque:
            target = &run.wheel_torque;
            scale = model.moment_scale();
            break;
          case TireRole::BrakeTorque:
            target = &run.brake_torque;
            scale = model.moment_scale();
            break;
          // A normalized demand is a *fraction*, so nothing scales it: the
          // document's length unit turns millimetres into metres and its moment
          // unit turns a stated torque into newton-metres, but "half the pedal"
          // is half whatever the unit system is.
          case TireRole::ThrottleDemand:
            target = &run.wheel_demand;
            scale = 1.0;
            break;
          case TireRole::BrakePressure:
            target = &run.brake_demand;
            scale = 1.0;
            break;
          case TireRole::None:
            break;
        }
        // The two unit systems are not interchangeable *within one channel*,
        // and a case that states both for one tire's wheel or brake is
        // describing two actuators in one place.  It is refused by name rather
        // than merged -- silently picking one would apply either a 0.6 N*m
        // couple or a 0.6 fraction, and those are not the same physical claim.
        // A driven *and* braked wheel is not this case: the two live in
        // different channels and are tracked apart.
        std::vector<int>* units_for_tire = nullptr;
        int units = 0;
        switch (tire_role) {
          case TireRole::WheelTorque:
          case TireRole::ThrottleDemand:
            units_for_tire = &wheel_units;
            units = tire_role == TireRole::WheelTorque ? 1 : 2;
            break;
          case TireRole::BrakeTorque:
          case TireRole::BrakePressure:
            units_for_tire = &brake_units;
            units = tire_role == TireRole::BrakeTorque ? 1 : 2;
            break;
          default:
            break;
        }
        if (units_for_tire != nullptr) {
          int& seen = (*units_for_tire)[static_cast<std::size_t>(index)];
          if (seen != 0 && seen != units) {
            return fail(error,
                        "role " + *role + " on tire " + *tire +
                        " states a unit its channel was already given in the "
                        "other system; a wheel's torque is stated in newton-metres "
                        "or as a normalized demand, not both");
          }
          seen = units;
        }
        // A demand table is allocated the first time a demand role names
        // it and stays empty otherwise, so "empty" continues to mean
        // "this run declares no demand" for every reader of the case --
        // which is what keeps an un-opted-in model byte-for-byte what it
        // was.  Allocating it up front would make every run look like it
        // declared a demand of zero.
        if (role_is_demand(tire_role) && target->empty()) {
          target->assign(sample_count * tire_count, 0.0);
        }
        if (!read_column_table(descriptor, blob, sample_count,
                               static_cast<std::size_t>(index), tire_count, scale,
                               *target, "role " + *role, error)) {
          return false;
        }
        // A normalized demand has a declared range, and a value outside it
        // is a mistake rather than something to clip: the brake's own
        // subsystem refuses `brake_input` outside [0, 1] and the drive's
        // refuses `drive_input` outside [-1, 1] (subsystems/brake.py,
        // subsystems/drive.py), so the case has to say the same thing or
        // the two layers would disagree about what a legal demand is.
        if (role_is_demand(tire_role)) {
          const double low = tire_role == TireRole::ThrottleDemand ? -1.0 : 0.0;
          for (std::size_t sample = 0; sample < sample_count; ++sample) {
            const double value = (*target)[sample * tire_count + static_cast<std::size_t>(index)];
            if (!std::isfinite(value) || value < low || value > 1.0) {
              return fail(error,
                          "role " + *role + " on tire " + *tire +
                          " must be within [" + (low < 0.0 ? "-1" : "0") +
                          ", 1]; got " + std::to_string(value));
            }
          }
        }

        continue;
      }

      if (*role == "steering_target" || *role == "steering_rate") {
        const std::string* actuator = descriptor.find_string("actuator");
        if (actuator == nullptr) {
          return fail(error, "role " + *role + " needs an actuator name");
        }
        const int index = model.steering_index(*actuator);
        if (index < 0) {
          return fail(error, "role " + *role + " names unknown actuator " + *actuator);
        }
        // A prescribed translation moves along the document's length unit; a
        // prescribed rotation is an angle, which does not scale at all.
        const std::string* quantity = descriptor.find_string("quantity");
        const double scale =
            (quantity != nullptr && *quantity == "translation") ? model.length_scale()
                                                                : 1.0;
        std::vector<double>& target =
            (*role == "steering_target") ? run.steering_target : run.steering_rate;
        if (!read_column_table(descriptor, blob, sample_count,
                               static_cast<std::size_t>(index), actuator_count, scale,
                               target, "role " + *role, error)) {
          return false;
        }
        continue;
      }


      if (*role == "driven_offset" || *role == "driven_offset_rate") {
        const std::string* coordinate = descriptor.find_string("coordinate");
        if (coordinate == nullptr) {
          return fail(error, "role " + *role + " needs a coordinate name");
        }
        const int index = model.driven_index(*coordinate);
        if (index < 0) {
          return fail(error,
                      "role " + *role + " names unknown coordinate " + *coordinate);
        }
        const std::size_t signal = static_cast<std::size_t>(index);
        if (model.driven_count() == 0) {
          return fail(error, "the model declares no driven coordinates");
        }
        // A translation is a length and follows the document's unit; a rotation
        // is an angle and does not.
        const bool rotation = model.driven_is_rotation(signal);
        const double scale = rotation ? 1.0 : model.length_scale();
        const bool is_offset = *role == "driven_offset";
        std::vector<double>& table =
            is_offset ? run.driven_target : run.driven_target_rate;
        if (!read_column_table(descriptor, blob, sample_count, signal,
                               model.driven_count(), scale, table,
                               "role " + *role, error)) {
          return false;
        }
        if (is_offset && !rotation) {
          // The constraint row measures the *absolute* separation, so the
          // assembling separation comes back: the case says "ten millimetres
          // more" and the geometry stays where the geometry lives.
          const double separation = model.driven_separation(signal);
          for (std::size_t sample = 0; sample < sample_count; ++sample) {
            table[sample * model.driven_count() + signal] += separation;
          }
        }
        continue;
      }

      if (*role == "body_wrench") {
        const std::string* body = descriptor.find_string("body");
        if (body == nullptr) {
          return fail(error, "role body_wrench needs a body name");
        }
        const int index = model.body_index(*body);
        if (index < 0) {
          return fail(error, "role body_wrench names unknown body " + *body);
        }
        if (!read_body_table(descriptor, blob, sample_count,
                             static_cast<std::size_t>(index), body_count * 6,
                             model.moment_scale(), run.body_wrench,
                             "role " + *role, error)) {
          return false;
        }
        continue;
      }

      return fail(error, "unknown blob role " + *role);
    }
  }

  out.cases.push_back(std::move(run));
  return true;
}

}  // namespace case_detail
}  // namespace axle_kernel
