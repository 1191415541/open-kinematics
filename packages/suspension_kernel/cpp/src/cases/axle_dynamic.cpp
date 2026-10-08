/// Expand an `axle_dynamic` case document into one solver run.
///
/// Unlike the quasi-static family, this one does not sweep anything: the
/// document is already a single time history, and what it needs from the case
/// layer is the sampled tables the kernel interpolates -- road height, road
/// velocity, wheel torque and body wrenches.
///
/// Those tables are large, so they do not travel in the JSON.  They live in the
/// container's blob and the document describes them: each entry of `blobs`
/// names a role, the tire or body it belongs to, and the byte range that holds
/// it.  The reader checks every range against the blob before it is read, so a
/// truncated or mis-sized payload is an error rather than a short read.
///
/// Units follow the document: length is millimetres, so a road height is mm, a
/// road velocity mm/s, a torque N*mm and the moment half of a body wrench N*mm.
/// Force stays in newtons because it is not a length-derived quantity.

#include "case_common.hpp"

#include <string>
#include <vector>

namespace axle_kernel {
namespace case_detail {
namespace {

using Json = JsonValue;

/// The per-tire roles a sample table can play.
///
/// `wheel_torque` is the Newton-metre excitation the axle family has always
/// applied.  The two normalized demand roles were added with the closed-loop
/// controller (p5-04): an ABS law modulates the *driver's* demand rather than a
/// pre-computed torque, and the demand reaches the element through the same
/// per-tire tables the vehicle family already carries.  They are dimensionless
/// fractions, so nothing scales them -- a role's unit is a property of the role
/// and the two unit systems must not be mixed inside one channel.
enum class TireRole {
  RoadHeight,
  RoadVelocity,
  WheelTorque,
  ThrottleDemand,
  BrakePressure,
  None
};

TireRole tire_role_of(const std::string& role) {
  if (role == "road_height") return TireRole::RoadHeight;
  if (role == "road_velocity") return TireRole::RoadVelocity;
  if (role == "wheel_torque") return TireRole::WheelTorque;
  if (role == "throttle_demand") return TireRole::ThrottleDemand;
  if (role == "brake_pressure") return TireRole::BrakePressure;
  return TireRole::None;
}

/// Whether a role carries a normalized driver demand.  The vehicle family defines
/// the same predicate; restating it here keeps the two boundaries reading one
/// rule rather than two that can drift.
bool role_is_demand(TireRole role) {
  return role == TireRole::ThrottleDemand || role == TireRole::BrakePressure;
}

}  // namespace

bool expand_axle_dynamic(const Json& document, const std::string& blob,
                         const ContractModel& model, ContractPlan& out,
                         std::string& error) {
  if (!read_identity(document, "axle_dynamic", out, error)) return false;

  std::vector<double> times;
  if (!read_time(document, blob, times, error)) return false;
  const std::size_t sample_count = times.size();
  const std::size_t tire_count = model.tire_order().size();
  const std::size_t body_count = model.body_count();

  ContractCase run;
  run.name = out.name;
  run.sample_count = sample_count;
  run.sample_times = times;
  run.body_wrench.assign(sample_count * body_count * 6, 0.0);
  run.road_z.assign(sample_count * tire_count, 0.0);
  run.road_velocity.assign(sample_count * tire_count, 0.0);
  run.wheel_torque.assign(sample_count * tire_count, 0.0);
  // The normalized demand tables stay *empty* unless a role fills them: the
  // element law reads "empty" as "no driver signal" rather than as a zero
  // demand of a declared one, which is what keeps a model that declares no
  // demand byte-for-byte what it was (subtask p2-10).
  // The driven tables are read per coordinate by the `driven_offset`
  // role, so they are sized once here for the whole history.
  run.driven_target.assign(sample_count * model.driven_count(), 0.0);
  run.driven_target_rate.assign(sample_count * model.driven_count(), 0.0);

  if (!read_case_overrides(document, run, error)) return false;

  const Json* blobs = document.find("blobs");
  if (blobs != nullptr) {
    if (!blobs->is_array()) return fail(error, "blobs must be an array");
    for (const Json& descriptor : blobs->items) {
      if (!descriptor.is_object()) {
        return fail(error, "a blob descriptor is not an object");
      }
      const std::string* role = descriptor.find_string("role");
      if (role == nullptr) return fail(error, "a blob descriptor has no role");
      if (*role == "sample_times" || *role == "motion_target" || *role == "motion_rate") {
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
          // A normalized demand is a *fraction*, so nothing scales it: the
          // document's length unit turns millimetres into metres and its moment
          // unit turns a stated torque into newton-metres, but "half the pedal"
          // is half whatever the unit system is.  The same rule the vehicle
          // family states, restated here so the two boundaries cannot drift.
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
        // A demand table is allocated *when a role first fills it*, not up
        // front: an empty table is how every reader learns "this run declares
        // no demand", which is what keeps an un-opted-in model byte-for-byte
        // what it was.  Allocating it in the constructor would make every run
        // look like it declared a demand of zero.  `read_column_table` writes
        // through `column_count` slots, so an unallocated table would be an
        // out-of-range write rather than a wrong number.
        if (role_is_demand(tire_role) && target->empty()) {
          target->assign(sample_count * tire_count, 0.0);
        }
        if (!read_column_table(descriptor, blob, sample_count,
                             static_cast<std::size_t>(index), tire_count, scale,
                             *target, "role " + *role, error)) {
          return false;
        }
        // A normalized demand has a declared range, and a value outside it is a
        // mistake rather than something to clip: the brake's own subsystem
        // refuses `brake_input` outside [0, 1] and the drive's refuses
        // `drive_input` outside [-1, 1], so the case has to say the same thing
        // or the two layers would disagree about what a legal demand is.
        if (role_is_demand(tire_role)) {
          const double low = tire_role == TireRole::ThrottleDemand ? -1.0 : 0.0;
          for (std::size_t sample = 0; sample < sample_count; ++sample) {
            const double value =
                (*target)[sample * tire_count + static_cast<std::size_t>(index)];
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
