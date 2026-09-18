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

/// The per-tire roles a sample table can play.  The axle family has no brake
/// channel: a wheel torque is the whole excitation it applies.
enum class TireRole { RoadHeight, RoadVelocity, WheelTorque, None };

TireRole tire_role_of(const std::string& role) {
  if (role == "road_height") return TireRole::RoadHeight;
  if (role == "road_velocity") return TireRole::RoadVelocity;
  if (role == "wheel_torque") return TireRole::WheelTorque;
  return TireRole::None;
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
          case TireRole::None:
            break;
        }
        if (!read_column_table(descriptor, blob, sample_count,
                             static_cast<std::size_t>(index), tire_count, scale,
                             *target, "role " + *role, error)) {
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
