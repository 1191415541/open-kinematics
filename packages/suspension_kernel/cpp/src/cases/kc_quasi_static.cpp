/// Expand a `kc_quasi_static` case document into concrete solver runs.
///
/// The document is declarative: it says which wheel-travel and rack values to
/// sweep, or which load paths to walk, and nothing about how many samples that
/// is.  The expansion -- the cartesian product, the absolute targets, the
/// wrench transfer from the wheel centre to the body origin -- happens here, on
/// the kernel side of the boundary, for one reason: a case document that
/// carried its own expanded table could disagree with the model it runs on.
///
/// Only the families this build implements are accepted.  Everything else is
/// rejected by the dispatcher, by name, before this file is reached.

#include "case_common.hpp"

#include "mb_config/constants.hpp"

namespace axle_kernel {
namespace case_detail {
namespace {

/// The driven targets and rates of one case, in model coordinate order.
///
/// A rate is the explicit time derivative the solver's velocity- and
/// acceleration-level rows need.  Every case this reader expands moves its
/// driven coordinates to a constant target, so the derivative is exactly zero;
/// deriving it from a difference quotient would only add rounding noise.
void constant_targets(const ContractModel& model, const std::vector<double>& offsets,
                      std::vector<double>& target, std::vector<double>& rate,
                      const std::vector<double>& times, double ramp) {
  const std::size_t driven = model.driven_count();
  const std::size_t sample_count = times.size();
  target.assign(sample_count * driven, 0.0);
  rate.assign(sample_count * driven, 0.0);
  for (std::size_t index = 0; index < driven; ++index) {
    const double separation = model.driven_separation(index);
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      if (ramp <= 0.0) {
        target[sample * driven + index] = separation + offsets[index];
        continue;
      }
      // A sweep may declare that its travel is reached over a ramp rather than
      // applied at the first sample.  A static trim starts from the assembling
      // pose, so a full-travel step at t=0 asks it for a state it cannot reach
      // in one Newton step; ramping asks for the same end state by a path.
      const double raw = times[sample] / ramp;
      const double clamped = raw <= 0.0 ? 0.0 : (raw >= 1.0 ? 1.0 : raw);
      // A raised cosine rather than a straight line: its rate is zero at both
      // ends, so the initial state the static trim produces -- zero velocity --
      // is consistent with the velocity-level row, and the sweep still arrives
      // at the requested travel.  A linear ramp would demand a velocity at t=0
      // that no trimmed state has.
      const double fraction = 0.5 - 0.5 * std::cos(kPi * clamped);
      target[sample * driven + index] = separation + offsets[index] * fraction;
      rate[sample * driven + index] =
          clamped < 1.0
              ? offsets[index] * 0.5 * kPi * std::sin(kPi * clamped) / ramp
              : 0.0;
    }
  }
}

/// One body and the constant wrench applied to it, in the model's own frame.
struct BodyLoad {
  std::size_t body = 0;
  std::array<double, 6> wrench{};
};

/// Read `body_wrench`: constant wrenches applied to named bodies for the whole
/// run.  Forces are N and moments N*mm, the units a wheel load is quoted in, so
/// the moment half is converted once here rather than at every use.
bool read_body_loads(const JsonValue& entries, const ContractModel& model,
                     std::vector<BodyLoad>& out, std::string& error) {
  if (!entries.is_array()) return fail(error, "body_wrench must be an array");
  for (const JsonValue& entry : entries.items) {
    if (!entry.is_object()) return fail(error, "a body_wrench entry is not an object");
    const std::string* name = entry.find_string("body");
    if (name == nullptr) return fail(error, "a body_wrench entry is missing its body");
    const int index = model.body_index(*name);
    if (index < 0) return fail(error, "body_wrench names unknown body " + *name);
    const JsonValue* values = entry.find("wrench");
    if (values == nullptr) {
      return fail(error, "body_wrench entry " + *name + " needs a wrench");
    }
    std::vector<double> wrench;
    if (!read_numbers(*values, wrench, "body_wrench.wrench", error)) return false;
    if (wrench.size() != 6) {
      return fail(error, "body_wrench entry " + *name + " needs six components");
    }
    BodyLoad load;
    load.body = static_cast<std::size_t>(index);
    for (std::size_t i = 0; i < 6; ++i) {
      load.wrench[i] = i < 3 ? wrench[i] : wrench[i] * kMillimetreScale;
    }
    out.push_back(load);
  }
  return true;
}

/// Apply the run's constant body wrenches to every sample of one case.
void fill_body_loads(const std::vector<BodyLoad>& loads, std::size_t sample_count,
                     std::size_t bodies, std::vector<double>& body_wrench) {
  body_wrench.assign(sample_count * bodies * 6, 0.0);
  for (const BodyLoad& load : loads) {
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      for (std::size_t i = 0; i < 6; ++i) {
        body_wrench[(sample * bodies + load.body) * 6 + i] = load.wrench[i];
      }
    }
  }
}
/// A name for one case of a general grid.  The grid names its axes by driven
/// coordinate, so a value-based name would have to encode an arbitrary number
/// of arbitrary reals; the position in the cartesian product is the identity
/// the caller already has, because it is the order the product was taken in.
std::string padded_case_name(const char* prefix, std::size_t index) {
  std::string digits = std::to_string(index);
  while (digits.size() < 4) digits.insert(digits.begin(), '0');
  return std::string(prefix) + digits;
}

/// Expand `k.axes`: one entry per driven coordinate, each with its own values.
///
/// Every combination is one case and every coordinate the grid does not name
/// stays at the separation the model was assembled with -- which is what makes
/// the combination with all-zero offsets the neutral, zero-residual state the
/// family measures its deformations against.
bool expand_k_axes(const JsonValue& axes, const ContractModel& model,
                   const std::vector<double>& times, double ramp,
                   const std::vector<BodyLoad>& loads, ContractPlan& plan,
                   std::string& error) {
  if (!axes.is_array() || axes.items.empty()) {
    return fail(error, "k.axes must be a non-empty array");
  }
  std::vector<std::size_t> coordinates;
  std::vector<std::vector<double>> values;
  for (const JsonValue& axis : axes.items) {
    if (!axis.is_object()) return fail(error, "a k.axes entry is not an object");
    const std::string* name = axis.find_string("coordinate");
    if (name == nullptr) {
      return fail(error, "a k.axes entry is missing its coordinate name");
    }
    const int index = model.driven_index(*name);
    if (index < 0) return fail(error, "k.axes names unknown coordinate " + *name);
    const JsonValue* list = axis.find("values_mm");
    if (list == nullptr) {
      return fail(error, "k.axes entry " + *name + " needs values_mm");
    }
    std::vector<double> axis_values;
    if (!read_numbers(*list, axis_values, "k.axes.values_mm", error)) return false;
    if (axis_values.empty()) {
      return fail(error, "k.axes entry " + *name + " must not be empty");
    }
    coordinates.push_back(static_cast<std::size_t>(index));
    values.push_back(std::move(axis_values));
  }

  const std::size_t sample_count = times.size();
  std::vector<std::size_t> cursor(coordinates.size(), 0);
  std::size_t case_index = 0;
  for (;;) {
    std::vector<double> offsets(model.driven_count(), 0.0);
    for (std::size_t axis = 0; axis < coordinates.size(); ++axis) {
      // `values_mm` is millimetres by name, so it converts with the millimetre
      // and not with the document's own length unit.
      offsets[coordinates[axis]] = values[axis][cursor[axis]] * kMillimetreScale;
    }
    ContractCase run;
    run.name = padded_case_name("k", case_index);
    run.sample_count = sample_count;
    run.sample_times = times;
    constant_targets(model, offsets, run.driven_target, run.driven_target_rate,
                     times, ramp);
    fill_body_loads(loads, sample_count, model.body_count(), run.body_wrench);
    plan.cases.push_back(std::move(run));
    ++case_index;
    // The last axis varies fastest, so the first axis varies slowest -- the
    // order `itertools.product` produces and the order the shorthand below
    // produces as well.
    std::size_t axis = coordinates.size();
    for (;;) {
      if (axis == 0) return true;
      --axis;
      if (++cursor[axis] < values[axis].size()) break;
      cursor[axis] = 0;
    }
  }
}

bool expand_k(const JsonValue& document, const ContractModel& model,
              const std::vector<double>& times, ContractPlan& plan, std::string& error) {
  const JsonValue* k = document.find("k");
  if (k == nullptr || !k->is_object()) {
    return fail(error, "a kc_quasi_static case needs a k or a c section");
  }

  // An optional ramp: a sweep that reaches its travel gradually says so, and a
  // sweep that does not keeps the historical step-at-zero behaviour.
  double ramp = 0.0;
  if (const Json* value = k->find("ramp_s");
      value != nullptr && value->kind == JsonKind::Number) {
    ramp = value->number;
  }
  if (ramp < 0.0) return fail(error, "k.ramp_s must not be negative");

  // `drive` says which point of the wheel the travel is measured at.  The
  // kernel drives the wheel centre, so a document that asks for the contact
  // point is refused rather than answered with a wheel-centre result: those are
  // different questions and the difference is a force application offset.
  if (const std::string* drive = k->find_string("drive");
      drive != nullptr && *drive != "wheel_center") {
    return fail(error, "k.drive " + *drive +
                            " is not implemented; only \"wheel_center\" is");
  }

  // Two ways to name the axes, both expanding to the same cartesian product.
  // The general form names any driven coordinates and gives each its own list,
  // which is what a caller driving the two sides independently -- a roll, a
  // single-wheel bump -- needs.  The shorthand names two *semantic* axes and is
  // what the frozen K/C baselines are written in, so it stays.
  // A K sweep can carry a constant external wrench per body, which is what the
  // product's "load the knuckle while you move the wheel" case is.  It is one
  // wrench for the whole run rather than a per-sample table: a K state is a
  // kinematic solution, and a force that changed inside it would not be one.
  std::vector<BodyLoad> body_loads;
  if (const JsonValue* entries = k->find("body_wrench"); entries != nullptr) {
    if (!read_body_loads(*entries, model, body_loads, error)) return false;
  }

  if (const JsonValue* axes = k->find("axes"); axes != nullptr) {
    return expand_k_axes(*axes, model, times, ramp, body_loads, plan, error);
  }

  const JsonValue* wheels_json = k->find("wheel_values_mm");
  const JsonValue* rack_json = k->find("rack_values_mm");
  if (wheels_json == nullptr || rack_json == nullptr) {
    return fail(error, "the k section needs wheel_values_mm and rack_values_mm");
  }
  std::vector<double> wheels;
  std::vector<double> racks;
  if (!read_numbers(*wheels_json, wheels, "k.wheel_values_mm", error) ||
      !read_numbers(*rack_json, racks, "k.rack_values_mm", error)) {
    return false;
  }
  if (wheels.empty() || racks.empty()) {
    return fail(error, "the k grid must not be empty");
  }

  // The grid axes name the driven coordinates they move.  Without this the
  // kernel would have to recognise coordinate *names*, which would make the
  // authoring layer's naming scheme part of the ABI.
  const JsonValue* axis_map = k->find("axis_map");
  if (axis_map == nullptr || !axis_map->is_object()) {
    return fail(error, "the k section needs an axis_map naming its driven coordinates");
  }
  const JsonValue* wheel_names = axis_map->find("wheel");
  const std::string* rack_name = axis_map->find_string("rack");
  if (wheel_names == nullptr || !wheel_names->is_array() || rack_name == nullptr) {
    return fail(error, "k.axis_map needs a wheel list and a rack name");
  }

  std::vector<std::size_t> wheel_indices;
  std::vector<std::size_t> rack_indices;
  const std::size_t driven = model.driven_count();
  for (const JsonValue& name : wheel_names->items) {
    if (name.kind != JsonKind::String) {
      return fail(error, "k.axis_map.wheel must contain coordinate names");
    }
    const int index = model.driven_index(name.text);
    if (index < 0) {
      return fail(error, "k.axis_map.wheel names unknown coordinate " + name.text);
    }
    wheel_indices.push_back(static_cast<std::size_t>(index));
  }
  {
    const int index = model.driven_index(*rack_name);
    if (index < 0) {
      return fail(error, "k.axis_map.rack names unknown coordinate " + *rack_name);
    }
    rack_indices.push_back(static_cast<std::size_t>(index));
  }
  if (wheel_indices.empty()) {
    return fail(error, "k.axis_map.wheel must name at least one coordinate");
  }

  // How the sweep is distributed across the wheels it names.  A symmetric bump
  // moves them together, a single-wheel case moves one, and an opposite case
  // rolls the axle by alternating the sign.  Axle K/C has always been
  // symmetric, which is why that is the default and why adding the modes leaves
  // its numbers untouched.
  std::vector<double> wheel_signs(wheel_indices.size(), 1.0);
  if (const std::string* mode = k->find_string("left_right_mode");
      mode != nullptr) {
    if (*mode == "single") {
      for (std::size_t slot = 1; slot < wheel_signs.size(); ++slot) {
        wheel_signs[slot] = 0.0;
      }
    } else if (*mode == "opposite") {
      for (std::size_t slot = 0; slot < wheel_signs.size(); ++slot) {
        wheel_signs[slot] = (slot % 2 == 0) ? 1.0 : -1.0;
      }
    } else if (*mode != "symmetric") {
      return fail(error, "unknown k.left_right_mode " + *mode);
    }
  }

  const std::size_t sample_count = times.size();
  for (const double wheel : wheels) {
    for (const double rack : racks) {
      std::vector<double> offsets(driven, 0.0);
      // `wheel_values_mm` and `rack_values_mm` are millimetres by name, so they
      // convert with the millimetre, not with the document's own length unit.
      // A document written in metres would otherwise turn a 10 mm travel into
      // ten metres, which is exactly the mistake a field name is there to stop.
      for (std::size_t slot = 0; slot < wheel_indices.size(); ++slot) {
        offsets[wheel_indices[slot]] =
            wheel * kMillimetreScale * wheel_signs[slot];
      }
      for (std::size_t index : rack_indices) {
        offsets[index] = rack * kMillimetreScale;
      }
      ContractCase run;
      run.name = "k-w" + format("%+.0f", wheel) + "-r" + format("%+.0f", rack);
      run.sample_count = sample_count;
      run.sample_times = times;
      constant_targets(model, offsets, run.driven_target, run.driven_target_rate,
                       times, ramp);
      fill_body_loads(body_loads, sample_count, model.body_count(), run.body_wrench);
      plan.cases.push_back(std::move(run));
    }
  }
  return true;
}
/// One load to run and the name its case gets.  A path sweep and an explicit
/// load list are two spellings of the same thing -- a list of six-component
/// loads -- so they converge here rather than in two copies of the case loop.
struct LoadCase {
  std::array<double, 6> load{};
  std::string name;
};

/// Read `c.loads`: explicit six-component loads, forces in N and moments in
/// N*mm, matching the units the scalar path sweep gives its levels.
bool read_explicit_loads(const JsonValue& loads, std::vector<LoadCase>& out,
                         std::string& error) {
  if (!loads.is_array() || loads.items.empty()) {
    return fail(error, "c.loads must be a non-empty array");
  }
  for (const JsonValue& entry : loads.items) {
    if (!entry.is_object()) return fail(error, "a c.loads entry is not an object");
    LoadCase item;
    for (int axis = 0; axis < 6; ++axis) {
      if (const Json* value = entry.find(load_axes()[axis]);
          value != nullptr && value->kind == JsonKind::Number) {
        item.load[static_cast<std::size_t>(axis)] = value->number;
      }
    }
    out.push_back(std::move(item));
  }
  return true;
}

bool expand_c(const JsonValue& document, const ContractModel& model,
              const std::vector<double>& times, ContractPlan& plan, std::string& error) {
  const JsonValue* c = document.find("c");
  if (c == nullptr || !c->is_object()) {
    return fail(error, "a kc_quasi_static case needs a k or a c section");
  }

  // How the load is distributed over the two sides.  `single` loads the marker
  // the document names; the other two also load the mirror marker, in phase or
  // in anti-phase, which is a roll or a symmetric squeeze.  The mirror has to
  // be named rather than guessed: a document that loads one side and means two
  // is a silent half-answer.
  std::string mode = "single";
  if (const std::string* side_mode = c->find_string("side_mode");
      side_mode != nullptr) {
    mode = *side_mode;
  }
  if (mode != "single" && mode != "symmetric" && mode != "opposite") {
    return fail(error, "load-path side mode " + mode +
                            " is not implemented; single, symmetric and "
                            "opposite are");
  }
  const std::string* marker_name = c->find_string("load_marker");
  if (marker_name == nullptr) {
    return fail(error, "the c section needs load_marker, the wheel centre it loads");
  }
  ContractMarker marker;
  if (!model.find_marker(*marker_name, marker)) {
    return fail(error, "c.load_marker names unknown marker " + *marker_name);
  }
  ContractMarker mirror;
  bool has_mirror = false;
  if (mode != "single") {
    const std::string* mirror_name = c->find_string("mirror_marker");
    if (mirror_name == nullptr) {
      return fail(error, "c.side_mode " + mode +
                              " needs c.mirror_marker, the other side's wheel centre");
    }
    if (!model.find_marker(*mirror_name, mirror)) {
      return fail(error, "c.mirror_marker names unknown marker " + *mirror_name);
    }
    has_mirror = true;
  }

  // The two spellings of "which loads to run": a scalar sweep along named axes,
  // or an explicit list that may put several components in one load.
  std::vector<LoadCase> loads;
  if (const Json* explicit_loads = c->find("loads"); explicit_loads != nullptr) {
    if (!read_explicit_loads(*explicit_loads, loads, error)) return false;
    for (std::size_t index = 0; index < loads.size(); ++index) {
      loads[index].name = padded_case_name("c", index);
    }
  } else {
    std::vector<std::string> paths;
    if (const JsonValue* values = c->find("paths"); values != nullptr) {
      for (const JsonValue& value : values->items) {
        if (value.kind != JsonKind::String) {
          return fail(error, "c.paths must contain load-path names");
        }
        paths.push_back(value.text);
      }
    } else {
      for (int index = 0; index < 6; ++index) paths.push_back(load_axes()[index]);
    }
    if (paths.empty()) return fail(error, "c.paths must not be empty");

    const int levels = integer_or(*c, "levels", 0);
    if (levels < 3) return fail(error, "c.levels must be at least three");
    const double maximum = number_or(*c, "maximum", 0.0);
    if (!(maximum > 0.0)) return fail(error, "c.maximum must be positive");

    std::vector<double> level_values;
    linspace(-maximum, maximum, static_cast<std::size_t>(levels), level_values);
    for (const std::string& path : paths) {
      const int axis = load_axis_index(path);
      if (axis < 0) return fail(error, "unknown load path " + path);
      for (const double level : level_values) {
        LoadCase item;
        item.load[static_cast<std::size_t>(axis)] = level;
        item.name = "c-" + path + "-" + format("%+.2f", level);
        loads.push_back(std::move(item));
      }
    }
  }

  // The body wrench the kernel takes is in the model's own units -- metres and
  // N*m -- while a load path states its moments in N*mm, the unit wheel-centre
  // loads are quoted in.  Convert the moment half once, here, so that no later
  // write has to remember which half is which.  Forces are unit-agnostic in
  // length, which is why only half the vector moves.
  for (LoadCase& item : loads) {
    for (std::size_t index = 3; index < 6; ++index) {
      item.load[index] *= kMillimetreScale;
    }
  }
  // The load is given at a point of the upright, not at the upright's origin.
  // The kernel resolves that lever arm at the pose it solves for, so the model
  // has to declare the marker; a model that has not is refused rather than
  // silently given the origin convention, which is wrong by the first-order
  // geometry of the swept upright (2.8e-8 rad on a 2.5e-5 rad response).
  const std::size_t body = static_cast<std::size_t>(marker.body);
  if (!model.has_body_wrench_point(marker.body)) {
    return fail(error, "c.load_marker " + *marker_name +
                            " has no declared application point; name it in the "
                            "model's body_wrench_markers so the lever arm is "
                            "resolved at the pose being solved for");
  }
  if (has_mirror && !model.has_body_wrench_point(mirror.body)) {
    return fail(error, "c.mirror_marker has no declared application point; name it "
                       "in the model's body_wrench_markers as well");
  }

  const std::size_t sample_count = times.size();
  const std::size_t bodies = model.body_count();
  const std::vector<double> offsets(model.driven_count(), 0.0);
  for (const LoadCase& item : loads) {
    // A load path holds its driven coordinates where they were assembled: it
    // is the wrench that changes, not the travel.
    ContractCase run;
    run.name = item.name;
    run.sample_count = sample_count;
    run.sample_times = times;
    run.body_wrench.assign(sample_count * bodies * 6, 0.0);
    const double mirror_sign = mode == "opposite" ? -1.0 : 1.0;
    for (std::size_t sample = 0; sample < sample_count; ++sample) {
      for (std::size_t index = 0; index < 6; ++index) {
        run.body_wrench[(sample * bodies + body) * 6 + index] = item.load[index];
      }
      if (has_mirror) {
        for (std::size_t index = 0; index < 6; ++index) {
          run.body_wrench[(sample * bodies + static_cast<std::size_t>(mirror.body)) * 6 +
                          index] = mirror_sign * item.load[index];
        }
      }
    }
    constant_targets(model, offsets, run.driven_target, run.driven_target_rate,
                     times, 0.0);
    plan.cases.push_back(std::move(run));
  }
  return true;
}
}  // namespace

bool expand_driven_grid(const Json& document, const ContractModel& model,
                        ContractPlan& out, std::string& error) {
  std::vector<double> times;
  // A sweep declares its grid, so it carries no payload: an irregular history
  // spells its sample instants out in one, and reading that would need a blob.
  if (!read_time(document, "", times, error)) return false;

  const bool has_k = document.find("k") != nullptr;
  const bool has_c = document.find("c") != nullptr;
  if (!has_k && !has_c) {
    return fail(error, "a grid case needs a k or a c section");
  }
  if (has_k && !expand_k(document, model, times, out, error)) return false;
  if (has_c && !expand_c(document, model, times, out, error)) return false;
  if (out.cases.empty()) return fail(error, "the case document expanded to no cases");
  return true;
}

bool expand_kc_quasi_static(const Json& document, const ContractModel& model,
                            ContractPlan& out, std::string& error) {
  error.clear();
  out = ContractPlan{};
  if (!read_identity(document, "kc_quasi_static", out, error)) return false;

  return expand_driven_grid(document, model, out, error);
}

}  // namespace case_detail
}  // namespace axle_kernel
