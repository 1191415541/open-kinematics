/// Evaluate the rotational-torque element and print what it applied.
///
/// This is a standalone probe rather than a pure-Python test because the
/// observable lives below the product C ABI: `Model::rotational_torques` and the
/// couple `assemble_rotational_torque_forces` accumulates into the torque buffer
/// are internal, and the product deliberately exposes no element-level wrench
/// channel for a single law.  The probe is compiled at test time and linked
/// against the static libraries the kernel build already produced, so it reads
/// the same translation units the shipped library was built from.
///
/// It answers two questions, and prints one line per fact the test asserts:
///
/// * `reader ...` -- does a block of `ELEMENT_ROTATIONAL_TORQUE` survive
///   `read_element_blocks` and reach `Model::rotational_torques` with the values
///   the block carried?
/// * `eval ...` -- for a given relative angular rate, what couple did the law
///   apply to each body?  The three states the epic requires assertions for are
///   one line each: a stationary pair (`rate` inside `kEps`), and the two
///   spinning directions, each with a magnitude below the cap and one above it.

#include "mb_assembly/functions.hpp"
#include "mb_element/functions.hpp"
#include "mb_input/functions.hpp"
#include "mb_model/types.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

// `ELEMENT_ROTATIONAL_TORQUE_*`, `ElementBlock`, `ElementCurveReference` and
// `kElementCurveSlots` all live at global scope in `mb_input/types.hpp`, inside
// its `extern "C"` block: they are the ABI's own names, so they carry no C++
// namespace and need no `using`.  Only the model layer's types are namespaced.
using axle_kernel::Model;
using axle_kernel::Quat;
using axle_kernel::RotationalTorque;
using axle_kernel::SampleInput;
using axle_kernel::State;
using axle_kernel::Vec3;

/// Two free bodies at the origin with the identity pose.  Both must be free:
/// `add_torque_on_body` suppresses the accumulation on a fixed body, so a probe
/// with a grounded `a` would print the reaction as zero and prove nothing about
/// the equal-and-opposite pair.
void seed_bodies(Model& model) {
  model.bodies.resize(2);
  model.bodies[0].fixed = false;
  model.bodies[1].fixed = false;
}

/// A state with the given angular velocities, everything else at rest.
State seeded_state(double omega_a_along_axis, double omega_b_along_axis) {
  State state;
  state.r.assign(2, Vec3{});
  state.v.assign(2, Vec3{});
  state.a.assign(2, Vec3{});
  state.q.assign(2, Quat{});
  state.alpha.assign(2, Vec3{});
  // The axis is +y in the body frame, so an `omega` along +y is "spinning
  // forward" for this element and an `omega` along -y is "spinning backwards".
  state.omega.assign(2, Vec3{});
  state.omega[0] = Vec3{0.0, omega_a_along_axis, 0.0};
  state.omega[1] = Vec3{0.0, omega_b_along_axis, 0.0};
  return state;
}

/// Build the block a reader has to accept, so the two questions above are asked
/// of the same parameters.
ElementBlock make_block(double stiffness, double max_torque, int body_a,
                        int body_b) {
  ElementBlock block{};
  block.kind = ELEMENT_ROTATIONAL_TORQUE;
  block.body_a = body_a;
  block.body_b = body_b;
  block.parameters[ELEMENT_ROTATIONAL_TORQUE_STIFFNESS] = stiffness;
  block.parameters[ELEMENT_ROTATIONAL_TORQUE_DAMPING] = 0.0;
  block.parameters[ELEMENT_ROTATIONAL_TORQUE_AXIS_A] = 0.0;
  block.parameters[ELEMENT_ROTATIONAL_TORQUE_AXIS_A + 1] = 1.0;
  block.parameters[ELEMENT_ROTATIONAL_TORQUE_AXIS_A + 2] = 0.0;
  // The reference quaternion is left at zero on purpose: this law never reads
  // it, so the probe proves the reader accepts a block that omits it.
  block.parameters[ELEMENT_ROTATIONAL_TORQUE_REFERENCE_QUATERNION] = 0.0;
  block.parameters[ELEMENT_ROTATIONAL_TORQUE_MAX_TORQUE] = max_torque;
  return block;
}

void dump_reader() {
  ElementBlock block = make_block(250.0, 900.0, 0, 1);
  // The p2-08 demand slots, so the reader's own values are asserted below.
  block.ints[ELEMENT_INT_TORQUE_DEMAND_SOURCE] = TORQUE_DEMAND_BRAKE;
  block.ints[ELEMENT_INT_TORQUE_TIRE] = 2;
  std::vector<ElementCurveReference> curves(
      kElementCurveSlots);
  Model model;
  seed_bodies(model);
  std::string error;
  const bool ok = axle_kernel::read_element_blocks(
      &block, 1, curves.data(), 0, nullptr, model, error
  );
  std::printf("reader ok %d\n", ok ? 1 : 0);
  if (!ok) {
    std::printf("reader error %s\n", error.c_str());
    return;
  }
  std::printf("reader count %zu\n", model.rotational_torques.size());
  const RotationalTorque& actuator = model.rotational_torques[0];
  std::printf("reader stiffness %.17g\n", actuator.stiffness);
  std::printf("reader damping %.17g\n", actuator.damping);
  std::printf("reader max_torque %.17g\n", actuator.max_torque);
  std::printf("reader axis %.17g %.17g %.17g\n",
              actuator.axis_a.x, actuator.axis_a.y, actuator.axis_a.z);
  std::printf("reader bodies %d %d\n", actuator.a, actuator.b);
  std::printf("reader demand_source %d %d\n", actuator.demand_source,
              actuator.demand_tire);
}

/// Apply the law once and print the couple each body received.  `body_a` is
/// grounded only for the reaction-less variant; both spellings are exercised so
/// the pair behaviour and the ground behaviour are each observed.
///
/// `demand_source` / `demand_tire` / `demand_value` / `slip` are the p2-08
/// channel: a source other than the unit demand reads the sampled input, so the
/// probe builds one holding nothing but the tire column under test.  They default
/// to the unit demand, which is what the pre-p2-08 calls meant.
void dump_eval(const char* label, double stiffness, double max_torque,
               double omega_a, double omega_b, int body_b,
               int demand_source = TORQUE_DEMAND_UNIT, int demand_tire = -1,
               double demand_value = 0.0, double slip = 0.0,
               double stale_torque = 0.0) {
  Model model;
  seed_bodies(model);
  RotationalTorque actuator;
  actuator.a = 0;
  actuator.b = body_b;
  actuator.axis_a = Vec3{0.0, 1.0, 0.0};
  actuator.stiffness = stiffness;
  actuator.damping = 0.0;
  actuator.max_torque = max_torque;
  actuator.demand_source = demand_source;
  actuator.demand_tire = demand_tire;
  model.rotational_torques.push_back(actuator);

  State state = seeded_state(omega_a, omega_b);
  // One slot per tire would be the production shape; one is enough here because
  // every probe state names tire 0.
  state.tire_sx.assign(1, slip);
  std::vector<Vec3> torque(2, Vec3{});
  double external_power = 0.0;
  axle_kernel::SampleInput input;
  // The demand channel reads its *own* vectors (subtask p2-10).  The N*m
  // tables are filled with the same number on purpose: a law that still read
  // them would print a couple for the `decoupled` state below and the test
  // would catch it, which is what makes "the two unit systems are separate"
  // an assertion rather than a comment.
  if (demand_source == TORQUE_DEMAND_WHEEL) {
    input.wheel_demand.assign(1, demand_value);
    input.torque.assign(1, stale_torque);
  } else if (demand_source == TORQUE_DEMAND_BRAKE) {
    input.brake_demand.assign(1, demand_value);
    input.brake_torque.assign(1, stale_torque);
  }
  axle_kernel::assemble_rotational_torque_forces(
      model, state, input, torque, nullptr, false, false, external_power
  );
  // A fixed body's accumulator is never written, so the probe leaves `a` free
  // here and reports both ends.
  std::printf("eval %s tau_a %.17g tau_b %.17g\n",
              label, torque[0].y, torque[1].y);
}

/// The two unit systems are separate, at the sampling layer as well.
///
/// `interpolate_input` is where a case's tables become the sample a force law
/// reads, and the whole point of p2-10 is that the N*m tables and the
/// normalized demand tables stay apart there too: same input structure, two
/// destinations, no bleed either way.  This prints all four destinations for
/// one sample so the test can assert each independently.
void dump_input_isolation() {
  double times[2] = {0.0, 1.0};
  double wheel_torque[2] = {10.0, 20.0};
  double brake_torque[2] = {30.0, 40.0};
  double wheel_demand[2] = {0.25, 0.75};
  double brake_demand[2] = {0.5, 1.0};

  AxleInput in{};
  in.sample_count = 2;
  in.body_count = 0;
  in.tire_count = 1;
  in.sample_times = times;
  in.wheel_torque = wheel_torque;

  // No demand declared: the two demand vectors are zero and the N*m ones are
  // exactly the case's own values.
  {
    Model model;
    model.vehicle_brake_torque = brake_torque;
    SampleInput sample;
    axle_kernel::interpolate_input(model, in, 0.5, sample);
    std::printf("isolate none torque %.17g brake_torque %.17g wheel_demand %.17g brake_demand %.17g\n",
                sample.torque[0], sample.brake_torque[0],
                sample.wheel_demand[0], sample.brake_demand[0]);
  }
  // Demands declared: both demand vectors carry the case's fractions and the
  // N*m destinations are untouched -- the demand is not written into them.
  {
    Model model;
    model.vehicle_brake_torque = brake_torque;
    model.vehicle_wheel_demand = wheel_demand;
    model.vehicle_brake_demand = brake_demand;
    SampleInput sample;
    axle_kernel::interpolate_input(model, in, 0.5, sample);
    std::printf("isolate both torque %.17g brake_torque %.17g wheel_demand %.17g brake_demand %.17g\n",
                sample.torque[0], sample.brake_torque[0],
                sample.wheel_demand[0], sample.brake_demand[0]);
  }
}

} // namespace

int main() {
  dump_reader();
  dump_input_isolation();
  // A stationary pair: the rate is exactly zero, so no couple at all.  This is
  // the "does not accelerate a stopped pair" state.
  dump_eval("still", 400.0, 1000.0, 0.0, 0.0, 1);
  // A pair spinning forwards, demand below the cap: the couple opposes it, so
  // the body's own couple is negative along the axis.
  dump_eval("forward", 400.0, 1000.0, 0.0, 3.0, 1);
  // Reversing: the couple flips sign with the rate, which is the "correct sign
  // when the pair spins backwards" state.
  dump_eval("reverse", 400.0, 1000.0, 0.0, -3.0, 1);
  // The demand above the cap: the couple saturates at `max_torque` instead of
  // growing with the demand, which is the locked-pair behaviour.
  dump_eval("capped", 5000.0, 1000.0, 0.0, 3.0, 1);
  // And the uncapped comparison, so "capped" is shown to be the cap binding and
  // not the law ignoring its demand.
  dump_eval("uncapped", 400.0, 1000.0, 0.0, 3.0, 1);
  // A grounded reaction: `b` is -1, so only `a` receives a couple, and that
  // couple reacts against the ground rather than against a second body.
  dump_eval("grounded", 400.0, 1000.0, 3.0, 0.0, -1);
  // The p2-08 demand channel.  `driven` follows the case's own driver signal
  // instead of the block's stiffness, so the couple is the gain times the demand
  // at this sample -- the half-stiffness state below shows which of the two the
  // law read, because the unit demand would return the full 400.
  dump_eval("driven", 400.0, 1000.0, 0.0, 3.0, 1, TORQUE_DEMAND_BRAKE, 0, 0.5);
  // A driver who asks for nothing gets nothing, even with the pair spinning:
  // that is what makes the demand the magnitude rather than a constant.
  dump_eval("no_demand", 400.0, 1000.0, 0.0, 3.0, 1, TORQUE_DEMAND_BRAKE, 0, 0.0);
  // The locked wheel: the pair has stopped turning (`rate` inside `kEps`) but the
  // tire is still slipping, so the couple stays at the demanded magnitude in the
  // direction that opposes the slip instead of falling to zero.  This is the
  // state the road map's 2.1 section calls out -- a wheel held at zero angular
  // rate under a full brake demand must still be braked.
  dump_eval("locked", 400.0, 1000.0, 0.0, 0.0, 1, TORQUE_DEMAND_BRAKE, 0, 1.0, 0.8);
  // Reversing under the same demand: the sign follows the slip when the pair is
  // not turning.
  dump_eval("locked_reverse", 400.0, 1000.0, 0.0, 0.0, 1, TORQUE_DEMAND_BRAKE, 0,
            1.0, -0.8);
  // The unit-demand block is untouched by the slip branch: with no source it has
  // no tire to follow, so a stopped pair still gets exactly nothing.
  dump_eval("unit_still", 400.0, 1000.0, 0.0, 0.0, 1);
  // The unit systems are separate (p2-10): the same wheel is given a full
  // brake demand of 1.0 *and* a stale 250 N*m in the old table.  A law that
  // read the N*m buffer would apply its own 250 as the fraction (capped at
  // 1000, so the couple would be -1000); a law that reads the demand buffer
  // prints the 400 the gain asks for, and the 250 is ignored.
  dump_eval("decoupled", 400.0, 1000.0, 0.0, 3.0, 1, TORQUE_DEMAND_BRAKE, 0,
            1.0, 0.0, 250.0);
  return 0;
}
