# Force-element interface freeze (C++ is the source of truth)

This file is the frozen interface for the three force-element structures. The
C++ kernel defines them; Python mirrors them field for field. Both sides change
together, and this file is what they are checked against.

Agreed with the user on 2026-09-26:

* **option 乙-1** — the kernel's single `Spring` splits into three structures
  (`Spring`, `Damper`, `BumpStop`), and the four damper terms Python already
  carries and the kernel does not (`gas_stiffness`, `gas_reference_force`,
  `friction`, `extension_sign`) are **added to the kernel's `Damper`**, so no
  physics is dropped.
* **ABI 15 → 16 is authorised**, together with re-collecting
  `tasks/01-baseline/BASELINE.json`. Only the descriptive fingerprints may move
  (`abi_version`, library hash, kernel source fingerprints); every entry under
  `frozen_baselines` must stay byte-identical.
* The anti-roll bar's Python runtime class is realigned to the kernel's
  torsional law (`axis_a` + reference quaternion), while the user-facing model
  keeps its six hardpoints and the assembly derives the axis and reference.

## Why the split is a net simplification

The kernel's `Spring` currently carries three different physical laws in one
record: an elastic law (`k`, `free_length`, elastic curve), a dissipative law
(`c_compression`/`c_rebound`, damper curve), and two unilateral stops
(`minimum_length`/`maximum_length` plus their curves). Python already models
them as three classes. Splitting the kernel record makes the two sides name the
same three things, which is the point.

## The frozen structures (C++)

```cpp
struct Spring {
    int a{-1}, b{-1};
    Vec3 pa{}, pb{};
    double k{0.0};
    double free_length{0.0};
    // Constant offset force, `k*(L - free_length) + preload`.
    double preload{0.0};
    // Optional measured elastic curve, on compression (`free_length - L`).
    std::vector<double> deflection;
    std::vector<double> force;
};

struct Damper {
    int a{-1}, b{-1};
    Vec3 pa{}, pb{};
    double c_compression{0.0}, c_rebound{0.0};
    // Gas-spring terms: the force is
    //   gas_reference_force + gas_stiffness*(L - gas_reference_length)
    // with `gas_reference_length` NaN meaning "no gas term".
    double gas_stiffness{0.0};
    double gas_reference_length{std::numeric_limits<double>::quiet_NaN()};
    double gas_reference_force{0.0};
    double preload{0.0};
    double friction{0.0};
    double extension_sign{1.0};
    // Optional measured force-velocity curve, strictly increasing in velocity.
    std::vector<double> velocity;
    std::vector<double> force;
};

struct BumpStop {
    int a{-1}, b{-1};
    Vec3 pa{}, pb{};
    // Unilateral: active only while `L < clearance` (direction +1) or
    // `L > clearance` (direction -1).
    double clearance{std::numeric_limits<double>::quiet_NaN()};
    double stiffness{0.0};
    double direction{1.0};
    double damping{0.0};
    // Optional measured curve, on penetration.
    std::vector<double> penetration;
    std::vector<double> force;
};
```

`Model` holds `std::vector<Spring> springs; std::vector<Damper> dampers;
std::vector<BumpStop> bump_stops;`.

## The frozen contract element types

The contract document's `type` field and the parameter names, one type per
structure. These are what the Python emitters write and the kernel reader reads.

| `type` | required | optional |
|---|---|---|
| `spring` | `body_a`, `body_b` | `point_a`, `point_b`, `stiffness`, `free_length`, `preload`, `elastic_curve` |
| `damper` | `body_a`, `body_b` | `point_a`, `point_b`, `compression_damping`, `rebound_damping`, `gas_stiffness`, `gas_reference_length`, `gas_reference_force`, `preload`, `friction`, `extension_sign`, `damper_curve` |
| `bump_stop` | `body_a`, `body_b` | `point_a`, `point_b`, `clearance`, `stiffness`, `direction`, `damping`, `stop_curve` |

Curves are arrays of `[x, y]` pairs, in the document's own units on x.

`contract_element_known` in `contract_registry.cpp` must accept exactly these
three names **plus** the ones already there (`bushing`, `anti_roll_bar`,
`aerodynamic_drag`, `steering_actuator`, `wheel_torque`, `point_wrench`,
`gravity`), and must no longer accept `spring_damper`.

## The frozen ABI field groups

`AxleInput`/`VehicleInput` carry one array group per structure. Each field below
is a pointer array of one value per element, in this order.

```
spring_count, spring_body_a, spring_body_b, spring_point_a, spring_point_b,
spring_stiffness, spring_free_length, spring_preload,
spring_curve_offset, spring_curve_count, spring_curve_deflection, spring_curve_force

damper_count, damper_body_a, damper_body_b, damper_point_a, damper_point_b,
damper_compression_damping, damper_rebound_damping,
damper_gas_stiffness, damper_gas_reference_length, damper_gas_reference_force,
damper_preload, damper_friction, damper_extension_sign,
damper_curve_offset, damper_curve_count, damper_curve_velocity, damper_curve_force

bump_stop_count, bump_stop_body_a, bump_stop_body_b, bump_stop_point_a, bump_stop_point_b,
bump_stop_clearance, bump_stop_stiffness, bump_stop_direction, bump_stop_damping,
bump_stop_curve_offset, bump_stop_curve_count, bump_stop_penetration, bump_stop_force
```

The `*_curve_offset`/`*_curve_count`/value arrays keep the existing
concatenation convention: `offset[i]` is where element `i`'s points start and
`count[i]` how many there are; a count of zero means the constant coefficients
are used.

## ABI version

`kAxleKernelAbiVersion` goes **15 → 16** in `mb_config/version.hpp`, with
`kAxleInputFieldAbiVersion` in `kernel_abi.cpp` following it (the `static_assert`
between them is what keeps the two honest).
`packages/suspension_multibody/src/suspension_multibody/kernel/native.py` mirrors
it as `_NATIVE_KERNEL_ABI_VERSION = 16`.
`kVehicleKernelAbiVersion` and `kCoreKernelAbiVersion` do not move.

## What must not change

* `frozen_baselines` in `BASELINE.json`.
* `suspension_kinematics` — untouched.
* The **physics**. Each of the three structures must compute exactly the terms the
  fused `Spring` computed, with the same coefficients and the same activation
  conditions. A coefficient that ends up in the wrong structure is a physics
  change, not a rename.

## What is *expected* to change (user decision, 2026-09-26)

The earlier draft of this file demanded a byte-identical `dynamic_hash`. That is
not achievable, because the split reaches three things the hash covers, and the
user has accepted the movement on all three. Re-recording
`dynamic_hash_baseline.json` is therefore the expected outcome, not a finding of
infidelity; the report must still name what moved and why.

1. **The SI record is split too.** The user-facing `AxleSpringDamper` becomes
   three records (`AxleSpring`, `AxleDamper`, `AxleBumpStop`). It modelled a
   combined part, but the kernel's structures are the ones that must be mirrored,
   so the Python schema mirrors them field for field. The model document changes,
   so `model_sha256` changes.
2. **The output blocks are split too.** `spring_output` keeps the elastic
   columns only (length, rate, elastic force, elastic+total scalar), and two new
   blocks carry the dissipative and the unilateral results. The fused 7-column
   row was the fused structure's own output; keeping it would mean rebuilding
   the fused record inside the output layer. The three blocks are
   `spring_output` (elastic), `damper_output` (dissipative) and
   `bump_stop_output` (unilateral).
3. **The element-wrench channel gains two codes**, as frozen above.

The floating-point summation-order difference measured on 2026-09-25
(`e*(a+b)` vs `e*a + e*b`, worst 4.547e-13 N over 158590/200000 samples) is a
separate, smaller movement on top of these three. It must not be used to excuse
a change in any physical quantity.

## What Python actually calls

`packages/suspension_multibody` does not construct `AxleInput`. It sends the
**contract document** to `suspension_kernel_run` (`kernel/__init__.py:153`,
`kernel/native.py:140`), and the kernel reads it in the `elements` array of
`ContractModel::read` (`src/cases/contract_model.cpp:526-668`). So the JSON
branch is the surface the Python kernel path depends on, and it is the one that
must accept `spring` / `damper` / `bump_stop`.

The `AxleInput` field groups are still frozen above and still must be split:
`src/assembly/build_model.cpp:206-263` and the generic element-block path
(`src/assembly/element_reader.cpp`, reached from `src/abi/kernel_core.cpp:202`)
read them, `sizeof(AxleInput)` is checked at `src/abi/kernel_abi.cpp:34`, and
leaving a fused group in place while the model layer moves on is exactly the
kind of half-done split this epic is against.

## The `element_wrench` observation channel

This channel records, per sample, which law applied what wrench. It sizes its
rows from the **model's own counts**, so splitting one structure into three
changes the row arithmetic. The spec has to say how, or the two sides will
disagree about the block's shape.

Today (`mb_config/element_wrench.hpp`): seven frozen type codes
(`Spring=1`, `Bushing=2`, `AntiRoll=3`, `Steering=4`, `DriveBrake=5`,
`Tire=6`, `External=7`), and `element_wrench_rows_per_element` returns 2 for
spring/bushing/anti-roll/steering, 4 for drive-brake, 1 otherwise. The counts
come from `ElementWrenchCounts`, filled at
`abi/kernel_contract_run.cpp:580-581` (from the contract) and
`output/kernel_output.cpp:388-389` (from `model.springs.size()`).

**Frozen decision:** the two *new* structures get **two new type codes**, appended
so no existing code moves (`Spring` keeps code 1 and now counts only the elastic
structure):

```
kElementWrenchDamper = 8,
kElementWrenchBumpStop = 9,
```

`kElementWrenchSpring = 1` keeps its meaning and now counts only the elastic
structure. `ElementWrenchCounts` gains `dampers` and `bump_stops`, and
`element_wrench_rows_per_element` returns 2 for both (they apply a wrench on
each end, like a spring). The channel stays **off by default**, so no result
document changes shape unless the environment switch asks for it.

This is the one place where the split is *not* purely internal: the channel's
row layout is read by `results/element_wrench.py`. Because the channel is off
in production, the Python decoder needs the two new codes added but no
production number moves.

## The three structures' evaluation order

The kernel evaluates all springs, then all bushings, then all anti-roll bars,
then steering, then drive-brake; the split must keep the **spring and damper
structures in one pass, in the order (spring, damper) per element**, so the
force accumulates in the same sequence it does today apart from the one
authorised 1e-13 rounding difference. Do not reorder the pass.

## The anti-roll bar: six hardpoints to `axis_a` + reference

The user-facing `AntiRollBar` keeps six hardpoints. The kernel's `AntiRollBar`
takes `axis_a` and a reference quaternion. Something has to bridge them, and
there is **no existing precedent**: the C++-shaped `AxleAntiRollBar` is only
ever built by hand in tests, and the user's six-hardpoint spec has always been
mapped to the vertical-travel law that is being retired. So the derivation is
frozen here rather than left to each side to invent.

**Frozen derivation.** The bar is a torsion section clamped at
`left_body_mount` and `right_body_mount`, with its arms reaching
`left_arm_end`/`right_arm_end` and its drop links at
`left_link_point`/`right_link_point`.

* the two bodies are the ones the drop links reach, exactly as today's element
  row already has them: `body_a` is the left link's body, `body_b` the right
  (`subsystems/suspension.py:285-297` builds the row that way);
* `axis_a` is the bar's own axis, `normalize(right_body_mount - left_body_mount)`,
  expressed in `body_a`'s frame — that is the axis the bar twists about, and it
  is what the two body mounts are for;
* `reference_quaternion` is the relative rotation of `body_b` to `body_a` at
  the assembling pose, so the angle the kernel measures is the deviation from
  the pose the model was authored in;
* `stiffness` is `torsional_stiffness`, a moment per radian;
* `damping` is 0: the user-facing spec declares no bar damping.

**Refuse rather than guess.** If the two body mounts coincide (a zero-length
axis), or the mount points are absent for the assembly at hand, the assembly
must raise naming the bar — not pick an axis, and not fall back to a vertical
law. A silently different physical quantity is worse than a refusal; that is
the same rule the dynamic bridge already follows when it refuses an element it
cannot read (`studies/bridge.py:400`).

**This is a real physics change.** The retired law measured the vertical
difference of the two link points (`right_z - left_z` minus a reference); the
kernel's measures a rotation about the bar axis. For a symmetric bar in roll
they are related, but they are not the same function, so a model that declares
an anti-roll bar will not produce the same component load it used to. State
this in the report; do not describe it as a pure rename.

## The frozen Python records (mirror, field for field)

`packages/suspension_multibody/src/suspension_multibody/axle_dynamics/schema.py`
replaces `AxleSpringDamper` (currently `:675-735`) with three records. The
assembly forms are `body_a`/`body_b`/`point_a_m`/`point_b_m`; the SI coefficient
units are the suffixed names, as the current record already uses them.

```python
class AxleSpring(StrictModel):
    """Elastic axial element: LinearSpring."""
    name: str
    body_a: str
    body_b: str
    point_a_m: Vec3Tuple
    point_b_m: Vec3Tuple
    stiffness_n_per_m: float = Field(ge=0)
    free_length_m: float = Field(ge=0)
    preload_n: float = 0.0
    elastic_curve_deflection_m: tuple[float, ...] = ()   # compression positive
    elastic_curve_force_n: tuple[float, ...] = ()

class AxleDamper(StrictModel):
    """Dissipative axial element."""
    name: str
    body_a: str
    body_b: str
    point_a_m: Vec3Tuple
    point_b_m: Vec3Tuple
    compression_damping_n_s_per_m: float = Field(default=0.0, ge=0)
    rebound_damping_n_s_per_m: float = Field(default=0.0, ge=0)
    gas_stiffness_n_per_m: float = Field(default=0.0, ge=0)
    gas_reference_length_m: float | None = None
    gas_reference_force_n: float = 0.0
    preload_n: float = 0.0
    friction_n: float = Field(default=0.0, ge=0)
    extension_sign: float = 1.0
    damper_curve_velocity_m_per_s: tuple[float, ...] = ()
    damper_curve_force_n: tuple[float, ...] = ()

class AxleBumpStop(StrictModel):
    """Unilateral axial stop."""
    name: str
    body_a: str
    body_b: str
    point_a_m: Vec3Tuple
    point_b_m: Vec3Tuple
    clearance_m: float = Field(ge=0)
    stiffness_n_per_m: float = Field(default=0.0, ge=0)
    direction: float = 1.0                 # +1 active below clearance, -1 above
    damping_n_s_per_m: float = Field(default=0.0, ge=0)
    stop_curve_penetration_m: tuple[float, ...] = ()
    stop_curve_force_n: tuple[float, ...] = ()
```

The validators the fused record carried move with their fields: the damper curve
still needs two points and strictly increasing velocity; the elastic and stop
curves still need either zero points or at least two, and finite values. The
fused record's `_valid_damper_curve` and `_valid_force_curve_abscissa`
(`:707-737`) are the source for both.

The gas terms are the kernel's `Damper` fields the Python side does not yet
carry, and `friction`/`extension_sign` are Python's terms the kernel does not
yet carry. Both directions are filled in, so no term is dropped from either
side; `gas_reference_length_m = None` is the kernel's NaN.

## The three output blocks

Widths are frozen here because both sides derive the block shape from them
(`mb_config/constants.hpp:25` today, and `contract_run.py:244-246` on the Python
side):

| block | width | columns |
|---|---|---|
| `spring_output` | 4 | length, rate, elastic force, preload |
| `damper_output` | 4 | length, rate, damping force, dissipation rate |
| `bump_stop_output` | 5 | length, rate, penetration, stop force, active flag |

Each block reports **its own structure's** terms, because after the split that is
all it can compute: the damper needs a velocity and the stop a penetration, and
neither is available to the elastic law. Column by column:

* `spring_output` — `L`, `dL`, the elastic force (`k*compression`, or the curve
  value), and the record's `preload`. The first three are today's columns 0, 1 and
  2 unchanged. Adding the last two gives the elastic scalar, which is exactly
  what today's column 6 held before the damper and stop terms were added.
* `damper_output` — `L`, `dL`, the damping force (today's column 3), and the
  power it removes (`damping_force * dL`). The gas, preload and friction terms
  are part of this structure's force and are included in column 2.
* `bump_stop_output` — `L`, `dL`, the penetration (0 when inactive), the stop
  force (today's columns 4 and 5, the one that is active), and a flag that is 1
  while the stop is engaged. A record carries one direction, so at most one of
  the two is ever non-zero; the flag is what distinguishes "not engaged" from
  "engaged with zero stiffness".

Nothing that was computed today is dropped, and no term is computed twice. The
three blocks together carry every number today's single 7-column row carried.
`spring_output` keeps today's `spring_names` order and count, so a reader that
only wants the elastic channel still indexes it the same way.

`AxleDynamicsResult` gains `damper_names`, `damper_output`, `bump_stop_names` and
`bump_stop_output` beside the existing `spring_names`/`spring_output`, and
`io/artifacts.py` writes all six arrays plus their `layouts` entries. The
`AxleRunLedger` in `contract_run.py` widens accordingly, reading the block width
from the kernel's block descriptor rather than hard-coding it where the
descriptor is available.

## Evaluation order

Per element the order is spring, then damper, then bump stop, and the force is
accumulated in that order so the sum matches today's
(`elastic + damping + compression_stop + rebound_stop`) term for term. The
structures stay in the one pass `assemble_spring_forces` already runs; do not
split it into three passes over the body list.
