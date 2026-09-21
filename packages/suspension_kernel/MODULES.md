# Kernel modules

`suspension_kernel` is one C++ library built from twenty-three modules.  This
file is the map the source comments point at: what each module owns, which way
the dependencies run, and where a new capability is supposed to arrive.

## The rule

1. **A module includes its own header.**  `mb_model/functions.hpp` declares the
   free functions of `mb_model`; it does not re-export another module's.  There
   are no aggregate headers left, and `check_module_layering.py` counts both
   "self-including headers" and "cross-aggregate includes" as zero.
2. **The dependency graph is a DAG.**  No module reaches back to one above it,
   and no pair of modules includes each other.  The gate reports the cycle count
   and compares the edge set against `layering_baseline.json`, so an edge can be
   removed but not quietly added.
3. **Translation units include what they call.**  A `.cpp` lists the module
   headers for the functions it actually uses rather than relying on include
   order.

```
uv run python packages/suspension_kernel/scripts/check_module_layering.py --strict
```

## The modules

| Module | Owns | Depends on |
|---|---|---|
| `mb_config` | the runtime floor: the standard-library prelude, the version constants, the env switches, the tolerance/width constants, the profiling counters.  It depends on nothing. | — |
| `mb_numeric` | vectors, matrices, quaternions, rotations, plain and monotone-cubic curve helpers, small whole-vector utilities. | `mb_config` |
| `mb_dual` | the dual-number algebra and the dual geometry (`DVec3`/`DMat3`/`DQuat`) the directional pass is written in. | `mb_config`, `mb_numeric` |
| `mb_energy` | the energy ledger types (rates and storage).  Values only, no dynamics. | — |
| `mb_model` | the assembled model: bodies, states, constraints, the element descriptors, `SampleInput`, the model accessors.  Subtask 04 added `static_rotation_gauge_for_pivot`: a pure accessor over the assembled gauge list. | `mb_config`, `mb_dual`, `mb_numeric` |
| `mb_input` | the kernel's internal input/output payload types and the element layout blocks.  No longer a cross-boundary layout: the contract reader fills them. | `mb_model` |
| `mb_linear` | factorisation and linear solves. | `mb_config`, `mb_dual`, `mb_numeric` |
| `mb_tire_common` | tire kinematics shared by the tire laws. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_model` |
| `mb_tire_state` | the per-tire carried states (relaxation, contact mass, turn slip). | `mb_config`, `mb_dual`, `mb_numeric`, `mb_model` |
| `mb_tire_fiala`, `mb_tire_pac2002`, `mb_tire_brush` | one tire force law each, including its parameter layout and mode table. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_model`, `mb_tire_common`, `mb_tire_state` |
| `mb_element` | the force elements and their constitutive laws, scalar and directional together: springs, bushings (with their curves), anti-roll bars, the steering actuator, drive/brake torques, and the force-assembly primitives (`add_force_on_body`, `add_torque_on_body`, `mat6_mul`, `bushing_deformation`) the tire laws share.  Subtask 04: this is the old `mb_suspension` plus the element half of `mb_vehicle`; the scalar and derivative implementations travel together. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_energy`, `mb_model`, `mb_joint`, the tire modules |
| `mb_force` | the external-force layer: the scalar bus `external_force_vector`, the directional bus `external_force_directional`, the applied-wrench/aerodynamic/gravity assembly, the generalized-force assembly.  Constitutive laws live in `mb_element`/`mb_tire`; this module only orchestrates them.  Subtask 04: split out of `mb_vehicle`. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_energy`, `mb_model`, `mb_joint`, `mb_element`, `mb_tire`, `mb_tire_fiala`, `mb_tire_pac2002`, `mb_tire_state` |
| `mb_tire` | the tire force assembly and the dispatcher that calls the laws above, including the tire half of the directional pass that used to live in `vehicle/kernel_directional.cpp`. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_energy`, `mb_model`, `mb_joint`, `mb_element`, the tire modules |
| `mb_joint` | constraint rows and their analytic Jacobians, the joint-type registry, and the constraint-system audit (`audit_constraint_system`, moved here from `mb_solve_static` at subtask 04: it is a property of the assembled rows, not of the solver).  Subtask 03: `constraint_rows` moved here from the model accessors, removing the model<->joint cycle. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_linear`, `mb_model`, `mb_tire_state` |
| `mb_assembly` | the vehicle-level assembly: the registration functions, the element reader, `build_model`.  It reads neutral input and builds a `Model`; it does not solve.  Subtask 04: this is the assembly half of the old `mb_vehicle`. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_model`, `mb_input`, `mb_joint`, `mb_tire`, `mb_tire_pac2002` |
| `mb_solve_dynamic` | the dynamic solve: the residual, the Newton step, the step controller, events. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_model`, `mb_joint`, `mb_tire`, `mb_force`, `mb_linear`, `mb_energy`, `mb_input`, the tire modules |
| `mb_solve_static` | the static solve: `static_trim`, the projection and least-squares helpers, contact pretrimming.  Subtask 03: no implementation dependency on `mb_solve_dynamic`'s module headers; the shared input sampling lives in `mb_input`.  Subtask 04: `audit_constraint_system` moved to `mb_joint`, `static_rotation_gauge_for_pivot` to `mb_model`. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_model`, `mb_joint`, `mb_force`, `mb_linear`, `mb_tire_common`, `mb_tire_state`, `mb_input`, `mb_solve_dynamic` |
| `mb_output` | the measurement and result writers. | `mb_config`, `mb_dual`, `mb_numeric`, `mb_model`, `mb_joint`, `mb_force`, `mb_solve_dynamic`, `mb_energy`, `mb_input`, `mb_tire_state` |
| `mb_contract` | the contract layer: canonical JSON, containers, SHA-256, and the four registries. | `mb_config` |
| `mb_cases` | the case families: how a declarative case document expands into concrete runs. | `mb_config`, `mb_numeric`, `mb_contract`, `mb_input`, `mb_model` |
| `abi` | the entry points.  `suspension_kernel_run` parses two documents, builds the model, expands the cases and drives the solver. | everything above |

`mb_vehicle` and `mb_suspension` no longer exist.  The registration, layout and
`build_model` half of `mb_vehicle` became `mb_assembly`; the external-force
buses and the generalized-force assembly became `mb_force`; the suspension
elements and the vehicle force elements became `mb_element`; the tire semantics
of `vehicle/kernel_directional.cpp` became the tire half of the directional pass
in `mb_tire`.  No forwarding shells were left behind.

`mb_cases` is deliberately not allowed to reach the solver: it produces the
`ContractCase` tables and stops.  A family that solved would make "what was
asked for" and "what was run" the same expression, and a case document could no
longer be inspected, compared or handed to a different solver.

## Extension points

Four registries in `mb_contract` are the places a new capability is meant to
arrive.  Each is queried by the contract reader, and each is exercised by the
self-test (`mb_contract_selftest`):

| Registry | Query | A new entry is |
|---|---|---|
| joint types | `contract_joint_rows(name)` | a row count for a constraint kind the model documents may name |
| element types | `contract_element_known(name)` | an element the model document may declare |
| tire models | `contract_tire_known(name)` | a force law the model document may name |
| case families | `contract_case_family_known(name)` | a family the case document may declare |

Adding a case family is a new file in `mb_cases` plus a row in that table; it
does not change the ABI, and it does not change the solver.  The three
capabilities the K/C family gained during the takeover -- a general
driven-coordinate grid, an explicit load list with side modes, and constant body
wrenches -- all arrived that way, through the case document and the model
document, with no new ABI entry point and no ABI version bump.

## What the kernel tells the outside world about itself

`suspension_kernel_contract_version` returns the contract version, and
`suspension_kernel_capabilities` returns a JSON document describing the PAC2002
scope the kernel implements and the coefficient families it refuses.  The
authoring layer reads the second one instead of carrying its own copy: a copy
drifting in either direction is silent, and both directions are wrong.

## Reading order

For the boundary: `src/abi/kernel_contract_run.cpp`, then `src/cases/`, then
`include/mb_cases/functions.hpp`.

For the physics: `include/mb_model/types.hpp` (what a model is), then
`src/model/`, then `src/assembly/` (how a model is built), then `src/element/`,
`src/force/` and `src/tire/`, then `src/solve_static/` and `src/solve_dynamic/`.
