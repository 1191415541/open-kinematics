# Kernel modules

`suspension_kernel` is one C++ library built from twenty modules.  This file is
the map the source comments point at: what each module owns, which way the
dependencies run, and where a new capability is supposed to arrive.

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
| `mb_base` | vectors, quaternions, rotations, small utilities, dual numbers, the version constants.  The floor: it depends on nothing. | — |
| `mb_energy` | the energy ledger types (rates and storage).  Values only, no dynamics. | — |
| `mb_model` | the assembled model: bodies, states, constraints, the element descriptors, `SampleInput`. | `mb_base` |
| `mb_input` | the kernel's internal input/output payload types and the element layout blocks.  No longer a cross-boundary layout: the contract reader fills them. | `mb_model` |
| `mb_linalg` | factorisation and linear solves. | `mb_base` |
| `mb_tire_common` | tire kinematics shared by the tire laws. | `mb_base`, `mb_model` |
| `mb_tire_state` | the per-tire carried states (relaxation, contact mass, turn slip). | `mb_base`, `mb_model` |
| `mb_tire_fiala`, `mb_tire_pac2002`, `mb_tire_brush` | one tire force law each, including its parameter layout and mode table. | `mb_base`, `mb_model`, `mb_tire_common`, `mb_tire_state` |
| `mb_suspension` | the suspension force elements: springs, dampers, bushings, anti-roll bars, stops. | `mb_base`, `mb_energy`, `mb_model` |
| `mb_tire` | the tire force assembly and the dispatcher that calls the laws above. | the tire modules, `mb_model`, `mb_energy` |
| `mb_constraint` | constraint rows and their analytic Jacobians, plus the joint-type registry. | `mb_base`, `mb_linalg`, `mb_model` |
| `mb_vehicle` | the vehicle-level assembly: layout, external loads, gravity, aerodynamics, steering, drive/brake. | `mb_model`, `mb_tire`, `mb_constraint`, `mb_energy`, `mb_input` |
| `mb_integrator` | the dynamic solve: the residual, the Newton step, the step controller, events. | `mb_model`, `mb_constraint`, `mb_tire`, `mb_linalg`, `mb_energy`, `mb_input` |
| `mb_static` | the static solve: `static_trim`, the projection and least-squares helpers, contact pretinning. | `mb_model`, `mb_constraint`, `mb_integrator`, `mb_linalg`, `mb_tire_common`, `mb_tire_state`, `mb_input` |
| `mb_output` | the measurement and result writers. | `mb_model`, `mb_constraint`, `mb_integrator`, `mb_energy`, `mb_input`, `mb_tire_state` |
| `mb_contract` | the contract layer: canonical JSON, containers, SHA-256, and the four registries. | `mb_base` |
| `mb_cases` | the case families: how a declarative case document expands into concrete runs. | `mb_base`, `mb_contract`, `mb_input` |
| `abi` | the entry points.  `suspension_kernel_run` parses two documents, builds the model, expands the cases and drives the solver. | everything above |

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
`src/model/`, then `src/suspension/` and `src/tire/`, then `src/static/` and
`src/integrator/`.
