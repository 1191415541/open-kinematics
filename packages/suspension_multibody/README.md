# suspension-multibody

Independent quasi-static suspension K&C and load solver for a symmetric front
double-wishbone suspension with rack steering.  The package has no dependency
on `suspension_kinematics`; shared geometry enters through
`suspension_contracts`.

## Run

```powershell
uv run --project packages/suspension_multibody suspension-multibody validate `
  --model model.yaml --case case.yaml
uv run --project packages/suspension_multibody suspension-multibody run `
  --model model.yaml --case case.yaml --out results
```

Each case selects exactly one mode: `K` for ideal suspension joints or `C`
for linear 6x6 compliant mounts.  K supports wheel-center and contact-point
drives; C supports six-component load paths and symmetric/opposite/single-side
load modes.  Results contain a manifest plus independent states,
component-load, bushing and diagnostic Parquet/CSV tables.

The fixed local performance gates are available through
`suspension_multibody.analysis.benchmarks` and cover 100 K states and 6600 C states.

## Adams validation

The strict K gate discovers Adams/Car 2024.1 through the environment, `PATH`, or
the Windows uninstall registry and verifies the license with a real unattended
`acar` product start. It creates temporary suspension and steering subsystem
copies with kinematic joints, generates the fixed 3x3 wheel-travel/rack grid,
and reruns every state with Adams Solver `simulate/kinematics`. The independent
`suspension_multibody` result is generated from the same normalized hardpoint input;
neither runner can read the other result.

```powershell
uv run --project packages/suspension_multibody suspension-multibody validate-adams `
  --profile adams-car-2024.1 --strict-k --require-installed `
  --evidence-dir artifacts/adams/strict-k
```

The strict C gate writes and executes a native Adams Solver model from the
same canonical hardpoints and element set: eight diagonal 6x6 inboard
bushings, ideal outboard/tie-rod joints, a neutral locked rack, a left
wheel-center six-axis wrench, and no gravity, contact, spring, damper, stop,
or stock-template compliance objects. Each physical bushing is emitted as a
reversed pair of half-rate native `BUSHING` elements so Adams' moving-J-frame
asymmetry does not alter the shared 6x6 constitutive law. It compares all 66 load states across
left/right wheel-center translation, rotation vector, toe, and camber.
The native command first solves zero load, then preconditions to the negative
endpoint before recording the 11-state negative-to-positive response sweep;
this keeps the largest moment paths on a continuous quasi-static equilibrium
branch.

    uv run --project packages/suspension_multibody suspension-multibody validate-adams `
      --profile adams-car-2024.1 --strict-c --require-installed `
      --evidence-dir artifacts/adams/strict-c

It is intentionally not a comparison against the stock TR template's complete
ride system, whose springs, dampers, stops, and additional bushings are outside
the current MBD element set. The legacy `--full` command also remains available
for regression evidence, but its built-in C fields are left/right symmetry
residuals rather than full compliance magnitudes and therefore do not
constitute strict C/load acceptance.

`--reference` and `--runner` override the built-in baseline and batch runner.
An external runner receives the request JSON and output directory as its final
two arguments (also exposed as `SUSPENSION_MULTIBODY_ADAMS_REQUEST` and
`SUSPENSION_MULTIBODY_ADAMS_OUTPUT`) and writes `adams_results.json` or CSV. Missing
groups or fields fail the gate instead of producing a profile-only pass.

## Geometry contract

`suspension_multibody.adapters.front_axle_model_from_contract` consumes Geometry
Contract V1. Mass properties remain multibody-specific input, so the contract
does not silently define compliance, force elements, tires, or solver settings.
