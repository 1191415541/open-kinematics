# suspension-mbd

Independent quasi-static suspension K&C and load solver for a symmetric front
double-wishbone suspension with rack steering.  The package has no dependency
on the repository's existing `kinematics` domain code.

## Run

```powershell
uv run --project packages/suspension_mbd suspension-mbd validate `
  --model model.yaml --case case.yaml
uv run --project packages/suspension_mbd suspension-mbd run `
  --model model.yaml --case case.yaml --out results
```

Each case selects exactly one mode: `K` for ideal suspension joints or `C`
for linear 6x6 compliant mounts.  K supports wheel-center and contact-point
drives; C supports six-component load paths and symmetric/opposite/single-side
load modes.  Results contain a manifest plus independent states,
component-load, bushing and diagnostic Parquet/CSV tables.

The fixed local performance gates are available through
`suspension_mbd.analysis.benchmarks` and cover 100 K states and 6600 C states.

## Adams validation

The strict K gate discovers Adams/Car 2024.1 through the environment, `PATH`, or
the Windows uninstall registry and verifies the license with a real unattended
`acar` product start. It creates temporary suspension and steering subsystem
copies with kinematic joints, generates the fixed 3x3 wheel-travel/rack grid,
and reruns every state with Adams Solver `simulate/kinematics`. The independent
`suspension_mbd` result is generated from the same normalized hardpoint input;
neither runner can read the other result.

```powershell
uv run --project packages/suspension_mbd suspension-mbd validate-adams `
  --profile adams-car-2024.1 --strict-k --require-installed `
  --evidence-dir .codex-tasks/20260729-suspension-mbd/raw/adams-strict-k
```

The legacy `--full` command remains available for regression evidence, but its
built-in C fields are left/right symmetry residuals rather than full compliance
magnitudes and therefore do not constitute strict C/load acceptance.

`--reference` and `--runner` override the built-in baseline and batch runner.
An external runner receives the request JSON and output directory as its final
two arguments (also exposed as `SUSPENSION_MBD_ADAMS_REQUEST` and
`SUSPENSION_MBD_ADAMS_OUTPUT`) and writes `adams_results.json` or CSV. Missing
groups or fields fail the gate instead of producing a profile-only pass.
