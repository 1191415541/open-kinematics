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

The `--full` gate requires an explicit non-proprietary numeric reference and an
external runner. The runner receives the generated request JSON and output
directory as its final two arguments (also exposed as
`SUSPENSION_MBD_ADAMS_REQUEST` and `SUSPENSION_MBD_ADAMS_OUTPUT`) and must write
`adams_results.json` or `adams_results.csv`. Results contain the three required
groups `K_geometry`, `C_compliance`, and `static_load`; missing results or fields
fail the gate instead of being treated as a profile-only pass.

```powershell
uv run --project packages/suspension_mbd suspension-mbd validate-adams `
  --profile adams-car-2024.1 --full --require-installed `
  --reference .\adams-reference.json --runner .\run-adams.bat
```
