"""
One real native K solve, run with no repository path on ``sys.path``.

This file is executed by ``scripts/check_composable_release.py`` from a directory
*outside* the repository, by the interpreter of a scratch virtual environment
that has the three released wheels installed and nothing else.  It is the
isolation half of the release check: an editable install would let a missing
package or a missing native library pass, and a run inside the tree would let a
missing data file pass.

What it asserts, and why each is worth asserting:

* the three packages import **without** the source tree on the path, so the
  wheels really carry what they need;
* one K state converges through the native kernel, so the shipped
  ``native/suspension_kernel.dll`` resolves from inside ``site-packages`` and the
  ABI gate in the loader accepts it;
* the solved state is finite, so "converged" is not a flag with no numbers behind
  it.

It prints one JSON object on stdout and nothing else, because the probe reads the
numbers rather than parsing prose.  It does not touch a fixture file: the model is
built in memory, so nothing here can depend on the repository's data directory.
"""

from __future__ import annotations

import json
import sys

from suspension_multibody.api import run_case
from suspension_multibody.kernel.native import native_build_metadata
from suspension_multibody.schema import (
    CaseSpec,
    DisplacementControl,
    FrontAxleModel,
    MassSpec,
)

#: The hardpoints of the package's own public K example, in millimetres.  Named
#: here rather than read from a fixture: this script has to run where the
#: repository is not.
HARDPOINTS: dict[str, list[float]] = {
    "uca_front": [-100.0, -500.0, 400.0],
    "uca_rear": [100.0, -500.0, 400.0],
    "uca_outer": [0.0, -700.0, 450.0],
    "lca_front": [-120.0, -500.0, 150.0],
    "lca_rear": [120.0, -500.0, 150.0],
    "lca_outer": [0.0, -700.0, 150.0],
    "tierod_inner": [100.0, -400.0, 250.0],
    "tierod_outer": [50.0, -700.0, 250.0],
    "wheel_center": [0.0, -700.0, 300.0],
    "rack_center": [0.0, 0.0, 250.0],
}


def main() -> int:
    """Run one native K state and print what happened, as JSON."""
    model = FrontAxleModel(
        hardpoints=HARDPOINTS, mass=MassSpec(sprung_mass=1000.0)
    )
    case = CaseSpec(
        mode="K",
        controls=(
            DisplacementControl(target="wheel_travel_left", values=(0.0, 20.0)),
        ),
    )
    bundle = run_case(model, case)
    states = list(bundle.states)
    payload = {
        # A repository path here would mean the isolation was not real.
        "repo_on_path": [entry for entry in sys.path if "open-kinematics" in entry],
        "state_count": len(states),
        "converged": all(state.converged for state in states),
        "residual": max(
            max(state.constraint_residual, state.force_residual) for state in states
        ),
        "finite": all(
            abs(float(state.metrics["left_wheel_center_z"])) < 1e6 for state in states
        ),
        "native": {
            key: native_build_metadata().get(key)
            for key in ("abi_version", "vehicle_abi_version")
        },
    }
    print(json.dumps(payload))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
