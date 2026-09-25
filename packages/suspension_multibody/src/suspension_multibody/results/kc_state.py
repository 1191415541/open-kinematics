"""
Decode one K/C contract sample into the reporting model.

`api` used to do this inline: it read `row[:3]` and `row[3:7]` out of the native
`body_state` array, converted metres to millimetres, and read a literal column
index out of the tire block.  That is decoding, and decoding belongs here -- the
module that owns the column layout is the module that may know it.  `api` kept
the knowledge because the K/C family was written before this layer existed, and
the result was two places that had to agree about a layout neither of them owned.

Three conversions happen here and are stated once:

* **metres to millimetres.**  The contract reports body poses in metres and every
  reporting helper carries millimetres.  Doing it twice is how a factor of a
  thousand gets applied twice;
* **row slices to a state object.**  The state row is a flat array; its layout is
  the contract's, so the slices live here rather than at each caller;
* **block column to a named quantity.**  A tire's compression is column 2 of
  `tire_output`; the name is what a caller should read, not the index.

Nothing here solves, evaluates an element law, or computes a metric.  It reads
what the kernel reported.
"""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

__all__ = [
    "MM",
    "body_state_vector",
    "rigid_state_from_row",
    "tire_compression_from_run",
]

#: Millimetres per metre.  The contract is SI and the reporting model is mm.
MM = 1000.0

#: The body-state row's own layout: position, then the body-to-world quaternion,
#: then linear and angular velocity.  Named rather than sliced with literals so
#: a reader can see which half `_rigid_state` takes.
_POSITION = slice(0, 3)
_QUATERNION = slice(3, 7)

#: The tire block's column holding the penetration the contact law used, in
#: metres, already clamped at zero by the kernel.  It is the same column in the
#: static branch as in the dynamic one, which is what lets a K/C run read it.
_TIRE_PENETRATION_COLUMN = 2


def rigid_state_from_row(assembly: Any, bodies: list[str], row: Any):
    """
    Rebuild the reporting state from one contract sample, metres to millimetres.

    ``row`` is the sample's body-state slice: one row per body, in the kernel's
    own body order.  The width beyond the first seven columns -- velocities,
    accelerations and the rest -- is carried by other readers and is not this
    function's business, which is why the row is indexed by body rather than
    sliced to a fixed shape: a caller that hands over a wider contract row still
    gets the pose it asked for.

    ``bodies`` is that kernel order, which the assembly's own bodies are then
    indexed by.  A row naming a body the assembly does not carry would put a pose
    under the wrong name, so the mismatch is refused rather than skipped.
    """
    from ..modeling.primitives.joints import RigidBody, RigidBodyState
    from ..modeling.primitives.spatial import SE3

    values = np.asarray(row, dtype=float)
    if values.ndim != 2 or values.shape[0] != len(bodies) or values.shape[1] < 7:
        raise ValueError(
            f"a contract sample is {values.shape} and the kernel named "
            f"{len(bodies)} bodies; a state needs one row of at least seven "
            "values per body"
        )
    rebuilt: dict[str, RigidBody] = {}
    for index, name in enumerate(bodies):
        entry = values[index]
        source = assembly.bodies[name]
        rebuilt[name] = RigidBody(
            name=name,
            pose=SE3(
                translation=np.asarray(entry[_POSITION], dtype=float) * MM,
                quaternion=np.asarray(entry[_QUATERNION], dtype=float),
            ),
            mass=source.mass,
            inertia=source.inertia,
            center_of_mass=source.center_of_mass,
            fixed=source.fixed,
        )
    return RigidBodyState(rebuilt)


def body_state_vector(row: np.ndarray) -> np.ndarray:
    """Return the state half of one contract row, in the contract's own units."""
    return np.asarray(row, dtype=float)[:7].copy()


def tire_compression_from_run(
    run: Any, case_index: int = 0
) -> dict[str, float]:
    """
    Return the vertical tire compression the solve produced, by side.

    A side the kernel did not report is *absent* rather than zero: a run whose
    model declares no tire has no compression, and reporting `0.0` would claim a
    measured value that does not exist -- the same distinction the rig's absent
    rack axis is built on.
    """
    try:
        names = run.tire_names
        block = run.block("tire_output")
    except Exception:  # noqa: BLE001 - a kernel without the block has no tires
        return {}
    if not names:
        return {}
    entry: Mapping[str, Any] = run.cases[case_index]
    offset = int(entry["sample_count"]) - 1
    first = int(entry["sample_offset"])
    by_side: dict[str, float] = {}
    for index, name in enumerate(names):
        compression = float(block[first + offset, index, _TIRE_PENETRATION_COLUMN])
        for side, label in (("L", "left"), ("R", "right")):
            if str(name).endswith(f"_{side}"):
                by_side[label] = compression
    return by_side
