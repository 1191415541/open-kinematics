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
    "pad_contact_from_run",
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

#: The tire block's column holding the vertical force the contact law produced, in
#: newtons.  Named for the same reason as the penetration column: a caller should
#: read a name, not an index.
_TIRE_LOAD_COLUMN = 4


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


def pad_contact_from_run(run: Any, case_index: int = 0) -> dict[str, dict[str, float]]:
    """
    Return what a pad reading owes per side: wheel centre, tire load, contact point.

    The pad reading is the one where the ground carries the wheel, so the three
    quantities a caller asks it for are the three the geometry defines:

    * **wheel centre** -- the tire's own centre point on its body, resolved at the
      solved pose.  It is read from the tire definition rather than from a body
      origin, because the centre sits at a body-local offset
      (``center_local``) and an offset of zero would name a different point;
    * **tire load** -- the vertical force the contact law produced, in newtons;
    * **contact point** -- derived, not encoded.  The kernel reports how far the
      tire is compressed and says nothing about where the patch is, because the
      patch is a consequence of the geometry::

          delta   = penetration
          contact = center - [0, 0, radius - delta]

      so the contact sits directly below the centre, ``radius - delta`` away.
      The identity that makes this checkable is ``contact.z == pad height``: the
      pad is what the wheel rests on, so the patch is on it.

    A side the model declares no tire for is **absent**, not zero -- the same
    distinction the compression channel is built on.  A run with no tire block at
    all returns an empty mapping rather than a fabricated one.
    """
    try:
        names = run.tire_names
        block = run.block("tire_output")
    except Exception:  # noqa: BLE001 - a kernel without the block has no tires
        return {}
    if not names:
        return {}

    entry: Mapping[str, Any] = run.cases[case_index]
    first = int(entry["sample_offset"])
    last = first + int(entry["sample_count"]) - 1

    # The tire's own declaration: the body it sits on, its centre offset and its
    # unloaded radius.  Read from the model document rather than guessed, so a tire
    # mounted anywhere other than a body origin is reported where it is.
    declared: dict[str, Mapping[str, Any]] = {}
    document = getattr(run, "model_document", None) or {}
    for tire in (document or {}).get("tires", ()) or ():
        declared[str(tire["name"])] = tire

    states: dict[str, np.ndarray] = {}
    result: dict[str, dict[str, float]] = {}
    for index, name in enumerate(names):
        tire = declared.get(str(name))
        if tire is None:
            continue
        body = str(tire["body"])
        if body not in states:
            states[body] = np.asarray(run.body_state(body), dtype=float)
        row = states[body][last]
        origin = row[0:3] * MM
        center_local = np.asarray(
            tire["parameters"]["center_local"], dtype=float
        )
        radius = float(tire["parameters"]["unloaded_radius"])
        # The body rotates, so the centre's offset turns with it.
        quaternion = row[3:7]
        center = origin + _rotate(center_local, quaternion)

        delta = float(block[last, index, _TIRE_PENETRATION_COLUMN]) * MM
        load = float(block[last, index, _TIRE_LOAD_COLUMN])
        contact = np.array(
            [center[0], center[1], center[2] - (radius - delta)], dtype=float
        )
        for side, label in (("L", "left"), ("R", "right")):
            if str(name).endswith(f"_{side}"):
                result[label] = {
                    "wheel_center_x_mm": float(center[0]),
                    "wheel_center_y_mm": float(center[1]),
                    "wheel_center_z_mm": float(center[2]),
                    "tire_load_n": load,
                    "contact_x_mm": float(contact[0]),
                    "contact_y_mm": float(contact[1]),
                    "contact_z_mm": float(contact[2]),
                }
    return result


def _rotate(vector: np.ndarray, quaternion: np.ndarray) -> np.ndarray:
    """Rotate a body-local offset into the world, from a (w, x, y, z) quaternion."""
    w, x, y, z = (float(value) for value in quaternion)
    rotation = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )
    return rotation @ vector
