"""
Versioned centre and instantaneous-axis definitions.

The centres here are *definitions*, and each carries its version because a
centre is not a physical point waiting to be discovered -- it depends on a
declared load distribution.  Two definitions of "the roll centre" that disagree
are not a bug as long as each says which one it is, which is why
``roll_center_legacy_v1`` is kept beside the new force-equivalent definition
rather than being replaced by it.

The force-equivalent definitions follow the design exactly:

* ``roll_force_equivalent_v2`` declares a lateral weight ``w_i`` per contact
  point, forbids longitudinal force and pure couples, and writes the virtual
  force ``F_i = (0, w_i, f_zi)``.  The ``f_zi`` are fixed by requiring the
  virtual work to vanish on **every** allowed contact motion ``V_i``, not merely
  on the one column that happens to be convenient.
* ``pitch_force_equivalent_v1`` is the same construction with a longitudinal
  weight and ``F_i = (w_i, 0, f_zi)``.

Two consequences of "every allowed motion" are worth stating because they are
easy to get wrong: the check covers the whole admissible basis (a centre that is
only verified on one column is not verified at all), and an inconsistent set of
motions is *reported* rather than fitted by least squares.  Scaling all the
weights together does not move a centre; changing how the load is shared between
points can.

``solve_rigid_motion`` in ``vehicle/screw_kinematics.py`` remains the migration
source for R5, but nothing here calls it -- these functions take the allowed
motion basis as data, so the centre arithmetic is independent of how the motion
was produced.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from .values import RESULTANT_FLOOR, CharacteristicsError

#: The centre definition versions this module implements.  Each is separately
#: nameable so a caller can ask for the legacy numbers explicitly.
ROLL_CENTER_LEGACY_VERSION = "roll_center_legacy_v1"
ROLL_CENTER_FORCE_VERSION = "roll_force_equivalent_v2"
PITCH_CENTER_FORCE_VERSION = "pitch_force_equivalent_v1"

#: How far the virtual work may miss zero before the motion set is called
#: inconsistent with the declared load distribution.
WORK_TOLERANCE = 1e-9

#: The lateral force the historical ``roll_center_legacy_v1`` report divides the
#: roll moment by.  It is a fixed convention, not a measured quantity, so it is
#: named once here rather than written as a literal in the report and the tests.
LEGACY_LATERAL_FORCE = -2.0


@dataclass(frozen=True)
class ForceEquivalentLine:
    """
    An equivalent line of action, its load distribution and its resultants.

    ``weights`` and ``contact_points`` are returned so a consumer can see the
    declared distribution the centre came from: a reported centre without the
    weights that produced it cannot be compared against another run.  ``force``
    and ``moment`` are the full resultants about the world origin, computed from
    the *complete* contact-point positions, so a contact patch that is not at
    ``z = 0`` is handled rather than assumed away.
    """

    kind: str
    version: str
    height: float
    section_coordinate: float
    section_axis: str
    weights: np.ndarray
    contact_points: np.ndarray
    vertical_forces: np.ndarray
    force: np.ndarray
    moment: np.ndarray
    line_point: np.ndarray
    line_direction: np.ndarray
    residual_couple: np.ndarray

    def __post_init__(self) -> None:
        for name in ("weights", "contact_points", "vertical_forces", "force", "moment"):
            object.__setattr__(self, name, np.array(getattr(self, name), dtype=float))


def _as_points(points: Any, label: str) -> np.ndarray:
    array = np.asarray(points, dtype=float)
    if array.ndim != 2 or array.shape[1] != 3:
        raise CharacteristicsError(f"{label} must have shape (points, 3)")
    if not np.isfinite(array).all():
        raise CharacteristicsError(f"{label} contains non-finite values")
    return array


def _as_columns(motions: Any, count: int, label: str) -> np.ndarray:
    """Validate the allowed contact motions as ``(points, 3, directions)``."""
    array = np.asarray(motions, dtype=float)
    if array.ndim != 3 or array.shape[0] != count or array.shape[1] != 3:
        raise CharacteristicsError(
            f"{label} must have shape ({count}, 3, directions); the contact motion of "
            "each declared point is a three-vector per admissible direction"
        )
    if not np.isfinite(array).all():
        raise CharacteristicsError(f"{label} contains non-finite values")
    if array.shape[2] == 0:
        raise CharacteristicsError(
            f"{label} has no admissible directions; a centre cannot be checked "
            "against an empty motion set"
        )
    return array


def _weights(weights: Any, count: int) -> np.ndarray:
    array = np.asarray(weights, dtype=float).ravel()
    if array.shape[0] != count:
        raise CharacteristicsError(f"{array.shape[0]} weights for {count} contact points")
    if not np.isfinite(array).all():
        raise CharacteristicsError("load weights must be finite")
    if float(np.linalg.norm(array)) <= RESULTANT_FLOOR:
        raise CharacteristicsError("a centre needs a nonzero load distribution")
    return array


def _solve_vertical_forces(
    motions: np.ndarray,
    weights: np.ndarray,
    *,
    longitudinal: bool,
) -> np.ndarray:
    """
    Fix each point's vertical force so the virtual work vanishes on every motion.

    For a lateral distribution the virtual force at point ``i`` is
    ``F_i = (0, w_i, f_zi)``, so the work along admissible direction ``k`` is
    ``w_i * v_y(i,k) + f_zi * v_z(i,k)``.  The forces are per *point* and the
    directions are the equations they must satisfy, which gives point ``i`` the
    system ``v_z(i,:) * f_zi = -w_i * v_y(i,:)``.

    Solving it as one stacked system over all points would be wrong: it would let
    a large lateral motion at one wheel be cancelled by a vertical force at
    another, which is not a load path the mechanism has.  Each point carries its
    own distribution and must satisfy every direction on its own.

    A point with no vertical motion cannot carry a vertical force, so a direction
    that requires one is reported as inconsistent rather than fitted.
    """
    # A roll centre balances a *lateral* distribution, so its drive axis is the
    # y component; a pitch centre uses the longitudinal x component.  Getting
    # this backwards is silent: the solve still succeeds and returns a plausible
    # height, because an all-zero drive column simply yields zero vertical
    # forces rather than an error.
    drive = 0 if longitudinal else 1
    horizontal = motions[:, drive, :]  # (points, directions)
    vertical = motions[:, 2, :]  # (points, directions)
    count = motions.shape[0]
    forces = np.zeros(count)
    for index in range(count):
        row = vertical[index]
        target = -weights[index] * horizontal[index]
        norm = float(np.linalg.norm(row))
        if norm <= RESULTANT_FLOOR:
            # The point does not move vertically in any admissible direction.
            if float(np.abs(target).max(initial=0.0)) > WORK_TOLERANCE:
                raise CharacteristicsError(
                    f"contact point {index} has no admissible vertical motion but the "
                    "declared weight requires a vertical force to make the virtual work "
                    "vanish; the load distribution is inconsistent with the boundary"
                )
            continue
        solution, *_ = np.linalg.lstsq(row.reshape(-1, 1), target, rcond=None)
        residual = row * float(solution[0]) - target
        if float(np.abs(residual).max(initial=0.0)) > WORK_TOLERANCE * max(
            1.0, float(np.abs(target).max(initial=0.0))
        ):
            raise CharacteristicsError(
                f"the declared load weight at contact point {index} is inconsistent with "
                "its admissible vertical motions: the virtual work does not vanish "
                f"(residual {float(np.abs(residual).max()):.3e})"
            )
        forces[index] = float(solution[0])
    return forces


def force_equivalent_roll_center(
    contact_points: Any,
    contact_motions: Any,
    weights: Any,
    *,
    section_coordinate: float,
) -> ForceEquivalentLine:
    """
    Return the geometric roll centre as a force-equivalent line in the YZ section.

    ``contact_motions`` is ``(points, 3, directions)``: the admissible velocity of
    each point, expressed as three-vectors, one column per allowed suspension
    motion.  Every column constrains the distribution; the centre is reported at
    the declared ``section_coordinate`` (the ``y`` of the YZ section) and the
    height is ``z = (y_ref * F_z - M_x) / F_y``.  A zero total lateral force
    leaves the line undefined, which is reported rather than returned as a huge
    number.
    """
    points = _as_points(contact_points, "contact points")
    weights = _weights(weights, points.shape[0])
    motions = _as_columns(contact_motions, points.shape[0], "contact motions")
    fz = _solve_vertical_forces(motions, weights, longitudinal=False)
    forces = np.column_stack((np.zeros_like(weights), weights, fz))
    force = forces.sum(axis=0)
    moment = np.cross(points, forces).sum(axis=0)
    lateral = force[1]
    if abs(lateral) <= RESULTANT_FLOOR:
        raise CharacteristicsError(
            "the total lateral force is zero, so the roll centre has no finite height"
        )
    height = (section_coordinate * force[2] - moment[0]) / lateral
    return _line(
        kind="roll_center",
        version=ROLL_CENTER_FORCE_VERSION,
        height=height,
        section_coordinate=section_coordinate,
        section_axis="y",
        weights=weights,
        points=points,
        fz=fz,
        force=force,
        moment=moment,
    )


def force_equivalent_pitch_center(
    contact_points: Any,
    contact_motions: Any,
    weights: Any,
    *,
    section_coordinate: float,
) -> ForceEquivalentLine:
    """
    Return the geometric pitch centre in the XZ section.

    The same construction as the roll centre with a longitudinal distribution;
    the height is ``z = (M_y + x_ref * F_z) / F_x`` at the declared
    ``section_coordinate`` (the ``x`` of the XZ section).  This is deliberately a
    separate definition from the single-wheel side-view instant centre: they are
    different quantities and must not be substituted for one another.
    """
    points = _as_points(contact_points, "contact points")
    weights = _weights(weights, points.shape[0])
    motions = _as_columns(contact_motions, points.shape[0], "contact motions")
    fz = _solve_vertical_forces(motions, weights, longitudinal=True)
    forces = np.column_stack((weights, np.zeros_like(weights), fz))
    force = forces.sum(axis=0)
    moment = np.cross(points, forces).sum(axis=0)
    longitudinal = force[0]
    if abs(longitudinal) <= RESULTANT_FLOOR:
        raise CharacteristicsError(
            "the total longitudinal force is zero, so the pitch centre has no finite height"
        )
    height = (moment[1] + section_coordinate * force[2]) / longitudinal
    return _line(
        kind="pitch_center",
        version=PITCH_CENTER_FORCE_VERSION,
        height=height,
        section_coordinate=section_coordinate,
        section_axis="x",
        weights=weights,
        points=points,
        fz=fz,
        force=force,
        moment=moment,
    )


def _line(
    *,
    kind: str,
    version: str,
    height: float,
    section_coordinate: float,
    section_axis: str,
    weights: np.ndarray,
    points: np.ndarray,
    fz: np.ndarray,
    force: np.ndarray,
    moment: np.ndarray,
) -> ForceEquivalentLine:
    """
    Package the resultants into a line of action, refusing to hide a free couple.

    The line through ``r0 = (F x M)/|F|^2`` parallel to ``F`` represents the
    resultant only if no couple remains perpendicular to it.  Where one does, it
    is reported rather than discarded: dropping it and announcing a unique pure
    force centre would be a claim the numbers do not support.
    """
    magnitude = float(np.linalg.norm(force))
    if magnitude <= RESULTANT_FLOOR:
        raise CharacteristicsError("the equivalent force is zero; there is no line of action")
    direction = force / magnitude
    line_point = np.cross(force, moment) / (magnitude * magnitude)
    residual = moment - np.cross(line_point, force)
    residual = residual - direction * float(residual @ direction)
    return ForceEquivalentLine(
        kind=kind,
        version=version,
        height=float(height),
        section_coordinate=float(section_coordinate),
        section_axis=section_axis,
        weights=weights,
        contact_points=points,
        vertical_forces=fz,
        force=force,
        moment=moment,
        line_point=line_point,
        line_direction=direction,
        residual_couple=residual,
    )


def section_intersection(
    line_point: Any,
    line_direction: Any,
    *,
    axis: str,
    coordinate: float,
) -> np.ndarray:
    """
    Intersect a line of action with a coordinate plane, or say why there is none.

    Two degenerate cases are distinguished rather than collapsed:

    * the line is parallel to the plane, so it never meets it;
    * the line lies *in* the plane, so every point of it is an intersection and
      no single one can be reported.
    """
    point = np.asarray(line_point, dtype=float).ravel()
    direction = np.asarray(line_direction, dtype=float).ravel()
    if point.shape != (3,) or direction.shape != (3,):
        raise CharacteristicsError("a line needs a three-vector point and direction")
    index = {"x": 0, "y": 1, "z": 2}.get(axis)
    if index is None:
        raise CharacteristicsError(f"unknown section axis {axis!r}; use 'x', 'y' or 'z'")
    component = direction[index]
    offset = coordinate - point[index]
    if abs(component) <= RESULTANT_FLOOR:
        if abs(offset) <= RESULTANT_FLOOR:
            raise CharacteristicsError(
                "the line of action lies in the section, so its intersection is not unique"
            )
        raise CharacteristicsError("the line of action is parallel to the section")
    return point + direction * (offset / component)


def side_view_instant_center(
    contact_point: Any,
    contact_motion: Any,
    reference_motion: Any,
    *,
    angular_rate: float,
) -> np.ndarray:
    """
    Return the single-wheel side-view instant centre in the XZ plane.

    The relative motion of the wheel in the side view is a planar rigid motion:
    a velocity ``(vx, vz)`` at the contact point plus an angular rate
    ``angular_rate`` about the ``y`` axis.  The instant centre is the point whose
    relative velocity is zero, and for a planar motion it is found by moving from
    the contact point perpendicular to its velocity by ``|v| / |omega|``.

    A motion with no angular rate is a pure translation in the side view and has
    no finite instant centre.  That is reported, because a centre at a very large
    distance is a different statement from "there is no centre".

    ``angular_rate`` is passed explicitly rather than inferred from ``v`` and the
    reference point: inferring it would make the result depend on a choice of
    lever arm that the caller never declared, and a wrong lever arm silently
    changes the answer while still producing a plausible-looking point.
    """
    point = np.asarray(contact_point, dtype=float).ravel()
    reference = np.asarray(reference_motion, dtype=float).ravel()
    velocity = np.asarray(contact_motion, dtype=float).ravel()
    if point.shape != (3,):
        raise CharacteristicsError("the contact point must be a three-vector")
    if velocity.shape != (3,) or reference.shape != (3,):
        raise CharacteristicsError("the contact and reference motions must be three-vectors")
    if not np.isfinite(angular_rate):
        raise CharacteristicsError("the angular rate must be finite")
    if abs(angular_rate) <= RESULTANT_FLOOR:
        raise CharacteristicsError(
            "the side-view relative motion is a pure translation (zero angular rate), so "
            "it has no finite instant centre"
        )
    # In the (x, z) plane a rotation about +y maps a radius r to omega * (r_z, -r_x).
    # Solving v + omega * (r_z, -r_x) = 0 gives r = (v_z, -v_x) / omega.
    relative = velocity - reference
    radius = np.array([relative[2], -relative[0]]) / angular_rate
    return np.array([point[0] + radius[0], point[1], point[2] + radius[1]])


def legacy_roll_center(
    patches: Any,
    slopes: Any,
    *,
    lateral_force: float = LEGACY_LATERAL_FORCE,
) -> np.ndarray:
    """
    Return the ``roll_center_legacy_v1`` centre from contact patches and slopes.

    This preserves the original least-squares-free arithmetic and sign convention
    exactly: the centre is the mean lateral position of the patches together with
    the height ``-sum(y_i * slope_i) / lateral_force``.  It is kept as its own
    version because it makes no identifiability claim -- the newer definition is
    the one that checks the full admissible basis, and quietly upgrading these
    numbers would change every stored legacy value.
    """
    patch_array = _as_points(patches, "contact patches")
    slope_array = _as_slopes(slopes, patch_array.shape[0])
    if abs(lateral_force) <= RESULTANT_FLOOR:
        raise CharacteristicsError("the legacy roll centre needs a nonzero lateral force")
    return np.array(
        [patch_array[:, 1].mean(), -_legacy_roll_moment(patch_array, slope_array) / lateral_force]
    )


def _as_slopes(slopes: Any, count: int) -> np.ndarray:
    array = np.asarray(slopes, dtype=float).ravel()
    if array.shape[0] != count:
        raise CharacteristicsError(f"{array.shape[0]} slopes for {count} contact patches")
    if not np.isfinite(array).all():
        raise CharacteristicsError("the contact slopes must be finite")
    return array


def _legacy_roll_moment(patches: np.ndarray, slopes: np.ndarray) -> float:
    """Return ``sum(y_i * slope_i)``: the legacy roll moment about the x axis."""
    return float(patches[:, 1] @ slopes)


def legacy_roll_center_envelope_values(readings: Any) -> np.ndarray:
    """
    Aggregate primitive readings the way ``ResultEnvelope.measure`` always has.

    ``readings`` is ``(samples, points, 4)``.  The envelope reports
    ``[mean(point y), mean(y_i * slope_i)]`` -- a **mean** of the per-point moment
    terms, and *without* dividing by a lateral force.

    That is deliberately different from the vehicle report, which sums the same
    terms and then divides by ``LEGACY_LATERAL_FORCE`` to obtain a height.  With
    two contact points the two differ by a factor of four.  They are two
    published quantities, so both aggregations are kept side by side and named;
    folding either into the other would silently rescale stored readings.
    """
    array = np.asarray(readings, dtype=float)
    if array.ndim != 3 or array.shape[2] != 4:
        raise CharacteristicsError("roll-center readings must be (samples, points, 4)")
    lateral = array[:, :, 1]
    slopes = array[:, :, 3]
    return np.column_stack((lateral.mean(axis=1), (lateral * slopes).mean(axis=1)))


def legacy_roll_center_report_values(
    readings: Any,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    """
    Aggregate one sample's primitive readings the way the vehicle report always has.

    ``readings`` is a single sample of ``(points, 4)``.  The report needs the
    individual pieces -- both patch positions, both slopes, the summed moment and
    the derived height -- so they are returned rather than compressed into a
    centre point, which is what ``RollCenterResult`` publishes.

    The height is ``-sum(y_i * slope_i) / lateral_force``: the roll moment is
    *summed* over the contact points.  See
    ``legacy_roll_center_envelope_values`` for why the envelope's mean-based
    aggregation is a separate function rather than a shared one.
    """
    array = np.asarray(readings, dtype=float)
    if array.ndim != 2 or array.shape[1] != 4:
        raise CharacteristicsError("one sample of roll-center readings must be (points, 4)")
    patches = array[:, :3]
    slopes = array[:, 3].copy()
    lateral = float(patches[:, 1].mean())
    moment = float(patches[:, 1] @ slopes)
    return patches, slopes, np.array([lateral, -moment / LEGACY_LATERAL_FORCE]), moment


def legacy_roll_center_primitives(
    *,
    jacobians: Any,
    selected_rows: Any,
    body_offsets: Any,
    reference_rotations: Any,
    reference_origins: Any,
    body_positions: Any,
    drive_points: Any,
    contact_points: Any,
    reference_body_offset: int | None = None,
    tolerance: float = 1e-8,
) -> np.ndarray:
    """
    Assemble the ``roll_center_legacy_v1`` per-sample readings from raw arrays.

    This is the retired ``ResultEnvelope._roll_center_primitives`` body, moved
    here so the formula has exactly one owner.  The envelope and the vehicle
    report both consumed it -- the report through a private attribute access --
    and this function is the shared implementation they now call instead.

    Everything arrives as plain arrays, so the algorithm stays independent of how
    the motion was produced: the caller extracts the constraint rows, the body
    states and the frame positions it already has.  ``characteristics`` must not
    import the results layer, so passing data rather than a result object is what
    keeps this function in its own layer.

    For each sample and each declared contact/drive pair it applies a unit
    vertical drive at the drive point, holds the selected constraint rows at
    zero (plus, optionally, the six coordinates of a declared reference body),
    and least-squares solves for the generalized velocity.  The contact's
    vertical velocity must be nonzero, and its slope is ``v_y / v_z``.

    The returned array is ``(samples, points, 4)``: the contact position in the
    reference frame followed by the slope.  Both remaining legacy consumers read
    columns 1 and 3, and the layout is preserved so their numbers do not move.

    It is deliberately *not* re-derived from a force equilibrium.  These values
    depend on this exact assembly -- which rows are held, the unit drive, and the
    least-squares choice when the system is redundant -- so substituting a
    better-founded construction would change every stored reading.  The audited
    definition is ``force_equivalent_roll_center``, a different function on
    purpose.
    """
    motion = np.asarray(jacobians, dtype=float)
    if motion.ndim != 3:
        raise CharacteristicsError("the constraint jacobian must be (samples, rows, dof)")
    samples, _, dof = motion.shape
    rows = np.asarray(selected_rows, dtype=int).ravel()
    if rows.size and (rows.min() < 0 or rows.max() >= motion.shape[1]):
        raise CharacteristicsError("the selected constraint rows fall outside the jacobian")
    offsets = np.asarray(body_offsets, dtype=int).ravel()
    rotations = np.asarray(reference_rotations, dtype=float)
    origins = np.asarray(reference_origins, dtype=float)
    positions = np.asarray(body_positions, dtype=float)
    drives = np.asarray(drive_points, dtype=float)
    contacts = np.asarray(contact_points, dtype=float)
    count = offsets.shape[0]
    if count == 0:
        raise CharacteristicsError("a roll centre needs at least one contact/drive pair")
    for array, label, shape in (
        (rotations, "reference rotations", (samples, 3, 3)),
        (origins, "reference origins", (samples, 3)),
        (positions, "body positions", (samples, count, 3)),
        (drives, "drive points", (samples, count, 3)),
        (contacts, "contact points", (samples, count, 3)),
    ):
        if array.shape != shape:
            raise CharacteristicsError(f"{label} must have shape {shape}, got {array.shape}")
    if reference_body_offset is not None and not 0 <= reference_body_offset <= dof - 6:
        raise CharacteristicsError("the reference body offset does not fit the coordinate vector")
    readings = np.empty((samples, count, 4))
    for sample in range(samples):
        frame = rotations[sample]
        origin = origins[sample]
        for point in range(count):
            offset = int(offsets[point])
            if not 0 <= offset <= dof - 6:
                raise CharacteristicsError(
                    f"free-body offset {offset} does not fit a {dof}-column coordinate vector"
                )
            up = frame[:, 2]
            arm = drives[sample, point] - positions[sample, point]
            drive_row = np.zeros(dof)
            drive_row[offset : offset + 3] = up
            drive_row[offset + 3 : offset + 6] = np.cross(arm, up)
            matrix = [motion[sample][rows], drive_row[None, :]]
            targets = [0.0] * rows.size + [1.0]
            if reference_body_offset is not None:
                held = np.zeros((6, dof))
                for axis in range(6):
                    held[axis, reference_body_offset + axis] = 1.0
                matrix.append(held)
                targets.extend([0.0] * 6)
            assembled = np.vstack(matrix)
            wanted = np.asarray(targets)
            twist = np.linalg.lstsq(assembled, wanted, rcond=None)[0]
            residual = float(np.max(np.abs(assembled @ twist - wanted)))
            if residual > tolerance:
                raise CharacteristicsError(
                    "the roll-center drive is inconsistent with the selected constraints "
                    f"(residual {residual:.3e})"
                )
            linear = twist[offset : offset + 3]
            angular = twist[offset + 3 : offset + 6]
            velocity = frame.T @ (
                linear + np.cross(angular, contacts[sample, point] - positions[sample, point])
            )
            if abs(velocity[2]) <= RESULTANT_FLOOR:
                raise CharacteristicsError("the roll-center contact has no vertical motion")
            readings[sample, point, :3] = frame.T @ (contacts[sample, point] - origin)
            readings[sample, point, 3] = velocity[1] / velocity[2]
    return readings


def named_operators() -> dict[str, Any]:
    """
    Return the explicit ``version -> operator`` catalogue.

    A registry rather than a chain of ``if`` statements: every definition has a
    name, a version and a callable, so a request naming an unimplemented version
    is refused by name instead of silently falling back to another definition.
    """
    return {
        ROLL_CENTER_FORCE_VERSION: force_equivalent_roll_center,
        PITCH_CENTER_FORCE_VERSION: force_equivalent_pitch_center,
        ROLL_CENTER_LEGACY_VERSION: legacy_roll_center,
    }


def require_operator(version: str) -> Any:
    """Return the operator for an exact definition version, or refuse."""
    operators = named_operators()
    if version not in operators:
        available = ", ".join(sorted(operators))
        raise CharacteristicsError(
            f"unknown centre definition version {version!r}; implemented: {available}"
        )
    return operators[version]


def contact_motion_basis(motions: Sequence[Any]) -> np.ndarray:
    """
    Stack per-direction contact motions into the ``(points, 3, directions)`` basis.

    A helper so callers do not each invent their own axis order, which is the
    kind of difference that silently swaps the lateral and vertical components
    of a centre calculation.
    """
    columns = [np.asarray(item, dtype=float) for item in motions]
    if not columns:
        raise CharacteristicsError("a contact motion basis needs at least one direction")
    for column in columns:
        if column.ndim != 2 or column.shape[1] != 3:
            raise CharacteristicsError("each direction must give one three-vector per point")
    return np.stack(columns, axis=2)


__all__ = [
    "LEGACY_LATERAL_FORCE",
    "PITCH_CENTER_FORCE_VERSION",
    "ROLL_CENTER_FORCE_VERSION",
    "ROLL_CENTER_LEGACY_VERSION",
    "WORK_TOLERANCE",
    "ForceEquivalentLine",
    "contact_motion_basis",
    "force_equivalent_pitch_center",
    "force_equivalent_roll_center",
    "legacy_roll_center",
    "legacy_roll_center_envelope_values",
    "legacy_roll_center_primitives",
    "named_operators",
    "require_operator",
    "section_intersection",
    "side_view_instant_center",
]
