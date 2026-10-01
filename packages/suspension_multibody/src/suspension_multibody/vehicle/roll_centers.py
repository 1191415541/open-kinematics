"""
Front-view roll centre per axle, solved from the suspension's own force transmission.

The roll centre is a property of the *constraint set* an assembly carries, not of a
set of hard-point names.  Nothing here looks up a hard-point role, and nothing here
intersects arm lines.  For each wheel end of an axle:

1. the differential-kinematics engine (``vehicle/screw_kinematics.py``) solves the
   assembly's own constraints for the motion in which that wheel end rises, and the
   front-view velocity ``(v_y, v_z)`` of the contact patch is read out of the twist
   the solve returned;
2. the suspension passes a lateral force to the sprung mass along the line joining
   the patch to the wheel end's front-view centre of rotation; that line is
   perpendicular to the patch's own front-view path.  The published
   ``contact_patch_slope`` is the path's ``d y / d z``, i.e. the patch's lateral
   travel per unit of its rise, ``r = v_y / v_z``; the force line's own ``d z / d y``
   is then ``-r``.  A wheel end that translates in the front view (``r = 0``) has its
   centre of rotation at infinity and its force line horizontal -- which is a fact
   about the linkage, not a failure, and is why the ratio, not the centre, is what
   the arithmetic uses;
3. those slopes build the **force-to-generalized-displacement derivative matrix** over
   the two generalized coordinates ``q = (u_y, phi)`` -- a lateral translation of the
   sprung mass and its roll about the ``x`` axis through the road plane:

       B[i, u_y] = d y_patch,i / d u_y = -1
       B[i, phi] = d y_patch,i / d phi = y_i * r_i

   (the patch is a material point of the wheel end, so a lateral translation of the
   sprung mass carries it the opposite way; rolling the sprung mass by ``phi`` about
   the longitudinal axis through the road plane lifts each wheel end by ``y_i * phi``
   relative to it, and the linkage turns that rise into a lateral travel of
   ``r_i * y_i * phi``, since ``r_i`` is the patch's lateral travel per unit rise);
4. with a lateral force ``dF_i`` at each patch the generalized loads are
   ``Q = B^T dF``, and the height at which those forces produce no roll moment is

       h = -Q_phi / Q_uy

That last line is what ``docs/multibody_architecture_evolution.md`` §2.6 asks for: the
roll-centre height is solved from the virtual-work derivative of the roll reaction
against the wheel's lateral forces, not from a geometric intersection.  A mechanism
that passes no lateral force at all (``Q_uy = 0``) has no roll centre and is refused by
name rather than divided by.

The reported contact patches and their path slopes are the reads the matrix is built
from, so a reader can check the arithmetic without re-solving anything.

It sits in ``vehicle/`` for the same reason ``static_loads`` does: the construction
reads the assembled vehicle, and ``vehicle/`` is the layer allowed to import
``subsystems``.  ``report/`` is not -- putting it there would add a
``report -> preparation`` edge the report boundary forbids.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from . import screw_kinematics as _engine

if TYPE_CHECKING:  # pragma: no cover - typing only
    from ..schema import FrontAxleModel, VehicleModel

#: Below this the patch's front-view velocity carries no slope: the ratio would be
#: rounding rather than geometry.
_FRONT_VIEW_EPSILON = 1e-12

#: Below this the axle passes no lateral force to the sprung mass, so the height at
#: which that force produces no roll moment is not defined rather than infinite.
_LATERAL_TRANSFER_EPSILON = 1e-12

#: The direction a wheel end is driven in to read one side: straight up.
_UP = np.array([0.0, 0.0, 1.0])

#: A per-side instant-centre construction a caller may route in.  ``None`` uses the
#: engine; the hook lets a caller compare constructions without this module owning a
#: second one.
InstantCentreEngine = Callable[["FrontAxleModel", str], np.ndarray]


@dataclass(frozen=True)
class RollCenterResult:
    """
    Front-view roll-centre geometry for one axle.

    ``center`` is ``[y, z]`` in the vehicle frame: ``center[1]`` is the height solved
    from the force-to-generalized-displacement derivative matrix and ``center[0]`` is
    the lateral position the lateral loads act through (zero for a symmetric axle).
    """

    axle: str
    center: np.ndarray
    #: Each side's contact patch, as ``[y, z]``, and the slope of the patch's own
    #: front-view path, ``d y / d z`` -- its lateral travel per unit of its rise.
    #: These are the two reads the matrix is built from.  The force line joins the
    #: patch to that side's centre of rotation and is perpendicular to this path, so
    #: its own ``d z / d y`` is the negative of this slope.  A zero slope means the
    #: patch rises without moving laterally, and its force line is horizontal.
    left_contact_patch: np.ndarray
    left_contact_patch_slope: float
    right_contact_patch: np.ndarray
    right_contact_patch_slope: float
    #: The two generalized loads the height divides: lateral force and roll moment.
    lateral_force: float
    roll_moment: float


def compute_vehicle_roll_centers(
    vehicle: "VehicleModel",
    *,
    road_z: float = 0.0,
    instant_center_engine: InstantCentreEngine | None = None,
) -> dict[str, RollCenterResult]:
    """
    Compute the front-view roll centre of every axle of ``vehicle``.

    The height comes from ``h = -Q_phi / Q_uy`` over the force-to-generalized-
    displacement derivative matrix the axle's own constraint set builds; see the
    module docstring.  ``instant_center_engine`` routes the per-side slope to another
    construction and is otherwise unused -- the default is the engine's.

    A side whose patch does not move in the front view, and an axle whose lateral
    force cannot be transmitted, are refused by name rather than reported as a centre
    picked from nothing.
    """
    runtime = _assembly_for(vehicle)
    results: dict[str, RollCenterResult] = {}
    for placement, axle_runtime in runtime.axle_assemblies.items():
        sides = _front_view_sides(
            axle_runtime, vehicle, placement, road_z, instant_center_engine
        )
        if len(sides) != 2:
            raise ValueError(
                f"axle {placement!r} declares {len(sides)} wheel centres in its "
                "assembly; a front-view roll centre is defined for the two sides of one "
                "axle"
            )
        left, right = sides
        patches = np.array([left[1], right[1]], dtype=float)
        slopes = np.array([left[0], right[0]], dtype=float)
        matrix = np.column_stack((-np.ones(2), patches[:, 1] * slopes))
        increments = np.ones(2)
        generalized = matrix.T @ increments
        lateral_force = float(generalized[0])
        roll_moment = float(generalized[1])
        if abs(lateral_force) <= _LATERAL_TRANSFER_EPSILON:
            raise ValueError(
                f"axle {placement!r} passes no lateral force to the sprung mass "
                f"(dF_lateral/dy = {lateral_force!r}), so the height at which that "
                "force produces no roll moment is not defined"
            )
        height = -roll_moment / lateral_force
        lateral_position = float(patches[:, 1].mean())
        results[placement] = RollCenterResult(
            axle=placement,
            center=np.array([lateral_position, height], dtype=float),
            left_contact_patch=patches[0],
            left_contact_patch_slope=float(slopes[0]),
            right_contact_patch=patches[1],
            right_contact_patch_slope=float(slopes[1]),
            lateral_force=lateral_force,
            roll_moment=roll_moment,
        )
    return results


def _front_view_sides(
    axle_runtime: Any,
    vehicle: "VehicleModel",
    placement: str,
    road_z: float,
    hook: InstantCentreEngine | None,
) -> list[tuple[float, np.ndarray]]:
    """
    Return both sides' ``(patch path slope, contact patch)`` in the front view.

    The two sides are told apart by the sign of the patch's ``y``, which is what makes
    a pair of wheels a pair; no name and no hard-point role is consulted.
    sides are told apart by the sign of the patch's ``y``, which is what makes a
    pair of wheels a pair; no name and no hard-point role is consulted.
    """
    axle_model = _axle_model(vehicle, placement)
    axle_model = _axle_model(vehicle, placement)
    sides: list[tuple[float, np.ndarray]] = []
    for body, label in sorted(axle_runtime.points):
        if label != "wheel_center":
            continue
        sides.append(
            _side_front_view(axle_runtime, body, road_z, axle_model, placement, hook)
        )
    sides.sort(key=lambda side: float(side[1][1]))
    if len(sides) == 2 and float(sides[0][1][1]) > 0.0:
        sides.reverse()
    return sides


def _side_front_view(
    axle_runtime: Any,
    body: str,
    road_z: float,
    axle_model: "FrontAxleModel",
    placement: str,
    hook: InstantCentreEngine | None,
) -> tuple[float, np.ndarray]:
    """
    Return one wheel end's ``(patch path slope, contact patch)`` in the front view.

    The slope is read out of the twist the engine solved for, unless the caller routed
    another construction in.
    """
    local = np.asarray(axle_runtime.points[(body, "wheel_center")], dtype=float)
    centre_world = axle_runtime.state.point_world(body, local)
    patch = np.array([centre_world[0], centre_world[1], road_z], dtype=float)
    if hook is not None:
        instant = np.asarray(hook(axle_model, _side_label(float(centre_world[1]))), dtype=float)
        rise = float(instant[1] - patch[2])
        run = float(patch[1] - instant[0])
        if abs(run) <= _FRONT_VIEW_EPSILON:
            raise ValueError(
                f"the routed instant centre of {body!r} stands at the patch's own "
                f"lateral position ({instant[0]!r}), so its force line is vertical and "
                "carries no slope to read"
            )
        return rise / run, patch
    motion = _engine.solve_rigid_motion(
        axle_runtime.constraints,
        axle_runtime.state,
        [_engine.PointDrive(body, "wheel_center", _UP.copy())],
        points=axle_runtime.points,
        pivot_body=body,
        pivot_point=(body, "wheel_center"),
    )
    velocity = motion.twist_of(body).transform_point(patch)
    if abs(float(velocity[2])) <= _FRONT_VIEW_EPSILON:
        raise ValueError(
            f"the wheel end {body!r} of axle {placement!r} does not move vertically "
            f"when it is driven upward (v_z = {float(velocity[2])!r} at the contact "
            "patch); this mechanism carries no front view to read a roll centre from"
        )
    return float(velocity[1]) / float(velocity[2]), patch


def _side_label(y: float) -> str:
    """Return ``"L"`` or ``"R"`` for a side, by the sign of its lateral position."""
    return "L" if y < 0.0 else "R"


def _axle_model(vehicle: "VehicleModel", placement: str) -> Any:
    """Return the axle a placement names, without a name rule of its own."""
    for attribute in ("front_axle", "rear_axle"):
        axle = getattr(vehicle, attribute, None)
        if axle is not None and axle.name == placement:
            return axle
    for attribute in ("front_axle", "rear_axle"):
        axle = getattr(vehicle, attribute, None)
        if axle is not None and attribute.startswith(placement):
            return axle
    raise ValueError(
        f"the assembled vehicle has an axle at {placement!r} that its model does not "
        "declare; the two disagree about what the vehicle is"
    )


def _assembly_for(vehicle: "VehicleModel") -> Any:
    """Compose the vehicle and return its runtime."""
    from ..subsystems.entry import compose_vehicle

    return compose_vehicle(vehicle, "K")


__all__ = ["RollCenterResult", "compute_vehicle_roll_centers"]
