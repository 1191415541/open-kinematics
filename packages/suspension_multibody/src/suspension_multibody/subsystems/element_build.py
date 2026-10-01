"""
Element construction: a declared row becomes the runtime force element.

A subsystem decides *what* force element exists and *where* -- it emits
:class:`~.types.ResolvedElement` declarations.  Turning a declaration into the
object a document carries is this module's job, and it used to live in the axle
assembly, which is why a composition could not produce elements without calling
that assembly.

The recorded order matters as much as the content: the contract document lists
elements in the sequence the assembly appends them, so :func:`element_rows` owns
the sequence and the individual builders own only their own row.

Nothing here solves, submits native or decodes a result.  It builds values.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Literal, cast

import numpy as np

from ..modeling.primitives import (
    SE3,
    AntiRollBarElement,
    BumpStopElement,
    BushingElement,
    LinearSpringElement,
    RotationalTorqueElement,
    RotationalTorqueParameters,
    StaticDamperElement,
    VerticalTireElement,
)
from ..schema import (
    AntiRollBar,
    BumpStop,
    Bushing6x6,
    LinearSpring,
    StaticDamper,
    VerticalTire,
)
from . import suspension as suspension_subsystem
from . import wheel as wheel_subsystem
from .types import ResolvedElement, SubsystemContext

__all__ = [
    "build_element",
    "element_rows",
]


def build_element(row: ResolvedElement) -> object:
    """
    Build one declared elastic element.

    A subsystem decides what exists and where; this function constructs it.  An
    unknown kind is refused rather than skipped, because a silently dropped
    element is the one failure mode a solver cannot show: the run still
    converges and the missing spring is invisible.
    """
    if row.kind == "spring":
        return _spring(row, cast(LinearSpring, row.spec))
    if row.kind == "damper":
        return _damper(row, cast(StaticDamper, row.spec))
    if row.kind == "bump_stop":
        return _bump_stop(row, cast(BumpStop, row.spec))
    if row.kind == "anti_roll_bar":
        return _anti_roll_bar(row, cast(AntiRollBar, row.spec))
    if row.kind == "tire":
        return _tire(row, cast(VerticalTire, row.spec))
    if row.kind == "bushing":
        return _bushing(row)
    if row.kind == "rotational_torque":
        return _rotational_torque(row)
    raise ValueError(f"unsupported element kind {row.kind!r}")


def _spring(row: ResolvedElement, spec: LinearSpring) -> LinearSpringElement:
    """Build one declared linear spring."""
    return LinearSpringElement(
        name=row.name,
        body_a=cast(str, row.body_a),
        point_a=cast(np.ndarray, row.point_a),
        body_b=cast(str, row.body_b),
        point_b=cast(np.ndarray, row.point_b),
        stiffness=spec.stiffness,
        free_length=spec.free_length,
        reference_length=spec.reference_length,
        preload=spec.preload or 0.0,
        force_curve=spec.force_curve,
    )


def _damper(row: ResolvedElement, spec: StaticDamper) -> StaticDamperElement:
    """Build one declared quasi-static damper."""
    return StaticDamperElement(
        name=row.name,
        body_a=cast(str, row.body_a),
        point_a=cast(np.ndarray, row.point_a),
        body_b=cast(str, row.body_b),
        point_b=cast(np.ndarray, row.point_b),
        gas_stiffness=spec.gas_stiffness,
        gas_reference_length=spec.gas_reference_length,
        gas_reference_force=spec.gas_reference_force,
        preload=spec.preload,
        friction=spec.friction,
        viscous_damping=spec.viscous_damping,
        force_curve=spec.force_curve,
    )


def _bump_stop(row: ResolvedElement, spec: BumpStop) -> BumpStopElement:
    """Build one declared bump stop."""
    return BumpStopElement(
        name=row.name,
        body_a=cast(str, row.body_a),
        point_a=cast(np.ndarray, row.point_a),
        body_b=cast(str, row.body_b),
        point_b=cast(np.ndarray, row.point_b),
        clearance=spec.clearance,
        stiffness=spec.stiffness,
        direction=spec.direction,
        force_curve=spec.force_curve,
    )


def _anti_roll_bar(row: ResolvedElement, spec: AntiRollBar) -> AntiRollBarElement:
    """Build one declared anti-roll bar."""
    return AntiRollBarElement(
        name=row.name,
        left_body=cast(str, row.body_a),
        left_point=cast(np.ndarray, row.point_a),
        right_body=cast(str, row.body_b),
        right_point=cast(np.ndarray, row.point_b),
        stiffness=spec.torsional_stiffness,
    )


def _tire(row: ResolvedElement, spec: VerticalTire) -> VerticalTireElement:
    """Build one declared vertical tire."""
    return VerticalTireElement(
        name=row.name,
        wheel_body=cast(str, row.body_a),
        wheel_center_local=cast(np.ndarray, row.point_a),
        stiffness=spec.stiffness,
        unloaded_radius=spec.unloaded_radius,
    )


def _bushing(row: ResolvedElement) -> BushingElement:
    """
    Build one declared bushing.

    A `spec` that is already an array is a template slot bushing: the template
    declared the connection's bushing column and the stiffness came from its
    property slot (zero for the built-in template, a real number for one that
    declares one).  The local poses are the identity-rotation slots the build has
    always written, so only the stiffness can move.
    """
    if isinstance(row.spec, np.ndarray):
        return BushingElement(
            name=row.name,
            body_a=cast(str, row.body_a),
            body_b=cast(str, row.body_b),
            local_pose_a=cast(SE3, row.local_pose_a),
            local_pose_b=cast(SE3, row.local_pose_b),
            stiffness=row.spec,
        )
    if row.spec is None:
        return BushingElement(
            name=row.name,
            body_a=cast(str, row.body_a),
            body_b=cast(str, row.body_b),
            local_pose_a=cast(SE3, row.local_pose_a),
            local_pose_b=cast(SE3, row.local_pose_b),
            stiffness=np.zeros((6, 6)),
        )
    spec = cast(Bushing6x6, row.spec)
    return BushingElement(
        name=row.name,
        body_a=cast(str, row.body_a),
        body_b=cast(str, row.body_b),
        local_pose_a=cast(SE3, row.local_pose_a),
        local_pose_b=cast(SE3, row.local_pose_b),
        stiffness=np.asarray(spec.stiffness, dtype=float),
        damping=np.diag(np.asarray(spec.damping, dtype=float)),
        preload=np.asarray(spec.preload, dtype=float),
        force_curves=spec.force_curves,
        force_curve_interpolation=spec.force_curve_interpolation,
        rotation_coordinates=spec.rotation_coordinates,
    )


def _rotational_torque(row: ResolvedElement) -> RotationalTorqueElement:
    """
    Build one declared rotational torque.

    The two bodies come from the row verbatim.  Which one is the driven side is
    decided where the row was built -- from the port pairing that resolved the two
    ends -- and never here: a rule here that read either name would be the
    name-based identity guess the assembly layer is not allowed to have.
    """
    return RotationalTorqueElement(
        name=row.name,
        body_a=cast(str, row.body_a),
        body_b=cast(str, row.body_b),
        parameters=cast(RotationalTorqueParameters, row.spec),
    )


def element_rows(
    model: object,
    mode: Literal["K", "C"],
    context: SubsystemContext,
    placeholders: Iterable[ResolvedElement],
) -> list[ResolvedElement]:
    """
    Collect the symmetric-proxy element declarations in their recorded order.

    The order is the contract: per side springs, dampers, tires, stops; then the
    anti-roll bar once; then the user's C-mode bushings; then the C-mode slot
    placeholders, which the original build appends last.  Each slot is filled by
    the subsystem that owns that content; the slot positions are fixed here.
    """
    rows: list[ResolvedElement] = []
    for side in context.request.sides:
        for kind in ("spring", "damper"):
            rows.extend(suspension_subsystem.elements(context, side, kind))
        # The wheel end exists only when this assembly carries the wheel role.
        # A vehicle composes its axles without it -- the vehicle owns the wheel
        # ends -- and asking the wheel template for a tire there would declare a
        # second one for a wheel somebody else already described.
        if context.request.carries("wheel"):
            rows.extend(wheel_subsystem.tires(context, side))
        rows.extend(suspension_subsystem.elements(context, side, "bump_stop"))
    rows.extend(suspension_subsystem.global_elements(context))
    if mode == "C":
        for side in context.request.sides:
            rows.extend(suspension_subsystem.compliance_elements(context, side))
        rows.extend(placeholders)
    return rows
