"""
Geometric adaptation: where a rig instance ends up, given where a port is.

A rig is written once and mounted on whatever assembly it is given.  When that
assembly's hardpoints move, the rig must follow -- and the way it follows is not
"recompute the rig's coordinates" but "recompute the transform that mounts it".
The rig's own definition keeps its local geometry; only the derived mounting pose
changes.  That is what ``A4`` tests: perturb the hardpoints, and the attachment
must equal an independently computed transform rather than a cached coordinate.

The transform is composed from two poses:

* the **world pose of the assembly's port**, which the assembly knows (it comes
  from the body the port hangs on, which the hardpoint moved);
* the **local installation pose declared by the rig**, which is constant.

so ``rig_instance = port_world @ installation``.  A template that needs more than
one attachment point declares a build rule per point; this module refuses to fit
a single rigid transform to several non-coincident points, because that is a
different (and wrong) answer dressed as a convenience.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..modeling.ports import GeometryPort
from ..modeling.primitives.spatial import SE3

__all__ = [
    "GeometryMismatchError",
    "MountSolution",
    "solve_mount",
]


class GeometryMismatchError(ValueError):
    """A template cannot be mounted on this port as written."""


@dataclass(frozen=True)
class MountSolution:
    """
    Where a rig instance lands, and the evidence for it.

    ``pose`` is the rig instance's world pose.  ``port_world`` and
    ``installation`` are kept so the arithmetic can be rechecked without
    rebuilding the assembly -- which is exactly how ``A4`` verifies it.
    """

    pose: SE3
    port_world: SE3
    installation: SE3

    def matches(self, expected: SE3, *, tolerance: float) -> bool:
        """
        Return whether this solution agrees with an independently computed pose.

        Compared on translation and on the rotation's principal vector rather
        than on quaternions: ``q`` and ``-q`` are the same rotation, and a
        quaternion comparison would report a mismatch between two identical
        orientations that happen to have opposite signs.
        """
        from ..modeling.primitives.spatial import (
            quaternion_to_rotation_vector,
        )

        translation_error = float(
            np.linalg.norm(np.asarray(self.pose.translation) - np.asarray(expected.translation))
        )
        rotation_error = float(
            np.linalg.norm(
                quaternion_to_rotation_vector(self.pose.quaternion)
                - quaternion_to_rotation_vector(expected.quaternion)
            )
        )
        return translation_error <= tolerance and rotation_error <= tolerance


def solve_mount(port: GeometryPort, *, port_world: SE3, installation: SE3) -> MountSolution:
    """
    Mount a rig instance on ``port``.

    ``port_world`` is the port's pose in world coordinates -- the assembly
    derives it from the body the port hangs on, so a hardpoint change moves it.
    ``installation`` is the rig's own local offset, constant for the template.

    A port declared ``cardinality="none"`` is refused: it exists to be reported,
    not connected.
    """
    if port.cardinality == "none":
        raise GeometryMismatchError(
            f"port {port.id} is declared unconnectable; it must not be mounted on"
        )
    return MountSolution(
        pose=port_world.compose(installation),
        port_world=port_world,
        installation=installation,
    )


def port_world_pose(body_pose: SE3, port_local: SE3) -> SE3:
    """
    Return a port's world pose from the pose of the body that owns it.

    Stated as its own function because it is the one line that makes the whole
    adaptive-interface story true: the port's *local* offset is constant, and
    everything that moves comes from the owner.  Inlining it at each call site is
    how a stale cached coordinate gets introduced.
    """
    return body_pose.compose(port_local)
