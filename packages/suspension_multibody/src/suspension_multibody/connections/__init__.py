"""
Ports, matching, geometric adaptation and the global assembly rules.

This package is where a declared model becomes a *connected* one. It sits above
``modeling`` (which describes entities) and beside ``templates`` (which describe
what a subsystem contributes), and it owns three things:

``policy``
    the global root rules -- what an axle may not carry and a vehicle must. Stated
    once, applied to the root's category, not overridable.
``matcher``
    which offered port satisfies which requirement: explicit mapping first, then
    role/capability/label filtering, ambiguity reported rather than guessed at.
``geometry``
    where a mounted instance lands, derived from the owner's live pose so that a
    hardpoint change moves the rig without editing the rig.

The word "connections" covers the *decisions*; the entities those decisions
produce are owned by the ``SimulationAssembly`` that results, not by this package.
"""

from __future__ import annotations

from .geometry import GeometryMismatchError, MountSolution, port_world_pose, solve_mount
from .matcher import (
    AmbiguousBindingError,
    Binding,
    BindingError,
    MatchReport,
    UnmetRequirementError,
    match_requirements,
)
from .policy import (
    AXLE_RULE,
    ROLES,
    ROOT_KINDS,
    VEHICLE_RULE,
    RootRule,
    RuleViolation,
    check_root,
    rule_for,
)

__all__ = [
    "AXLE_RULE",
    "ROLES",
    "ROOT_KINDS",
    "VEHICLE_RULE",
    "AmbiguousBindingError",
    "Binding",
    "BindingError",
    "GeometryMismatchError",
    "MatchReport",
    "MountSolution",
    "RootRule",
    "RuleViolation",
    "UnmetRequirementError",
    "check_root",
    "match_requirements",
    "port_world_pose",
    "rule_for",
    "solve_mount",
]
