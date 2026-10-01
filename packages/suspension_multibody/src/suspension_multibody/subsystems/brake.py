"""
The brake subsystem: a torque-element brake under the `brake` role.

The simplified template owns **no bodies at all**.  It does not build a caliper,
a rotor or a brake disc; what it contributes is a *torque element*: one couple
per braked wheel, between the wheel it slows and the body that reacts it.
Adams Car's own simplified brake is the same thing: the 0-body
`_brake_system_4Wdisk.tpl` sits under `MAJOR_ROLE='brake_system'` next to the
4-body `_brake_system_4Wdisk_calipers.tpl`, and the assembly does not know which
one it got.

Three things are deliberately true of this module, because they are what makes
the simplified brake replaceable rather than a dead end (requirement 20 / D11):

* the torque is an element **declaration**, not a sampled signal.  The couple's
  magnitude comes from the role's own slots and its direction from the real-time
  relative angular rate, which the kernel's ``rotational_torque`` family reads
  (``cpp/src/element/anti_roll.cpp``, ``assemble_rotational_torque_forces``).
  Nothing here is pre-computed per sample and nothing here flips a sign;
* the slot set is the standardized one of the roadmap's 2.1 section:
  ``piston_area`` / ``effective_radius`` / ``friction_coeff`` / ``rotor_inertia``.
  The source document's own constants survive as this module's recorded defaults
  for the three that it states, so the arithmetic below is still the transcribed
  ``SFORCE/31-34`` shape;
* **which body reacts the couple is not decided here.**  The wheel end is the
  requiring side's own body and is passed in; the reaction end is read off a
  matched port's owner by
  :func:`~suspension_multibody.compilation.element_blocks.pair_torque_bodies`.
  No rule in this module reads a body name, which is what the assembly layer
  forbids (``EPIC.md:233``).

The magnitude is transcribed from the frozen Adams document
``artifacts/adams-fiala-handling/step_steer/adams_raw/handling_step_steer_dynamic.adm``
(``SFORCE/31-34``)::

    T = 2 * piston_area * share * demand * DEMAND_SCALE
        * friction_coeff * effective_radius

Two facts about that document are recorded rather than smoothed over:

* the constant ``0.1`` (this module's :data:`DEMAND_SCALE`) comes from the source
  document's demand scaling; its own provenance in the ``.sub`` files it was
  generated from could not be verified here -- it is an *unverified* constant,
  not a derived one.  It is a module constant and **not** a slot because the
  standardized slot set names the brake's physical parameters and this is the
  source document's normalization of its driver input;
* the document carries two effective piston radii (145.0 front, 130.0 rear),
  while the standardized set allows one.  This module takes the parameter the
  role defines and does not invent a second one; the rear-axle deviation that
  follows is a recorded deviation, not a hidden correction.

``share`` replaces the deleted ``front_brake_bias`` slot.  The front/rear split
is no longer a parameter *of the brake role*: with one element per wheel the
split is stated where the per-wheel demand is allocated, which is the caller's
argument here and the case document's brake signal in the preparation layer
(subtask p2-05).  A caller that reproduces the recorded 60/40 car passes
``share=0.6`` for a front wheel, and the recorded 17400 N*mm comes back exactly.

``rotor_inertia`` is the fourth standardized slot.  It is declared and carried
because the roadmap names it; the landed kernel law does not read an inertia (the
couple is applied to the wheel body the model already gives an inertia to), so
the slot is a declaration with a plausible default rather than a term of the
arithmetic above.  The default is derived and says so: a solid steel disc,
0.28 m outside diameter, 12 mm thick at 7850 kg/m^3, is 5.80 kg and
``0.5 * m * r^2`` is 0.0568 kg*m^2.

The authoritative arithmetic check for the pre-element form lives in
``raw/brake_torque_evidence.md`` in subtask 04's directory; the standardized-slot
form is checked by ``tests/subsystems/test_brake_subsystem.py``.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import cast

import numpy as np

from ..modeling.ports import PortSpec
from ..modeling.primitives.elements import RotationalTorqueParameters
from ..templates import (
    SubsystemInstance,
    Template,
    register,
)
from ..templates.builtin import BRAKE
from .assembly import assemble_from_template
from .types import WHEELS, ResolvedElement, SubsystemContext, SubsystemOutput

__all__ = [
    "BRAKE_REACTION_ROLE",
    "DEMAND_SCALE",
    "SIMPLIFIED_BRAKE",
    "SIMPLIFIED_BRAKE_NAME",
    "brake_amplitude",
    "build",
    "register_simplified",
    "role",
    "torque_parameters",
    "wheel_torque_element",
]

#: The role this subsystem implements.
role = "brake"

#: The name the simplified template registers under.  A detailed template is a
#: different name in the same registry, not a different code path.
SIMPLIFIED_BRAKE_NAME = "brake_4wdisk_simplified"

#: The requirement role the couple's reaction end comes from.
#:
#: A *role*, never a body name: the reaction body is the matched port's owner,
#: which is how a caliper bracket or a suspension's steerable carrier meets this
#: requirement without anything here knowing either name.  Declaring the matching
#: need on the template belongs to the composition path; measuring the file route
#: shows ``template_document_from`` does not write ``needs`` today, so a need
#: stated here would be dropped by the export/read round trip -- recorded in
#: ``raw/reaction_paths.md`` rather than papered over.
BRAKE_REACTION_ROLE = "brake_reaction"

#: The source document's demand scaling (``SFORCE/31-34``'s ``0.1``).
#:
#: A constant rather than a slot: see the module docstring.  Dropping it would
#: have scaled every recorded magnitude by ten, which is a physics change, not a
#: naming one.
DEMAND_SCALE = 0.1

#: The simplified brake's own slots.  These are the role's required slots with
#: the Adams simple-brake values as defaults, so the template carries its own
#: parameters rather than referring to a properties file.  The names are the
#: standardized set (roadmap 2.1); the values are the recorded ones
#: (``friction_coeff`` 0.4, ``effective_radius`` 145.0 mm, ``piston_area``
#: 2500 mm^2), and ``rotor_inertia``'s derivation is in the module docstring.

SIMPLIFIED_BRAKE = BRAKE


def register_simplified() -> Template:
    """
    Register the simplified brake template, replacing any previous one.

    Registration is the whole substitution mechanism: swapping this template for
    a detailed one is a `register` call under the same role, and nothing in the
    assembly layer changes.
    """
    return register(SIMPLIFIED_BRAKE, replace=True)


def build(instance: SubsystemInstance, context: SubsystemContext) -> SubsystemOutput:
    """
    Contribute the bodies this brake instance declares.

    The simplified template declares none, so this returns none -- but it returns
    them *through the shared role path*, so a detailed template that declares
    calipers gets them without this function changing.
    """
    return assemble_from_template(instance, context)


def brake_amplitude(
    parameters: Mapping[str, float], *, demand: float, share: float = 1.0
) -> float:
    """
    Return one braked wheel's torque magnitude for a stated demand and share.

    The arithmetic is the Adams ``SFORCE/31-34`` shape, evaluated in the source's
    own multiplication order so the recorded numbers reproduce exactly::

        T = 2 * piston_area * share * demand * DEMAND_SCALE
            * friction_coeff * effective_radius

    ``demand`` is the normalized brake input and must be within ``[0, 1]``; the
    range check names ``brake_input`` so a bad case file says which signal is
    wrong.  ``share`` is the wheel's part of one demand -- 1.0 for a wheel that
    takes the whole of it, the front axle's 0.6 for the recorded 60/40 car.

    No sign is applied: the couple's direction belongs to the element law, which
    takes it from the real-time relative rate about the wheel's spin axis.
    """
    input_value = float(demand)
    if input_value < 0.0 or input_value > 1.0:
        raise ValueError(
            f"brake_input must be normalized to [0, 1]; got {input_value!r}"
        )
    piston_area = float(parameters["piston_area"])
    friction_coeff = float(parameters["friction_coeff"])
    radius = float(parameters["effective_radius"])
    return (
        2.0
        * piston_area
        * float(share)
        * input_value
        * DEMAND_SCALE
        * friction_coeff
        * radius
    )


def torque_parameters(
    parameters: Mapping[str, float],
    *,
    wheel: str,
    demand: float,
    share: float = 1.0,
    axis_a: object | None = None,
) -> RotationalTorqueParameters:
    """
    Return the couple one braked wheel's element carries.

    ``stiffness`` is the magnitude at this demand and ``max_torque`` the
    magnitude at full demand, so the element law's own cap
    (``min(stiffness * demand, max_torque)``) can never bite below the demand the
    caller asked for -- and the full-demand figure is the recorded ``SFORCE``
    amplitude, which is what the cap means physically.

    ``axis_a`` is the wheel's spin axis **in the reaction body's frame**, because
    that is the frame the element law reads.  ``None`` means the body-frame y
    axis, which is the lateral axis for the built-in topology the recorded car
    reacts against; a model whose spin axis is not its body's y axis passes the
    axis explicitly -- the role requires a ``spin_axis`` mount for exactly this.

    ``wheel`` names the corner the element belongs to, and it is refused by name
    when it is not one of the four: an element named for a wheel the model does
    not have would apply its couple to whatever body the caller passed anyway.
    """
    if wheel not in WHEELS:
        raise ValueError(
            f"unknown wheel {wheel!r}; a brake element is named for one of the "
            f"four corner names {list(WHEELS)}"
        )
    amplitude = brake_amplitude(parameters, demand=demand, share=share)
    full_demand = brake_amplitude(parameters, demand=1.0, share=share)
    axis = (
        np.array([0.0, 1.0, 0.0])
        if axis_a is None
        else np.asarray(axis_a, dtype=float)
    )
    return RotationalTorqueParameters(
        stiffness=amplitude,
        max_torque=full_demand,
        axis_a=axis,
    )


def wheel_torque_element(
    parameters: Mapping[str, float],
    *,
    wheel: str,
    own_body: str,
    report: object,
    ports: Mapping[str, PortSpec],
    demand: float,
    share: float = 1.0,
    axis_a: object | None = None,
) -> ResolvedElement:
    """
    Return the brake torque element one wheel carries, as a declared row.

    ``own_body`` is the body the couple slows -- the wheel end, which the needing
    side states exactly as a ``LinkSpec`` states ``body_a``.  The reaction end is
    the matched port's owner, read by the pairing step, so a caliper bracket and
    a suspension's steerable carrier both meet this requirement without either
    name appearing here.

    The compilation step is imported inside the call rather than at module
    scope: ``compilation`` reaches ``rigs`` and ``studies`` through its own
    package import, and a subsystem that pulled that chain in at import time
    would make the composition of a bare axle pay for it.  The same lazy import
    the composition layer already uses for the matcher.
    """
    from ..compilation.element_blocks import pair_torque_bodies, torque_element_row

    pairing = pair_torque_bodies(
        name=f"brake_{wheel}",
        role=BRAKE_REACTION_ROLE,
        own_body=own_body,
        report=report,
        ports=ports,
    )
    element = torque_element_row(
        pairing,
        torque_parameters(
            parameters, wheel=wheel, demand=demand, share=share, axis_a=axis_a
        ),
    )
    return cast(ResolvedElement, element)
