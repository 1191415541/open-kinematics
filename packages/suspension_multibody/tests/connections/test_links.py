"""
Pairings become physical rows, and a composition that states none is unchanged.

The matcher decides which offered port satisfies which requirement; these tests
are about the other half of that decision -- the part that *builds* the joint or
the bushing.  Three properties are asserted, because they are the three ways this
can go wrong:

1. a recipe the match cannot fill fails by name (role, port, candidates), rather
   than producing a model with a connection missing;
2. an explicit pairing and an inferred one produce the *same* entities, which is
   what makes the pairing an override rather than a second assembly path;
3. a contribution with no recipe produces no rows at all, so every assembly that
   never mentioned a link keeps exactly the entities it had.
"""

from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.connections.links import (
    LinkSpec,
    build_links,
    explicit_bindings_from_pairings,
)
from suspension_multibody.connections.matcher import (
    AmbiguousBindingError,
    BindingError,
)
from suspension_multibody.modeling.identity import EntityId
from suspension_multibody.modeling.ports import GeometryPort, PortRequirement
from suspension_multibody.modeling.primitives import RigidBody
from suspension_multibody.modeling.primitives.spatial import SE3
from suspension_multibody.subsystems.composition import (
    SubsystemContribution,
    compose_simulation_assembly,
    fingerprint_assembly,
)
from suspension_multibody.subsystems.types import SubsystemOutput

INSTANCE = ("axle",)

#: The body the body-side entry carries.  Deliberately *not* ``chassis``: the
#: pairing has to work on a name no rule in the assembly knows, or it is not a
#: pairing but a lookup table with extra steps.
CHASSIS_BODY = "subframe"

Point = tuple[float, float, float]


def _output(bodies: tuple[str, ...], points: dict[tuple[str, str], Point]):
    return SubsystemOutput(
        bodies={name: RigidBody(name=name, mass=1.0) for name in bodies},
        points={key: np.asarray(value, dtype=float) for key, value in points.items()},
    )


def _port(local: str, role: str, owner: str, *, offset: Point = (0.0, 0.0, 0.0)):
    """One geometric port on ``owner``, offering ``role`` from a local frame."""
    return GeometryPort(
        id=EntityId(INSTANCE, local),
        owner=EntityId(INSTANCE, owner),
        role=role,
        pose=SE3(
            translation=np.asarray(offset, dtype=float),
            quaternion=np.array([1.0, 0.0, 0.0, 0.0], dtype=float),
        ),
    )


def _chassis(*, ports: tuple[str, ...] = ("mount",)) -> SubsystemContribution:
    """Return the body-side contribution: one body, a port per name it is offered under."""
    return SubsystemContribution(
        role="chassis",
        output=_output((CHASSIS_BODY,), {(CHASSIS_BODY, "centre"): (0.0, 0.0, 0.5)}),
        ports={name: _port(name, "mount", CHASSIS_BODY) for name in ports},
    )


def _suspension(
    *, links: tuple[LinkSpec, ...] = (), requirement_role: str = "mount"
) -> SubsystemContribution:
    """Return the needing side: one body, one requirement, and how to join it."""
    return SubsystemContribution(
        role="suspension",
        output=_output(("upright_L",), {("upright_L", "mount"): (0.1, -0.7, 0.3)}),
        ports={},
        needs=(PortRequirement(role=requirement_role),),
        links=links,
    )


def _mount_link(**overrides) -> LinkSpec:
    fields = {
        "role": "mount",
        "kind": "weld",
        "body_a": "upright_L",
        "point_a_local": (0.1, -0.7, 0.3),
        "point_a_label": "mount",
    }
    fields.update(overrides)
    return LinkSpec(**fields)


def test_a_composition_that_states_no_recipe_builds_no_rows() -> None:
    composed = compose_simulation_assembly([_chassis(), _suspension()])
    assert composed.generated["links"] == ()
    assert composed.assembly.fragment.joints == {}
    assert composed.assembly.fragment.bushings == {}
    assert composed.assembly.fragment.connections == {}


def test_a_weld_recipe_becomes_a_weld_joint_between_the_two_bodies() -> None:
    composed = compose_simulation_assembly(
        [_chassis(), _suspension(links=(_mount_link(),))]
    )
    rows = composed.generated["links"]
    assert len(rows) == 1
    row = rows[0]
    assert row.kind == "weld"
    assert row.row_type == "WeldJoint"
    assert row.body_a == "upright_L"
    assert row.body_b == CHASSIS_BODY
    assert row.point_a_label == "mount"
    assert row.point_b_label == "mount"
    assert row.point_a == pytest.approx((0.1, -0.7, 0.3))
    # The joint is in both columns -- active and ideal -- because a link is an
    # ideal constraint, and a K reading of a C assembly still has to see it.
    assert set(composed.assembly.fragment.joints) == {row.row_name}
    assert set(composed.assembly.fragment.ideal_constraints) == {row.row_name}
    connection = composed.assembly.fragment.connections[row.row_name]
    assert connection.kind == "ideal"
    assert (connection.body_a, connection.body_b) == ("upright_L", CHASSIS_BODY)


def test_the_explicit_pairing_and_the_inferred_one_agree() -> None:
    """A pairing is an override, not a second assembly path."""
    recipe = _mount_link()
    inferred = compose_simulation_assembly([_chassis(), _suspension(links=(recipe,))])
    stated = compose_simulation_assembly(
        [_chassis(), _suspension(links=(recipe,))], pairings={"mount": "mount"}
    )
    assert inferred.bindings == stated.bindings
    assert inferred.generated["links"] == stated.generated["links"]
    assert fingerprint_assembly(inferred.assembly) == fingerprint_assembly(
        stated.assembly
    )
    assert sorted(inferred.assembly.fragment.joints) == sorted(
        stated.assembly.fragment.joints
    )
    assert (
        inferred.assembly.fragment.points.keys() == stated.assembly.fragment.points.keys()
    )


def test_a_pairing_works_on_a_body_name_no_rule_knows() -> None:
    """``subframe`` is in no hardcoded set; the pairing is what makes it work."""
    composed = compose_simulation_assembly(
        [_chassis(), _suspension(links=(_mount_link(),))], pairings={"mount": "mount"}
    )
    row = composed.generated["links"][0]
    assert row.body_b == CHASSIS_BODY
    assert CHASSIS_BODY not in {"chassis", "ground"}


def test_an_ambiguous_requirement_is_refused_until_the_pairing_says_which() -> None:
    two_ports = _chassis(ports=("mount_left", "mount_right"))
    with pytest.raises(AmbiguousBindingError) as caught:
        compose_simulation_assembly([two_ports, _suspension(links=(_mount_link(),))])
    assert "mount_left" in str(caught.value)
    assert "mount_right" in str(caught.value)
    # Naming one settles it, and the link follows the pairing rather than the
    # order the ports happened to be offered in.
    composed = compose_simulation_assembly(
        [_chassis(ports=("mount_left", "mount_right")), _suspension(links=(_mount_link(),))],
        pairings={"mount": "mount_right"},
    )
    assert composed.generated["links"][0].port_id.endswith("mount_right")


def test_a_pairing_that_names_no_offered_port_is_refused_by_name() -> None:
    with pytest.raises(BindingError) as caught:
        compose_simulation_assembly(
            [_chassis(), _suspension(links=(_mount_link(),))],
            pairings={"mount": "somewhere_else"},
        )
    message = str(caught.value)
    assert "somewhere_else" in message
    assert "mount" in message


def test_a_recipe_whose_requirement_was_never_bound_is_refused_by_role() -> None:
    with pytest.raises(BindingError) as caught:
        compose_simulation_assembly(
            [_chassis(), _suspension(links=(_mount_link(role="damper"),))]
        )
    assert "damper" in str(caught.value)


def test_a_revolute_recipe_carries_the_axis_each_body_states() -> None:
    composed = compose_simulation_assembly(
        [
            _chassis(),
            _suspension(
                links=(
                    _mount_link(
                        kind="revolute",
                        axis_a=(0.0, 1.0, 0.0),
                        axis_b=(0.0, 1.0, 0.0),
                    ),
                )
            ),
        ]
    )
    row = composed.generated["links"][0]
    assert row.row_type == "RevoluteJoint"
    joint = composed.assembly.fragment.joints[row.row_name]["constraint"]
    assert np.allclose(np.asarray(joint.axis_a, dtype=float), (0.0, 1.0, 0.0))


def test_a_bushing_recipe_becomes_a_compliance_row_not_a_joint() -> None:
    composed = compose_simulation_assembly(
        [
            _chassis(),
            _suspension(
                links=(_mount_link(kind="bushing", stiffness=tuple(np.eye(6).tolist())),)
            ),
        ]
    )
    row = composed.generated["links"][0]
    assert row.row_type == "BushingElement"
    assert row.row_name in composed.assembly.fragment.bushings
    assert composed.assembly.fragment.joints == {}
    connection = composed.assembly.fragment.connections[row.row_name]
    assert connection.kind == "bushing"


def test_pairing_names_are_rendered_into_port_ids_once() -> None:
    port = _port("mount", "mount", CHASSIS_BODY)
    ports = {str(port.id): port}
    rendered = explicit_bindings_from_pairings(
        {"mount": "mount"}, ports, instance=INSTANCE
    )
    assert rendered == {"mount": str(EntityId(INSTANCE, "mount"))}
    with pytest.raises(BindingError) as caught:
        explicit_bindings_from_pairings({"mount": "nope"}, ports, instance=INSTANCE)
    assert "nope" in str(caught.value)


def test_build_links_without_recipes_returns_an_empty_fragment() -> None:
    rows = build_links((), {}, (), instance=INSTANCE)
    assert rows.rows == ()
    assert rows.fragment.joints == {}
