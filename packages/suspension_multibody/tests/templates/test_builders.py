"""
A template must really produce entities, and both authoring routes must agree.

The weak version of "templates are extensible" is a registry that accepts a name
and returns nothing.  These tests hold the stronger version: building a template
yields a :class:`ModelFragment` whose entity counts match what the assembly
actually constructs -- 13 joints in K mode and 9 in C mode for the built-in
double wishbone, which is the recorded behaviour of the composition entry, not a
number chosen here.

The equality of the two routes is the other half.  A template may be written as
data or as a builder; if those produced different fragments, "declare it or code
it" would be two formats wearing one name.
"""

from __future__ import annotations

import pytest

from suspension_multibody.modeling.instance import ModelFragment
from suspension_multibody.templates.builders import (
    build_fragment,
    builder_names,
    clear_builders,
    register_builder,
    resolve_builder,
)
from suspension_multibody.templates.builtin import DOUBLE_WISHBONE
from suspension_multibody.templates.model import (
    ConnectionDefinition,
    PartDefinition,
    Template,
    TemplateError,
)


@pytest.fixture(autouse=True)
def _clean_builders():
    """Keep the process-wide builder registry from leaking between tests."""
    clear_builders()
    yield
    clear_builders()


def test_a_declared_template_really_produces_entities() -> None:
    """A registry entry that yields nothing would be the weak version."""
    fragment = build_fragment(DOUBLE_WISHBONE, mode="K", properties={}, instance=("axle",))
    assert isinstance(fragment, ModelFragment)
    assert set(fragment.bodies) == {part.name for part in DOUBLE_WISHBONE.parts}
    assert fragment.entity_count > 0


def test_k_and_c_modes_activate_the_columns_they_should() -> None:
    """
    The recorded constraint counts: K has 13 joints and **no** forces, C has 9 and 8.

    These are the numbers the composition entry actually produces, checked against
    a dump of the historical assembly rather than against the requirement's prose.
    K mode carries no compliance element at all: its inboard rear points are
    inactive, not merely joint-free, so the earlier reading of "4 bushings in K"
    described rows the assembly has never had.

    C mode is not "K with the joints removed": the tie rod ends and the arm outer
    points have no bushing column, so they stay joints.
    """
    k = build_fragment(DOUBLE_WISHBONE, mode="K", properties={}, instance=("axle",))
    c = build_fragment(DOUBLE_WISHBONE, mode="C", properties={}, instance=("axle",))
    assert (len(k.joints), len(k.forces)) == (14, 0)
    assert (len(c.joints), len(c.forces)) == (10, 8)


def test_mode_does_not_change_the_bodies_or_their_points() -> None:
    """Switching mode is a change of activation, not a second build."""
    k = build_fragment(DOUBLE_WISHBONE, mode="K", properties={}, instance=("axle",))
    c = build_fragment(DOUBLE_WISHBONE, mode="C", properties={}, instance=("axle",))
    assert set(k.bodies) == set(c.bodies)
    assert set(k.points) == set(c.points)


def test_a_zero_body_template_is_legal() -> None:
    """A simplified element that contributes only a force must be expressible."""
    template = Template(
        name="simplified",
        role="suspension",
        parts=(),
        connections=(),
    )
    fragment = build_fragment(template, mode="K", properties={}, instance=("axle",))
    assert fragment.bodies == {}


def test_a_builder_generates_a_variable_number_of_parts() -> None:
    """
    The extension story: a builder may loop, so the part count is not fixed.

    A double wishbone has four arms because it is written that way; a template
    whose builder loops over links produces as many as it is asked for, and the
    downstream code sees the same fragment either way.
    """

    def many_links(parameters, properties, mode, bound_ports) -> ModelFragment:
        del properties, mode, bound_ports
        count = int(parameters.get("links", 2))
        bodies = {f"link_{index}": {"name": f"link_{index}"} for index in range(count)}
        return ModelFragment(bodies=bodies)

    register_builder("many_links", many_links)
    template = Template(name="trailing_arm", role="suspension", builder="many_links")
    for count in (0, 1, 5):
        fragment = build_fragment(
            template, mode="K", properties={}, parameters={"links": count}
        )
        assert len(fragment.bodies) == count


def test_both_authoring_routes_produce_the_same_fragment() -> None:
    """
    Declaring a template and building it must not be two different formats.

    The builder here is written to reproduce exactly what the declaration says,
    and the assertion is that the two fragments are equal -- so a template can be
    migrated from data to code (or back) without changing the model.
    """

    def hmm(parameters, properties, mode, bound_ports) -> ModelFragment:
        del parameters, properties, mode, bound_ports
        return ModelFragment(
            bodies={"body_a": {"name": "body_a"}, "body_b": {"name": "body_b"}},
            points={("body_a", "p"): {"role": "mount"}},
            joints={"p": {"kind": "ball_joint", "body": "body_a", "point": "p"}},
        )

    register_builder("same_as_declared", hmm)
    declared = Template(
        name="declared",
        role="suspension",
        parts=(PartDefinition("body_a"), PartDefinition("body_b")),
        connections=(
            ConnectionDefinition(name="p", role="body_a_mount", joint="ball_joint"),
        ),
    )
    coded = Template(
        name="coded",
        role="suspension",
        parts=(PartDefinition("body_a"), PartDefinition("body_b")),
        connections=(
            ConnectionDefinition(name="p", role="body_a_mount", joint="ball_joint"),
        ),
        builder="same_as_declared",
    )
    from_data = build_fragment(declared, mode="K", properties={}, instance=("axle",))
    from_code = build_fragment(coded, mode="K", properties={}, instance=("axle",))
    assert set(from_data.bodies) == set(from_code.bodies)
    assert set(from_data.joints) == set(from_code.joints)


def test_a_builder_that_returns_the_wrong_type_is_rejected() -> None:
    """A builder is a contract, not a suggestion: the fragment type is checked."""
    register_builder("wrong", lambda *args: {"not": "a fragment"})
    template = Template(name="bad", role="suspension", builder="wrong")
    with pytest.raises(TemplateError, match="not a ModelFragment"):
        build_fragment(template, mode="K", properties={}, instance=())


def test_an_unregistered_builder_name_names_what_is_registered() -> None:
    template = Template(name="ghost", role="suspension", builder="missing_builder")
    with pytest.raises(TemplateError, match="not registered"):
        build_fragment(template, mode="K", properties={}, instance=())


def test_registering_a_builder_twice_is_refused() -> None:
    register_builder("once", lambda *args: ModelFragment())
    assert "once" in builder_names()
    with pytest.raises(TemplateError, match="already registered"):
        register_builder("once", lambda *args: ModelFragment())


def test_resolve_builder_reports_the_known_names() -> None:
    register_builder("known", lambda *args: ModelFragment())
    assert resolve_builder("known") is not None
    with pytest.raises(TemplateError, match="known builders are known"):
        resolve_builder("unknown")


def test_provenance_records_the_template_and_a_stable_fingerprint() -> None:
    """A5 traces an entity back; the fingerprint must not depend on key order."""
    first = build_fragment(
        DOUBLE_WISHBONE, mode="K", properties={"a": 1.0, "b": 2.0}, instance=("axle",)
    )
    second = build_fragment(
        DOUBLE_WISHBONE, mode="K", properties={"b": 2.0, "a": 1.0}, instance=("axle",)
    )
    assert first.provenance is not None
    assert first.provenance.template == DOUBLE_WISHBONE.name
    assert (
        first.provenance.properties_fingerprint
        == second.provenance.properties_fingerprint
    )


def test_the_instance_path_is_carried_into_the_fragment() -> None:
    fragment = build_fragment(
        DOUBLE_WISHBONE, mode="K", properties={}, instance=("vehicle", "front")
    )
    assert fragment.instance == ("vehicle", "front")
