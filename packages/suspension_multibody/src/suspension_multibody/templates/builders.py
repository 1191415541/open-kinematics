"""
The builder registry: how a template that carries Python resolves its builder.

A template may be authored two ways, and both must produce the *same*
:class:`~suspension_multibody.modeling.instance.ModelFragment`:

* by **declaring** its parts, connections, slots and outputs -- pure data, which
  is what an expert who does not want to write Python uses, and what
  ``builtin.py`` does today; or
* by **naming a builder** -- a registered callable that walks a loop and
  generates the topology, which is what a template with a variable number of
  links or arms needs.

The template document stores only the builder's *name*.  Storing the callable
would make the document unserialisable and would smuggle executable behaviour
into the model contract, which is exactly what ``DESIGN.md`` forbids.  The
registry maps the name back to the function at instantiation time.

Both routes end where the declarations end, so the validation is shared: a
builder's fragment is checked by the same code that checks a declared one.
"""

from __future__ import annotations

import hashlib
from collections.abc import Mapping
from dataclasses import replace
from typing import Any

from ..modeling.instance import FragmentProvenance, ModelFragment
from .instantiate import activated_column
from .model import Builder, TemplateError
from .ports import need_to_requirement

__all__ = [
    "build_fragment",
    "builder_names",
    "clear_builders",
    "resolve_builder",
    "register_builder",
]


#: Builder name -> callable.  Process-wide, like the template registry.
_BUILDERS: dict[str, Builder] = {}


def register_builder(name: str, builder: Builder) -> Builder:
    """
    Register a builder under ``name``.

    Re-registering a name is an error: two experts registering ``wishbone`` and
    silently getting one of the two is the failure this registry exists to
    prevent, and a test author who needs a clean slate calls ``clear_builders``.
    """
    if not name:
        raise TemplateError("a builder must carry a name")
    if not callable(builder):
        raise TemplateError(f"builder {name!r} is not callable")
    if name in _BUILDERS:
        raise TemplateError(
            f"builder {name!r} is already registered; pick another name or clear "
            "the registry"
        )
    _BUILDERS[name] = builder
    return builder


def resolve_builder(name: str) -> Builder:
    """Return a registered builder, naming the unknown one if it is missing."""
    try:
        return _BUILDERS[name]
    except KeyError as exc:
        known = ", ".join(sorted(_BUILDERS)) or "(none registered)"
        raise TemplateError(
            f"template names builder {name!r}, which is not registered; known "
            f"builders are {known}"
        ) from exc


def builder_names() -> tuple[str, ...]:
    """Return the registered builder names in a stable order."""
    return tuple(sorted(_BUILDERS))


def clear_builders() -> None:
    """Empty the builder registry.  For tests that need a known starting state."""
    _BUILDERS.clear()


def build_fragment(
    template,
    *,
    mode: str,
    properties: Mapping[str, float],
    parameters: Mapping[str, Any] | None = None,
    instance: tuple[str, ...] = (),
    bound_ports: Mapping[str, Any] | None = None,
) -> ModelFragment:
    """
    Produce the fragment a template contributes, by either authoring route.

    A template with a builder calls it; a template with only declarations is
    expanded by :func:`declared_fragment`.  Both return a ``ModelFragment`` and
    both are then checked the same way, so "the builder must really create
    entities" is enforced structurally rather than asserted in a comment.

    The builder receives read-only inputs -- resolved properties, the active
    mode, and the ports bound so far -- and may not read another subsystem's
    private state.  That is what keeps two instances independent and lets the
    same template be mounted twice.
    """
    if template.builder:
        builder = resolve_builder(template.builder)
        fragment = builder(
            dict(parameters or {}),
            dict(properties),
            mode,
            dict(bound_ports or {}),
        )
        if not isinstance(fragment, ModelFragment):
            raise TemplateError(
                f"builder {template.builder!r} of template {template.name!r} "
                f"returned {type(fragment).__name__}, not a ModelFragment"
            )
    else:
        fragment = declared_fragment(template, mode=mode, properties=properties)

    provenance = FragmentProvenance(
        template=template.name,
        revision=template.suspension_kind,
        instance=instance,
        properties_fingerprint=fingerprint_properties(properties),
    )
    mounted = fragment.mounted(instance)
    if mounted.provenance is not None:
        # A builder that recorded its own provenance keeps it: it knows more
        # about what it generated than this layer does.
        return mounted
    return replace(mounted, provenance=provenance)


def declared_fragment(
    template, *, mode: str, properties: Mapping[str, float]
) -> ModelFragment:
    """
    Expand a data-only template into a fragment.

    This is the route that lets a template be written without Python.  Every
    declared part becomes a body, every connection becomes a point on the body
    it attaches to, and the joint/bushing columns are resolved for ``mode`` by
    the same :func:`~suspension_multibody.templates.instantiate.activated_column`
    rule the instance layer uses -- one rule, one place.
    """
    bodies: dict[str, Any] = {}
    points: dict[tuple[str, str], Any] = {}
    joints: dict[str, Any] = {}
    forces: dict[str, Any] = {}

    for part in template.parts:
        bodies[part.name] = {
            "name": part.name,
            "mass": float(part.mass),
            "fixed": bool(part.fixed),
        }

    for connection in template.connections:
        # A connection's point sits on the part its role names; that is the whole
        # meaning of `role` here, and it is why the declaration carries no
        # coordinates: the assembly resolves the role against a real model.
        owner = _owner_for(template, connection)
        points[(owner, connection.name)] = {"role": connection.role}
        column = activated_column(connection, mode)
        if column == "joint" and connection.joint:
            joints[connection.name] = {
                "kind": connection.joint,
                "body": owner,
                "point": connection.name,
            }
        elif column == "bushing" and connection.bushing:
            forces[connection.name] = {
                "kind": "bushing",
                "body": owner,
                "point": connection.name,
                "slot": _slot_name_for(template, connection),
            }

    return ModelFragment(
        bodies=bodies,
        points=points,
        joints=joints,
        forces=forces,
        outputs={output.name: output for output in template.outputs},
        ports={port.name: port for port in template.ports},
        requirements=tuple(
            need_to_requirement(need)
            for need in template.needs
        ),
    )


def _owner_for(template, connection) -> str:
    """
    Return the part a connection's point is attached to.

    The role names the hardpoint; the part is the first declared part whose name
    the role refers to.  A template that names its parts after its mounts -- as
    the built-in double wishbone does -- resolves here without extra metadata;
    anything else falls back to the first part, which keeps a purely geometric
    connection addressable rather than failing the build.
    """
    for part in template.parts:
        if part.name and part.name in connection.role:
            return part.name
    return template.parts[0].name if template.parts else ""


def _slot_name_for(template, connection) -> str:
    """Return the property slot feeding a connection's bushing column."""
    for slot in template.property_slots:
        if connection.name in slot.connections:
            return slot.name
    return connection.bushing or ""


def fingerprint_properties(properties: Mapping[str, float]) -> str:
    """
    Fingerprint resolved properties so a template instance is reproducible.

    Sorted and formatted rather than hashed through ``repr`` of a dict, because
    the fingerprint has to be identical for two runs that supplied the same
    values in a different insertion order -- otherwise "same input, same model"
    would depend on how a caller happened to build its mapping.
    """
    canonical = "|".join(
        f"{name}={float(value)!r}" for name, value in sorted(properties.items())
    )
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16]
