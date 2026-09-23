"""
The template registry.

Experts register templates; users refer to them by name.  Registration is where a
template's role contract is first checked, so a malformed template fails at the
point it is written rather than when someone tries to assemble a vehicle with it.
"""

from __future__ import annotations

from .model import Template, TemplateError

__all__ = [
    "clear",
    "get",
    "names",
    "register",
    "registered",
]


#: The registry itself.  Module-level because templates are process-wide
#: declarations, like the joint table.
_REGISTRY: dict[str, Template] = {}


def register(template: Template, *, replace: bool = False) -> Template:
    """
    Register a template by name.

    Checks the role contract first: a template that does not satisfy its role is
    rejected here, naming the missing mount or slot.  Re-registering an existing
    name is an error unless `replace` is set, so a silent overwrite cannot hide
    one expert's template behind another's.
    """
    if not template.name:
        raise TemplateError("a template must carry a name")
    try:
        template.check_role_contract()
    except TemplateError:
        raise
    existing = _REGISTRY.get(template.name)
    if existing is not None and not replace:
        raise TemplateError(
            f"template {template.name!r} is already registered "
            f"(role {existing.role!r}); pass replace=True to overwrite"
        )
    _REGISTRY[template.name] = template
    return template


def get(name: str) -> Template:
    """Return a registered template, naming the unknown one if it is missing."""
    try:
        return _REGISTRY[name]
    except KeyError as exc:
        known = ", ".join(sorted(_REGISTRY)) or "(none registered)"
        raise TemplateError(
            f"template {name!r} is not registered; known templates are {known}"
        ) from exc


def names() -> tuple[str, ...]:
    """Return the registered template names in a stable order."""
    return tuple(sorted(_REGISTRY))


def registered() -> dict[str, Template]:
    """Return a copy of the registry, for iteration and assertions."""
    return dict(_REGISTRY)


def clear() -> None:
    """Empty the registry.  For tests that need a known starting state."""
    _REGISTRY.clear()
