"""
Stable entity identity.

An entity is named by *where it came from* and *what it is inside that place*,
never by its index in a list or its display name.  A list index changes when an
unrelated body is inserted; a display name can be edited for presentation.  Both
would silently re-point a reference at a different entity, which is the failure
mode this type exists to make impossible.

The path is the instance route -- the chain of instance names from the root
assembly down to the instance that owns the entity, e.g.
``("front_axle", "L", "suspension")``.  The local id is stable within that
instance and is chosen by whoever declares the entity.  Together they form the
identity that crosses instance boundaries.
"""

from __future__ import annotations

from dataclasses import dataclass

#: Separator used by ``__str__``.  Chosen because it cannot appear in an
#: instance name (those are Python-ish identifiers), so a rendered id parses
#: back unambiguously.
_SEPARATOR = "/"


@dataclass(frozen=True, order=True)
class EntityId:
    """An entity's stable identity: its instance path plus a local id."""

    path: tuple[str, ...]
    local: str

    def __post_init__(self) -> None:
        if not isinstance(self.path, tuple):
            raise TypeError("EntityId.path must be a tuple of instance names")
        if not self.local:
            raise ValueError("EntityId.local must be a non-empty string")
        for step in self.path:
            if not step:
                raise ValueError("EntityId.path must not contain an empty step")
        if _SEPARATOR in self.local:
            raise ValueError(
                f"EntityId.local must not contain {_SEPARATOR!r}: {self.local!r}"
            )

    @classmethod
    def root(cls, local: str) -> EntityId:
        """Identify an entity owned by the root assembly itself."""
        return cls((), local)

    def child(self, instance: str, local: str) -> EntityId:
        """Identify an entity inside a named nested instance."""
        if not instance:
            raise ValueError("instance name must be a non-empty string")
        return EntityId((*self.path, instance), local)

    def under(self, *instances: str) -> EntityId:
        """Prefix this identity with one or more instance names."""
        if not instances:
            return self
        return EntityId((*instances, *self.path), self.local)

    def __str__(self) -> str:
        return _SEPARATOR.join((*self.path, self.local))

    def belongs_to(self, path: tuple[str, ...]) -> bool:
        """Return whether this entity sits at or below ``path``."""
        return self.path[: len(path)] == path


def qualified(*parts: str) -> EntityId:
    """
    Build an id from a rendered chain, the last part being the local id.

    Convenience for callers that hold the path pieces in hand; it exists so
    they do not reach into the dataclass fields positionally.
    """
    if not parts:
        raise ValueError("qualified() needs at least a local id")
    return EntityId(tuple(parts[:-1]), parts[-1])


__all__ = ["EntityId", "qualified"]
