"""
Family-owned preparation implementations for the unified simulation lifecycle.

Each module in this package owns exactly one assembly/family pair: it assembles
the domain inputs, normalises units and input signals, and returns the context
its family compiler consumes.  A module authors no physics of its own and never
submits native.

The modules deliberately do not import one another, and this package imports
none of them: ``simulation.preparation.default_preparation_registry`` holds the
lazy import table and pulls in exactly one family module when that family is
prepared.  Key enumeration, document bypass and prepared-simulation reuse
therefore stay free of family imports.
"""

__all__: tuple[str, ...] = ()
