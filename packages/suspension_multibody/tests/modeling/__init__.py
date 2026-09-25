"""
The low modelling layer: identity, fragments, assemblies, ports and units.

These tests hold the layer to the promises the rest of the architecture is built
on, so they are written against the *contract* rather than against a caller:

* an entity keeps its identity across a rebuild, so a reference cannot silently
  re-point at a different entity;
* a fragment is immutable data and refuses a collision rather than resolving it
  by last-write-wins;
* an assembly keeps its nested parts addressable instead of flattening them;
* the unit boundary has exactly one factor, in one place.
"""
