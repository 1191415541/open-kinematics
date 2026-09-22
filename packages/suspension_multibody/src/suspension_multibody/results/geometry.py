"""
The result side of the coordinate split.

A result-side conversion reads what the kernel reports -- a global wrench on a
body -- and expresses it in the body's own frame.  That is the report's job,
and ``report`` (07) must reach it without calling back into preparation: the
edge only ever points from results to the report, never the other way.

Beware two sizes of the same name.  The *result-side* transform that exists
today is ``wrench_global_to_local``, and the only live caller of it
(``api.py``'s element report) reads the same function the input assembly uses.
There is therefore one implementation, and it lives in
``preparation/geometry.py``; duplicating it here just to fill a directory would
be a second implementation of the same transform, which is exactly what the
boundary forbids.  ``wrench_local_to_global``, the mirror-image transform, has
no production caller at all: its only user was ``core/reactions.py``, which 08
deleted with the rest of the retired solver surface.

So this module is the boundary, not yet a home: it names the side, and the
report boundary 07 lands its converters here.  It deliberately re-exports
nothing -- a re-export would be the reverse ``results -> preparation`` edge the
architecture is trying to remove.
"""

__all__: tuple[str, ...] = ()
