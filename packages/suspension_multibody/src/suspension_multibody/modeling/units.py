"""
Units, in one place.

The internal model is SI: metres, kilograms, seconds, newtons.  The *schemas*
users write are millimetres, because that is how suspension geometry is
published and how every hardpoint table in the field is written.  The conversion
therefore happens exactly once, at the boundary where a user document becomes a
model, and nothing downstream converts again.

Keeping the factor here rather than spelling ``1e-3`` at each call site is not
tidiness for its own sake: a second conversion somewhere in the middle of the
pipeline is invisible in a result (the numbers stay plausible, they are just
wrong by 1000) and it is exactly the kind of mistake a unit boundary exists to
make impossible.  ``A5`` checks the SI claim by constructing the same physical
input through two schemas and comparing the models; that only means something if
there is one place the factor can be wrong.
"""

from __future__ import annotations

__all__ = [
    "KG_PER_TONNE",
    "METRES_PER_MILLIMETRE",
    "N_PER_KN",
    "to_kilograms",
    "to_metres",
    "to_newtons",
]

#: Millimetres to metres.  The schema's length unit versus the model's.
METRES_PER_MILLIMETRE = 1e-3

#: Kilograms to tonnes, for the mass values some vehicle tables publish.
KG_PER_TONNE = 1e3

#: Kilonewtons to newtons.
N_PER_KN = 1e3


def to_metres(millimetres: float) -> float:
    """Convert a schema length to the model's SI length."""
    return float(millimetres) * METRES_PER_MILLIMETRE


def to_kilograms(tonnes: float) -> float:
    """Convert a tonne value to kilograms."""
    return float(tonnes) * KG_PER_TONNE


def to_newtons(kilonewtons: float) -> float:
    """Convert a kilonewton value to newtons."""
    return float(kilonewtons) * N_PER_KN
