"""
The reporting-side assembly of force-element wrenches.

The element *declarations* moved to ``modeling/primitives/elements.py``, where
they sit beside the joint and body declarations.  What is left here is
``evaluate_generalized_forces``: the one place that walks a set of elements,
evaluates each of them in Python and expresses the total per body.  That is a
recomputation of constitutive behaviour the native kernel already answered, so
this package exists only until ``api.py`` reads the native element-wrench channel
instead -- it is not a home for new code.
"""

from .assembly import evaluate_generalized_forces

__all__ = ["evaluate_generalized_forces"]
