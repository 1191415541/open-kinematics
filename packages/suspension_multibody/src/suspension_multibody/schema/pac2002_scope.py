"""
PAC2002 scope validation for the authoring schemas.

A tire whose coefficients request a feature the native kernel cannot run exactly
has to fail closed at the schema boundary: accepting it would produce a result
that silently omits the feature.  *Which* features those are is a property of
the kernel, so the answer comes from :mod:`suspension_multibody.kernel.capabilities`
and nothing is restated here.
"""

from __future__ import annotations

from collections.abc import Mapping

from ..kernel.capabilities import pac2002_unsupported_native_reasons

__all__ = ["validate_pac2002_native_scope"]


def validate_pac2002_native_scope(coefficients: Mapping[str, float]) -> None:
    """Raise if coefficients request Adams PAC2002 features native cannot run exactly."""
    reasons = pac2002_unsupported_native_reasons(coefficients)
    if reasons:
        raise ValueError("unsupported native PAC2002 scope: " + "; ".join(reasons))
