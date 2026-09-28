"""
Element property documents: springs, dampers and bump stops.

A property file is where an expert writes a constitutive law once and a subsystem
or assembly refers to it.  The file is *not* part of a template's topology: a
subsystem may repoint ``property_bindings`` from a linear law to a nonlinear one
without changing which bodies, joints or elements exist, and the model's topology
hash must not move when it does.

Two boundaries this module keeps:

* the document is validated against the bundled schema *and* against the model's
  own element classes, so "stiffness must be positive" has one home rather than
  two that can disagree;
* a loaded document yields an explicit ``resolved`` mapping of kernel-facing
  parameters, so the solve path consumes a law, not a free-form dict.
"""

from __future__ import annotations

import copy
import hashlib
import math
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Sequence

from suspension_contracts import (
    ContractError,
    canonical_json_bytes,
    validate_element_properties,
)

from ..properties.load import ENTRY_KINDS
from .errors import ElementPropertyError

__all__ = [
    "ELEMENT_MODELS",
    "ElementPropertyDocument",
    "ElementPropertyError",
]

#: The constitutive models a property file may declare, and what each one means.
#:
#: ``linear`` carries numbers in ``parameters``; ``nonlinear`` and ``piecewise``
#: carry a ``curve``.  The distinction is recorded rather than inferred from
#: curve means the threshold softens the curve, and reading it as one or the
#: other would change the force.
ELEMENT_MODELS: dict[str, str] = {
    "linear": "parameters",
    "nonlinear": "curve",
    "piecewise": "curve",
}

#: Which parameter name carries the scalar stiffness or damping of each element
#: type, which of those must be strictly positive, and the model class whose
#: bounds the file therefore obeys.
_SCALAR_FIELD: dict[str, tuple[str, bool]] = {
    "spring": ("stiffness", True),
    "damper": ("viscous_damping", True),
    # A mount bushing's one number is its translational stiffness.  The six-axis
    # form needs a matrix, which is why `matrix` exists as its own section: without
    # it a mount could only be this scalar form, whose rotational diagonals are
    # zero, and a compliant assembly built from a file would be a mechanism.
    "bushing": ("stiffness", True),
    # A tire's number is its vertical stiffness; its unloaded radius rides beside it
    # in `parameters`, because a tire needs both to be a tire.
    "tire": ("stiffness", True),
    # ``BumpStop.stiffness`` is ``ge=0``: a stop that is declared and currently
    # carries no force is a real state, and refusing it here would be a second,
    # stricter rule than the model's own.
    "bump_stop": ("stiffness", False),
}


def _hash(value: Any) -> str:
    return hashlib.sha256(canonical_json_bytes(value)).hexdigest()


def _read(path: Path) -> dict[str, Any]:
    import json

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ElementPropertyError(f"{path}: cannot read document: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ElementPropertyError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ElementPropertyError(f"{path}: document root must be an object")
    return payload


def _check_finite(value: Any, path: str) -> None:
    """Reject a non-finite number anywhere in a parameter or curve payload."""
    if isinstance(value, bool) or value is None:
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise ElementPropertyError(f"{path}: numeric value must be finite")
        return
    if isinstance(value, Mapping):
        for key, child in value.items():
            _check_finite(child, f"{path}.{key}")
        return
    if isinstance(value, Sequence) and not isinstance(value, str):
        for index, child in enumerate(value):
            _check_finite(child, f"{path}[{index}]")


@dataclass(frozen=True)
class ElementPropertyDocument:
    """One loaded constitutive law for one element type."""

    path: Path
    payload: dict[str, Any]
    #: Kernel-facing parameters, already converted to the reference units.
    resolved: Mapping[str, Any]

    @classmethod
    def load(
        cls,
        path: str | Path,
        *,
        expected_type: str | None = None,
        allowed_models: Sequence[str] | None = None,
    ) -> "ElementPropertyDocument":
        """
        Read, validate and resolve one property file.

        ``expected_type`` is the element type the *referring* template declares, so
        a spring slot bound to a damper file fails where the binding is written.
        ``allowed_models`` is the template slot's declared model list, checked with
        the same intent.
        """
        target = Path(path).resolve()
        payload = _read(target)
        try:
            validate_element_properties(payload)
        except ContractError as exc:
            raise ElementPropertyError(f"{target}: {exc}") from exc

        element_type = str(payload["element_type"])
        model = str(payload["model"])
        if expected_type is not None and element_type != expected_type:
            raise ElementPropertyError(
                f"{target}: expected element_type {expected_type!r}, "
                f"got {element_type!r}"
            )
        if allowed_models is not None and model not in allowed_models:
            raise ElementPropertyError(
                f"{target}: model {model!r} is not allowed here; "
                f"allowed models are {sorted(allowed_models)}"
            )

        _check_finite(payload.get("parameters", {}), f"{target}: parameters")
        curve = payload.get("curve")
        if curve is not None:
            _check_finite(curve["points"], f"{target}: curve.points")
            points = [(float(x), float(y)) for x, y in curve["points"]]
            if any(right[0] <= left[0] for left, right in zip(points, points[1:])):
                raise ElementPropertyError(
                    f"{target}: curve independent values must be strictly increasing"
                )

        needs = ELEMENT_MODELS[model]
        if needs == "curve" and curve is None:
            raise ElementPropertyError(
                f"{target}: model {model!r} requires a 'curve' object"
            )
        if (
            needs == "parameters"
            and not payload.get("parameters")
            and payload.get("matrix") is None
        ):
            raise ElementPropertyError(
                f"{target}: model 'linear' requires a non-empty 'parameters' object "
                "or a 'matrix'"
            )

        resolved = _resolve(target, element_type, model, payload)
        # Bounds are the model's own: the resolved parameters are handed to the
        # element class the solver will construct, so "a stiffness is positive"
        # has one home rather than a second copy here that could drift from it.
        _check_with_model_class(target, element_type, resolved)
        return cls(target, copy.deepcopy(payload), MappingProxyType(resolved))

    @property
    def name(self) -> str:
        return str(self.payload["name"])

    @property
    def element_type(self) -> str:
        return str(self.payload["element_type"])

    @property
    def model(self) -> str:
        return str(self.payload["model"])

    @property
    def content_hash(self) -> str:
        """Hash of the file's own content: the law, not its binding."""
        return _hash(self.payload)

    @property
    def effective_values_hash(self) -> str:
        """Hash of the resolved kernel-facing values this law produces."""
        return _hash(dict(self.resolved))

    def curve_points(self) -> tuple[tuple[float, float], ...]:
        """Return the curve samples, or an empty tuple for a linear law."""
        curve = self.payload.get("curve")
        if curve is None:
            return ()
        return tuple((float(x), float(y)) for x, y in curve["points"])


#: The model class a property file's element type is checked against.  The file
#: speaks of a `bushing` -- the element the assembly builds -- while the model's
#: own entry table calls the six-axis form `bushing6x6`, so the two spellings are
#: translated here rather than in every reader.
_MODEL_CLASS_KIND: dict[str, str] = {"bushing": "bushing6x6"}


def _check_with_model_class(
    path: Path, element_type: str, resolved: Mapping[str, Any]
) -> None:
    """
    Run the resolved law through the model's own element class.

    The classes live in ``properties/load.py``'s entry table (``ENTRY_KINDS``) so
    the bounds a property file obeys are the same ones the model's inline numbers
    obey -- including the ones that are not simple bounds: a spring needs exactly
    one length definition, and a file that declares a curve without one leaves the
    kernel with no reference length, which is a missing model rather than a
    formatting problem.  Placement fields are placeholders because a property file
    carries numbers and not geometry.
    """
    kind = _MODEL_CLASS_KIND.get(element_type, element_type)
    model_class = ENTRY_KINDS.get(kind)
    if model_class is None:
        raise ElementPropertyError(
            f"{path}: no model class is registered for element type {element_type!r}"
        )
    from pydantic import ValidationError

    fields = set(model_class.model_fields)
    payload: dict[str, Any] = {
        key: value for key, value in resolved.items() if key in fields
    }
    # A mount's single number is the isotropic six-axis mount the assembly builds
    # from it: the three translational diagonals the runtime fills, and nothing else.
    # Validating the scalar *as* that matrix keeps the file's bounds the model's
    # bounds rather than a second set invented here.  A file that states the table
    # itself is already in the shape the class wants.
    if kind == "bushing6x6" and isinstance(payload.get("stiffness"), float):
        payload["stiffness"] = _isotropic_mount(payload["stiffness"])
    # Placeholders for the fields that are placement rather than law: a file carries
    # numbers, not geometry, and only the fields the model actually has are filled,
    # so a spec without a name or a body (a tire) is not handed one.
    for name in ("name", "body_a", "body_b"):
        if name in fields:
            payload.setdefault(name, "validation")
    for name in ("point_a", "point_b", "contact_point"):
        if name in fields:
            payload.setdefault(name, {"x": 0.0, "y": 0.0, "z": 0.0})
    try:
        model_class.model_validate(payload)
    except ValidationError as exc:
        raise ElementPropertyError(
            f"{path}: the {element_type} law was rejected by the "
            f"{kind} model: {_first_problem(exc)}"
        ) from exc


def _isotropic_mount(stiffness: float) -> tuple[tuple[float, ...], ...]:
    """Return the six-axis mount a single stiffness stands for."""
    return tuple(
        tuple(stiffness if row == column and row < 3 else 0.0 for column in range(6))
        for row in range(6)
    )


def _first_problem(exc: Any) -> str:
    errors = exc.errors()
    if not errors:
        return str(exc)
    first = errors[0]
    location = ".".join(str(part) for part in first.get("loc", ()))
    return f"{location}: {first.get('msg', 'invalid')}"


def _resolve(
    path: Path,
    element_type: str,
    model: str,
    payload: Mapping[str, Any],
) -> dict[str, Any]:
    """
    Convert a validated law into the parameters the element build path consumes.

    The result is what ``subsystems/element_build.py`` hands the kernel: a scalar
    stiffness or damping for a linear law, a six-axis matrix for a mount, plus the
    curve when the file declares one.  A linear law keeps its declared numbers and
    carries no curve, so a later swap to a nonlinear file changes these values and
    only these values.
    """
    resolved: dict[str, Any] = {
        "element_type": element_type,
        "model": model,
        "units": dict(payload["units"]),
        "source": str(path),
    }
    parameters = dict(payload.get("parameters", {}))
    for key, value in parameters.items():
        if not isinstance(value, (int, float)) or isinstance(value, bool):
            raise ElementPropertyError(
                f"{path}: parameter {key!r} must be a number, found "
                f"{type(value).__name__}"
            )
        resolved[key] = float(value)

    # A matrix is the one shape `parameters` cannot state -- a mount's six axes and
    # a wheel's inertia are tables, not numbers -- so it rides in its own section,
    # naming the parameter it fills.  Without it, a mount could only ever be the
    # scalar form, whose rotational diagonals are zero, and a compliant assembly
    # built from a file would be a mechanism.
    matrix = payload.get("matrix")
    if matrix is not None:
        rows = tuple(tuple(float(item) for item in row) for row in matrix["rows"])
        widths = {len(row) for row in rows}
        if len(widths) != 1:
            raise ElementPropertyError(
                f"{path}: matrix {matrix['name']!r} has rows of differing length "
                f"{sorted(widths)}"
            )
        resolved[str(matrix["name"])] = rows

    points = tuple((float(x), float(y)) for x, y in payload.get("curve", {}).get("points", ()))
    if points:
        resolved["force_curve"] = points

    scalar, must_be_positive = _SCALAR_FIELD[element_type]
    if scalar not in resolved:
        if not points:
            raise ElementPropertyError(
                f"{path}: {element_type} law provides neither a {scalar!r} "
                "parameter nor a curve; the solve path has no force to apply"
            )
        # A pure curve law still needs a positive stiffness for the kernel's
        # parameter block: the curve supplies the shape and this supplies the
        # slope the kernel uses before its own interpolation takes over.
        slope = _first_slope(points)
        if slope <= 0.0:
            raise ElementPropertyError(
                f"{path}: {element_type} curve must have a positive initial "
                f"slope, found {slope!r}"
            )
        resolved[scalar] = slope
    if isinstance(resolved[scalar], tuple):
        # The matrix *is* the law, so there is no single number to bound here: the
        # model's own class checks its shape, and a mount whose numbers are all zero
        # is a mount that carries nothing -- a legal state the class is free to
        # accept, so no second rule is invented for it.
        return resolved
    value = float(resolved[scalar])
    if must_be_positive and value <= 0.0:
        raise ElementPropertyError(
            f"{path}: {element_type} {scalar} must be positive, found {value!r}"
        )
    if not must_be_positive and value < 0.0:
        raise ElementPropertyError(
            f"{path}: {element_type} {scalar} must not be negative, found {value!r}"
        )
    return resolved


def _first_slope(points: Sequence[tuple[float, float]]) -> float:
    """Return the secant slope of the first curve interval."""
    (x0, y0), (x1, y1) = points[0], points[1]
    if x1 == x0:
        return 0.0
    return (y1 - y0) / (x1 - x0)
