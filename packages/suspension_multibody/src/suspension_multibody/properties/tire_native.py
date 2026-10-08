"""Compile resolved tire laws into the existing native parameter payload."""

from __future__ import annotations

import copy
from typing import Any

import numpy as np

from ..axle_dynamics.schema import PAC2002_PARAMETER_DEFAULTS, PAC2002_PARAMETER_NAMES


def compile_tires(
    rows: tuple[dict[str, Any], ...],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], bytes]:
    """Return native tires, indexed array descriptors and their binary payload."""
    tires = copy.deepcopy(list(rows))
    descriptors: list[dict[str, Any]] = []
    blob = bytearray()

    def append(values: Any) -> str:
        array = np.ascontiguousarray(values, dtype="<f8")
        index = len(descriptors)
        name = f"generic-tire-array-{index}"
        descriptors.append(
            {
                "name": name,
                "offset": len(blob),
                "length": array.nbytes,
                "dtype": "float64",
                "shape": list(array.shape),
                "order": "C",
            }
        )
        blob.extend(array.tobytes())
        return name

    for tire in tires:
        parameters = tire["parameters"]
        length_scale = parameters.pop("_coefficient_length_scale", .001)
        coefficients = parameters.pop("pac2002_coefficients", {})
        fiala = parameters.pop("fiala_parameters", {})
        tables = parameters.pop("pac2002_tables", {})
        if tire["model"] != "native_brush":
            values = [
                float(coefficients.get(name, PAC2002_PARAMETER_DEFAULTS[name]))
                for name in PAC2002_PARAMETER_NAMES
            ]
            if "BOTTOMING_RADIUS" in coefficients:
                values[PAC2002_PARAMETER_NAMES.index("BOTTOMING_RADIUS")] *= length_scale
            if tire["model"] == "fiala":
                head = [
                    fiala.get("CSLIP", 1000),
                    fiala.get("CALPHA", 800),
                    fiala.get("CGAMMA", 0),
                    0,
                    fiala.get("USE_MODE", 2),
                    fiala.get("UMIN", 0.9),
                    fiala.get("UMAX", 1),
                    fiala.get("RELAX_LENGTH_X", parameters["longitudinal_relaxation_length"]/length_scale) * length_scale,
                    fiala.get("RELAX_LENGTH_Y", parameters["lateral_relaxation_length"]/length_scale) * length_scale,
                    fiala.get("WIDTH", 235) * length_scale,
                    fiala.get("ROLLING_RESISTANCE", 0) * length_scale,
                    fiala.get("LOW_SPEED_THRESHOLD", 0.001),
                    0,
                    0,
                ]
                values[: len(head)] = head
            parameters["blob"] = append(values)
        for field, key in (
            ("deflection_curve", "deflection_load_curve"),
            ("bottoming_curve", "bottoming_curve"),
        ):
            if tables.get(key):
                parameters[field] = append(tables[key])
    return tires, descriptors, bytes(blob)
