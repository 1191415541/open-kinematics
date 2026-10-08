"""Neutral TIR coefficients, SI tables and tire model conversion."""
from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Literal, Mapping

from ..schema.dynamic import TireModelSpec

_ADAMS_NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[EeDd][-+]?\d+)?"


_LENGTH_TO_MM: dict[str, float] = {
    "m": 1_000.0,
    "meter": 1_000.0,
    "meters": 1_000.0,
    "mm": 1.0,
    "millimeter": 1.0,
    "millimeters": 1.0,
    "cm": 10.0,
    "centimeter": 10.0,
    "centimeters": 10.0,
    "in": 25.4,
    "inch": 25.4,
    "inches": 25.4,
}


_FORCE_TO_N: dict[str, float] = {
    "n": 1.0,
    "newton": 1.0,
    "newtons": 1.0,
    "kn": 1_000.0,
    "kilonewton": 1_000.0,
    "kilonewtons": 1_000.0,
    "lbf": 4.4482216152605,
    # Long spelling shipped by four stock tires (AA_small_*_relax,
    # AA_small_trr64_rim, mdi_pac94).  Without it _unit_factor raises and those
    # files fail to import at all instead of being converted or rejected.
    "pound_force": 4.4482216152605,
}


_TIME_TO_S: dict[str, float] = {
    "s": 1.0,
    "sec": 1.0,
    "second": 1.0,
    "seconds": 1.0,
    "ms": 1.0e-3,
    "millisecond": 1.0e-3,
    "milliseconds": 1.0e-3,
}


_ANGLE_TO_RAD: dict[str, float] = {
    "rad": 1.0,
    "radian": 1.0,
    "radians": 1.0,
    "deg": math.pi / 180.0,
    "degree": math.pi / 180.0,
    "degrees": math.pi / 180.0,
}


def _parse_text_units(text: str) -> dict[str, str]:
    """读取 Adams 文本文件中的单位段，并保留源文件声明."""
    units: dict[str, str] = {}
    in_units = False
    for line in text.splitlines():
        stripped = line.strip()
        upper = stripped.upper()
        if upper in {"[UNITS]", "UNITS/"}:
            in_units = True
            continue
        if not in_units:
            continue
        if stripped.startswith(("[", "$", "!", "(")):
            break
        for match in re.finditer(
            r"\b(LENGTH|FORCE|ANGLE|MASS|TIME)\s*=\s*['\"]?([^'\"$,\s]+)",
            line,
            re.IGNORECASE,
        ):
            units[match.group(1).lower()] = match.group(2).strip().lower()
    return units


def _unit_factor(
    unit: str | None,
    factors: Mapping[str, float],
    default: float,
) -> float:
    """将一个 Adams 单位转换为目标单位的倍率；未知单位必须显式失败."""
    if unit is None:
        return default
    normalized = unit.strip().lower()
    try:
        return factors[normalized]
    except KeyError as exc:
        raise ValueError(f"unsupported Adams unit: {unit!r}") from exc


def tire_model_spec(
    coefficients: Mapping[str, float],
    *,
    kind: Literal["pac2002", "native_brush", "fiala"],
    tables: Mapping[str, tuple[tuple[float, float], ...]] | None = None,
) -> TireModelSpec:
    """Convert shared PAC2002 values to either source or native proxy data."""
    radius = float(coefficients.get("UNLOADED_RADIUS_MM", 344.0))
    nominal_load = max(
        float(coefficients.get("FNOMIN_N", coefficients.get("FNOMIN", 4_850.0))),
        1e-9,
    )
    longitudinal_stiffness = abs(
        float(coefficients.get("PKX1", 22.303)) * nominal_load
    )
    cornering_stiffness = abs(
        float(coefficients.get("PKY1", -21.92)) * nominal_load
    )
    kwargs: dict[str, object] = {
        "kind": kind,
        "parameter_source": "adams_builtin",
        "unloaded_radius": radius,
        "maximum_compression": 0.99 * radius,
        "vertical_stiffness": float(
            coefficients.get("VERTICAL_STIFFNESS_N_MM", 210.0)
        ),
        "vertical_damping": float(
            coefficients.get("VERTICAL_DAMPING_N_S_MM", 0.05)
        ),
        "cornering_stiffness": cornering_stiffness,
        "longitudinal_stiffness": longitudinal_stiffness,
        # The native ABI has one friction coefficient for both brush axes.
        # Use the lower nominal PAC2002 peak and keep this limitation explicit
        # in the source-equivalence manifest.
        "friction_coefficient": min(
            abs(float(coefficients.get("PDX1", 1.0))),
            abs(float(coefficients.get("PDY1", 1.0))),
        ),
        "pneumatic_trail": float(coefficients.get("QDZ1", 0.0935)) * radius,
        "pac2002_coefficients": dict(coefficients),
        "pac2002_tables": dict(tables or {}),
    }
    if kind == "fiala":
        kwargs["kind"] = "fiala"
        kwargs["fiala_parameters"] = {
            "CSLIP": float(coefficients.get("CSLIP_N", coefficients.get("CSLIP", 1000.0))),
            "CALPHA": float(coefficients.get("CALPHA_N_PER_RAD", coefficients.get("CALPHA", 800.0))),
            "UMIN": float(coefficients.get("UMIN", 0.9)),
            "UMAX": float(coefficients.get("UMAX", 1.0)),
            "RELAX_LENGTH_X": float(coefficients.get("RELAX_LENGTH_X_MM", 50.0)),
            "RELAX_LENGTH_Y": float(coefficients.get("RELAX_LENGTH_Y_MM", 150.0)),
            "WIDTH": float(coefficients.get("WIDTH_MM", 235.0)),
            "ROLLING_RESISTANCE": float(
                coefficients.get(
                    "ROLLING_RESISTANCE_MM",
                    coefficients.get("ROLLING_RESISTANCE", 0.0),
                )
            ),
            # [MODEL] USE_MODE: smoothing (2, 12) and slip transient (11, 12).
            "USE_MODE": float(coefficients.get("USE_MODE", 2.0)),
        }
    kwargs.update(
        {
            # PAC 纯滑移路径不把该量用于轮胎力，但底层仍保留两个
            # 衰减状态，因此需要一个与源参数同量纲的正值。
            "relaxation_length": min(
                abs(float(coefficients.get("PTX1", 2.3657))) * radius,
                abs(float(coefficients.get("PTY1", 2.1439))) * radius,
            ),
            "detached_relaxation_s": 0.05,
        }
    )
    return TireModelSpec(**kwargs)


_SOURCE_FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[Ee][-+]?\d+)?"


def _bracket_sections(text: str) -> list[tuple[str, str]]:
    """按 Adams 方括号段落拆分文本，并保留同名段落."""
    sections: list[tuple[str, str]] = []
    current: str | None = None
    lines: list[str] = []
    for line in text.splitlines():
        match = re.match(r"^\s*\[([^]]+)\]\s*$", line)
        if match:
            if current is not None:
                sections.append((current, "\n".join(lines)))
            current = match.group(1).strip().upper()
            lines = []
        elif current is not None:
            lines.append(line)
    if current is not None:
        sections.append((current, "\n".join(lines)))
    return sections


def _source_fields(block: str) -> dict[str, str]:
    """读取 Adams 段落中的键值字段."""
    return {
        key.upper(): value.strip().strip("'").strip()
        for key, value in re.findall(
            r"^\s*([A-Za-z][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$",
            block,
            re.MULTILINE,
        )
    }


def parse_tire_tables_text(text: str) -> dict[str, tuple[tuple[float, float], ...]]:
    """
    Read a tire's tabulated curves in SI units.

    ``[DEFLECTION_LOAD_CURVE]`` and ``[BOTTOMING_CURVE]`` are two-column tables
    (vertical deflection, load) in the file's declared units.  The kernel needs SI and
    the tables cannot ride in the scalar coefficient payload, so they are returned
    separately: the ABI carries them as per-tire arrays.  Both curves are part of
    USE_MODE-independent vertical behaviour, which is why they are parsed here rather
    than in the mode-specific code.
    """
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    force_scale = _unit_factor(units.get("force"), _FORCE_TO_N, 1.0)
    sections = {name: block for name, block in _bracket_sections(text)}
    tables: dict[str, tuple[tuple[float, float], ...]] = {}
    for section, name in (
        ("DEFLECTION_LOAD_CURVE", "deflection_load_curve"),
        ("BOTTOMING_CURVE", "bottoming_curve"),
    ):
        rows = _source_numeric_table(sections.get(section, ""))
        if rows:
            tables[name] = tuple(
                (deflection*length_scale*1.0e-3, load*force_scale)
                for deflection, load in rows
            )
    return tables


def _source_numeric_table(block: str) -> tuple[tuple[float, float], ...]:
    """读取曲线段中的前两列数值."""
    rows: list[tuple[float, float]] = []
    pattern = re.compile(rf"^\s*({_SOURCE_FLOAT})\s+({_SOURCE_FLOAT})")
    for line in block.splitlines():
        match = pattern.match(line)
        if match:
            rows.append((float(match.group(1)), float(match.group(2))))
    return tuple(rows)


def _adams_float(value: str) -> float:
    """Parse an Adams number, including the Fortran ``D`` exponent marker."""
    return float(value.replace("D", "E").replace("d", "e"))


def parse_tire_text(text: str) -> dict[str, float]:
    """Parse coefficients and feature switches using the source unit conventions."""
    units = _parse_text_units(text)
    length_scale = _unit_factor(units.get("length"), _LENGTH_TO_MM, 1.0)
    force_scale = _unit_factor(units.get("force"), _FORCE_TO_N, 1.0)
    time_scale = _unit_factor(units.get("time"), _TIME_TO_S, 1.0)
    values: dict[str, float] = {}
    for key, raw in re.findall(
        rf"^\s*([A-Z][A-Z0-9_]*)\s*=\s*({_ADAMS_NUMBER})",
        text,
        re.MULTILINE,
    ):
        values[key] = _adams_float(raw)
    sections = {name: block for name, block in _bracket_sections(text)}
    model_fields = _source_fields(sections.get("MODEL", ""))
    for key in ("USE_MODE", "VXLOW", "LONGVL", "FE_METHOD"):
        raw = model_fields.get(key)
        if raw is not None and re.fullmatch(_ADAMS_NUMBER, raw):
            values[key] = _adams_float(raw)
    property_format = model_fields.get("PROPERTY_FILE_FORMAT", "").upper()
    if property_format:
        values["PROPERTY_FILE_FORMAT_PAC2002"] = float("PAC2002" in property_format)
        if "PAC-MC" in property_format or "PACMC" in property_format:
            values["PAC2002_UNSUPPORTED_PAC_MC"] = 1.0
    tyre_side = model_fields.get("TYRESIDE", "").upper()
    if tyre_side.startswith("R"):
        values["TYRESIDE_RIGHT"] = 1.0
        values["USE_MODE"] = -abs(values.get("USE_MODE", 14.0))
    elif tyre_side.startswith("L"):
        values["TYRESIDE_LEFT"] = 1.0
        values["USE_MODE"] = abs(values.get("USE_MODE", 14.0))
    def truthy_field(name: str) -> bool:
        raw = model_fields.get(name, "").strip().strip("'").upper()
        return raw not in {"", "0", "FALSE", "NO", "NONE", "OFF"}
    if truthy_field("BELT_DYNAMICS"):
        values["PAC2002_UNSUPPORTED_BELT_DYNAMICS"] = 1.0
    if truthy_field("LOCAL_SOLVER"):
        values["PAC2002_UNSUPPORTED_LOCAL_SOLVER"] = 1.0
    # The Maxwell element must fail closed.  Its enable switch is a quoted
    # string that the numeric extractor above cannot see, while
    # DYNAMIC_STIFFNESS and DYNAMIC_DAMPING are numbers it does copy into
    # ``values``.  None of the three reaches the C++ parameter array, so
    # without this flag the tire would be accepted and solved as if the
    # element were absent.  Adams documents the switch in the [VERTICAL]
    # section, so both sections are checked.
    vertical_fields = _source_fields(sections.get("VERTICAL", ""))
    maxwell_switch = (
        vertical_fields.get("USE_DYNAMIC_STIFFNESS")
        or model_fields.get("USE_DYNAMIC_STIFFNESS")
        or ""
    )
    if maxwell_switch.strip().strip("'").upper() not in {
        "",
        "0",
        "FALSE",
        "NO",
        "NONE",
        "OFF",
    }:
        values["PAC2002_UNSUPPORTED_DYNAMIC_STIFFNESS"] = 1.0
        # Carry the switch itself into the ABI as well, so the kernel can see that
        # the element was requested once it is implemented.  Until then the flag
        # above still fails the tire closed, so this changes nothing user-facing.
        values["USE_DYNAMIC_STIFFNESS"] = 1.0
    contact_model = model_fields.get("CONTACT_MODEL", "").strip().strip("'").upper()
    if contact_model and contact_model not in {"0", "POINT_FOLLOWER"}:
        values["PAC2002_UNSUPPORTED_CONTACT_MODEL"] = 1.0
    # FITTYP selects an alternative (legacy) rolling-resistance formulation.
    # Adams documents that FITTYP=5 determines rolling resistance differently and
    # that removing the keyword switches to the [ROLLING_COEFFICIENTS] equations.
    # The native kernel implements only the latter.  The keyword is absent from
    # every tire in the installed Adams libraries, so rather than implement an
    # unexercised legacy path this fails closed: silently applying the modern
    # formula to a FITTYP=5 tire would misstate rolling resistance.
    fit_type = model_fields.get("FITTYP", "").strip().strip("'").upper()
    if fit_type not in {"", "0"}:
        values["PAC2002_UNSUPPORTED_FITTYP"] = 1.0
    if abs(values.get("FE_METHOD", 0.0)) > 1.0e-12:
        values["PAC2002_UNSUPPORTED_FE_METHOD"] = 1.0
    def mark_curve(section: str, count_key: str, flag_key: str) -> None:
        """
        Record a curve's point count, and fail it closed only if asked to.

        An empty ``flag_key`` means the curve is implemented, so the count is purely
        diagnostic and no rejection flag is written.
        """
        rows = _source_numeric_table(sections.get(section, ""))
        if rows:
            values[count_key] = float(len(rows))
            if flag_key:
                values[flag_key] = 1.0
    # [DEFLECTION_LOAD_CURVE] is implemented: the kernel interpolates the table with a
    # monotone cubic and uses it as the vertical spring law (see
    # pac2002_curve_interpolate).  The point count is still recorded for diagnostics,
    # but the tire is no longer rejected for carrying the section.
    mark_curve("DEFLECTION_LOAD_CURVE", "DEFLECTION_LOAD_CURVE_POINT_COUNT", "")
    # Wheel bottoming is implemented: the kernel adds the rim reaction from
    # [BOTTOMING_CURVE] once the deflection reaches UNLOADED_RADIUS - BOTTOMING_RADIUS,
    # exactly as Adams documents Fz = min(0, Fzk+Fzc) + min(0, Fzrim).  The point count
    # stays for diagnostics and the tire is no longer rejected for carrying it.
    mark_curve("BOTTOMING_CURVE", "BOTTOMING_CURVE_POINT_COUNT", "")
    if "UNLOADED_RADIUS" in values:
        values["UNLOADED_RADIUS_MM"] = values["UNLOADED_RADIUS"] * length_scale
    if "FNOMIN" in values:
        values["FNOMIN_N"] = values["FNOMIN"] * force_scale
    if "VERTICAL_STIFFNESS" in values:
        values["VERTICAL_STIFFNESS_N_MM"] = (
            values["VERTICAL_STIFFNESS"] * force_scale / length_scale
        )
    if "VERTICAL_DAMPING" in values:
        values["VERTICAL_DAMPING_N_S_MM"] = (
            values["VERTICAL_DAMPING"] * force_scale * time_scale / length_scale
        )
    if "WIDTH" in values:
        values["WIDTH_MM"] = values["WIDTH"] * length_scale
    if "CALPHA" in values:
        values["CALPHA_N_PER_RAD"] = (
            values["CALPHA"] * force_scale
            / (length_scale / length_scale)
            / _unit_factor(units.get("angle"), _ANGLE_TO_RAD, 1.0)
        )
    if "CSLIP" in values:
        values["CSLIP_N"] = values["CSLIP"] * force_scale
    if "RELAX_LENGTH_X" in values:
        # Adams Fiala examples express relaxation length in metres even when
        # the surrounding tire file declares millimetres.
        values["RELAX_LENGTH_X_MM"] = values["RELAX_LENGTH_X"] * 1000.0
    if "RELAX_LENGTH_Y" in values:
        values["RELAX_LENGTH_Y_MM"] = values["RELAX_LENGTH_Y"] * 1000.0
    if "ROLLING_RESISTANCE" in values:
        # A length, like WIDTH: Adams' property reader converts it with the file's
        # length unit and forms Ty = -sign(omega)*STEP(...)*RR*Fz.  Carried in mm
        # so the vehicle path can scale it to metres alongside WIDTH.
        values["ROLLING_RESISTANCE_MM"] = values["ROLLING_RESISTANCE"] * length_scale
    values.setdefault("SPRING_STIFFNESS_N_MM", 125.0)
    values.setdefault("SPRING_FREE_LENGTH_MM", 300.0)
    return values


def parse_tire(path: Path) -> dict[str, float]:
    """Read coefficients from a TIR file."""
    return parse_tire_text(path.read_text(encoding="ascii", errors="replace"))

def parse_tire_tables(path: Path) -> dict[str, tuple[tuple[float, float], ...]]:
    """Read tabulated vertical laws from a TIR file in SI units."""
    return parse_tire_tables_text(path.read_text(encoding="ascii", errors="replace"))
