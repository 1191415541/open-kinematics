r"""
Decode Adams help MathJax SVG equations into readable text.

Adams documents its tire equations as MathJax SVGs whose glyphs are ``<path>``
outlines.  The files carry no ``<title>``, no MathML annotation and no alt text,
so the formulas look opaque -- but each ``<use>`` references a glyph id that
encodes the character (``MJMAIN-3D`` is ``=``, ``MJMATHI-3BA`` is lambda).  This
reconstructs the formula as text from those references.

Validated against a known equation: decoding ``Equation3919.svg`` reproduces the
rolling-resistance moment law that ``axle_kernel.cpp`` already implements, and
``--survey`` reports **zero unmapped character slots across all 7193 equation
images** Adams 2025.1.1 ships, with the seven stretchy-glyph pieces listed
separately as structural.

Structural elements (fraction bars, radicals, absolute-value bars, ``\left``
brackets) are separate ``<path>``/``<rect>``/``<use>`` elements.  The flat
character stream cannot express them, so it silently flattens grouping: it reads
``Equation3981.svg`` as ``1 1 + kappa' sqrt{...}``, which cannot say whether the
radical sits under the fraction's bar or multiplies it -- and the two readings are
different formulas.  ``--tree`` resolves that class of ambiguity: it walks the
transform hierarchy and prints every glyph with its absolute coordinates and scale.
The outermost group carries MathJax's ``matrix(1 0 0 -1 0 0)`` flip, so the printed
``y`` is already screen-oriented: **a numerator has y<0, a denominator y>0, a
subscript y>0 at reduced scale, and a fraction or radical bar shows up as a
``rect`` whose width reveals what it covers.**  Use ``--tree`` whenever the flat
reading admits more than one grouping; use ``--survey`` when a ``?`` appears, to
tell a missing letter from a lost bar.

Usage::

    uv run --package suspension-multibody python \
        packages/suspension_multibody/scripts/decode_adams_equations.py \
        Equation3952.svg Equation3953.svg
    uv run --package suspension-multibody python \
        packages/suspension_multibody/scripts/decode_adams_equations.py \
        --tree Equation3981.svg

With no arguments it decodes a small built-in sample.  Prefer the prose and
tables in the help pages for naming and units.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
import xml.etree.ElementTree as ElementTree
from collections.abc import Iterable, Iterator
from typing import NamedTuple

# TeX math-font slots whose Unicode differs from the code point.
#
# The map has to cover every Greek letter the tire equations use: an unmapped slot
# decodes as ``?``, and before sigma/rho/zeta/mu/epsilon were added the turn-slip
# relaxation equations read ``?c=a(1-theta-?)`` and ``theta=Ky0/3?yFy`` -- ambiguous
# enough that the formula could not be transcribed.  ASCII letters need no entry
# because ``chr(code)`` already yields them; ``.tmp``-style surveys over a set of
# equations find any remaining gaps.
TEX_TO_UNICODE = {
    0x0394: "\u0394",  # Delta
    0x0398: "\u0398",  # Theta
    0x03A0: "\u03a0",  # Pi
    0x03A3: "\u03a3",  # Sigma
    0x03A5: "\u03a5",  # Upsilon
    0x03A6: "\u03a6",  # Phi
    0x03A8: "\u03a8",  # Psi
    0x03A9: "\u03a9",  # Omega
    0x03B1: "\u03b1",  # alpha
    0x03B2: "\u03b2",  # beta
    0x03B3: "\u03b3",  # gamma
    0x03B4: "\u03b4",  # delta
    0x03B5: "\u03b5",  # epsilon (lunate)
    0x03B6: "\u03b6",  # zeta
    0x03B7: "\u03b7",  # eta
    0x03B8: "\u03b8",  # theta
    0x03BA: "\u03ba",  # kappa
    0x03BB: "\u03bb",  # lambda
    0x03BC: "\u03bc",  # mu
    0x03BD: "\u03bd",  # nu
    0x03BE: "\u03be",  # xi
    0x03C0: "\u03c0",  # pi -- the parking-moment block uses pi/2 factors
    0x03C1: "\u03c1",  # rho
    0x03C3: "\u03c3",  # sigma
    0x03C4: "\u03c4",  # tau
    0x03C5: "\u03c5",  # upsilon
    0x03C6: "\u03c6",  # phi
    0x03C7: "\u03c7",  # chi
    0x03C8: "\u03c8",  # psi
    0x03C9: "\u03c9",  # omega
    0x03D5: "\u03d5",  # phi symbol -- MathJax's slot for \phi, next to 3C6
    0x03D6: "\u03d6",  # pi symbol (varpi)
    0x03F5: "\u03f5",  # epsilon (closed)
    0x2202: "\u2202",  # partial
    0x2212: "\u2212",  # minus
    0x221A: "\u221a",  # radical
    0x22C5: "\u22c5",  # centered dot
}

# Glyphs that are *parts* of a stretchy character rather than characters: the
# vertical strokes MathJax's size-4 font uses to build a tall radical, and the
# horizontal pieces for long over/under-bars.  Decoding them to text would invent
# symbols, and letting them fall through to ``?`` would report a structural piece
# as a missing letter -- the exact confusion ``--survey`` exists to prevent.
STRUCTURAL_GLYPH_SLOTS = frozenset(
    {
        "MJSZ4-E000",
        "MJSZ4-E001",
        "MJSZ4-E150",
        "MJSZ4-E151",
        "MJSZ4-E152",
        "MJSZ4-E153",
        "MJSZ4-E154",
    }
)

DEFAULT_EQUATIONS_DIR = pathlib.Path(
    r"G:\MSC.Software\Adams\2025_1_1\help\GeneratedImages\Equations"
)
# Documented nonlinear-transient contact-mass dynamics (eqs 20-22).
SAMPLE = ("Equation3952.svg", "Equation3953.svg", "Equation3954.svg")

_XLINK_HREF = "{http://www.w3.org/1999/xlink}href"
_GLYPH_ID = re.compile(r"^([A-Za-z0-9]+)-([0-9A-Fa-f]+)$")
_TRANSFORM_TOKEN = re.compile(r"([a-zA-Z]+)\(([^)]*)\)")

# An affine map ``(a, b, c, d, e, f)``: ``x' = a*x + c*y + e``, ``y' = b*x + d*y + f``.
Affine = tuple[float, float, float, float, float, float]
IDENTITY: Affine = (1.0, 0.0, 0.0, 1.0, 0.0, 0.0)


def _multiply(parent: Affine, child: Affine) -> Affine:
    """Return the affine map that applies ``child`` and then ``parent``."""
    pa, pb, pc, pd, pe, pf = parent
    ca, cb, cc, cd, ce, cf = child
    return (
        pa * ca + pc * cb,
        pb * ca + pd * cb,
        pa * cc + pc * cd,
        pb * cc + pd * cd,
        pa * ce + pc * cf + pe,
        pb * ce + pd * cf + pf,
    )


def parse_transform(value: str | None) -> Affine:
    """Return the affine map described by an SVG ``transform`` attribute."""
    if not value:
        return IDENTITY
    result = IDENTITY
    for name, arguments in _TRANSFORM_TOKEN.findall(value):
        numbers = [float(item) for item in re.split(r"[\s,]+", arguments.strip()) if item]
        if name == "translate":
            tx = numbers[0] if numbers else 0.0
            ty = numbers[1] if len(numbers) > 1 else 0.0
            step: Affine = (1.0, 0.0, 0.0, 1.0, tx, ty)
        elif name == "scale":
            sx = numbers[0] if numbers else 1.0
            sy = numbers[1] if len(numbers) > 1 else sx
            step = (sx, 0.0, 0.0, sy, 0.0, 0.0)
        elif name == "matrix" and len(numbers) == 6:
            step = (numbers[0], numbers[1], numbers[2], numbers[3], numbers[4], numbers[5])
        else:
            continue
        result = _multiply(result, step)
    return result


def _href(element: ElementTree.Element) -> str | None:
    """Return the ``href`` of a ``<use>`` element, whichever namespace it uses."""
    for key, value in element.attrib.items():
        if key == "href" or key.endswith("}href") or key == _XLINK_HREF:
            return value
    return None


def _glyph(code: str, family: str) -> str:
    r"""
    Return the character for one glyph reference, or ``?`` when unmapped.

    ``MJMAIN``/``MJSZ`` slots are Unicode already (``0x2032`` is the prime of
    ``phi'``, ``0x2223`` the tall dividing bar of ``\left|``), so an unmapped
    slot still decodes through ``chr``.  ``MJMATHI`` is the math-italic alphabet,
    where an unmapped non-ASCII slot really is unknown and must stay ``?`` so a
    missing Greek letter cannot be mistaken for a formula.  Stretchy-glyph pieces
    decode to the empty string: they are strokes, not characters.
    """
    if f"{family}-{code}" in STRUCTURAL_GLYPH_SLOTS:
        return ""
    point = int(code, 16)
    mapped = TEX_TO_UNICODE.get(point)
    if mapped is not None:
        return mapped
    character = chr(point)
    if character.isprintable() and (family != "MJMATHI" or point < 0x7F):
        return character
    return "?"


def _walk(
    element: ElementTree.Element,
    transform: Affine,
    depth: int,
    rows: list[str],
    pieces: list[str],
) -> None:
    """Append one transcript row per glyph, and one flat character per glyph."""
    step = _multiply(transform, parse_transform(element.get("transform")))
    a, b, c, d, e, f = step
    tag = element.tag.rsplit("}", 1)[-1]
    indent = "  " * depth
    if tag in ("svg", "g", "defs"):
        if tag != "defs":  # glyph outlines carry no layout information
            rows.append(f"{indent}<{tag}>")
            for child in element:
                _walk(child, step, depth + 1, rows, pieces)
        return
    if tag == "rect":
        x = float(element.get("x", "0"))
        y = float(element.get("y", "0"))
        width = float(element.get("width", "0"))
        height = float(element.get("height", "0"))
        rows.append(
            f"{indent}[bar x={a * x + c * y + e:.0f} y={b * x + d * y + f:.0f} "
            f"w={a * width:.0f} h={height:.0f}]"
        )
        return
    if tag != "use":
        return
    href = _href(element)
    if href is None or not href.startswith("#"):
        return
    match = _GLYPH_ID.match(href[1:])
    if match is None:
        return
    family, code = match.groups()
    local_x = float(element.get("x", "0"))
    local_y = float(element.get("y", "0"))
    x = a * local_x + c * local_y + e
    y = b * local_x + d * local_y + f
    scale = abs(a)
    character = _glyph(code, family)
    pieces.append(character)
    rows.append(
        f"{indent}{character!r} at x={x:.0f} y={y:.0f} scale={scale:.3f}"
    )


def decode(svg_path: pathlib.Path) -> str:
    """Return the equation text encoded by a MathJax SVG."""
    raw = svg_path.read_text(encoding="utf-8", errors="replace")
    defined = {f"{family}-{code}" for family, code in re.findall(
        r'<path id="([A-Za-z0-9]+)-([0-9A-Fa-f]+)"', raw
    )}
    pieces: list[str] = []
    for family, code in re.findall(
        r'<use[^>]*?(?:xlink:)?href="#([A-Za-z0-9]+)-([0-9A-Fa-f]+)"[^>]*?/>', raw
    ):
        if f"{family}-{code}" not in defined:
            continue
        pieces.append(_glyph(code, family))
    return "".join(pieces)


def transcript(svg_path: pathlib.Path) -> Iterator[str]:
    """Yield the transform-aware layout of a MathJax SVG, one row per element."""
    root = ElementTree.parse(svg_path).getroot()
    rows: list[str] = []
    pieces: list[str] = []
    _walk(root, IDENTITY, 0, rows, pieces)
    yield f"flat: {''.join(pieces)}"
    yield from rows


class SurveyResult(NamedTuple):
    """Per-glyph-slot use counts, split by what a slot means."""

    unmapped: dict[str, int]
    structural: dict[str, int]


def survey(
    equations_dir: pathlib.Path, paths: Iterable[pathlib.Path] | None = None
) -> SurveyResult:
    """
    Count the glyph slots in the given equations, split by category.

    ``unmapped`` lists slots that still decode as ``?``: a real gap, because an
    unlisted Greek letter prints ``?`` exactly like a lost fraction bar.  An empty
    ``unmapped`` set is the only honest basis for "every character in this
    equation set decodes".  ``structural`` lists the stretchy-glyph pieces, which
    have no character to decode to and must not be counted as gaps.
    """
    unmapped: dict[str, int] = {}
    structural: dict[str, int] = {}
    candidates = (
        list(paths)
        if paths is not None
        else sorted(equations_dir.glob("Equation*.svg"))
    )
    for path in candidates:
        raw = path.read_text(encoding="utf-8", errors="replace")
        for family, code in re.findall(
            r'<use[^>]*?(?:xlink:)?href="#([A-Za-z0-9]+)-([0-9A-Fa-f]+)"', raw
        ):
            key = f"{family}-{code}"
            if key in STRUCTURAL_GLYPH_SLOTS:
                structural[key] = structural.get(key, 0) + 1
            elif _glyph(code, family) == "?":
                unmapped[key] = unmapped.get(key, 0) + 1
    return SurveyResult(
        unmapped=dict(sorted(unmapped.items(), key=lambda item: -item[1])),
        structural=dict(sorted(structural.items(), key=lambda item: -item[1])),
    )


def _selected_paths(names: list[str], equations_dir: pathlib.Path) -> Iterator[pathlib.Path]:
    for name in names:
        target = equations_dir / name
        if target.is_file():
            yield target


def main() -> None:
    """Decode the requested equations, or a documented sample."""
    # Greek glyphs cannot be encoded by a legacy console code page (GBK on this
    # host raises UnicodeEncodeError mid-equation), so pin the stream to UTF-8.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "equations",
        nargs="*",
        default=None,
        help=(
            "SVG file names inside the Adams GeneratedImages/Equations directory; "
            "with none, decoding uses a small built-in sample and --survey scans the "
            "whole directory"
        ),
    )
    parser.add_argument(
        "--equations-dir",
        type=pathlib.Path,
        default=DEFAULT_EQUATIONS_DIR,
    )
    parser.add_argument(
        "--tree",
        action="store_true",
        help="print the transform hierarchy so grouping and bars are visible",
    )
    parser.add_argument(
        "--survey",
        action="store_true",
        help="list glyph slots that still decode as '?' (with no names: all equations)",
    )
    args = parser.parse_args()

    if args.survey:
        # No names means "every equation", not the sample: a survey that silently
        # covered three files would report a clean bill of health for 7193.
        selected = list(_selected_paths(args.equations or [], args.equations_dir))
        found = survey(args.equations_dir, selected or None)
        for key, count in found.unmapped.items():
            print(f"unmapped\t{key}\t{count}")
        for key, count in found.structural.items():
            print(f"structural\t{key}\t{count}")
        print(f"files: {len(selected) if selected else 'all'}")
        print(f"unmapped slots: {len(found.unmapped)}")
        print(f"structural slots: {len(found.structural)}")
        return

    for name in args.equations or list(SAMPLE):
        target = args.equations_dir / name
        if not target.is_file():
            print(f"--- {name} --- MISSING at {target}")
            continue
        print(f"--- {name} ---")
        if args.tree:
            print("\n".join(transcript(target)))
        else:
            print(decode(target))


if __name__ == "__main__":
    main()
