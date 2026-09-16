"""
Pin the Adams equation decoder's grouping recovery.

The decoder is the only oracle for the tire equations Adams ships as MathJax
glyph outlines, so a silent regression in it would corrupt every transcription
that follows.  Two properties matter and both are behavioural:

* an unmapped glyph slot must stay distinguishable from a lost fraction bar,
  because both used to print ``?`` -- that confusion once made the turn-slip
  relaxation equation read ``?c = a(1-theta-?)`` and hid its transcription;
* nested transforms must be composed, not just parsed, or a denominator and a
  sibling radical collapse into the same flat string.

``Equation3981.svg`` in the Adams help is the worked case: the flat stream is
``1 1 + kappa' sqrt{...}`` and only the geometry says the radical multiplies the
fraction instead of sitting under its bar.  The fixture below reproduces that
shape with hand-written outlines, so the test needs no Adams installation.
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

import pytest

_SCRIPT = (
    pathlib.Path(__file__).resolve().parents[2]
    / "scripts"
    / "decode_adams_equations.py"
)


def _load_decoder():
    spec = importlib.util.spec_from_file_location(
        "decode_adams_equations_under_test", _SCRIPT
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


decoder = _load_decoder()


_GLYPHS = {
    "MJMATHI-3B6": "zeta",
    "MJMATHI-3BE": "xi",
    "MJMATHI-3C0": "pi",
    "MJMAIN-3D": "equals",
    "MJMAIN-31": "one",
    "MJMAIN-2B": "plus",
    "MJMAIN-32": "two",
    "MJMAIN-7B": "lbrace",
    "MJMAIN-7D": "rbrace",
    "MJMAIN-7C": "bar",
    "MJMATHI-3BA": "kappa",
    "MJMATHI-61": "a",
    "MJMATHI-3B1": "alpha",
    "MJSZ3-221A": "radical",
    "MJMAIN-2032": "prime",
}


def _definitions() -> str:
    return "".join(
        f'<path id="{name}" stroke-width="1" d="M 0 0" />' for name in _GLYPHS
    )


def _use(x: float, y: float, name: str, scale: float | None = None) -> str:
    transform = f' transform="scale({scale})"' if scale is not None else ""
    return (
        f'<use{transform} x="{x}" y="{y}" '
        f'xmlns:NS="http://www.w3.org/1999/xlink" NS:href="#{name}" />'
    )


# ``zeta = 1/(1 + kappa') * sqrt({|alpha|} + {a})`` with the same nesting as the
# real Equation3981.svg: the fraction bar is 1640 wide and the radical bar is
# 24576 wide, so the radical cannot belong to the denominator.
_FIXTURE = f"""<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 -1692 29262 2611">
<defs id="MathJax_SVG_glyphs">{_definitions()}</defs>
<g fill="currentColor" transform="matrix(1 0 0 -1 0 0)">
{_use(0, 0, "MJMATHI-3B6")}
{_use(749, 0, "MJMAIN-3D")}
<g transform="translate(1527)"><g transform="translate(397)">
<rect stroke="none" x="0" y="220" width="1640" height="60" />
{_use(909, 567, "MJMAIN-31", 0.707)}
<g transform="translate(60 -386)">
{_use(0, 0, "MJMAIN-31", 0.707)}
{_use(500, 0, "MJMAIN-2B", 0.707)}
<g transform="translate(904)">
{_use(0, 0, "MJMATHI-3BA", 0.707)}
{_use(815, 408, "MJMAIN-2032", 0.5)}
</g></g></g></g>
<g transform="translate(3686)">
{_use(0, 121, "MJSZ3-221A")}
<rect stroke="none" x="1000" y="1512" width="24576" height="60" />
<g transform="translate(1000)">
{_use(100, 0, "MJMAIN-7B")}
{_use(700, -1, "MJMAIN-7C")}
{_use(1145, 0, "MJMATHI-3B1")}
{_use(2247, -1, "MJMAIN-7C")}
{_use(2626, 0, "MJMAIN-7B")}
{_use(3404, 0, "MJMATHI-61")}
{_use(10731, 0, "MJMAIN-7D")}
{_use(16025, 675, "MJMAIN-32", 0.707)}
{_use(12007, 0, "MJMAIN-2B")}
{_use(20232, 0, "MJMATHI-3C0")}
</g></g></g></svg>
"""


@pytest.fixture
def fixture_path(tmp_path: pathlib.Path) -> pathlib.Path:
    path = tmp_path / "Equation3981.svg"
    path.write_text(_FIXTURE, encoding="utf-8")
    return path


def test_transform_composition_multiplies_nested_offsets() -> None:
    """A child transform must apply on top of its parent's, not replace it."""
    parent = decoder.parse_transform("translate(1000)")
    child = decoder.parse_transform("scale(0.707) translate(60 -386)")
    combined = decoder._multiply(parent, child)
    # x' = 1000 + 0.707*(60 + x), y' = 0.707*(-386 + y)
    assert combined[0] == pytest.approx(0.707)
    assert combined[4] == pytest.approx(1000 + 0.707 * 60)
    assert combined[5] == pytest.approx(0.707 * -386)


def test_glyph_slots_map_greek_and_keep_unknowns_visible() -> None:
    """Greek slots decode; an unmapped math-italic slot stays '?'."""
    assert decoder._glyph("3B6", "MJMATHI") == "\u03b6"
    assert decoder._glyph("3BE", "MJMATHI") == "\u03be"
    assert decoder._glyph("3C0", "MJMATHI") == "\u03c0"
    assert decoder._glyph("3BD", "MJMATHI") == "\u03bd"
    assert decoder._glyph("3D5", "MJMATHI") == "\u03d5"
    assert decoder._glyph("3D6", "MJMATHI") == "\u03d6"
    assert decoder._glyph("3A6", "MJMATHI") == "\u03a6"
    # MJMAIN is already Unicode, so the prime of phi' must survive.
    assert decoder._glyph("2032", "MJMAIN") == "\u2032"
    assert decoder._glyph("2223", "MJMAIN") == "\u2223"
    # An unknown math-italic Greek letter must not silently become a character.
    assert decoder._glyph("3C2", "MJMATHI") == "?"
    # A stretchy-glyph piece is neither a character nor a gap.
    assert decoder._glyph("E150", "MJSZ4") == ""


def test_flat_decode_flattens_grouping(fixture_path: pathlib.Path) -> None:
    """The flat stream is order only: it cannot show where the radical sits."""
    flat = decoder.decode(fixture_path)
    # Note the trailing ``2``: even the *exponent* flattens into the baseline run,
    # so a reader cannot tell ``{a}2`` from ``{a}`` squared.  Only ``--tree`` can.
    assert flat == "\u03b6=1" + "1+\u03ba\u2032" + "\u221a{" + "|\u03b1|" + "{a}" + "2+\u03c0"


def test_tree_decode_separates_denominator_from_sibling_radical(
    fixture_path: pathlib.Path,
) -> None:
    """Geometry, not glyph order, must place the radical outside the fraction."""
    rows = list(decoder.transcript(fixture_path))

    def indent(row: str) -> int:
        return len(row) - len(row.lstrip())

    def find(predicate: str) -> str:
        matches = [row for row in rows if predicate in row]
        assert len(matches) == 1, matches
        return matches[0]

    numerator = find("'1' at x=2567 y=-401")  # above the baseline
    denominator = find("'1' at x=1984 y=386")  # below the baseline
    radical = find("'\u221a'")
    fraction_bar = find("w=1640")
    radical_bar = find("w=24576")

    # The equation is 1/(1+kappa') * sqrt(...), not 1/(1+kappa'*sqrt(...)).  The
    # denominator nests deeper than the radical sign, and the fraction's own bar
    # sits between the two depths -- so the radical cannot be under that bar.
    assert indent(denominator) > indent(radical)
    assert indent(radical) < indent(fraction_bar) < indent(denominator)
    # The numerator shares the bar's group, so it is shallower than the denominator
    # while still belonging to the fraction.
    assert indent(radical) < indent(numerator) < indent(denominator)
    # The radical owns the wide bar, sharing the radical sign's own group; its
    # width is what rules out the competing reading.
    assert indent(radical_bar) == indent(radical)


def test_survey_reports_unmapped_slots(tmp_path: pathlib.Path) -> None:
    """A survey must name the slot, not just count the '?' it produces."""
    path = tmp_path / "Equation0001.svg"
    path.write_text(
        '<svg xmlns="http://www.w3.org/2000/svg"><defs id="g">'
        '<path id="MJMATHI-3C2" stroke-width="1" d="M 0 0" />'
        '<path id="MJMATHI-3B6" stroke-width="1" d="M 0 0" />'
        '<path id="MJSZ4-E150" stroke-width="1" d="M 0 0" />'
        "</defs><g>"
        '<use x="0" y="0" xmlns:NS="http://www.w3.org/1999/xlink" '
        'NS:href="#MJMATHI-3C2" />'
        '<use x="100" y="0" xmlns:NS="http://www.w3.org/1999/xlink" '
        'NS:href="#MJMATHI-3B6" />'
        '<use x="200" y="0" xmlns:NS="http://www.w3.org/1999/xlink" '
        'NS:href="#MJSZ4-E150" />'
        "</g></svg>",
        encoding="utf-8",
    )
    result = decoder.survey(tmp_path)
    # A gap and a stretchy piece must not be lumped together: the survey exists to
    # tell "this letter is missing" from "this is a stroke of a tall radical".
    assert result.unmapped == {"MJMATHI-3C2": 1}
    assert result.structural == {"MJSZ4-E150": 1}
    assert decoder.decode(path) == "?\u03b6"
