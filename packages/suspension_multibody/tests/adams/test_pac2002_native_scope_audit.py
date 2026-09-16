"""
Permanent audit of the native PAC2002 coverage the ``exact_pac2002`` label claims.

The manifest publishes ``native_tire_model_scope`` next to the ``exact_pac2002``
label, so that label is only meaningful while the published scope agrees with what
the code actually enforces.  That agreement used to be maintained by hand and had
drifted: the published list still claimed only modes 1-4 and 11-14 were gated and
never mentioned ``USE_MODE 0`` or the validity-range clamp.

These tests derive the expectation from the enforcement constants and fail when
the declaration and the enforcement stop matching, so a feature cannot be added,
removed or re-blocked without updating the claim in the same change.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from suspension_multibody.pac2002_scope import (
    PAC2002_FAMILY_DECLARED_GAP,
    PAC2002_FEATURE_FLAG_DECLARED_GAP,
    PAC2002_NATIVE_IMPLEMENTED_FEATURES,
    PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES,
    PAC2002_PARAMETER_DECLARED_GAP,
    PAC2002_SUPPORTED_NATIVE_USE_MODES,
    PAC2002_UNSUPPORTED_FEATURE_FAMILIES,
    PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS,
    PAC2002_UNSUPPORTED_NATIVE_PARAMETERS,
    pac2002_unsupported_native_reasons,
)

# Modes Adams documents that the advanced transient contact-mass model covers.
# 21/22 are the pure-axis variants and 25 adds turn-slip; none of them is in the
# supported set, and each has to be declared as a gap.
_ADVANCED_TRANSIENT_MODES = frozenset({21, 22, 23, 24, 25})


def test_every_fail_closed_family_names_the_gap_it_enforces() -> None:
    family_names = {family.name for family in PAC2002_UNSUPPORTED_FEATURE_FAMILIES}
    assert family_names == set(PAC2002_FAMILY_DECLARED_GAP), (
        "a fail-closed family was added or renamed without declaring the gap it "
        "enforces in PAC2002_FAMILY_DECLARED_GAP"
    )
    for name, gap in PAC2002_FAMILY_DECLARED_GAP.items():
        assert gap in PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES, (name, gap)


def test_unsupported_scalar_parameters_are_declared_as_gaps() -> None:
    """The Maxwell element is the only scalar blocker, and it is declared."""
    assert PAC2002_UNSUPPORTED_NATIVE_PARAMETERS == frozenset(
        {"DYNAMIC_STIFFNESS", "DYNAMIC_DAMPING"}
    )
    assert "maxwell_non_rolling_vertical_element" in (
        PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES
    )


def test_declared_mode_coverage_matches_the_scope_gate() -> None:
    """Every advanced-transient mode is either supported or declared missing."""
    supported = PAC2002_SUPPORTED_NATIVE_USE_MODES
    assert supported == frozenset({0, 1, 2, 3, 4, 11, 12, 13, 14, 23, 24, 25}), (
        "the supported USE_MODE set changed; update the feature declaration and "
        "the manifest audit together"
    )
    # The contact-mass modes split into the ones the kernel runs and the ones it
    # declares unavailable; together they must cover the documented range.  Mode 25
    # moved from the second group to the first when the turn-slip / parking family
    # was implemented and gated (tasks/D/TURN_SLIP_PARKING_NOTES.md steps 68-71).
    claimed = {23, 24, 25}
    declared = {"advanced_transient_pure_axis_modes_21_22"}
    assert claimed <= supported
    assert declared <= set(PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES)
    assert (_ADVANCED_TRANSIENT_MODES - claimed) == {21, 22}


def test_implemented_and_missing_feature_lists_are_disjoint_and_named() -> None:
    implemented = set(PAC2002_NATIVE_IMPLEMENTED_FEATURES)
    missing = set(PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES)
    assert not implemented & missing
    assert implemented and missing
    for name in (*implemented, *missing):
        assert name and name == name.strip().lower(), name


def test_comparison_manifest_republishes_the_declared_scope() -> None:
    """
    The manifest must read the declaration instead of transcribing it.

    Asserting object identity is the point: if the manifest builder ever goes back
    to a hand-written tuple, these become copies and the assertion fails, which is
    exactly the drift that let the published scope disagree with the code.
    """
    module = __import__(
        "suspension_multibody.adams.full_vehicle_model", fromlist=["_"]
    )
    assert (
        module.PAC2002_NATIVE_IMPLEMENTED_FEATURES
        is PAC2002_NATIVE_IMPLEMENTED_FEATURES
    )
    assert (
        module.PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES
        is PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES
    )
    assert (
        module.PAC2002_SUPPORTED_NATIVE_USE_MODES
        is PAC2002_SUPPORTED_NATIVE_USE_MODES
    )


def test_every_scalar_blocker_declares_its_gap() -> None:
    """A flag or parameter cannot be blocked without naming what is missing."""
    assert set(PAC2002_FEATURE_FLAG_DECLARED_GAP) == set(
        PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS
    )
    assert set(PAC2002_PARAMETER_DECLARED_GAP) == set(
        PAC2002_UNSUPPORTED_NATIVE_PARAMETERS
    )
    declared = set(PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES)
    for gap in (
        *PAC2002_FEATURE_FLAG_DECLARED_GAP.values(),
        *PAC2002_PARAMETER_DECLARED_GAP.values(),
    ):
        assert gap in declared, gap


def _permitted_rejection_reasons() -> set[str]:
    """
    Reconstruct the only reason texts the registry is allowed to emit.

    ``pac2002_unsupported_native_reasons`` builds each reason from the registry,
    so this is derived rather than transcribed: adding a blocker without declaring
    its gap makes the library audit below fail.
    """
    reasons = {family.reason for family in PAC2002_UNSUPPORTED_FEATURE_FAMILIES}
    reasons |= {
        name.replace("PAC2002_UNSUPPORTED_", "unsupported ").lower()
        for name in PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS
    }
    for name in PAC2002_UNSUPPORTED_NATIVE_PARAMETERS:
        reasons.add(f"unsupported parameter {name}")
        reasons.add(f"non-finite unsupported parameter {name}")
    for name in PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS:
        reasons.add(f"non-finite unsupported feature flag {name}")
    return reasons


@pytest.mark.adams
def test_every_rejection_measured_on_the_stock_library_is_declared() -> None:
    """
    Measure the fail-closed gate against the tires Adams actually ships.

    This is the evidence behind the coverage claim: it walks the installed tire
    library, asks the scope gate about each PAC2002 tire, and requires every
    rejection to trace back to a reason the registry is allowed to emit -- either
    an out-of-scope USE_MODE or one of the declared gaps.  A feature silently
    starting to block tires, or a tire being rejected for an undeclared reason,
    fails here.
    """
    from suspension_multibody.adams import discover_profile
    from suspension_multibody.adams.full_vehicle_model import (
        _parse_text_units,
        _parse_tire,
    )

    profile = discover_profile()
    if not profile.available or profile.database_path is None:
        pytest.skip(f"Adams/Car is unavailable: {profile.message}")
    root = Path(profile.database_path) / "tires.tbl"
    if not root.is_dir():
        pytest.skip(f"Adams tire library unavailable: {root}")

    permitted = _permitted_rejection_reasons()
    accepted: list[str] = []
    rejected: list[tuple[str, tuple[str, ...]]] = []
    for path in sorted(root.glob("*.tir")):
        text = path.read_text(encoding="ascii", errors="replace")
        if "PAC2002" not in text.upper():
            continue
        try:
            coefficients = _parse_tire(path)
        except ValueError as error:  # pragma: no cover - surfaces unit gaps
            pytest.fail(f"{path.name}: unit declaration is unsupported: {error}")
        reasons = pac2002_unsupported_native_reasons(coefficients)
        if not reasons:
            accepted.append(path.name)
            continue
        rejected.append((path.name, reasons))
        for reason in reasons:
            if reason.startswith("unsupported USE_MODE "):
                mode = int(reason.rsplit(" ", 1)[1])
                assert mode not in PAC2002_SUPPORTED_NATIVE_USE_MODES
                continue
            head = reason.split(" (")[0]
            assert head in permitted, (path.name, reason)

    # The maneuver the correlation gate replays must stay inside the scope, and
    # the library must not collapse to "everything is rejected".
    assert "pac2002_235_60R16.tir" in accepted
    assert accepted, "no stock PAC2002 tire is in native scope"
    assert rejected, "the fail-closed gate accepted every stock PAC2002 tire"

    # Everything the stock library asks for that native cannot do must be
    # declared.  ``_parse_text_units`` is exercised above via ``_parse_tire``.
    assert _parse_text_units("[UNITS]\nFORCE = 'pound_force'\n")["force"] == (
        "pound_force"
    )


def test_pound_force_and_pound_mass_declarations_are_converted(
    tmp_path: Path,
) -> None:
    """
    Four stock tires declare ``pound_force``/``pound_mass``.

    Before these spellings were in the unit tables, ``_parse_tire`` raised
    ``unsupported Adams unit`` on those files, so a vehicle using one of them
    failed to import at all instead of converting -- neither working nor failing
    closed.  ``lbf``/``lbm`` were already handled, so the conversion is pinned
    against them.
    """
    from suspension_multibody.adams.full_vehicle_model import _parse_tire

    tire = tmp_path / "pound_units.tir"
    tire.write_text(
        "\n".join(
            [
                "[UNITS]",
                "LENGTH = 'inch'",
                "FORCE = 'pound_force'",
                "MASS = 'pound_mass'",
                "TIME = 'second'",
                "ANGLE = 'degree'",
                "[MODEL]",
                "FNOMIN = 1000",
                "UNLOADED_RADIUS = 10",
            ]
        ),
        encoding="ascii",
    )
    coefficients = _parse_tire(tire)
    # The parser keeps the source values and exposes the normalized ones: 1000
    # pound_force -> N, and a 10 inch radius -> mm.
    assert coefficients["FNOMIN"] == pytest.approx(1000.0)
    assert coefficients["FNOMIN_N"] == pytest.approx(1000.0 * 4.4482216152605)
    assert coefficients["UNLOADED_RADIUS_MM"] == pytest.approx(254.0)


def test_every_supported_mode_is_gated_or_accounted_for() -> None:
    """
    A supported mode with no Adams gate is a hole in the exactness claim.

    Modes 0/1/2/11/12 cannot be referenced with any maneuver the harness can run
    (each is missing a force axis a steering maneuver needs), so they are named with
    the reason instead.  This asserts the two sets partition the supported modes --
    adding a mode to either list without the other, or claiming a mode with no gate
    and no recorded reason, fails here.
    """
    from suspension_multibody.pac2002_scope import (
        PAC2002_ADAMS_GATED_USE_MODES,
        PAC2002_UNGATED_USE_MODE_REASONS,
    )

    gated = set(PAC2002_ADAMS_GATED_USE_MODES)
    ungated = set(PAC2002_UNGATED_USE_MODE_REASONS)
    assert not gated & ungated
    assert gated | ungated == set(PAC2002_SUPPORTED_NATIVE_USE_MODES), (
        gated | ungated, set(PAC2002_SUPPORTED_NATIVE_USE_MODES)
    )
    assert gated <= set(PAC2002_SUPPORTED_NATIVE_USE_MODES)
    for mode, reason in PAC2002_UNGATED_USE_MODE_REASONS.items():
        assert reason.strip(), mode


@pytest.mark.parametrize(
    "feature",
    [
        "advanced_transient_pure_axis_modes_21_22",
        # What is left of the spin/parking family once USE_MODE 25 is implemented:
        # the two second-order trail coefficients no documented factor reads.
        "turn_slip_second_order_trail_coefficients",
        "belt_dynamics_modes_21_25",
        "maxwell_non_rolling_vertical_element",
        "non_point_contact_models",
    ],
)
def test_documented_gaps_stay_declared(feature: str) -> None:
    """Removing a gap from the claim requires removing the fail-closed check."""
    assert feature in PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES
