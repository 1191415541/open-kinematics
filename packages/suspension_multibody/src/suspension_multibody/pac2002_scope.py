"""
Native PAC2002 feature-scope validation helpers.

The native kernel implements the steady-state Magic Formula and the linear
transient model.  Every other PAC2002 feature Adams documents must fail closed:
a tire that requests it has to be rejected, because accepting it would produce a
result that silently omits the feature.

Two enforcement layers exist here:

``PAC2002_UNSUPPORTED_NATIVE_PARAMETERS`` / ``..._FEATURE_FLAGS``
    Scalar switches and coefficients that must be zero.  These are the original
    checks and cover the PAC-MC pressure extensions and the non-zero feature
    flags the importer emits.

``PAC2002_UNSUPPORTED_FEATURE_FAMILIES``
    Whole coefficient families keyed by the ``.tir`` section that declares them
    (turn-slip, advanced-transient contact mass, belt dynamics, validity
    ranges).  These were previously invisible: the generic numeric extractor
    copies every ``NAME = number`` pair into the coefficient dict, so a tire
    could request belt dynamics or a contact-mass model and be solved without
    it, with no diagnostic at all.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass

# USE_MODE 0 is the vertical-spring-and-damper-only mode; the rest are the
# steady-state, linear-transient and (23/24) advanced-transient contact-mass
# modes the kernel runs exactly enough to be gated by the coefficient families
# below.
#
# Deliberately still absent:
#   21, 22 -- pure longitudinal / pure lateral advanced transient.  The contact
#             body always has both in-plane degrees of freedom, so these modes
#             would need a per-mode slot layout the kernel does not have.
#   25     -- adds turn-slip and the contact body's yaw degree of freedom, which
#             is not modelled at all (subtask D).
PAC2002_SUPPORTED_NATIVE_USE_MODES = frozenset(
    {0, 1, 2, 3, 4, 11, 12, 13, 14, 23, 24, 25}
)

# Which of those modes a live automated Adams correlation gate covers, and why the
# rest cannot be covered by any maneuver the installed Adams ships.
#
# A steering maneuver needs both force axes to hold the vehicle, but that is not the
# only obstacle: adding the straight-line `acceleration` case (see
# ``STRAIGHT_LINE_CASES``) still leaves modes 0/1/11 failing Adams' *static*
# equilibrium, with "Static equilibrium analysis has not been successful ... the
# equation with the largest error was model.TR_Brake_System.*_wheel_omega".  The
# same case runs to 801 samples in mode 14, so the maneuver works and the failure is
# specific to those modes.  A single-axis USE_MODE has no route to an Adams
# reference in this vehicle assembly.
#
# Claiming a mode as supported while it has no gate is a real gap in the exactness
# claim, so it is recorded here rather than left implicit; an audit asserts that the
# gated and accounted-for sets together cover every supported mode and do not
# overlap.
PAC2002_ADAMS_GATED_USE_MODES = frozenset({3, 4, 13, 14, 23, 24, 25})

PAC2002_UNGATED_USE_MODE_REASONS: dict[int, str] = {
    0: (
        "vertical spring and damper only; no slip force, and Adams' static "
        "equilibrium fails for it under both a steering and a straight-line maneuver"
    ),
    1: (
        "Fx and My only; Adams' static equilibrium fails for it under both a "
        "steering and a straight-line maneuver"
    ),
    2: "Fy, Mx and Mz only; cannot hold the vehicle through a steering maneuver",
    11: (
        "Fx and My only (transient); Adams' static equilibrium fails for it under "
        "both a steering and a straight-line maneuver"
    ),
    12: "Fy, Mx and Mz only (transient); cannot hold the vehicle",
}

# These Adams PAC2002/PAC-MC extensions are not part of the native ABI yet.  A
# non-zero value would otherwise be accepted by the Python schema and silently
# dropped before the C++ parameter array is built.
PAC2002_UNSUPPORTED_NATIVE_PARAMETERS = frozenset(
    {
        # Maxwell non-rolling vertical element (subtask A5, unimplemented).  The
        # quoted ``USE_DYNAMIC_STIFFNESS`` switch has its own feature flag below;
        # these two are unquoted numbers, so the generic extractor does copy them
        # into the coefficient dict and they are checked here as well.
        "DYNAMIC_STIFFNESS",
        "DYNAMIC_DAMPING",
    }
)

PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS = frozenset(
    {
        "PAC2002_UNSUPPORTED_BELT_DYNAMICS",
        "PAC2002_UNSUPPORTED_CONTACT_MODEL",
        # The Maxwell element (USE_DYNAMIC_STIFFNESS = 'YES' plus
        # DYNAMIC_STIFFNESS/DYNAMIC_DAMPING) changes non-rolling vertical
        # stiffness.  DYNAMIC_STIFFNESS and DYNAMIC_DAMPING are unquoted
        # numbers, so the generic extractor reads them into the coefficient
        # dict, but neither name is part of the PAC2002 ABI payload and the
        # enable switch is quoted and therefore invisible to that extractor.
        # Without this flag a tire requesting the element was accepted and
        # silently solved without it.
        "PAC2002_UNSUPPORTED_DYNAMIC_STIFFNESS",
        "PAC2002_UNSUPPORTED_FE_METHOD",
        # FITTYP selects the legacy rolling-resistance formulation, which the
        # native kernel does not implement.  The keyword is absent from every
        # tire in the installed Adams libraries, so it fails closed instead of
        # silently applying the modern [ROLLING_COEFFICIENTS] equations.
        "PAC2002_UNSUPPORTED_FITTYP",
        "PAC2002_UNSUPPORTED_LOCAL_SOLVER",
        "PAC2002_UNSUPPORTED_PAC_MC",
    }
)


@dataclass(frozen=True)
class Pac2002FeatureFamily:
    """
    A documented PAC2002 feature the native kernel cannot run exactly.

    ``coefficients`` lists the tire-property keywords that only carry a non-zero
    value when the feature is actually requested.
    """

    name: str
    reason: str
    coefficients: frozenset[str]


# The steady-state spin/parking coefficients of Eq3329-Eq3356 are consumed by the
# kernel for USE_MODE 25, with exactly two exceptions: QBRP2 and QDTP2 appear in the
# ABI enum but no documented factor reads them (Eq3347 gives ``DDrgamma`` from QDTP1
# alone), so a tire that sets either one still fails closed rather than being solved
# with a term dropped.
_TURN_SLIP_SECOND_ORDER = frozenset({"QBRP2", "QDTP2"})

# [DYNAMIC_COEFFICIENTS] and [CONTACT_COEFFICIENTS] together carry the
# non-linear (advanced) transient contact-mass model from Adams sections
# "Transient Behavior in PAC2002" and "PAC2002 with Belt Dynamics".
_CONTACT_MASS = frozenset(
    {
        "MC",
        "IC",
        "KX",
        "KY",
        "KP",
        "CX",
        "CY",
        "CP",
        "CXZ1",
        "CXZ2",
        "CXX1",
        "CYZ1",
        "CYZ2",
        "CYY1",
        "PA1",
        "PA2",
        "EP",
        "EP12",
        "BF2",
        "BP1",
        "BP2",
        "BP3",
        "BP4",
        "N_LENGTH",
        "N_WIDTH",
        "PAE",
        "PB1",
        "PB2",
        "PB3",
        "PBE",
        "PCE",
        "PLS",
    }
)

# Subset of the contact-mass family the native kernel still cannot consume.
# MC/KX/KY/CX/CY drive the contact patch's force balance, CXZ*/CXX*/CYZ*/CYY* scale
# its carcass stiffness with load and slip (eqs 48-50), PA1/PA2 give the half
# contact length of the non-linear relaxation lengths (eqs 3978, 3982-3984) and
# IC/KP/CP are the yaw degree of freedom (eqs 3954 and 3961).  What remains is the
# turn-slip relaxation set (EP/EP12/BF2/BP*) and the enveloping helpers
# (N_*/PAE/PB*/PBE/PCE/PLS).  A tire that declares any of them still fails closed.
# Subset of the contact-mass family the native kernel still cannot consume.
# MC/KX/KY/CX/CY drive the contact patch's force balance, CXZ*/CXX*/CYZ*/CYY* scale
# its carcass stiffness with load and slip (eqs 48-50), PA1/PA2 give the half
# contact length of the non-linear relaxation lengths (eqs 3978, 3982-3984), IC/KP/CP
# are the yaw degree of freedom (eqs 3954, 3961) and EP/EP12/BF2/BP1/BP2 are the
# turn-slip relaxation set (eqs 3969-3977) the USE_MODE 25 kernel now evaluates.  What
# remains is BP3/BP4 (the third and fourth moment relaxation factors, which no
# documented filter uses and no installed tire declares) and the enveloping helpers
# (N_*/PAE/PB*/PBE/PCE/PLS) of the non-point contact models.  A tire that declares
# any of them still fails closed.
_CONTACT_MASS_UNIMPLEMENTED = _CONTACT_MASS - frozenset(
    {
        "MC",
        "KX",
        "KY",
        "CX",
        "CY",
        "CXZ1",
        "CXZ2",
        "CXX1",
        "CYZ1",
        "CYZ2",
        "CYY1",
        "PA1",
        "PA2",
        "IC",
        "KP",
        "CP",
        # Eq3969-Eq3977: the four filters, their composite turn slips and the
        # relaxation lengths they use, all evaluated in the USE_MODE 25 kernel.
        "EP",
        "EP12",
        "BF2",
        "BP1",
        "BP2",
    }
)

# [BELT_PARAMETERS] carries the rigid-ring belt model (rim-to-belt six degree of
# freedom bushing) described by Adams section "PAC2002 with Belt Dynamics".
_BELT_DYNAMICS = frozenset(
    {
        "QBVTH",
        "QBVXZ",
        "QCBGM",
        "QCBTH",
        "QCBXZ",
        "QCBY",
        "QCCFI",
        "QCCX",
        "QCCY",
        "QIBXZ",
        "QIBY",
        "QIC",
        "QKBGM",
        "QKBTH",
        "QKBXZ",
        "QKBY",
        "QKCFI",
        "QKCX",
        "QKCY",
        "QMB",
        "QMC",
        "TYRE_MASS",
    }
)

# Validity ranges are declared in the tire file and Adams clamps the inputs to
# them.  The native kernel copies these into the ABI payload but never applies
# them, so a request that relies on clamping is silently unclamped.
_VALIDITY_RANGES = frozenset(
    {
        "KPUMIN",
        "KPUMAX",
        "ALPMIN",
        "ALPMAX",
        "CAMMIN",
        "CAMMAX",
        "FZMIN",
        "FZMAX",
    }
)

PAC2002_UNSUPPORTED_FEATURE_FAMILIES: tuple[Pac2002FeatureFamily, ...] = (
    Pac2002FeatureFamily(
        name="turn_slip_second_order_coefficients",
        reason="unsupported second-order turn-slip coefficients",
        coefficients=_TURN_SLIP_SECOND_ORDER,
    ),
    Pac2002FeatureFamily(
        name="contact_mass_advanced_transient",
        reason="unsupported non-linear transient contact-mass coefficients",
        coefficients=_CONTACT_MASS_UNIMPLEMENTED,
    ),
    Pac2002FeatureFamily(
        name="belt_dynamics",
        reason="unsupported belt dynamics coefficients",
        coefficients=_BELT_DYNAMICS,
    ),
)

# ``_VALIDITY_RANGES`` is deliberately NOT part of the fail-closed check above.
# Every tire property file declares [LONG_SLIP_RANGE], [SLIP_ANGLE_RANGE],
# [INCLINATION_ANGLE_RANGE] and [VERTICAL_FORCE_RANGE], so treating their
# presence as "the feature was requested" rejects every valid tire, including
# the reference case.  Declaring a range is not requesting a feature.
#
# The gap is real but different in kind: the kernel copies these bounds into the
# ABI payload and never applies them, so inputs outside the declared range are
# not clamped the way Adams clamps them.  For a simulation whose inputs stay
# inside the declared ranges — including the reference case — behaviour matches
# Adams.  Closing this properly means implementing the clamp in the kernel, which
# is tracked as its own subtask rather than approximated by a rejection.
UNCLAMPED_VALIDITY_RANGE_COEFFICIENTS = _VALIDITY_RANGES

_FAMILY_COEFFICIENTS = frozenset(
    name
    for family in PAC2002_UNSUPPORTED_FEATURE_FAMILIES
    for name in family.coefficients
)

# Every coefficient that must be zero for a tire to run natively.
PAC2002_MUST_BE_ZERO_COEFFICIENTS = (
    PAC2002_UNSUPPORTED_NATIVE_PARAMETERS | _FAMILY_COEFFICIENTS
)

# ---------------------------------------------------------------------------
# Declared native coverage
# ---------------------------------------------------------------------------
#
# The comparison manifest publishes these two tuples as
# ``native_tire_model_scope``.  They used to be hand-written inside the
# manifest builder and had drifted: they still claimed only modes 1-4/11-14 were
# gated and did not mention the validity-range clamp or USE_MODE 0, so the
# ``exact_pac2002`` label advertised a wider scope than the code had and a
# narrower one than it actually had.
#
# Keeping the declaration next to the enforcement lets a permanent audit derive
# both from one source.  ``test_pac2002_native_scope_audit.py`` asserts that every
# fail-closed family is declared below, that the mode list matches
# ``PAC2002_SUPPORTED_NATIVE_USE_MODES``, and that the manifest republishes these
# exact tuples.
PAC2002_NATIVE_IMPLEMENTED_FEATURES: tuple[str, ...] = (
    "steady_state_pure_and_combined_slip_modes_1_4",
    "selected_combined_slip_coefficients",
    "standard_rbx_rby_rvy_combined_slip_coefficients",
    "linear_transient_modes_11_14",
    "advanced_transient_contact_mass_modes_23_24",
    # The contact body's yaw state is carried and integrated (Eq3954, driven by Mz)
    # because a mode-23/24 tire has to keep that degree of freedom non-singular:
    # Adams aborts its own mode-24 solve with "NaN value detected in
    # AsMath::step()" if IC/KP/CP are zeroed.  Its *effect* on the forces is nil at
    # those modes -- Eq26's ``- Vx*beta`` belongs to turn-slip -- and that is
    # measured rather than assumed: feeding beta into the lateral slip target gives
    # 5.63/2.17/25.5/22.6/28.0/23.2 % NRMSE over the 5 s mode-24 reference where
    # leaving it out gives 0.24/0.69/1.03/0.85/0.65/1.16 %.  The coupling that makes
    # the yaw state act on the slip is declared below as part of the turn-slip gap,
    # and mode 25 stays rejected.
    "contact_body_yaw_degree_of_freedom",
    "nonlinear_transient_relaxation_lengths",
    "vertical_spring_and_damper_use_mode_0",
    "input_validity_range_clamping",
    "contact_body_load_and_slip_carcass_stiffness_corrections",
    "first_order_relaxation",
    "vertical_contact",
    "qv2_qfc_vxlow_vertical_and_low_speed_scaling",
    "negative_use_mode_and_tyreside_side_mirroring",
    "source_phx_pvx_phy_pvy_offsets",
    "standard_aligning_moment_coefficients",
    "overturning_rolling_and_gyroscopic_moments",
    # The tire file's [DEFLECTION_LOAD_CURVE] replaces the vertical stiffness
    # polynomial with a monotone cubic interpolation of the tabulated load.
    "deflection_load_curve_vertical_force",
    # [BOTTOMING_CURVE] with BOTTOMING_RADIUS adds the rim reaction once the rim
    # reaches the road (Adams: min(0, Fzk+Fzc) + min(0, Fzrim)).
    "wheel_bottoming_rim_reaction",
    # USE_MODE 25: the turn-slip relaxation set of Eq3963-Eq3966, the composite
    # channels of Eq3969-Eq3970, the steady-state spin/parking factors of
    # Eq3329-Eq3356 and the yaw coupling of Eq3961/Eq3967.
    #
    # What makes the family's *effect* measurable at these modes is the axle
    # antisymmetric channel: mode 24 under-produces the left-minus-right lateral force
    # tenfold (ratio 0.10 against Adams) with the front's correlation negative
    # (-0.686), while mode 25 reaches ratio 0.46-2.39 with a positive correlation on
    # both axles.  The gate is
    # ``test_parking_steer_pins_the_turn_slip_and_parking_torque``.
    "turn_slip_and_parking_mode_25",
    "turn_slip_relaxation_set",
)

PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES: tuple[str, ...] = (
    "pac_mc_extensions",
    "advanced_transient_pure_axis_modes_21_22",
    "turn_slip_second_order_trail_coefficients",
    "belt_dynamics_modes_21_25",
    "non_point_contact_models",
    "fe_method_friction_ellipse",
    "adams_local_solver_tire_model",
    "maxwell_non_rolling_vertical_element",
    "legacy_fityp_rolling_resistance",
)

# Each fail-closed family must name the declared gap it enforces, so a new family
# cannot be added without saying what it is that the kernel does not do.
#
# The turn-slip/parking family is gone because the kernel now evaluates it; what
# remains of the contact-mass family is BP3/BP4 and the enveloping helpers of the
# non-point contact models, and what remains of the spin family is the two
# second-order trail coefficients no documented factor reads.
PAC2002_FAMILY_DECLARED_GAP: dict[str, str] = {
    "turn_slip_second_order_coefficients": (
        "turn_slip_second_order_trail_coefficients"
    ),
    "contact_mass_advanced_transient": "non_point_contact_models",
    "belt_dynamics": "belt_dynamics_modes_21_25",
}

# The same rule for the two scalar registries.  ``pac2002_unsupported_native_reasons``
# turns each of these into a rejection reason, and the library audit asserts that
# every rejection measured on the stock Adams tires traces back to one of these
# declared gaps -- so nothing can be blocked without being named in the claim.
PAC2002_FEATURE_FLAG_DECLARED_GAP: dict[str, str] = {
    "PAC2002_UNSUPPORTED_BELT_DYNAMICS": "belt_dynamics_modes_21_25",
    "PAC2002_UNSUPPORTED_CONTACT_MODEL": "non_point_contact_models",
    "PAC2002_UNSUPPORTED_DYNAMIC_STIFFNESS": (
        "maxwell_non_rolling_vertical_element"
    ),
    "PAC2002_UNSUPPORTED_FE_METHOD": "fe_method_friction_ellipse",
    "PAC2002_UNSUPPORTED_FITTYP": "legacy_fityp_rolling_resistance",
    "PAC2002_UNSUPPORTED_LOCAL_SOLVER": "adams_local_solver_tire_model",
    "PAC2002_UNSUPPORTED_PAC_MC": "pac_mc_extensions",
}

PAC2002_PARAMETER_DECLARED_GAP: dict[str, str] = {
    "DYNAMIC_STIFFNESS": "maxwell_non_rolling_vertical_element",
    "DYNAMIC_DAMPING": "maxwell_non_rolling_vertical_element",
}


def pac2002_native_use_mode(coefficients: Mapping[str, float]) -> int:
    """Return the Adams USE_MODE magnitude used for native scope checks."""
    raw = abs(float(coefficients.get("USE_MODE", 14.0)))
    if not math.isfinite(raw):
        raise ValueError("PAC2002 USE_MODE must be finite")
    return int(round(raw))


def _is_absent_or_zero(value: float) -> bool:
    if not math.isfinite(value):
        return False
    return abs(value) <= 1.0e-12


def pac2002_unsupported_native_reasons(
    coefficients: Mapping[str, float],
) -> tuple[str, ...]:
    """List native PAC2002 blockers that must not be silently approximated."""
    reasons: list[str] = []

    use_mode = pac2002_native_use_mode(coefficients)
    if use_mode not in PAC2002_SUPPORTED_NATIVE_USE_MODES:
        reasons.append(f"unsupported USE_MODE {use_mode}")

    for name in sorted(PAC2002_UNSUPPORTED_NATIVE_PARAMETERS):
        value = float(coefficients.get(name, 0.0))
        if not math.isfinite(value):
            reasons.append(f"non-finite unsupported parameter {name}")
        elif abs(value) > 1.0e-12:
            reasons.append(f"unsupported parameter {name}")

    for name in sorted(PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS):
        value = float(coefficients.get(name, 0.0))
        if not math.isfinite(value):
            reasons.append(f"non-finite unsupported feature flag {name}")
        elif abs(value) > 1.0e-12:
            reasons.append(name.replace("PAC2002_UNSUPPORTED_", "unsupported ").lower())

    for family in PAC2002_UNSUPPORTED_FEATURE_FAMILIES:
        requested = [
            name
            for name in sorted(family.coefficients)
            if not _is_absent_or_zero(float(coefficients.get(name, 0.0)))
        ]
        if requested:
            reasons.append(f"{family.reason} ({', '.join(requested)})")

    return tuple(reasons)


def validate_pac2002_native_scope(coefficients: Mapping[str, float]) -> None:
    """Raise if coefficients request Adams PAC2002 features native cannot run exactly."""
    reasons = pac2002_unsupported_native_reasons(coefficients)
    if reasons:
        raise ValueError("unsupported native PAC2002 scope: " + "; ".join(reasons))
