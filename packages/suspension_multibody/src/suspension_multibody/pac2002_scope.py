"""
Native PAC2002 feature-scope validation helpers.

The kernel exposes the steady-state Magic Formula, the linear transient model and
the advanced transient contact-mass modes.  Every other PAC2002 feature Adams
documents has to fail closed: a tire that requests it must be rejected, because
accepting it would produce a result that silently omits the feature.

*Which* features those are is a property of the kernel, so the kernel declares
it: ``suspension_kernel_capabilities`` returns the supported USE_MODEs and the
coefficient families the kernel refuses, and everything below is derived from
that document.  The lists used to be copied into this module by hand, and the
copy drifted in the direction that matters -- this file claimed the kernel never
applies the validity-range clamps while the kernel was calling
``pac2002_clamp_load`` on every tire step, and it still carried a stale comment
saying USE_MODE 25 was refused after the kernel had implemented and gated it.

Two things stay here because they are not properties of the kernel:

``PAC2002_ADAMS_GATED_USE_MODES`` / ``PAC2002_UNGATED_USE_MODE_REASONS``
    which modes the live Adams correlation gate can cover, and why the rest
    cannot be covered by any maneuver the installed Adams ships.

``PAC2002_NATIVE_IMPLEMENTED_FEATURES`` / ``..._NOT_IMPLEMENTED_FEATURES``
    the human-readable coverage claim the comparison manifest publishes next to
    the ``exact_pac2002`` label.  A permanent audit derives both from the
    enforcement and fails when they stop matching.
"""

from __future__ import annotations

import functools
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

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


@functools.lru_cache(maxsize=1)
def _kernel_scope() -> tuple[
    frozenset[int], frozenset[str], frozenset[str], tuple[Pac2002FeatureFamily, ...]
]:
    """
    Return the kernel's capability declaration.

    Read from the shared library rather than carried here, and read lazily: the
    authoring schemas import this module, and there is no reason for them to need
    a built kernel until a tire actually reaches the scope check.
    """
    import ctypes

    from .kernel.native import load_library

    reader = load_library().suspension_kernel_capabilities
    reader.argtypes = [
        ctypes.c_char_p,
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_size_t),
    ]
    reader.restype = ctypes.c_int32
    needed = ctypes.c_size_t(0)
    status = int(reader(None, 0, ctypes.byref(needed)))
    if status != 11:
        raise RuntimeError(
            f"kernel capability probe returned {status}; expected the size request"
        )
    buffer = ctypes.create_string_buffer(int(needed.value))
    status = int(reader(buffer, int(needed.value), ctypes.byref(needed)))
    if status != 0:
        raise RuntimeError(f"kernel capability read returned {status}")
    document = json.loads(buffer.value.decode("utf-8"))
    families = tuple(
        Pac2002FeatureFamily(
            name=str(entry["name"]),
            reason=str(entry["reason"]),
            coefficients=frozenset(str(value) for value in entry["coefficients"]),
        )
        for entry in document["pac2002_refused_families"]
    )
    return (
        frozenset(int(value) for value in document["pac2002_supported_use_modes"]),
        frozenset(str(value) for value in document["pac2002_refused_parameters"]),
        frozenset(str(value) for value in document["pac2002_refused_feature_flags"]),
        families,
    )


#: The constants the kernel owns.  They are resolved on first access rather than
#: at import, so importing the authoring schemas does not load the library.
_KERNEL_SOURCED = (
    "PAC2002_SUPPORTED_NATIVE_USE_MODES",
    "PAC2002_UNSUPPORTED_NATIVE_PARAMETERS",
    "PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS",
    "PAC2002_UNSUPPORTED_FEATURE_FAMILIES",
    "PAC2002_MUST_BE_ZERO_COEFFICIENTS",
)


def __getattr__(name: str) -> Any:
    """Resolve the kernel-sourced constants on first access."""
    if name not in _KERNEL_SOURCED:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    modes, parameters, flags, families = _kernel_scope()
    if name == "PAC2002_SUPPORTED_NATIVE_USE_MODES":
        value: Any = modes
    elif name == "PAC2002_UNSUPPORTED_NATIVE_PARAMETERS":
        value = parameters
    elif name == "PAC2002_UNSUPPORTED_NATIVE_FEATURE_FLAGS":
        value = flags
    elif name == "PAC2002_UNSUPPORTED_FEATURE_FAMILIES":
        value = families
    else:
        value = parameters | frozenset(
            coefficient for family in families for coefficient in family.coefficients
        )
    globals()[name] = value
    return value


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
    # The scope comes from the kernel, so this can only refuse what the kernel
    # itself says it cannot run.  A bare module-level name would not do: the
    # kernel-sourced constants resolve through `__getattr__`, which a global
    # lookup inside a function does not consult.
    supported_modes, refused_parameters, refused_flags, refused_families = _kernel_scope()
    reasons: list[str] = []

    use_mode = pac2002_native_use_mode(coefficients)
    if use_mode not in supported_modes:
        reasons.append(f"unsupported USE_MODE {use_mode}")

    for name in sorted(refused_parameters):
        value = float(coefficients.get(name, 0.0))
        if not math.isfinite(value):
            reasons.append(f"non-finite unsupported parameter {name}")
        elif abs(value) > 1.0e-12:
            reasons.append(f"unsupported parameter {name}")

    for name in sorted(refused_flags):
        value = float(coefficients.get(name, 0.0))
        if not math.isfinite(value):
            reasons.append(f"non-finite unsupported feature flag {name}")
        elif abs(value) > 1.0e-12:
            reasons.append(name.replace("PAC2002_UNSUPPORTED_", "unsupported ").lower())

    for family in refused_families:
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
