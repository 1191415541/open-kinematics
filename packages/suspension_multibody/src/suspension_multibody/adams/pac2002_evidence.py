"""
The Adams evidence behind the PAC2002 coverage claim.

Two things stay on this side because they are not properties of the kernel:

``PAC2002_ADAMS_GATED_USE_MODES`` / ``PAC2002_UNGATED_USE_MODE_REASONS``
    which modes the live Adams correlation gate can cover, and why the rest
    cannot be covered by any maneuver the installed Adams ships.

``PAC2002_NATIVE_IMPLEMENTED_FEATURES`` / ``..._NOT_IMPLEMENTED_FEATURES``
    the human-readable coverage claim the comparison manifest publishes next to
    the ``exact_pac2002`` label.  A permanent audit derives both from the
    enforcement and fails when they stop matching.

``PAC2002_*_DECLARED_GAP``
    which declared gap each fail-closed family, feature flag and parameter
    enforces, so nothing can be blocked without being named in the claim.

The *enforcement* these describe is the kernel's own capability declaration
(read in :mod:`suspension_multibody.kernel.capabilities`); only the published
evidence lives here.
"""

from __future__ import annotations

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

__all__ = [
    "PAC2002_ADAMS_GATED_USE_MODES",
    "PAC2002_FAMILY_DECLARED_GAP",
    "PAC2002_FEATURE_FLAG_DECLARED_GAP",
    "PAC2002_NATIVE_IMPLEMENTED_FEATURES",
    "PAC2002_NATIVE_NOT_IMPLEMENTED_FEATURES",
    "PAC2002_PARAMETER_DECLARED_GAP",
    "PAC2002_UNGATED_USE_MODE_REASONS",
]
