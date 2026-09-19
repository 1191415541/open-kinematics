"""Neutral serialization for native solver settings."""

from __future__ import annotations


def solver_settings_document(settings) -> dict[str, object]:
    """Return the explicit 21-field solver block used by all case families."""
    return {
        "integrator": "hht" if settings.integrator == "hht" else "ggl_generalized_alpha",
        "rho_inf": float(settings.rho_inf),
        "hht_alpha": float(settings.hht_alpha),
        "initialization_mode": str(settings.initialization_mode),
        "adaptive_step": bool(settings.adaptive_step),
        "internal_step_s": float(settings.internal_step_s),
        "minimum_step_s": float(settings.minimum_step_s),
        "maximum_step_s": float(settings.maximum_step_s),
        "local_relative_tolerance": float(settings.local_relative_tolerance),
        "local_position_tolerance_m": float(settings.local_position_tolerance_m),
        "local_angle_tolerance_rad": float(settings.local_angle_tolerance_rad),
        "local_velocity_tolerance_m_per_s": float(settings.local_velocity_tolerance_m_per_s),
        "local_angular_velocity_tolerance_rad_per_s": float(
            settings.local_angular_velocity_tolerance_rad_per_s
        ),
        "local_brush_tolerance_m": float(settings.local_brush_tolerance_m),
        "contact_event_tolerance_s": float(settings.contact_event_tolerance_s),
        "max_newton_iterations": int(settings.max_newton_iterations),
        "max_line_search_iterations": int(settings.max_line_search_iterations),
        "position_tolerance_m": float(settings.position_tolerance_m),
        "velocity_tolerance_m_per_s": float(settings.velocity_tolerance_m_per_s),
        "dynamics_tolerance": float(settings.dynamics_tolerance),
        "increment_tolerance": float(settings.increment_tolerance),
    }
