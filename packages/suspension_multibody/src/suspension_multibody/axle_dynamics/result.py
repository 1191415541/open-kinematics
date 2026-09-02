"""Native axle dynamics result objects."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Literal

import numpy as np

BODY_STATE_COLUMNS = (
    "position_x_m",
    "position_y_m",
    "position_z_m",
    "quaternion_w",
    "quaternion_x",
    "quaternion_y",
    "quaternion_z",
    "velocity_x_m_per_s",
    "velocity_y_m_per_s",
    "velocity_z_m_per_s",
    "omega_x_rad_per_s",
    "omega_y_rad_per_s",
    "omega_z_rad_per_s",
    "acceleration_x_m_per_s2",
    "acceleration_y_m_per_s2",
    "acceleration_z_m_per_s2",
    "alpha_x_rad_per_s2",
    "alpha_y_rad_per_s2",
    "alpha_z_rad_per_s2",
)
CONSTRAINT_WRENCH_COLUMNS = (
    "force_x_n",
    "force_y_n",
    "force_z_n",
    "moment_x_n_m",
    "moment_y_n_m",
    "moment_z_n_m",
)
SPRING_OUTPUT_COLUMNS = (
    "length_m",
    "length_rate_m_per_s",
    "elastic_force_n",
    "damping_force_n",
    "compression_stop_elastic_force_n",
    "rebound_stop_elastic_force_n",
    "total_axial_force_n",
)
BUSHING_OUTPUT_COLUMNS = (
    "translation_x_m",
    "translation_y_m",
    "translation_z_m",
    "rotation_x_rad",
    "rotation_y_rad",
    "rotation_z_rad",
    "force_x_n",
    "force_y_n",
    "force_z_n",
    "moment_x_n_m",
    "moment_y_n_m",
    "moment_z_n_m",
)
ANTI_ROLL_OUTPUT_COLUMNS = (
    "relative_angle_rad",
    "relative_rate_rad_per_s",
    "torque_on_body_b_n_m",
)
TIRE_OUTPUT_COLUMNS = (
    "active",
    "gap_m",
    "penetration_m",
    "normal_velocity_m_per_s",
    "normal_force_n",
    "longitudinal_force_n",
    "lateral_force_n",
    "longitudinal_slip_velocity_m_per_s",
    "lateral_slip_velocity_m_per_s",
    "friction_utilization",
    "brush_longitudinal_m",
    "brush_lateral_m",
    "overturning_moment_n_m",
    "rolling_resistance_moment_n_m",
    "aligning_moment_n_m",
)
DIAGNOSTIC_COLUMNS = (
    "accepted",
    "accepted_step_count",
    "rejected_attempt_count",
    "maximum_newton_iterations",
    "minimum_accepted_step_s",
    "maximum_accepted_step_s",
    "last_accepted_step_s",
    "position_residual",
    "velocity_residual",
    "dynamics_residual",
    "active_contact_count",
    "contact_event_count",
    "maximum_local_error_ratio",
    "energy_residual_j",
    "failure_code",
    "static_trim_pinned_null_directions",
)
PERFORMANCE_COLUMNS = (
    "available",
    "residual_calls",
    "residual_time_s",
    "constraint_jacobian_calls",
    "constraint_jacobian_time_s",
    "force_evaluations",
    "force_time_s",
    "mass_inverse_calls",
    "mass_inverse_time_s",
    "reaction_time_s",
    "linear_factorizations",
    "linear_factorization_time_s",
    "linear_solves",
    "linear_solve_time_s",
    "line_search_trials",
    "newton_iterations",
    "accepted_steps",
    "rejected_attempts",
    "analytic_jacobian_columns",
    "finite_difference_jacobian_columns",
    "nonsmooth_fallback_columns",
    "analytic_jacobian_time_s",
    "finite_difference_jacobian_time_s",
)
ENERGY_COLUMNS = (
    "kinetic_energy_j",
    "potential_energy_j",
    "mechanical_energy_j",
    "interval_energy_residual_j",
    "interval_external_work_j",
    "interval_road_work_j",
    "interval_drive_work_j",
    "interval_damper_dissipation_j",
    "interval_friction_dissipation_j",
    "interval_contact_dissipation_j",
    "interval_algorithmic_dissipation_j",
    "interval_total_work_j",
    "interval_total_physical_dissipation_j",
    "status",
    "gravitational_potential_energy_j",
    "spring_elastic_energy_j",
    "stop_elastic_energy_j",
    "bushing_elastic_energy_j",
    "anti_roll_elastic_energy_j",
    "tire_normal_elastic_energy_j",
    "tire_brush_elastic_energy_j",
)


@dataclass(frozen=True)
class AxleRunDiagnostics:
    accepted: np.ndarray
    internal_steps: np.ndarray
    rejected_attempts: np.ndarray
    newton_iterations: np.ndarray
    minimum_accepted_step_s: np.ndarray
    maximum_accepted_step_s: np.ndarray
    last_accepted_step_s: np.ndarray
    position_residual: np.ndarray
    velocity_residual: np.ndarray
    dynamics_residual: np.ndarray
    active_contacts: np.ndarray
    contact_events: np.ndarray
    local_error_ratio: np.ndarray
    energy_residual: np.ndarray
    failure_code: np.ndarray
    pinned_null_directions: np.ndarray


@dataclass(frozen=True)
class AxleContactEventRecord:
    """One internally localized tire contact transition."""

    time_s: float
    tire: str
    transition: Literal["enter", "exit"]


@dataclass(frozen=True)
class AxleRunPerformance:
    """Optional aggregate native timing counters for one solver run."""

    available: bool = False
    residual_calls: int = 0
    residual_time_s: float = 0.0
    constraint_jacobian_calls: int = 0
    constraint_jacobian_time_s: float = 0.0
    force_evaluations: int = 0
    force_time_s: float = 0.0
    mass_inverse_calls: int = 0
    mass_inverse_time_s: float = 0.0
    reaction_time_s: float = 0.0
    linear_factorizations: int = 0
    linear_factorization_time_s: float = 0.0
    linear_solves: int = 0
    linear_solve_time_s: float = 0.0
    line_search_trials: int = 0
    newton_iterations: int = 0
    accepted_steps: int = 0
    rejected_attempts: int = 0
    analytic_jacobian_columns: int = 0
    finite_difference_jacobian_columns: int = 0
    nonsmooth_fallback_columns: int = 0
    analytic_jacobian_time_s: float = 0.0
    finite_difference_jacobian_time_s: float = 0.0


@dataclass(frozen=True)
class AxleDynamicsResult:
    times_s: np.ndarray
    body_names: tuple[str, ...]
    constraint_names: tuple[str, ...]
    spring_names: tuple[str, ...]
    bushing_names: tuple[str, ...]
    anti_roll_bar_names: tuple[str, ...]
    tire_names: tuple[str, ...]
    states: np.ndarray
    constraint_wrench: np.ndarray
    spring_output: np.ndarray
    bushing_output: np.ndarray
    anti_roll_output: np.ndarray
    diagnostics: AxleRunDiagnostics
    tire_output: np.ndarray
    energy: np.ndarray
    contact_events: tuple[AxleContactEventRecord, ...] = ()
    performance: AxleRunPerformance = field(default_factory=AxleRunPerformance)

    def body_state(self, body: str) -> np.ndarray:
        """Return pose, velocity, omega, acceleration, and alpha samples."""
        return self.states[:, self.body_names.index(body), :]

    def tire_state(self, tire: str) -> np.ndarray:
        """Return the documented 15-column contact state for one tire."""
        return self.tire_output[:, self.tire_names.index(tire), :]

    def joint_wrench_on_body_b(self, joint: str) -> np.ndarray:
        """Return world force and marker-referenced moment on joint body_b."""
        return self.constraint_wrench[:, self.constraint_names.index(joint), :]

    def spring_state(self, spring: str) -> np.ndarray:
        """Return length, rate, force components, and total axial force."""
        return self.spring_output[:, self.spring_names.index(spring), :]

    def bushing_state(self, bushing: str) -> np.ndarray:
        """Return local deformation and local wrench on body_b."""
        return self.bushing_output[:, self.bushing_names.index(bushing), :]

    def anti_roll_bar_state(self, bar: str) -> np.ndarray:
        """Return relative angle, rate, and torque on body_b."""
        return self.anti_roll_output[:, self.anti_roll_bar_names.index(bar), :]
