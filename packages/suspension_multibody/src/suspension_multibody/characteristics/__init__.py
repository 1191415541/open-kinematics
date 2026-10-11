"""
The characteristics layer: measurement, motion and centre algorithms.

This package owns the *pure operators* -- turning declared measurements and
already-computed native responses into named characteristic values.  Two
boundaries define it, and both are enforced by the architecture gate rather than
by convention:

* it depends only on low-level data and spatial algebra, and imports neither
  ``results`` nor the authoring layer, ``cases``, or the solver.  A module that
  could read a result envelope would be free to grow a second implementation of
  a formula that already has an owner;
* it performs no solving.  Asking the native side for a response is the
  compiler's and the runner's job; this layer answers questions about responses
  it is handed.

Nothing here is re-exported eagerly from the package root beyond the value types
and measurement operators, so importing ``characteristics`` does not drag in the
whole numeric stack.
"""

from __future__ import annotations

from .centers import (
    PITCH_CENTER_FORCE_VERSION,
    ROLL_CENTER_FORCE_VERSION,
    ROLL_CENTER_LEGACY_VERSION,
    ForceEquivalentLine,
    contact_motion_basis,
    force_equivalent_pitch_center,
    force_equivalent_roll_center,
    legacy_roll_center,
    legacy_roll_center_envelope_values,
    legacy_roll_center_primitives,
    legacy_roll_center_report_values,
    named_operators,
    require_operator,
    section_intersection,
    side_view_instant_center,
)
from .measurements import (
    ANGLE_UNITS,
    LENGTH_UNITS,
    axis_direction,
    camber_angle,
    direction_cosine,
    frame_position,
    legacy_lateral_axis_y,
    legacy_wheel_camber,
    legacy_wheel_toe,
    measurement_derivative,
    toe_angle,
    transformed_point,
    wheel_center,
)
from .motion import (
    ANGULAR_SPEED_FLOOR,
    RANK_TOLERANCE,
    RESULTANT_FLOOR,
    Screw,
    Twist,
    ackermann_error,
    audit_boundary,
    is_identifiable,
    kingpin_axis,
    kingpin_inclination,
    measurement_response,
    rank_and_null_space,
    relative_twist,
    sample_status_for,
    screw_from_twist,
    solve_motion,
)
from .registry import (
    MEASUREMENT_KINDS,
    Definition,
    definition,
    definitions,
    missing_requirements,
    require_domain,
)
from .values import (
    DOMAINS,
    SAMPLE_STATUSES,
    CharacteristicResult,
    CharacteristicsError,
    DerivativeValue,
    MeasurementValue,
    SampleIdentity,
    VectorValue,
    statuses_for,
)

__all__ = [
    "ANGLE_UNITS",
    "ANGULAR_SPEED_FLOOR",
    "DOMAINS",
    "LENGTH_UNITS",
    "MEASUREMENT_KINDS",
    "PITCH_CENTER_FORCE_VERSION",
    "RANK_TOLERANCE",
    "RESULTANT_FLOOR",
    "ROLL_CENTER_FORCE_VERSION",
    "ROLL_CENTER_LEGACY_VERSION",
    "SAMPLE_STATUSES",
    "CharacteristicResult",
    "CharacteristicsError",
    "Definition",
    "DerivativeValue",
    "ForceEquivalentLine",
    "MeasurementValue",
    "SampleIdentity",
    "Screw",
    "Twist",
    "VectorValue",
    "ackermann_error",
    "audit_boundary",
    "axis_direction",
    "camber_angle",
    "contact_motion_basis",
    "definition",
    "definitions",
    "direction_cosine",
    "force_equivalent_pitch_center",
    "force_equivalent_roll_center",
    "frame_position",
    "legacy_lateral_axis_y",
    "legacy_wheel_camber",
    "legacy_wheel_toe",
    "is_identifiable",
    "kingpin_axis",
    "kingpin_inclination",
    "legacy_roll_center",
    "legacy_roll_center_envelope_values",
    "legacy_roll_center_primitives",
    "legacy_roll_center_report_values",
    "measurement_derivative",
    "measurement_response",
    "missing_requirements",
    "named_operators",
    "rank_and_null_space",
    "relative_twist",
    "require_domain",
    "require_operator",
    "sample_status_for",
    "screw_from_twist",
    "section_intersection",
    "side_view_instant_center",
    "solve_motion",
    "statuses_for",
    "toe_angle",
    "transformed_point",
    "wheel_center",
]
