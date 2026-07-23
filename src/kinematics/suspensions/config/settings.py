"""
Suspension configuration models.

This module defines configuration structures for suspension systems, including units,
wheel parameters, and static alignment settings.
"""

from __future__ import annotations

from pydantic import BaseModel, ConfigDict, field_validator, model_validator

from kinematics.io.validation import PydanticVec3


class TireConfig(BaseModel):
    """
    Configuration parameters for a tire.

    Attributes:
        section_width: Section width in mm.
        static_radius_mm: Static wheel radius in mm.
        aspect_ratio: Optional legacy aspect ratio in [0, 1], used only for
            converting old rim-diameter tire specs. Not part of the active GUI
            tire definition.
    """

    model_config = ConfigDict(frozen=True, arbitrary_types_allowed=True)

    section_width: float
    static_radius_mm: float
    aspect_ratio: float = 0.55

    @field_validator("aspect_ratio")
    @classmethod
    def check_aspect_ratio(cls, v: float) -> float:
        if not 0 <= v <= 1:
            raise ValueError(f"aspect_ratio must be in [0, 1], got {v}")
        return v

    @field_validator("static_radius_mm")
    @classmethod
    def check_static_radius_mm(cls, v: float) -> float:
        if v <= 0:
            raise ValueError(f"static_radius_mm must be positive, got {v}")
        return v

    @field_validator("section_width")
    @classmethod
    def check_section_width(cls, v: float) -> float:
        if v <= 0:
            raise ValueError(f"section_width must be positive, got {v}")
        return v

    @property
    def nominal_radius(self) -> float:
        """
        Return static tire radius in mm.
        """
        return self.static_radius_mm


class WheelConfig(BaseModel):
    """
    Configuration parameters for a wheel and tire assembly.

    Attributes:
        offset: Wheel offset (ET) from hub mounting face to wheel center plane
            in mm. Positive means wheel centerline is inboard of the hub face.
        tire: Tire configuration parameters.
    """

    model_config = ConfigDict(frozen=True, arbitrary_types_allowed=True)

    offset: float
    tire: TireConfig


class CamberShimConfig(BaseModel):
    """
    Configuration for a camber shim adjustment.

    This type of shim sits outboard of the top balljoint, effectively splitting the
    upright in two. A local assembly solve rotates the UBJ-side shim block about the
    upper ball joint and the lower upright body about the lower ball joint while the
    shim faces remain parallel at the requested setup thickness.

    The shim geometry is defined by two ordered dowel datum points (A, B) on the
    nominal mid-thickness plane, plus a shared face normal. The design upper and
    lower face positions are derived by offsetting +/- 0.5 * design_thickness
    along the normal from each datum point.

    Attributes:
        shim_face_point_a: First dowel datum on the design mid-thickness plane (mm).
        shim_face_point_b: Second dowel datum on the design mid-thickness plane (mm).
        shim_face_normal: Unit vector perpendicular to the design shim faces.
        design_thickness: Shim stack thickness in mm at design condition.
        setup_thickness: Actual shim stack thickness in mm for this configuration.
    """

    model_config = ConfigDict(frozen=True, arbitrary_types_allowed=True)

    shim_face_point_a: PydanticVec3
    shim_face_point_b: PydanticVec3
    shim_face_normal: PydanticVec3
    design_thickness: float
    setup_thickness: float

    @model_validator(mode="after")
    def validate_face_definition(self) -> "CamberShimConfig":
        import numpy as np

        from kinematics.core.constants import EPS_GEOMETRIC

        magnitude = float(
            np.linalg.norm(np.asarray(self.shim_face_normal, dtype=np.float64))
        )
        if magnitude < EPS_GEOMETRIC:
            raise ValueError("shim_face_normal vector is near-zero")

        datum_separation = float(
            np.linalg.norm(
                np.asarray(self.shim_face_point_b, dtype=np.float64)
                - np.asarray(self.shim_face_point_a, dtype=np.float64)
            )
        )
        if datum_separation < EPS_GEOMETRIC:
            raise ValueError("shim_face_point_a and shim_face_point_b must be distinct")

        return self


class SuspensionConfig(BaseModel):
    """
    Complete configuration for a suspension system.

    Attributes:
        steered: Whether this suspension corner is steered.
        wheel: Wheel configuration parameters.
        cg_position: Center of gravity position in mm (required for anti-dive/squat).
        wheelbase: Wheelbase distance in mm.
        static_camber_deg: Design-condition camber in degrees. Negative tilts the
            top of the wheel inward.
        static_toe_deg: Design-condition toe in degrees. Positive is toe-in.
        axle_length_mm: Distance between axle inboard and outboard hardpoints in mm.
        camber_shim: Optional camber shim configuration.
        upright_mounted_points: List of point names mounted to the upright that should
            move when camber shims are applied.
    """

    model_config = ConfigDict(frozen=True, arbitrary_types_allowed=True)

    steered: bool
    wheel: WheelConfig
    cg_position: PydanticVec3
    wheelbase: float
    static_camber_deg: float = 0.0
    static_toe_deg: float = 0.0
    axle_length_mm: float = 150.0
    camber_shim: CamberShimConfig | None = None
    upright_mounted_points: list[str] = [
        "axle_inboard",
        "axle_outboard",
        "pushrod_outboard",
        "trackrod_outboard",
    ]

    @field_validator("axle_length_mm")
    @classmethod
    def check_axle_length_mm(cls, v: float) -> float:
        if v <= 0:
            raise ValueError(f"axle_length_mm must be positive, got {v}")
        return v
