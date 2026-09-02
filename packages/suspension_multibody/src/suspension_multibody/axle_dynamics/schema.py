"""Closed SI schemas for the native axle dynamics solver."""

from __future__ import annotations

import math
from typing import Literal

import numpy as np
from pydantic import Field, field_validator, model_validator

from ..schema.common import StrictModel

Vec3Tuple = tuple[float, float, float]
QuaternionTuple = tuple[float, float, float, float]
Matrix3Tuple = tuple[tuple[float, float, float], ...]
Vector6Tuple = tuple[float, float, float, float, float, float]
Matrix6Tuple = tuple[tuple[float, float, float, float, float, float], ...]

# 这组参数同时用于 Python 轮胎模型与 native 边界，顺序必须保持稳定。
# 这些参数覆盖当前 native 所需的 PAC2002 纯滑移、联合滑移、松弛和
    # 回正力矩项。前 83 个参数保持历史布局；参数顺序同时是 native ABI
    # 的固定内存布局，新增参数只能追加到末尾，不能调整已有位置。
PAC2002_PARAMETER_NAMES = (
    "FNOMIN",
    "PCX1",
    "PDX1",
    "PDX2",
    "PKX1",
    "PKX2",
    "PEX1",
    "PEX2",
    "PHX1",
    "PHX2",
    "PVX1",
    "PVX2",
    "PCY1",
    "PDY1",
    "PDY2",
    "PKY1",
    "PKY2",
    "PEY1",
    "PEY2",
    "PHY1",
    "PHY2",
    "PVY1",
    "PVY2",
    "PDX3",
    "PEX3",
    "PEX4",
    "PKX3",
    "PDY3",
    "PEY3",
    "PEY4",
    "PKY3",
    "PHY3",
    "PVY3",
    "PVY4",
    "RBX1",
    "RBX2",
    "RCX1",
    "REX1",
    "REX2",
    "RHX1",
    "RBY1",
    "RBY2",
    "RBY3",
    "RCY1",
    "REY1",
    "REY2",
    "RHY1",
    "RHY2",
    "RVY1",
    "RVY2",
    "RVY3",
    "RVY4",
    "RVY5",
    "RVY6",
    "PTX1",
    "PTX2",
    "PTX3",
    "PTY1",
    "PTY2",
    "QBZ1",
    "QBZ2",
    "QBZ3",
    "QBZ4",
    "QBZ5",
    "QBZ9",
    "QBZ10",
    "QCZ1",
    "QDZ1",
    "QDZ2",
    "QDZ3",
    "QDZ4",
    "QDZ6",
    "QDZ7",
    "QDZ8",
    "QDZ9",
    "QEZ1",
    "QEZ2",
    "QEZ3",
    "QEZ4",
    "QEZ5",
    "QHZ1",
    "QHZ2",
    "QHZ3",
    "QHZ4",
    "SSZ1",
    "SSZ2",
    "SSZ3",
    "SSZ4",
    # Chrono ChPac02Tire::CalcFxyMz/CalcSigma*/CalcM* 使用的缩放和
    # 附加力矩参数。只能追加，不能改变前面 88 项的 ABI 位置。
    "LCX",
    "LCY",
    "LEX",
    "LEY",
    "LFZO",
    "LGAX",
    "LGAY",
    "LGAZ",
    "LHX",
    "LHY",
    "LKX",
    "LKY",
    "LKYG",
    "LMUX",
    "LMUY",
    "LMX",
    "LMY",
    "LRES",
    "LS",
    "LSGAL",
    "LSGKP",
    "LTR",
    "LVMX",
    "LVX",
    "LVY",
    "LVYKA",
    "LXAL",
    "LYKA",
    "LIP",
    "IP",
    "IP_NOM",
    "PPX1",
    "PPX2",
    "PPX3",
    "PPX4",
    "PPY1",
    "PPY2",
    "PPY3",
    "PPY4",
    "QPX1",
    "QPZ1",
    "QPZ2",
    "QSX1",
    "QSX2",
    "QSX3",
    "QSX4",
    "QSX5",
    "QSX7",
    "QSX8",
    "QSX9",
    "QSX10",
    "QSX11",
    "QSY1",
    "QSY2",
    "QSY3",
    "QSY4",
    "QSY5",
    "QSY6",
    "QSY7",
    "QSY8",
    "QTZ1",
    # Adams PAC2002 垂向接触和有效滚动半径参数；只能追加。
    "LCZ",
    "BREFF",
    "DREFF",
    "FREFF",
    "QREO",
    "QV1",
    "QV2",
    "QFCX1",
    "QFCY1",
    "QFCG1",
    "QFZ1",
    "QFZ2",
    "QFZ3",
    "QPFZ1",
    "LONGVL",
    "VXLOW",
    # Adams PAC2002 无带动力学模式下的陀螺回正力矩参数；只能追加。
    "LGYR",
    "MBELT",
)

PAC2002_PARAMETER_DEFAULTS = {
    "FNOMIN": 4850.0,
    "PCX1": 1.65,
    "PDX1": 1.0,
    "PDX2": 0.0,
    "PKX1": 120000.0 / 4850.0,
    "PKX2": 0.0,
    "PEX1": 0.0,
    "PEX2": 0.0,
    "PHX1": 0.0,
    "PHX2": 0.0,
    "PVX1": 0.0,
    "PVX2": 0.0,
    "PCY1": 1.3,
    "PDY1": 1.0,
    "PDY2": 0.0,
    "PKY1": -80000.0 / 4850.0,
    "PKY2": 0.0,
    "PEY1": 0.0,
    "PEY2": 0.0,
    "PHY1": 0.0,
    "PHY2": 0.0,
    "PVY1": 0.0,
    "PVY2": 0.0,
    "PDX3": 0.0,
    "PEX3": 0.0,
    "PEX4": 0.0,
    "PKX3": 0.0,
    "PDY3": 0.0,
    "PEY3": 0.0,
    "PEY4": 0.0,
    "PKY3": 0.0,
    "PHY3": 0.0,
    "PVY3": 0.0,
    "PVY4": 0.0,
    "RBX1": 0.0,
    "RBX2": 0.0,
    "RCX1": 1.0,
    "REX1": 0.0,
    "REX2": 0.0,
    "RHX1": 0.0,
    "RBY1": 0.0,
    "RBY2": 0.0,
    "RBY3": 0.0,
    "RCY1": 1.0,
    "REY1": 0.0,
    "REY2": 0.0,
    "RHY1": 0.0,
    "RHY2": 0.0,
    "RVY1": 0.0,
    "RVY2": 0.0,
    "RVY3": 0.0,
    "RVY4": 0.0,
    "RVY5": 0.0,
    "RVY6": 0.0,
    "PTX1": 1.0,
    "PTX2": 0.0,
    "PTX3": 0.0,
    "PTY1": 1.0,
    "PTY2": 1.0,
    "QBZ1": 0.0,
    "QBZ2": 0.0,
    "QBZ3": 0.0,
    "QBZ4": 0.0,
    "QBZ5": 0.0,
    "QBZ9": 0.0,
    "QBZ10": 0.0,
    "QCZ1": 1.0,
    "QDZ1": 0.0,
    "QDZ2": 0.0,
    "QDZ3": 0.0,
    "QDZ4": 0.0,
    "QDZ6": 0.0,
    "QDZ7": 0.0,
    "QDZ8": 0.0,
    "QDZ9": 0.0,
    "QEZ1": 0.0,
    "QEZ2": 0.0,
    "QEZ3": 0.0,
    "QEZ4": 0.0,
    "QEZ5": 0.0,
    "QHZ1": 0.0,
    "QHZ2": 0.0,
    "QHZ3": 0.0,
    "QHZ4": 0.0,
    "SSZ1": 0.0,
    "SSZ2": 0.0,
    "SSZ3": 0.0,
    "SSZ4": 0.0,
    "LCX": 1.0,
    "LCY": 1.0,
    "LEX": 1.0,
    "LEY": 1.0,
    "LFZO": 1.0,
    "LGAX": 1.0,
    "LGAY": 1.0,
    "LGAZ": 1.0,
    "LHX": 1.0,
    "LHY": 1.0,
    "LKX": 1.0,
    "LKY": 1.0,
    "LKYG": 1.0,
    "LMUX": 1.0,
    "LMUY": 1.0,
    "LMX": 1.0,
    "LMY": 1.0,
    "LRES": 1.0,
    "LS": 1.0,
    "LSGAL": 1.0,
    "LSGKP": 1.0,
    "LTR": 1.0,
    "LVMX": 1.0,
    "LVX": 1.0,
    "LVY": 1.0,
    "LVYKA": 1.0,
    "LXAL": 1.0,
    "LYKA": 1.0,
    "LIP": 1.0,
    "IP": 200000.0,
    "IP_NOM": 200000.0,
    "PPX1": 0.0,
    "PPX2": 0.0,
    "PPX3": 0.0,
    "PPX4": 0.0,
    "PPY1": 0.0,
    "PPY2": 0.0,
    "PPY3": 0.0,
    "PPY4": 0.0,
    "QPX1": 0.0,
    "QPZ1": 0.0,
    "QPZ2": 0.0,
    "QSX1": 0.0,
    "QSX2": 0.0,
    "QSX3": 0.0,
    "QSX4": 0.0,
    "QSX5": 0.0,
    "QSX7": 0.0,
    "QSX8": 0.0,
    "QSX9": 0.0,
    "QSX10": 0.0,
    "QSX11": 0.0,
    "QSY1": 0.0,
    "QSY2": 0.0,
    "QSY3": 0.0,
    "QSY4": 0.0,
    "QSY5": 0.0,
    "QSY6": 0.0,
    "QSY7": 0.0,
    "QSY8": 0.0,
    "QTZ1": 0.0,
    "LCZ": 1.0,
    "BREFF": 8.4,
    "DREFF": 0.27,
    "FREFF": 0.07,
    "QREO": 1.0,
    "QV1": 0.0,
    "QV2": 0.0,
    "QFCX1": 0.0,
    "QFCY1": 0.0,
    "QFCG1": 0.0,
    "QFZ1": 0.0,
    "QFZ2": 0.0,
    "QFZ3": 0.0,
    "QPFZ1": 0.0,
    "LONGVL": 16.6,
    "VXLOW": 1.0,
    "LGYR": 1.0,
    "MBELT": 0.0,
}


def _finite(values: tuple[float, ...], label: str) -> tuple[float, ...]:
    if any(not math.isfinite(value) for value in values):
        raise ValueError(f"{label} must contain finite values")
    return values


class AxleBody(StrictModel):
    """One rigid body expressed in SI units."""

    name: str = Field(min_length=1)
    mass_kg: float = Field(ge=0)
    inertia_kg_m2: Matrix3Tuple
    position_m: Vec3Tuple = (0.0, 0.0, 0.0)
    quaternion_body_to_world: QuaternionTuple = (1.0, 0.0, 0.0, 0.0)
    linear_velocity_m_per_s: Vec3Tuple = (0.0, 0.0, 0.0)
    angular_velocity_rad_per_s: Vec3Tuple = (0.0, 0.0, 0.0)
    fixed: bool = False

    @field_validator("inertia_kg_m2", mode="before")
    @classmethod
    def _validate_inertia(cls, value: object) -> Matrix3Tuple:
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError("inertia_kg_m2 must be a finite 3x3 matrix")
        if not np.allclose(matrix, matrix.T, atol=1e-12):
            raise ValueError("inertia_kg_m2 must be symmetric")
        return tuple(tuple(float(item) for item in row) for row in matrix)

    @field_validator(
        "position_m",
        "linear_velocity_m_per_s",
        "angular_velocity_rad_per_s",
    )
    @classmethod
    def _validate_position(cls, value: Vec3Tuple) -> Vec3Tuple:
        return _finite(value, "position_m")  # type: ignore[return-value]

    @field_validator("quaternion_body_to_world")
    @classmethod
    def _validate_quaternion(
        cls, value: QuaternionTuple
    ) -> QuaternionTuple:
        _finite(value, "quaternion_body_to_world")
        norm = math.sqrt(sum(item * item for item in value))
        if norm <= 1e-12:
            raise ValueError("quaternion_body_to_world must be nonzero")
        return tuple(item / norm for item in value)  # type: ignore[return-value]

    @model_validator(mode="after")
    def _physical_mass(self) -> AxleBody:
        inertia = np.asarray(self.inertia_kg_m2, dtype=float)
        if not self.fixed:
            if self.mass_kg <= 0:
                raise ValueError("free bodies require positive mass_kg")
            if float(np.min(np.linalg.eigvalsh(inertia))) <= 0:
                raise ValueError("free body inertia_kg_m2 must be positive definite")
        return self


class AxleJoint(StrictModel):
    """One ideal joint with body-local markers and axes."""

    name: str = Field(min_length=1)
    kind: Literal[
        "spherical",
        "revolute",
        "prismatic",
        "fixed",
        "universal",
        "constant_velocity",
        "cylindrical",
        "inplane",
    ]
    body_a: str
    body_b: str
    point_a_m: Vec3Tuple
    point_b_m: Vec3Tuple
    axis_a: Vec3Tuple = (0.0, 0.0, 1.0)
    axis_b: Vec3Tuple = (0.0, 0.0, 1.0)
    axis_a_secondary: Vec3Tuple = (0.0, 1.0, 0.0)
    axis_b_secondary: Vec3Tuple = (1.0, 0.0, 0.0)
    # CONVEL 的初始交叉轴相位关系。
    constant_velocity_angle_target: float = Field(default=0.0, ge=-2.0, le=2.0)

    @model_validator(mode="after")
    def _valid_joint(self) -> AxleJoint:
        if self.body_a == self.body_b:
            raise ValueError("joint bodies must be different")
        for value, label in (
            (self.point_a_m, "point_a_m"),
            (self.point_b_m, "point_b_m"),
            (self.axis_a, "axis_a"),
            (self.axis_b, "axis_b"),
            (self.axis_a_secondary, "axis_a_secondary"),
            (self.axis_b_secondary, "axis_b_secondary"),
        ):
            _finite(value, label)
        if self.kind in {
            "revolute",
            "prismatic",
            "universal",
            "constant_velocity",
            "cylindrical",
            "inplane",
        }:
            if np.linalg.norm(self.axis_a) <= 1e-12:
                raise ValueError("axis_a must be nonzero")
            # An in-plane primitive is defined by body A's plane normal alone.
            if self.kind != "inplane" and np.linalg.norm(self.axis_b) <= 1e-12:
                raise ValueError("axis_b must be nonzero")
        return self


class AxleCoordinateCoupler(StrictModel):
    """One SI linear relation between two ideal-joint coordinates."""

    name: str = Field(min_length=1)
    joint_a: str = Field(min_length=1)
    coordinate_a: Literal["rotation", "translation"]
    scale_a: float
    joint_b: str = Field(min_length=1)
    coordinate_b: Literal["rotation", "translation"]
    scale_b: float

    @model_validator(mode="after")
    def _valid_relation(self) -> AxleCoordinateCoupler:
        if self.joint_a == self.joint_b:
            raise ValueError("a coordinate coupler requires two different joints")
        if not math.isfinite(self.scale_a) or not math.isfinite(self.scale_b):
            raise ValueError("coordinate coupler scales must be finite")
        if abs(self.scale_a) <= 1e-12 or abs(self.scale_b) <= 1e-12:
            raise ValueError("coordinate coupler scales must be non-zero")
        return self


class AxleSpringDamper(StrictModel):
    """Passive axial spring-damper between two body-local points."""

    name: str = Field(min_length=1)
    body_a: str
    body_b: str
    point_a_m: Vec3Tuple
    point_b_m: Vec3Tuple
    stiffness_n_per_m: float = Field(ge=0)
    compression_damping_n_s_per_m: float = Field(ge=0)
    rebound_damping_n_s_per_m: float = Field(ge=0)
    free_length_m: float = Field(ge=0)
    minimum_length_m: float | None = Field(default=None, ge=0)
    maximum_length_m: float | None = Field(default=None, ge=0)
    compression_stop_stiffness_n_per_m: float = Field(default=0.0, ge=0)
    compression_stop_damping_n_s_per_m: float = Field(default=0.0, ge=0)
    rebound_stop_stiffness_n_per_m: float = Field(default=0.0, ge=0)
    rebound_stop_damping_n_s_per_m: float = Field(default=0.0, ge=0)
    # Measured force-velocity curve. When given it replaces the two constant
    # damping coefficients: a real shock is neither linear nor symmetric about
    # zero velocity, so fitting one to two constants would not be the measured
    # element. Positive force resists extension.
    damper_curve_velocity_m_per_s: tuple[float, ...] = ()
    damper_curve_force_n: tuple[float, ...] = ()
    # 源力学曲线：弹性挠度在压缩时为正，限位块穿透量为正。
    elastic_curve_deflection_m: tuple[float, ...] = ()
    elastic_curve_force_n: tuple[float, ...] = ()
    compression_stop_curve_penetration_m: tuple[float, ...] = ()
    compression_stop_curve_force_n: tuple[float, ...] = ()
    rebound_stop_curve_penetration_m: tuple[float, ...] = ()
    rebound_stop_curve_force_n: tuple[float, ...] = ()

    @model_validator(mode="after")
    def _valid_damper_curve(self) -> AxleSpringDamper:
        velocity = self.damper_curve_velocity_m_per_s
        force = self.damper_curve_force_n
        if len(velocity) != len(force):
            raise ValueError("damper curve velocity and force must pair up")
        if not velocity:
            return self
        if len(velocity) < 2:
            raise ValueError("a damper curve needs at least two points")
        _finite(velocity, "damper_curve_velocity_m_per_s")
        _finite(force, "damper_curve_force_n")
        if any(b <= a for a, b in zip(velocity, velocity[1:])):
            raise ValueError("damper curve velocity must strictly increase")
        return self

    @field_validator(
        "elastic_curve_deflection_m",
        "compression_stop_curve_penetration_m",
        "rebound_stop_curve_penetration_m",
    )
    @classmethod
    def _valid_force_curve_abscissa(
        cls, value: tuple[float, ...]
    ) -> tuple[float, ...]:
        values = tuple(float(item) for item in value)
        _finite(values, "force curve abscissa")
        if values and len(values) < 2:
            raise ValueError("a force curve needs at least two points")
        if any(b <= a for a, b in zip(values, values[1:])):
            raise ValueError("force curve abscissas must strictly increase")
        return values

    @field_validator(
        "elastic_curve_force_n",
        "compression_stop_curve_force_n",
        "rebound_stop_curve_force_n",
    )
    @classmethod
    def _valid_force_curve_ordinate(
        cls, value: tuple[float, ...]
    ) -> tuple[float, ...]:
        return _finite(
            tuple(float(item) for item in value), "force curve force"
        )

    @model_validator(mode="after")
    def _valid_force_curves(self) -> AxleSpringDamper:
        for abscissa, ordinate, label in (
            (
                self.elastic_curve_deflection_m,
                self.elastic_curve_force_n,
                "elastic",
            ),
            (
                self.compression_stop_curve_penetration_m,
                self.compression_stop_curve_force_n,
                "compression stop",
            ),
            (
                self.rebound_stop_curve_penetration_m,
                self.rebound_stop_curve_force_n,
                "rebound stop",
            ),
        ):
            if len(abscissa) != len(ordinate):
                raise ValueError(f"{label} force curve arrays must pair up")
            if abscissa and len(abscissa) < 2:
                raise ValueError(f"{label} force curve needs at least two points")
        return self

    @model_validator(mode="after")
    def _valid_stops(self) -> AxleSpringDamper:
        if (
            self.minimum_length_m is not None
            and self.maximum_length_m is not None
            and self.minimum_length_m >= self.maximum_length_m
        ):
            raise ValueError("minimum_length_m must be below maximum_length_m")
        if self.minimum_length_m is None and (
            self.compression_stop_stiffness_n_per_m > 0
            or self.compression_stop_damping_n_s_per_m > 0
        ):
            raise ValueError("compression stop parameters require minimum_length_m")
        if self.maximum_length_m is None and (
            self.rebound_stop_stiffness_n_per_m > 0
            or self.rebound_stop_damping_n_s_per_m > 0
        ):
            raise ValueError("rebound stop parameters require maximum_length_m")
        return self


class AxleBushing(StrictModel):
    """Passive six-axis bushing with optional SI force curves."""

    name: str = Field(min_length=1)
    body_a: str
    body_b: str
    point_a_m: Vec3Tuple
    point_b_m: Vec3Tuple
    frame_a_to_body_quaternion: QuaternionTuple = (1.0, 0.0, 0.0, 0.0)
    frame_b_to_body_quaternion: QuaternionTuple = (1.0, 0.0, 0.0, 0.0)
    reference_translation_in_frame_a_m: Vec3Tuple
    reference_quaternion_a_to_b: QuaternionTuple
    stiffness: Matrix6Tuple
    damping: Matrix6Tuple
    preload_in_frame_a_n_n_m: Vector6Tuple = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    # 平移曲线坐标为 m、力为 N；转动曲线坐标为 rad、力矩为 N*m。
    force_curves: tuple[tuple[tuple[float, float], ...], ...] = ()
    force_curve_interpolation: Literal["piecewise_linear", "akima"] = (
        "piecewise_linear"
    )
    rotation_coordinates: Literal["rotation_vector", "cardan_xyz"] = (
        "rotation_vector"
    )

    @field_validator("stiffness", "damping", mode="before")
    @classmethod
    def _validate_matrix6(cls, value: object) -> Matrix6Tuple:
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (6, 6) or not np.all(np.isfinite(matrix)):
            raise ValueError("bushing matrices must be finite 6x6 matrices")
        if not np.allclose(matrix, matrix.T, atol=1e-12):
            raise ValueError("bushing matrices must be symmetric")
        if float(np.min(np.linalg.eigvalsh(matrix))) < -1e-10:
            raise ValueError("bushing matrices must be positive semidefinite")
        return tuple(tuple(float(item) for item in row) for row in matrix)

    @field_validator(
        "frame_a_to_body_quaternion",
        "frame_b_to_body_quaternion",
        "reference_quaternion_a_to_b",
    )
    @classmethod
    def _normalize_quaternion(cls, value: QuaternionTuple) -> QuaternionTuple:
        _finite(value, "bushing quaternion")
        norm = math.sqrt(sum(item * item for item in value))
        if norm <= 1e-12:
            raise ValueError("bushing quaternions must be nonzero")
        return tuple(item / norm for item in value)  # type: ignore[return-value]

    @field_validator("force_curves")
    @classmethod
    def _validate_force_curves(
        cls, value: object
    ) -> tuple[tuple[tuple[float, float], ...], ...]:
        curves = tuple(
            tuple((float(x), float(y)) for x, y in curve)  # type: ignore[misc]
            for curve in value  # type: ignore[union-attr]
        )
        if curves and len(curves) != 6:
            raise ValueError("bushing force_curves must contain six axis curves")
        for curve in curves:
            if curve and len(curve) < 2:
                raise ValueError("each bushing force curve requires at least two samples")
            if any(not math.isfinite(x) or not math.isfinite(y) for x, y in curve):
                raise ValueError("bushing force curves must contain finite samples")
            if any(right[0] <= left[0] for left, right in zip(curve, curve[1:])):
                raise ValueError("bushing force curve abscissas must be strictly increasing")
        return curves


class AxleAntiRollBar(StrictModel):
    """Passive torsional coupling between left and right bodies."""

    name: str = Field(min_length=1)
    body_a: str
    body_b: str
    axis_a: Vec3Tuple
    reference_quaternion_a_to_b: QuaternionTuple
    stiffness_n_m_per_rad: float = Field(ge=0)
    damping_n_m_s_per_rad: float = Field(ge=0)

    @field_validator("reference_quaternion_a_to_b")
    @classmethod
    def _normalize_reference(
        cls, value: QuaternionTuple
    ) -> QuaternionTuple:
        _finite(value, "reference_quaternion_a_to_b")
        norm = math.sqrt(sum(item * item for item in value))
        if norm <= 1e-12:
            raise ValueError("reference_quaternion_a_to_b must be nonzero")
        return tuple(item / norm for item in value)  # type: ignore[return-value]

    @model_validator(mode="after")
    def _valid_axis(self) -> AxleAntiRollBar:
        _finite(self.axis_a, "axis_a")
        if np.linalg.norm(self.axis_a) <= 1e-12:
            raise ValueError("axis_a must be nonzero")
        return self


class AxleAerodynamicDrag(StrictModel):
    """Quadratic drag element carried by one body."""

    body: str
    application_point_m: Vec3Tuple = (0.0, 0.0, 0.0)
    forward_axis_local: Vec3Tuple = (1.0, 0.0, 0.0)
    coefficient_n_s2_per_m2: float = Field(ge=0)

    @model_validator(mode="after")
    def _valid_axis(self) -> AxleAerodynamicDrag:
        _finite(self.application_point_m, "application_point_m")
        _finite(self.forward_axis_local, "forward_axis_local")
        if np.linalg.norm(self.forward_axis_local) <= 1e-12:
            raise ValueError("aerodynamic forward axis must be nonzero")
        return self


class AxleTire(StrictModel):
    """
    Unilateral compliant tire contact attached to a wheel body.

    ``body`` receives the contact wrench and carries wheel inertia. Vehicle
    models may optionally provide ``frame_body`` for the non-spinning carrier
    used to evaluate the tire contact frame and road position.
    """

    name: str = Field(min_length=1)
    body: str
    center_local_m: Vec3Tuple = (0.0, 0.0, 0.0)
    frame_body: str | None = None
    frame_center_local_m: Vec3Tuple | None = None
    drive_torque_body: str | None = None
    drive_torque_reaction_body: str | None = None
    drive_torque_axis_local: Vec3Tuple | None = None
    spin_axis_local: Vec3Tuple = (0.0, 1.0, 0.0)
    forward_axis_local: Vec3Tuple = (1.0, 0.0, 0.0)
    unloaded_radius_m: float = Field(gt=0)
    maximum_compression_m: float = Field(gt=0)
    vertical_stiffness_n_per_m: float = Field(gt=0)
    vertical_damping_n_s_per_m: float = Field(ge=0)
    longitudinal_friction_coefficient: float = Field(gt=0)
    lateral_friction_coefficient: float = Field(gt=0)
    longitudinal_brush_stiffness_n_per_m: float = Field(gt=0)
    lateral_brush_stiffness_n_per_m: float = Field(gt=0)
    longitudinal_relaxation_length_m: float = Field(gt=0)
    lateral_relaxation_length_m: float = Field(gt=0)
    detached_relaxation_s: float = Field(gt=0)
    model_kind: Literal["native_brush", "pac2002_pure_slip"] = "native_brush"
    pac2002_parameter_source: Literal["user", "adams_builtin"] = "user"
    pac2002_mirror: bool | None = None
    pac2002_coefficients: dict[str, float] = Field(default_factory=dict)

    @field_validator("pac2002_coefficients")
    @classmethod
    def _finite_pac2002_coefficients(cls, value: dict[str, float]) -> dict[str, float]:
        if any(not key or not math.isfinite(float(item)) for key, item in value.items()):
            raise ValueError("pac2002_coefficients must contain finite numeric values")
        return {str(key): float(item) for key, item in value.items()}

    @model_validator(mode="after")
    def _valid_axes(self) -> AxleTire:
        _finite(self.spin_axis_local, "spin_axis_local")
        _finite(self.forward_axis_local, "forward_axis_local")
        spin = np.asarray(self.spin_axis_local, dtype=float)
        forward = np.asarray(self.forward_axis_local, dtype=float)
        if np.linalg.norm(spin) <= 1e-12 or np.linalg.norm(forward) <= 1e-12:
            raise ValueError("tire axes must be nonzero")
        if np.linalg.norm(np.cross(spin, forward)) <= 1e-12:
            raise ValueError("tire spin and forward axes must not be parallel")
        if self.maximum_compression_m >= self.unloaded_radius_m:
            raise ValueError("maximum_compression_m must be below tire radius")
        if (self.drive_torque_body is None) != (
            self.drive_torque_axis_local is None
        ):
            raise ValueError(
                "drive_torque_body and drive_torque_axis_local must be provided together"
            )
        if self.drive_torque_reaction_body is not None:
            if self.drive_torque_body is None:
                raise ValueError(
                    "drive_torque_reaction_body requires drive_torque_body"
                )
            if self.drive_torque_reaction_body == self.drive_torque_body:
                raise ValueError("drive torque body and reaction body must differ")
        if self.drive_torque_axis_local is not None:
            _finite(self.drive_torque_axis_local, "drive_torque_axis_local")
            if np.linalg.norm(self.drive_torque_axis_local) <= 1e-12:
                raise ValueError("drive_torque_axis_local must be nonzero")
        return self


class AxleDynamicsModel(StrictModel):
    """Complete native axle physical model."""

    schema_version: Literal[1] = 1
    name: str = Field(min_length=1)
    units: Literal["SI"] = "SI"
    coordinate_system: Literal["vehicle_x_rear_y_right_z_up"] = (
        "vehicle_x_rear_y_right_z_up"
    )
    bodies: tuple[AxleBody, ...]
    joints: tuple[AxleJoint, ...]
    springs: tuple[AxleSpringDamper, ...] = ()
    bushings: tuple[AxleBushing, ...] = ()
    anti_roll_bars: tuple[AxleAntiRollBar, ...] = ()
    tires: tuple[AxleTire, ...] = ()
    aerodynamic_drags: tuple[AxleAerodynamicDrag, ...] = ()
    coordinate_couplers: tuple[AxleCoordinateCoupler, ...] = ()
    gravity_m_per_s2: Vec3Tuple = (0.0, 0.0, -9.80665)

    @model_validator(mode="after")
    def _closed_model(self) -> AxleDynamicsModel:
        names = [body.name for body in self.bodies]
        if len(names) != len(set(names)):
            raise ValueError("body names must be unique")
        if not any(body.fixed for body in self.bodies):
            raise ValueError("an axle model requires at least one fixed fixture body")
        known = set(names)
        element_names = [
            *(joint.name for joint in self.joints),
            *(spring.name for spring in self.springs),
            *(bushing.name for bushing in self.bushings),
            *(bar.name for bar in self.anti_roll_bars),
            *(tire.name for tire in self.tires),
        ]
        if len(element_names) != len(set(element_names)):
            raise ValueError("joint, spring, and tire names must be unique")
        for joint in self.joints:
            if joint.body_a not in known or joint.body_b not in known:
                raise ValueError(f"joint {joint.name!r} references an unknown body")
        for spring in self.springs:
            if spring.body_a not in known or spring.body_b not in known:
                raise ValueError(f"spring {spring.name!r} references an unknown body")
        for bushing in self.bushings:
            if bushing.body_a not in known or bushing.body_b not in known:
                raise ValueError(f"bushing {bushing.name!r} references an unknown body")
        for bar in self.anti_roll_bars:
            if bar.body_a not in known or bar.body_b not in known:
                raise ValueError(
                    f"anti-roll bar {bar.name!r} references an unknown body"
                )
        for tire in self.tires:
            if tire.body not in known:
                raise ValueError(f"tire {tire.name!r} references an unknown body")
            if tire.frame_body is not None and tire.frame_body not in known:
                raise ValueError(
                    f"tire {tire.name!r} references an unknown frame body"
                )
            if (
                tire.drive_torque_body is not None
                and tire.drive_torque_body not in known
            ):
                raise ValueError(
                    f"tire {tire.name!r} references an unknown drive torque body"
                )
            if (
                tire.drive_torque_reaction_body is not None
                and tire.drive_torque_reaction_body not in known
            ):
                raise ValueError(
                    f"tire {tire.name!r} references an unknown drive torque reaction body"
                )
            if tire.frame_center_local_m is not None:
                _finite(tire.frame_center_local_m, "frame_center_local_m")
        joint_names = {joint.name for joint in self.joints}
        coupler_names = [coupler.name for coupler in self.coordinate_couplers]
        if len(coupler_names) != len(set(coupler_names)):
            raise ValueError("coordinate coupler names must be unique")
        for coupler in self.coordinate_couplers:
            if coupler.joint_a not in joint_names or coupler.joint_b not in joint_names:
                raise ValueError(
                    f"coordinate coupler {coupler.name!r} references an unknown joint"
                )
        _finite(self.gravity_m_per_s2, "gravity_m_per_s2")
        return self


class AxleSolverSettings(StrictModel):
    """
    Native time-integration settings.

    ``ggl_generalized_alpha`` remains the native default.  The explicit HHT
    mode is used when a comparison manifest pins the same Adams HHT alpha.
    """

    integrator: Literal["ggl_generalized_alpha", "hht"] = (
        "ggl_generalized_alpha"
    )
    rho_inf: float = Field(default=0.8, gt=0, le=1)
    hht_alpha: float = Field(default=-0.3, ge=-1.0 / 3.0, le=0)
    initialization_mode: Literal[
        "static_equilibrium", "provided_consistent_state"
    ] = "static_equilibrium"
    adaptive_step: bool = True
    internal_step_s: float = Field(default=0.00025, gt=0)
    minimum_step_s: float = Field(default=1e-6, gt=0)
    maximum_step_s: float = Field(default=0.001, gt=0)
    local_relative_tolerance: float = Field(default=1e-5, gt=0)
    local_position_tolerance_m: float = Field(default=1e-7, gt=0)
    local_angle_tolerance_rad: float = Field(default=1e-7, gt=0)
    local_velocity_tolerance_m_per_s: float = Field(default=1e-6, gt=0)
    local_angular_velocity_tolerance_rad_per_s: float = Field(
        default=1e-6, gt=0
    )
    local_brush_tolerance_m: float = Field(default=1e-7, gt=0)
    contact_event_tolerance_s: float = Field(default=1e-6, gt=0)
    max_newton_iterations: int = Field(default=20, ge=1, le=100)
    max_line_search_iterations: int = Field(default=10, ge=1, le=30)
    position_tolerance_m: float = Field(default=1e-8, gt=0)
    velocity_tolerance_m_per_s: float = Field(default=1e-7, gt=0)
    dynamics_tolerance: float = Field(default=1e-8, gt=0)
    increment_tolerance: float = Field(default=1e-8, gt=0)

    @model_validator(mode="after")
    def _step_bounds(self) -> AxleSolverSettings:
        if self.minimum_step_s > self.internal_step_s:
            raise ValueError("minimum_step_s must not exceed internal_step_s")
        if self.internal_step_s > self.maximum_step_s:
            raise ValueError("internal_step_s must not exceed maximum_step_s")
        return self


class AxleHarmonicRoad(StrictModel):
    """
    A road height that both solvers evaluate from the same closed form.

    A sampled sine has to be approximated by one solver or the other, which
    makes the comparison a test of the interpolation rather than of the
    physics.  Declaring the harmonic analytically lets each side evaluate
    `offset + amplitude * sin(2*pi*frequency*t + phase)` exactly.
    """

    tire: str = Field(min_length=1)
    offset_m: float
    amplitude_m: float = Field(ge=0)
    frequency_hz: float = Field(gt=0)
    phase_rad: float = 0.0


class AxleDynamicsCase(StrictModel):
    """Sampled road and wheel-load inputs on one public time grid."""

    schema_version: Literal[1] = 1
    name: str = Field(min_length=1)
    times_s: tuple[float, ...]
    road_height_m: dict[str, tuple[float, ...]] = Field(default_factory=dict)
    road_velocity_m_per_s: dict[str, tuple[float, ...]] = Field(
        default_factory=dict
    )
    wheel_torque_n_m: dict[str, tuple[float, ...]] = Field(default_factory=dict)
    body_wrench_n_n_m: dict[str, tuple[Vector6Tuple, ...]] = Field(
        default_factory=dict
    )
    # When a tire appears here its road height and velocity come from the
    # closed form rather than from the sampled tables, so no interpolation
    # enters the comparison on either side.
    harmonic_roads: tuple[AxleHarmonicRoad, ...] = ()
    solver: AxleSolverSettings = Field(default_factory=AxleSolverSettings)

    @model_validator(mode="after")
    def _signals_match(self) -> AxleDynamicsCase:
        if len(self.times_s) < 2:
            raise ValueError("times_s requires at least two samples")
        if any(not math.isfinite(value) for value in self.times_s):
            raise ValueError("times_s must be finite")
        if any(b <= a for a, b in zip(self.times_s, self.times_s[1:])):
            raise ValueError("times_s must be strictly increasing")
        # A declared harmonic owns its tire's road signal outright: the sampled
        # tables are (re)filled from the same closed form the exported Adams
        # dataset uses, so a round trip through the manifest reproduces exactly
        # the same numbers rather than being rejected as a double definition.
        declared = [road.tire for road in self.harmonic_roads]
        if len(declared) != len(set(declared)):
            raise ValueError("each tire may declare at most one harmonic road")
        for road in self.harmonic_roads:
            angle = [
                2.0 * math.pi * road.frequency_hz * time + road.phase_rad
                for time in self.times_s
            ]
            rate = 2.0 * math.pi * road.frequency_hz
            object.__setattr__(
                self,
                "road_height_m",
                {
                    **self.road_height_m,
                    road.tire: tuple(
                        road.offset_m + road.amplitude_m * math.sin(value)
                        for value in angle
                    ),
                },
            )
            object.__setattr__(
                self,
                "road_velocity_m_per_s",
                {
                    **self.road_velocity_m_per_s,
                    road.tire: tuple(
                        road.amplitude_m * rate * math.cos(value)
                        for value in angle
                    ),
                },
            )
        for signals in (
            self.road_height_m,
            self.road_velocity_m_per_s,
            self.wheel_torque_n_m,
        ):
            for name, values in signals.items():
                if len(values) != len(self.times_s):
                    raise ValueError(f"signal {name!r} length must match times_s")
                if any(not math.isfinite(value) for value in values):
                    raise ValueError(f"signal {name!r} must be finite")
        for name, values in self.body_wrench_n_n_m.items():
            if len(values) != len(self.times_s):
                raise ValueError(f"signal {name!r} length must match times_s")
            if any(
                not math.isfinite(component)
                for wrench in values
                for component in wrench
            ):
                raise ValueError(f"signal {name!r} must be finite")
        return self
