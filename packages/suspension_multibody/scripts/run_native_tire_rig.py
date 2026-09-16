r"""
native 轮胎台架：单胎、规定工况、输出与 Adams 单胎台架同名的通道.

对标对象是随装 Adams 台架 ``install_dir/atire/models/tire_testrig_analysis_1``
（结构见 ``.codex-tasks/20260912-native-fiala-parity/tasks/11-native-tire-rig/INVESTIGATION.md``）。
Adams 台架用 MOTION 规定纵向速度与侧偏角；native 的轴级 ABI 没有通用运动学驱动，
所以这里改用：

* **自由体 + 大质量惰走** 代替规定纵向速度（ΔV = Fx·t/M，可按需收紧）；
* **逐工况几何设 `forward_axis_local`** 代替时变侧偏角 MOTION；
* **`body_wrench` 施加恒定 −Fz** 代替 Adams 的 SFORCE 恒力（垂向自由度自由，与 Adams 同）；
* **自旋自由**（与 Adams 的 ``DEACTIVATE/MOTION, ID=35`` 同）。

坐标：轴级模型声明 ``vehicle_x_rear_y_right_z_up``，即 **x 指向车尾**，故前向 = −x。

用法::

    uv run python packages/suspension_multibody/scripts/run_native_tire_rig.py \\
        --tir "G:/MSC.Software/Adams/2025_1_1/acar/acar_concept.cdb/tires.tbl/fiala_235_45R17.tir"
"""

from __future__ import annotations

import argparse
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np

from suspension_multibody.adams.full_vehicle_model import (
    _parse_tire,
    parse_tire_tables,
)
from suspension_multibody.axle_dynamics import (
    AxleBody,
    AxleDrivenCoordinate,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleTire,
)
from suspension_multibody.axle_dynamics.native import _run_native
from suspension_multibody.axle_dynamics.result import TIRE_OUTPUT_COLUMNS
from suspension_multibody.pac2002_scope import validate_pac2002_native_scope

#: 前向单位向量（轴级坐标 x 指向车尾）。
FORWARD = (-1.0, 0.0, 0.0)

_LENGTH_TO_M = {"M": 1.0, "METER": 1.0, "METRE": 1.0, "MM": 1.0e-3, "CM": 1.0e-2}


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _rotate_about(vector, axis, angle: float):
    """Rodrigues 旋转."""
    norm = math.sqrt(sum(c * c for c in axis))
    unit = tuple(c / norm for c in axis)
    cosine, sine = math.cos(angle), math.sin(angle)
    cross = _cross(unit, vector)
    dot = sum(u * v for u, v in zip(unit, vector))
    return tuple(
        vector[i] * cosine + cross[i] * sine + unit[i] * dot * (1.0 - cosine)
        for i in range(3)
    )


def parse_fiala_tir(path: Path) -> dict[str, float]:
    """
    读 Fiala ``.tir`` 的关键项，换算到 SI.

    只读 ``[MODEL]`` / ``[UNITS]`` / ``[DIMENSION]`` / ``[PARAMETER]`` 四段；
    ``[SHAPE]`` 与接触模型不参与力律（见 INVESTIGATION.md §5）。两列表
    （``[DEFLECTION_LOAD_CURVE]`` 等）走 :func:`parse_fiala_tables`；这里只带回点数，
    好让 :func:`build_rig` 能校验调用方没有漏传表。
    """
    text = path.read_text(encoding="utf-8", errors="replace")
    sections: dict[str, dict[str, str]] = {}
    current = ""
    for raw in text.splitlines():
        line = raw.split("$")[0].split("!")[0].strip()
        if not line:
            continue
        match = re.fullmatch(r"\[([A-Z_]+)\]", line)
        if match:
            current = match.group(1)
            sections.setdefault(current, {})
            continue
        if "=" in line and current:
            key, _, value = line.partition("=")
            sections[current][key.strip().upper()] = value.strip().strip("'\"")

    units = sections.get("UNITS", {})
    length_unit = units.get("LENGTH", "M").upper()
    angle_unit = units.get("ANGLE", "RAD").upper()
    length = _LENGTH_TO_M.get(length_unit)
    if length is None:
        raise ValueError(f"unsupported LENGTH unit {length_unit!r} in {path}")

    model = sections.get("MODEL", {})
    dimension = sections.get("DIMENSION", {})
    parameter = sections.get("PARAMETER", {})

    def number(section: dict[str, str], key: str, default: float | None = None) -> float:
        if key not in section:
            if default is None:
                raise ValueError(f"{path}: missing {key}")
            return default
        return float(section[key].split()[0])

    # CALPHA 的单位随文件的 ANGLE 声明走：本胎是 N/deg，内核按 N/rad 用。
    calpha = number(parameter, "CALPHA")
    if angle_unit.startswith("DEG"):
        calpha = calpha * 180.0 / math.pi

    unloaded_radius = number(dimension, "UNLOADED_RADIUS") * length
    # 本胎不写 MAXIMUM_COMPRESSION；缺省按半径的 0.99（与整车模型读出的 318.78 mm 一致）。
    maximum_compression = (
        number(parameter, "MAXIMUM_COMPRESSION") * length
        if "MAXIMUM_COMPRESSION" in parameter
        else 0.99 * unloaded_radius
    )

    return {
        "USE_MODE": number(model, "USE_MODE", 2.0),
        "UNLOADED_RADIUS": unloaded_radius,
        "WIDTH": number(dimension, "WIDTH") * length,
        "VERTICAL_STIFFNESS": number(parameter, "VERTICAL_STIFFNESS"),
        "VERTICAL_DAMPING": number(parameter, "VERTICAL_DAMPING"),
        "MAXIMUM_COMPRESSION": maximum_compression,
        "CSLIP": number(parameter, "CSLIP"),
        "CALPHA": calpha,
        "UMIN": number(parameter, "UMIN"),
        "UMAX": number(parameter, "UMAX"),
        "ROLLING_RESISTANCE": number(parameter, "ROLLING_RESISTANCE", 0.0) * length,
        # RELAX_LENGTH_X/Y 是唯一**不**按文件 LENGTH 换算的项：帮助文档对 Fiala 未给
        # 单位，而随装证据（tyr501.f 注释以米为准；车辆模型把 0.05 读成 50 mm）都指向
        # **米**。0.05 mm 对 235 mm 宽的胎不是有意义的松弛长度，0.05 m 才是。
        # 该项只在 USE_MODE ∈ {11,12}（瞬态开）下被读取，本胎是 2，故不影响当前对标。
        "RELAX_LENGTH_X": number(parameter, "RELAX_LENGTH_X", 0.05),
        "RELAX_LENGTH_Y": number(parameter, "RELAX_LENGTH_Y", 0.15),
        "DEFLECTION_LOAD_CURVE_POINTS": float(
            len(parse_fiala_tables(path).get("deflection_load_curve", ()))
        ),
    }


def parse_fiala_tables(path: Path) -> dict[str, tuple[tuple[float, float], ...]]:
    """
    读 ``[DEFLECTION_LOAD_CURVE]`` / ``[BOTTOMING_CURVE]``，换算到 SI.

    表不能搭在标量参数载荷里，内核按每胎的曲线数组收（``VehicleInput`` 的
    ``tire_deflection_curve_*``），Python 侧由 ``AxleTire.pac2002_tables`` 承载。复用
    导入器里已有的实现，避免对同一段文本写第二套单位换算。
    """
    return parse_tire_tables(path)


@dataclass(frozen=True)
class RigManeuver:
    """
    台架工况.

    Adams 官方机动（``solver_run.acf``）：V = 20 m/s、Fz = 3000 N、α = ±15° 正弦、
    γ = 0、自旋初值 58.139535 rad/s、驱动/制动矩 0、1 s / 100 步。本版的 α 是**逐工况
    常量**（见 INVESTIGATION.md §6.2），正弦扫掠要另做。
    """

    duration_s: float = 1.0
    step_s: float = 0.01
    speed_m_per_s: float = 20.0
    normal_load_n: float = 3000.0
    initial_spin_rad_per_s: float = 58.139535
    slip_angle_rad: float = 0.0
    #: 官方机动的侧偏是正弦：`MOTION/32 = 0.261799·sin(2πt)`。给成 `(幅值, 频率 Hz)`
    #: 就走正弦，`slip_angle_rad` 只用于常量工况。两种情况下 toe yoke 的初始姿态都由
    #: `slip_angle_rad` 之外的正弦零点决定（正弦在 t = 0 处为 0，故装配姿态是单位阵）。
    slip_angle_sine_rad: tuple[float, float] | None = None
    camber_rad: float = 0.0
    #: 规定车轮自旋（驱动轮/测功机工况，对应 Adams 打开 MOTION/35）。给了它，自旋就从
    #: 自由自由度变成**驱动坐标**：车轮不再自己滚到自由滚动，驱动反力就是维持该转速所需的
    #: 力矩。速率由 `_rig_driven_rates` 给，位移目标由调用方按积分给出。
    driven_spin_rad_per_s: float | None = None

    @property
    def initial_slip_angle_rate_rad_per_s(self) -> float:
        """目标侧偏在 t = 0 的时间导数（速度级行要求初速与之一致）."""
        if self.slip_angle_sine_rad is None:
            return 0.0
        amplitude, frequency_hz = self.slip_angle_sine_rad
        return amplitude * 2.0 * math.pi * frequency_hz
    spin_inertia_kg_m2: float = 1.0
    carriage_mass_kg: float = 1.0e5
    #: 垂向自由度上的质量。Adams 台架该自由度上是 10.632+20 = 30.632 kg。
    vertical_mass_kg: float = 30.632
    #: 车轮体质量（只影响平动，自旋由 spin_inertia 决定）。
    wheel_mass_kg: float = 20.0


def _curve_penetration_for_load(
    rows: tuple[tuple[float, float], ...], load_n: float
) -> float:
    """
    在单调的 ``(贯入, 载荷)`` 表上反查 ``load_n`` 对应的贯入（分段线性）.

    只用来给台架一个**一致的初态**：垂向自由度是自由的，内核按三次样条求 Fz，故初值
    略偏也只会引入一段衰减瞬态。但初值差一倍（下表把刚度翻倍）会让 Fz、R_loaded、ω
    整条曲线都有振铃，所以必须反查而不是继续用 ``Fz/VERTICAL_STIFFNESS``。
    """
    if load_n <= 0.0 or len(rows) < 2:
        return 0.0
    for (d0, f0), (d1, f1) in zip(rows, rows[1:]):
        if f1 <= f0:
            continue
        if load_n <= f1:
            return d0 + (d1 - d0) * (load_n - f0) / (f1 - f0)
    (d0, f0), (d1, f1) = rows[-2], rows[-1]
    return d1 + (load_n - f1) * (d1 - d0) / (f1 - f0)


def _quaternion(axis, angle: float) -> tuple[float, float, float, float]:
    """
    轴角转四元数（w, x, y, z），轴必须是单位向量.

    初始姿态用它构造：驱动量是从**装配姿态**量起的，所以要让某个关节的坐标为 θ，
    装配时该体就得已经转过 θ（见 `_rig_chain` 与 §13）。
    """
    norm = math.sqrt(sum(c * c for c in axis))
    unit = tuple(c / norm for c in axis)
    half = 0.5 * angle
    sine = math.sin(half)
    return (math.cos(half), unit[0] * sine, unit[1] * sine, unit[2] * sine)


def _quaternion_multiply(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> tuple[float, float, float, float]:
    """Hamilton 积，约定与内核一致：`rotate(q, v)` 用的是同一个约定."""
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def _rig_chain(
    maneuver: RigManeuver, center_z: float, *, body_name: str
):
    """
    台车链 fixture → carrier → slider → toe → camber → wheel 的体与关节.

    与 Adams 台架的关节顺序一致：JOINT/1 纵向 translational、JOINT/3 垂向
    translational、JOINT/2 侧偏与 JOINT/4 外倾 revolute、JOINT/5 车轮自旋 revolute。
    纵向、侧偏、外倾由**驱动坐标**规定（Adams 的 MOTION/31、/32、/34），垂向自由度
    自由并由恒力加载（Adams 关掉 MOTION/33、用 SFORCE/5），自旋自由（MOTION/35 停用）。

    两个 yoke 是必要的：驱动旋转量是从装配姿态量起的关节坐标，只有真的存在那个转动
    自由度，它才好被规定。它们按 Adams 的 dummy 件取 1 g / 1e-6 kg·m²，对垂向质量与
    自旋惯量的扰动可忽略（Adams 自己的 dummy 也是 `mass=0.001`）。
    """
    speed = maneuver.speed_m_per_s
    velocity = (FORWARD[0] * speed, FORWARD[1] * speed, 0.0)
    _unit_inertia = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    _yoke_inertia = ((1.0e-6, 0.0, 0.0), (0.0, 1.0e-6, 0.0), (0.0, 0.0, 1.0e-6))

    # 装配姿态：先把侧偏转出来（绕 z），再把外倾转出来（绕转过侧偏之后的轮胎前向轴）。
    toe_quaternion = _quaternion((0.0, 0.0, 1.0), maneuver.slip_angle_rad)
    camber_axis = _rotate_about(FORWARD, (0.0, 0.0, 1.0), maneuver.slip_angle_rad)
    camber_quaternion = _quaternion(camber_axis, maneuver.camber_rad)
    combined = _quaternion_multiply(camber_quaternion, toe_quaternion)

    # 自旋轴取 **camber yoke 的局部常量轴**：yoke 的世界姿态里已经含了侧偏与外倾，
    # 把世界系的自旋轴交给 revolute 关节会在速度级破坏它的 5 行（实测 status 7
    # "initial velocity violates velocity constraints"）。外倾绕前向轴转，前向轴不变、
    # 自旋轴随之倾斜，正是外倾应有的效果。
    spin = _cross((0.0, 0.0, 1.0), FORWARD)
    spin_world = _rotate_about(
        _rotate_about(spin, (0.0, 0.0, 1.0), maneuver.slip_angle_rad),
        camber_axis,
        maneuver.camber_rad,
    )
    angular_velocity = tuple(
        component * maneuver.initial_spin_rad_per_s for component in spin_world
    )

    # 侧偏若正弦驱动，t = 0 处的角速度不为零，而速度级行要求它就等于目标速率
    # （`J q_dot = target_rate`），否则初态门直接拒绝（status 7）。所以 yoke 的初速要
    # 与目标函数的导数一致；车轮的自旋是**相对** camber yoke 的，故要把 yoke 的角速度
    # 加到车轮的绝对角速度上去。
    toe_rate = maneuver.initial_slip_angle_rate_rad_per_s
    camber_rate = 0.0
    toe_angular_velocity = (0.0, 0.0, toe_rate)
    camber_angular_velocity = tuple(
        toe_angular_velocity[index] + camber_rate * camber_axis[index]
        for index in range(3)
    )
    wheel_angular_velocity = tuple(
        camber_angular_velocity[index] + angular_velocity[index]
        for index in range(3)
    )

    bodies = (
        # 轴级 schema 要求至少一个 fixed 基座体。台架路面静止，正好由它承载：
        # 轮胎力是唯一的作用，故它不参与任何动力学。
        AxleBody(
            name="fixture",
            mass_kg=0.0,
            inertia_kg_m2=((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
            fixed=True,
        ),
        AxleBody(
            name="carrier",
            mass_kg=maneuver.carriage_mass_kg,
            inertia_kg_m2=_unit_inertia,
            position_m=(0.0, 0.0, center_z),
            linear_velocity_m_per_s=velocity,
        ),
        AxleBody(
            name="slider",
            mass_kg=maneuver.vertical_mass_kg,
            inertia_kg_m2=_unit_inertia,
            position_m=(0.0, 0.0, center_z),
            linear_velocity_m_per_s=velocity,
        ),
        AxleBody(
            name="toe_yoke",
            mass_kg=1.0e-3,
            inertia_kg_m2=_yoke_inertia,
            position_m=(0.0, 0.0, center_z),
            quaternion_body_to_world=toe_quaternion,
            linear_velocity_m_per_s=velocity,
            angular_velocity_rad_per_s=toe_angular_velocity,
        ),
        AxleBody(
            name="camber_yoke",
            mass_kg=1.0e-3,
            inertia_kg_m2=_yoke_inertia,
            position_m=(0.0, 0.0, center_z),
            quaternion_body_to_world=combined,
            linear_velocity_m_per_s=velocity,
            angular_velocity_rad_per_s=camber_angular_velocity,
        ),
        AxleBody(
            name=body_name,
            mass_kg=maneuver.wheel_mass_kg,
            inertia_kg_m2=(
                (maneuver.spin_inertia_kg_m2, 0.0, 0.0),
                (0.0, maneuver.spin_inertia_kg_m2, 0.0),
                (0.0, 0.0, maneuver.spin_inertia_kg_m2),
            ),
            position_m=(0.0, 0.0, center_z),
            quaternion_body_to_world=combined,
            linear_velocity_m_per_s=velocity,
            angular_velocity_rad_per_s=wheel_angular_velocity,
        ),
    )
    joints = (
        AxleJoint(
            name="longitudinal",
            kind="prismatic",
            body_a="fixture",
            body_b="carrier",
            point_a_m=(0.0, 0.0, center_z),
            point_b_m=(0.0, 0.0, 0.0),
            axis_a=(1.0, 0.0, 0.0),
            axis_b=(1.0, 0.0, 0.0),
        ),
        AxleJoint(
            name="vertical",
            kind="prismatic",
            body_a="carrier",
            body_b="slider",
            point_a_m=(0.0, 0.0, 0.0),
            point_b_m=(0.0, 0.0, 0.0),
            axis_a=(0.0, 0.0, 1.0),
            axis_b=(0.0, 0.0, 1.0),
        ),
        AxleJoint(
            name="toe",
            kind="revolute",
            body_a="slider",
            body_b="toe_yoke",
            point_a_m=(0.0, 0.0, 0.0),
            point_b_m=(0.0, 0.0, 0.0),
            axis_a=(0.0, 0.0, 1.0),
            axis_b=(0.0, 0.0, 1.0),
        ),
        AxleJoint(
            name="camber",
            kind="revolute",
            body_a="toe_yoke",
            body_b="camber_yoke",
            point_a_m=(0.0, 0.0, 0.0),
            point_b_m=(0.0, 0.0, 0.0),
            axis_a=FORWARD,
            axis_b=FORWARD,
        ),
        AxleJoint(
            name="spin",
            kind="revolute",
            body_a="camber_yoke",
            body_b=body_name,
            point_a_m=(0.0, 0.0, 0.0),
            point_b_m=(0.0, 0.0, 0.0),
            axis_a=spin,
            axis_b=spin,
        ),
    )
    return bodies, joints


def _rig_driven_coordinates(
    tire_name: str, *, driven_spin: bool = False
) -> tuple[AxleDrivenCoordinate, ...]:
    """
    台架的三个驱动坐标：纵向平移、侧偏、外倾（对应 Adams 的 MOTION/31、/32、/34）.

    轴按约定表达在 **reaction body** 的坐标系里：纵向是车架系里的 x，侧偏是滑块系里的 z，
    外倾是侧偏 yoke 系里的轮胎前向轴。零角参考取单位四元数，也就是装配姿态——
    正因如此，`RigManeuver` 的常量侧偏/外倾必须同时写进 yoke 的初始姿态。
    """
    coordinates = [
        AxleDrivenCoordinate(
            name=f"{tire_name}:longitudinal",
            kind="translation",
            body="carrier",
            reaction_body="fixture",
            axis_local=(1.0, 0.0, 0.0),
        ),
        AxleDrivenCoordinate(
            name=f"{tire_name}:toe",
            kind="rotation",
            body="toe_yoke",
            reaction_body="slider",
            axis_local=(0.0, 0.0, 1.0),
        ),
        AxleDrivenCoordinate(
            name=f"{tire_name}:camber",
            kind="rotation",
            body="camber_yoke",
            reaction_body="toe_yoke",
            axis_local=FORWARD,
        ),
    ]
    if driven_spin:
        # 规定自旋：ADAMS 的 MOTION/35 打开时就是这一条。轴在 camber yoke 的局部系里，
        # 与自旋关节同轴，所以它锁住的正是那个 revolute 的转角。
        coordinates.append(
            AxleDrivenCoordinate(
                name=f"{tire_name}:spin",
                kind="rotation",
                body="wheel",
                reaction_body="camber_yoke",
                axis_local=_cross((0.0, 0.0, 1.0), FORWARD),
            )
        )
    return tuple(coordinates)


def _rig_driven_targets(
    maneuver: RigManeuver, times: tuple[float, ...], tire_name: str
) -> dict[str, tuple[float, ...]]:
    """
    三个驱动坐标逐样本目标值.

    纵向是**位移**函数而不是速度：Adams 的 `MOTION/31 = 20·t` 就是位移，且从装配位置量起，
    所以目标是 `−speed·t`（native 前向为 −x，车架沿 −x 前进）。
    """
    toe = maneuver.slip_angle_sine_rad
    camber = maneuver.camber_rad
    spin = maneuver.driven_spin_rad_per_s
    driven: dict[str, tuple[float, ...]] = {
        f"{tire_name}:longitudinal": tuple(
            -maneuver.speed_m_per_s * time for time in times
        ),
        f"{tire_name}:camber": tuple(camber for _ in times),
    }
    if toe is None:
        driven[f"{tire_name}:toe"] = tuple(
            maneuver.slip_angle_rad for _ in times
        )
    else:
        amplitude, frequency_hz = toe
        driven[f"{tire_name}:toe"] = tuple(
            amplitude * math.sin(2.0 * math.pi * frequency_hz * time)
            for time in times
        )
    if spin is not None:
        # 位置级目标 = 速率的积分（从装配姿态起算），与 Adams 的位移型 MOTION 同式。
        driven[f"{tire_name}:spin"] = tuple(spin * time for time in times)
    return driven


def _rig_driven_rates(
    maneuver: RigManeuver, times: tuple[float, ...], tire_name: str
) -> dict[str, tuple[float, ...]]:
    """
    三个驱动坐标的**解析**目标速率.

    不能依赖 `_driven_buffers` 的 `np.gradient` 兜底：单边差分在端点与解析导数差
    O(h²)（正弦首点约 0.13%），而速度级行要求初速与目标速率严格一致，差这一点就被初态门
    拒绝（status 7）。这里直接给导数，顺带把正弦的相位也写清楚。
    """
    toe = maneuver.slip_angle_sine_rad
    rates: dict[str, tuple[float, ...]] = {
        f"{tire_name}:longitudinal": tuple(
            -maneuver.speed_m_per_s for _ in times
        ),
        f"{tire_name}:camber": tuple(0.0 for _ in times),
    }
    if toe is None:
        rates[f"{tire_name}:toe"] = tuple(0.0 for _ in times)
    else:
        amplitude, frequency_hz = toe
        omega = 2.0 * math.pi * frequency_hz
        rates[f"{tire_name}:toe"] = tuple(
            amplitude * omega * math.cos(omega * time) for time in times
        )
    spin = maneuver.driven_spin_rad_per_s
    if spin is not None:
        rates[f"{tire_name}:spin"] = tuple(spin for _ in times)
    return rates


def _rig_case(
    maneuver: RigManeuver,
    spin,
    *,
    tire_name: str,
    wheel_brake_torque_n_m: float,
) -> AxleDynamicsCase:
    """
    台架算例：恒定垂向载荷 + 可选的驱/制动矩（与胎模型无关）.

    恒定垂向载荷（世界 −z）施加在**垂向自由度**上（slider），与 Adams 的
    SFORCE/5 同：垂向自由，靠恒力与轮胎法向力平衡。
    """
    samples = int(round(maneuver.duration_s / maneuver.step_s)) + 1
    times = tuple(i * maneuver.step_s for i in range(samples))
    wrench = np.zeros((samples, 6))
    wrench[:, 2] = -maneuver.normal_load_n
    if wheel_brake_torque_n_m != 0.0:
        for axis, component in zip(spin, (3, 4, 5)):
            wrench[:, component] = -wheel_brake_torque_n_m * axis
    return AxleDynamicsCase(
        name="native_tire_rig_case",
        times_s=times,
        road_height_m={tire_name: tuple(0.0 for _ in times)},
        road_velocity_m_per_s={tire_name: tuple(0.0 for _ in times)},
        body_wrench_n_n_m={"slider": tuple(tuple(row) for row in wrench)},
        driven_target_m=_rig_driven_targets(maneuver, times, tire_name),
        driven_target_rate=_rig_driven_rates(maneuver, times, tire_name),
        solver=AxleSolverSettings(
            # 台架初态就有 20 m/s 平动与自旋，不能走静平衡初始化（会报
            # "static equilibrium initialization requires zero initial velocity"）；
            # 初态已按 Fz = k·pen 的平衡贯入构造，直接作为一致状态提交。
            initialization_mode="provided_consistent_state",
            adaptive_step=False,
            internal_step_s=maneuver.step_s,
            minimum_step_s=1.0e-9,
            maximum_step_s=maneuver.step_s,
        ),
    )


def build_rig(
    tir: dict[str, float],
    maneuver: RigManeuver,
    *,
    tire_name: str = "tire",
    body_name: str = "wheel",
    wheel_brake_torque_n_m: float = 0.0,
    tables: dict[str, tuple[tuple[float, float], ...]] | None = None,
) -> tuple[AxleDynamicsModel, AxleDynamicsCase]:
    """构造单胎台架模型与算例."""
    radius = tir["UNLOADED_RADIUS"]
    deflection_curve = (tables or {}).get("deflection_load_curve", ())
    # `DEFLECTION_LOAD_CURVE_POINTS` 只由 `parse_fiala_tir` 写入，故手搓的 tir 字典取缺省 0。
    if tir.get("DEFLECTION_LOAD_CURVE_POINTS", 0.0) and len(deflection_curve) < 2:
        raise ValueError(
            f"{tire_name}: the .tir carries [DEFLECTION_LOAD_CURVE] with "
            f"{tir.get('DEFLECTION_LOAD_CURVE_POINTS', 0.0):.0f} rows but no table was passed; "
            "call parse_fiala_tables(path) and pass tables=..."
        )
    # VERTICAL_STIFFNESS 在 .tir 里是 N/mm；换算成 N/m 再求平衡贯入。
    stiffness_n_per_m = tir["VERTICAL_STIFFNESS"] * 1000.0
    if len(deflection_curve) >= 2:
        # 有表时 Adams 用表取代刚度："you must specify VERTICAL_STIFFNESS in the tire
        # property file, but it does not play any role"。
        penetration = _curve_penetration_for_load(
            deflection_curve, maneuver.normal_load_n
        )
    else:
        penetration = maneuver.normal_load_n / stiffness_n_per_m
    center_z = radius - penetration

    # 轮胎的 forward/spin 轴现在挂在 **camber yoke** 上，且是**常量**：侧偏与外倾由
    # toe/camber 两个 revolute 关节真的转出来，yoke 的世界姿态里已经含了它们，
    # 所以局部轴不需要再逐工况旋转（这才是 Adams 的定义）。
    spin = _cross((0.0, 0.0, 1.0), FORWARD)
    bodies, joints = _rig_chain(maneuver, center_z, body_name=body_name)
    tire = AxleTire(
        name=tire_name,
        # body 是**自旋**的车轮体（承接接触力偶），frame_body 是**不旋转**的承载件
        # （提供轮胎坐标系与接触中心）。真实模型同理：body=spindle、frame_body=upright。
        # 若把 forward/spin 轴挂在自旋体上，forward 会随轮子翻滚，内核再把它投影到
        # 水平面（forward.z=0）时方向会翻转，滑移量随之失真。
        body=body_name,
        frame_body="camber_yoke",
        center_local_m=(0.0, 0.0, 0.0),
        frame_center_local_m=(0.0, 0.0, 0.0),
        spin_axis_local=spin,
        forward_axis_local=FORWARD,
        unloaded_radius_m=radius,
        maximum_compression_m=min(tir["MAXIMUM_COMPRESSION"], radius * 0.99),
        vertical_stiffness_n_per_m=stiffness_n_per_m,
        vertical_damping_n_s_per_m=tir["VERTICAL_DAMPING"] * 1000.0,
        longitudinal_friction_coefficient=1.0,
        lateral_friction_coefficient=1.0,
        longitudinal_brush_stiffness_n_per_m=1000.0,
        lateral_brush_stiffness_n_per_m=1000.0,
        longitudinal_relaxation_length_m=tir["RELAX_LENGTH_X"],
        lateral_relaxation_length_m=tir["RELAX_LENGTH_Y"],
        detached_relaxation_s=0.05,
        model_kind="fiala",
        fiala_parameters={
            "CSLIP": tir["CSLIP"],
            "CALPHA": tir["CALPHA"],
            "CGAMMA": 0.0,
            "USE_MODE": tir["USE_MODE"],
            "UMIN": tir["UMIN"],
            "UMAX": tir["UMAX"],
            "RELAX_LENGTH_X": tir["RELAX_LENGTH_X"],
            "RELAX_LENGTH_Y": tir["RELAX_LENGTH_Y"],
            "WIDTH": tir["WIDTH"],
            "ROLLING_RESISTANCE": tir["ROLLING_RESISTANCE"],
            "LOW_SPEED_THRESHOLD": 1.0e-3,
        },
        pac2002_tables=dict(tables or {}),
    )
    model = AxleDynamicsModel(
        name="native_tire_rig",
        # 台架的垂向载荷完全由 body_wrench 施加，故重力置零。Adams 台架同样如此：
        # 它的 SFORCE/5 函数里显式补上了 (10.632+20)*9.80665 这一项来抵消配重。
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=bodies,
        joints=joints,
        tires=(tire,),
        driven_coordinates=_rig_driven_coordinates(
            tire_name, driven_spin=maneuver.driven_spin_rad_per_s is not None
        ),
    )
    case = _rig_case(
        maneuver,
        spin,
        tire_name=tire_name,
        wheel_brake_torque_n_m=wheel_brake_torque_n_m,
    )
    return model, case


@dataclass(frozen=True)
class Pac2002RigTire:
    """
    PAC2002 台架胎：内核系数载荷（文件单位）+ 台架需要的 SI 标量.

    系数保持**文件单位**，与 ``vehicle_dynamics.py:1217`` 一致：内核在每个使用点各自
    换算，提前换到 SI 反而会二次换算。
    """

    coefficients: dict[str, float]
    radius_m: float
    maximum_compression_m: float
    vertical_stiffness_n_per_m: float
    vertical_damping_n_s_per_m: float
    tables: dict[str, tuple[tuple[float, float], ...]]
    #: 随装 Adams 库里的胎走内置模型，对应内核的 tire kind **2**（见 ``native.py:936``）。
    parameter_source: Literal["user", "adams_builtin"] = "adams_builtin"


def parse_pac2002_tir(path: Path) -> Pac2002RigTire:
    """
    读 PAC2002 ``.tir``：系数交给内核，另取台架需要的 SI 标量.

    ``_parse_tire``（``full_vehicle_model.py:4312``）就是导入器里那个既产
    ``pac2002_coefficients``、又给 Fiala 产标量的解析器；这里复用同一份，避免对同一段
    文本写第二套单位换算。它顺带归一出 ``*_MM`` 系列键（长度 mm、力 N、时间 s），所以
    对任何 LENGTH/FORCE 声明的文件都能换到 SI。越出 native 范围的胎由
    ``validate_pac2002_native_scope`` 直接拒绝。
    """
    coefficients = _parse_tire(path)
    validate_pac2002_native_scope(coefficients)
    radius = coefficients["UNLOADED_RADIUS_MM"] * 1.0e-3
    stiffness_n_per_m = coefficients.get("VERTICAL_STIFFNESS_N_MM", 0.0) * 1000.0
    if stiffness_n_per_m <= 0.0:
        raise ValueError(
            f"{path.name}: the rig needs VERTICAL_STIFFNESS to seed the initial "
            "penetration; the file does not declare a positive one"
        )
    # 初态贯入按 Fz = k·pen 反查。对本胎是**精确**的：它没有 [VERTICAL_COEFFICIENTS]，
    # 所以 QFZ1 走内核的 k·R/(FNOMIN·LCZ) 回退、QFZ2/QFZ3、QPFZ1、QV2、QFC* 全为 0，
    # 弹性力恰好退化成 k·pen（axle_kernel.cpp:1466）。带这些项的胎不能再线性反查，必须
    # 显式拒绝，免得悄悄给出一个非平衡初态。
    non_linear = [
        name
        for name in ("QFZ2", "QFZ3", "QPFZ1", "QV2", "QFCX1", "QFCY1", "QFCG1")
        if abs(float(coefficients.get(name, 0.0))) > 1.0e-12
    ]
    if non_linear:
        raise ValueError(
            f"{path.name}: the rig seeds the initial penetration from Fz = k*pen, which "
            f"is only exact without these vertical terms: {', '.join(non_linear)}"
        )
    return Pac2002RigTire(
        coefficients=coefficients,
        radius_m=radius,
        maximum_compression_m=0.99 * radius,
        vertical_stiffness_n_per_m=stiffness_n_per_m,
        vertical_damping_n_s_per_m=(
            coefficients.get("VERTICAL_DAMPING_N_S_MM", 0.05) * 1000.0
        ),
        tables=parse_tire_tables(path),
    )


def build_pac2002_rig(
    pac: Pac2002RigTire,
    maneuver: RigManeuver,
    *,
    tire_name: str = "tire",
    body_name: str = "wheel",
    wheel_brake_torque_n_m: float = 0.0,
) -> tuple[AxleDynamicsModel, AxleDynamicsCase]:
    """
    与 :func:`build_rig` 同一条台车链，只把胎换成 PAC2002 纯滑移模型.

    台架本身与胎模型无关，所以这里只重写轮胎那一块：换 ``model_kind``、换系数载荷、按
    PAC2002 的弹性力律反查初态贯入。
    """
    center_z = pac.radius_m - (
        maneuver.normal_load_n / pac.vertical_stiffness_n_per_m
    )
    spin = _cross((0.0, 0.0, 1.0), FORWARD)
    bodies, joints = _rig_chain(maneuver, center_z, body_name=body_name)

    coefficients = pac.coefficients
    nominal_load = float(
        coefficients.get("FNOMIN_N", coefficients.get("FNOMIN", 4850.0))
    )
    # 与 ``_adams_tire_spec`` / ``vehicle_dynamics`` 同源的换算。这几项只喂底层的刷子
    # 状态与阻尼，PAC2002 纯滑移力律不读它们（弛豫长度对 kind 2 由 PTX1/PTY1 决定，
    # 见 axle_kernel.cpp:5680），但 schema 要求为正。
    reference_relaxation = min(
        abs(float(coefficients.get("PTX1", 2.3657))) * pac.radius_m,
        abs(float(coefficients.get("PTY1", 2.1439))) * pac.radius_m,
    )
    friction = min(
        abs(float(coefficients.get("PDX1", 1.0))),
        abs(float(coefficients.get("PDY1", 1.0))),
    )
    tire = AxleTire(
        name=tire_name,
        body=body_name,
        frame_body="camber_yoke",
        center_local_m=(0.0, 0.0, 0.0),
        frame_center_local_m=(0.0, 0.0, 0.0),
        spin_axis_local=spin,
        forward_axis_local=FORWARD,
        unloaded_radius_m=pac.radius_m,
        maximum_compression_m=pac.maximum_compression_m,
        vertical_stiffness_n_per_m=pac.vertical_stiffness_n_per_m,
        vertical_damping_n_s_per_m=pac.vertical_damping_n_s_per_m,
        longitudinal_friction_coefficient=friction,
        lateral_friction_coefficient=friction,
        longitudinal_brush_stiffness_n_per_m=(
            abs(float(coefficients.get("PKX1", 22.303)))
            * nominal_load
            / reference_relaxation
        ),
        lateral_brush_stiffness_n_per_m=(
            abs(float(coefficients.get("PKY1", -21.92)))
            * nominal_load
            / reference_relaxation
        ),
        longitudinal_relaxation_length_m=reference_relaxation,
        lateral_relaxation_length_m=reference_relaxation,
        detached_relaxation_s=0.05,
        model_kind="pac2002_pure_slip",
        pac2002_parameter_source=pac.parameter_source,
        pac2002_coefficients=dict(coefficients),
        pac2002_tables=dict(pac.tables),
    )
    model = AxleDynamicsModel(
        name="native_tire_rig",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=bodies,
        joints=joints,
        tires=(tire,),
        driven_coordinates=_rig_driven_coordinates(
            tire_name, driven_spin=maneuver.driven_spin_rad_per_s is not None
        ),
    )
    case = _rig_case(
        maneuver,
        spin,
        tire_name=tire_name,
        wheel_brake_torque_n_m=wheel_brake_torque_n_m,
    )
    return model, case


def run_rig(model: AxleDynamicsModel, case: AxleDynamicsCase):
    """
    跑一次台架并返回轴级结果.

    公开入口 ``run_axle_dynamics`` 只接受 ``native_brush``：``native.py:936-955`` 把
    ``fiala`` 映射为 tire kind **3**，而 ``native.py:1193`` 在非 ``vehicle_mode`` 下
    拒绝任何 ``tire_model_kind != 0``。因此 Fiala 台架必须走**版本化整车接口**——
    与 ``vehicle_dynamics.py:256`` 的做法相同：调用 ``_run_native`` 并传入
    ``brake_torque`` 把它切到该接口。这里传全零，故不施加任何制动力矩。
    """
    zeros = {tire.name: tuple(0.0 for _ in case.times_s) for tire in model.tires}
    return _run_native(model, case, brake_torque=zeros).result


def _rotate_vectors_by_quaternions(
    vector, quaternions: np.ndarray
) -> np.ndarray:
    """把同一个局部向量按逐样本四元数转到世界系（(w,x,y,z) 约定）."""
    vector = np.asarray(vector, float)
    w = quaternions[:, 0]
    q = quaternions[:, 1:4]
    # v' = v + 2w(q x v) + 2q x (q x v)
    cross_qv = np.cross(q, vector)
    return vector + 2.0 * (w[:, None] * cross_qv + np.cross(q, cross_qv))


def report(
    result,
    tire_name: str,
    radius_m: float,
    forward,
    spin_axis,
    *,
    frame_body: str | None = None,
) -> dict[str, np.ndarray]:
    """
    输出与 Adams 台架同名的通道表.

    `forward`/`spin_axis` 的解释随 `frame_body` 而变：给了 `frame_body` 就当作**该体的
    局部轴**并逐样本按它的姿态转到世界系，不给就是世界系的常量轴。台架的侧偏与外倾现在
    由 toe/camber 关节真的转出来，轮胎平面随时间变化，所以必须给 `frame_body`
    （`"camber_yoke"`），否则重建的 `rolling_speed`/`kappa`/`R_e` 会落在错误的轴上。
    """
    columns = {name: i for i, name in enumerate(TIRE_OUTPUT_COLUMNS)}
    state = result.tire_state(tire_name)
    out = {
        "t": np.asarray(result.times_s, float),
        "penetration_m": state[:, columns["penetration_m"]],
        "normal_force_n": state[:, columns["normal_force_n"]],
        "longitudinal_force_n": state[:, columns["longitudinal_force_n"]],
        "lateral_force_n": state[:, columns["lateral_force_n"]],
        "aligning_moment_n_m": state[:, columns["aligning_moment_n_m"]],
        "overturning_moment_n_m": state[:, columns["overturning_moment_n_m"]],
        "rolling_resistance_moment_n_m": state[
            :, columns["rolling_resistance_moment_n_m"]
        ],
        "vx_m_per_s": state[:, columns["longitudinal_slip_velocity_m_per_s"]],
        "vy_m_per_s": state[:, columns["lateral_slip_velocity_m_per_s"]],
        "rolling_speed_m_per_s": state[:, columns["rolling_speed_m_per_s"]],
    }
    wheel = result.body_state("wheel")
    # 轴级输出的 rolling_speed / slip_reference 两列在 Fiala 路径下不填（是 PAC2002 的
    # 槽位），故这里按内核的定义自行重构：rolling_speed = |dot(v_wheel, forward)|、
    # spin_rate = dot(omega, spin_axis)、kappa = -vx / rolling_speed。
    velocity = np.asarray(wheel[:, 7:10], float)
    omega = np.asarray(wheel[:, 10:13], float)
    if frame_body is not None:
        quaternions = np.asarray(result.body_state(frame_body)[:, 3:7], float)
        forward = _rotate_vectors_by_quaternions(forward, quaternions)
        spin_axis = _rotate_vectors_by_quaternions(spin_axis, quaternions)
    else:
        forward = np.broadcast_to(np.asarray(forward, float), velocity.shape)
        spin_axis = np.broadcast_to(np.asarray(spin_axis, float), velocity.shape)
    out["rolling_speed_m_per_s"] = np.abs(
        np.einsum("ij,ij->i", velocity, forward)
    )
    out["kappa"] = -out["vx_m_per_s"] / np.maximum(out["rolling_speed_m_per_s"], 1e-12)
    # 轮胎侧偏角：由内核给出的横向/纵向滑移速度重建，不是把驱动目标回填成"结果"。
    # 符号与 Adams 的 `tire_kinematics.slip_angle` 对齐——官方正弦机动上实测内核的横向
    # 滑移速度与该通道反号（err_rms ≈ √2·幅值，正是纯反号的特征），故取负。
    out["slip_angle_rad"] = -np.arctan2(
        out["vy_m_per_s"], np.maximum(out["rolling_speed_m_per_s"], 1e-12)
    )
    out["rolling_radius_m"] = radius_m - out["penetration_m"]
    out["spin_rate_rad_per_s"] = np.einsum("ij,ij->i", omega, spin_axis)
    # PAC2002 的 `rolling_radius`（Adams 的 `tire_rolling_states.rolling_radius`）是**有效
    # 滚动半径** R_e，与几何的 R₀−贯入 不是一回事（差 4.7 mm 量级）。由内核自己的滑移
    # 定义反解：vx = V − ω·R_e ⇒ R_e = (rolling_speed − vx)/spin_rate。
    out["effective_rolling_radius_m"] = (
        out["rolling_speed_m_per_s"] - out["vx_m_per_s"]
    ) / np.where(np.abs(out["spin_rate_rad_per_s"]) > 1e-9, out["spin_rate_rad_per_s"], np.nan)
    return out


def main() -> int:
    """跑一次命令行指定的台架工况并打印通道表."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tir", type=Path, required=True)
    parser.add_argument("--duration-s", type=float, default=1.0)
    parser.add_argument("--step-s", type=float, default=0.01)
    parser.add_argument("--speed", type=float, default=20.0)
    parser.add_argument("--load", type=float, default=3000.0)
    parser.add_argument("--spin0", type=float, default=58.139535)
    parser.add_argument("--slip-angle-deg", type=float, default=0.0)
    parser.add_argument("--camber-deg", type=float, default=0.0)
    parser.add_argument("--spin-inertia", type=float, default=1.0)
    parser.add_argument("--carriage-mass", type=float, default=1.0e5)
    parser.add_argument(
        "--startup-smoothing",
        action="store_true",
        help="打开 Fiala 起动平滑（env SUSPENSION_AXLE_FIALA_STARTUP_SMOOTHING）",
    )
    parser.add_argument("--json", type=Path, help="把通道表写成 npz")
    args = parser.parse_args()

    if args.startup_smoothing:
        os.environ["SUSPENSION_AXLE_FIALA_STARTUP_SMOOTHING"] = "1"
    else:
        os.environ.pop("SUSPENSION_AXLE_FIALA_STARTUP_SMOOTHING", None)

    # 胎模型由文件自己声明（``_parse_tire`` 的 PROPERTY_FILE_FORMAT_PAC2002），台架本身
    # 与胎模型无关，所以命令行不必再要一个开关。
    is_pac2002 = bool(_parse_tire(args.tir).get("PROPERTY_FILE_FORMAT_PAC2002", 0.0))
    maneuver = RigManeuver(
        duration_s=args.duration_s,
        step_s=args.step_s,
        speed_m_per_s=args.speed,
        normal_load_n=args.load,
        initial_spin_rad_per_s=args.spin0,
        slip_angle_rad=math.radians(args.slip_angle_deg),
        camber_rad=math.radians(args.camber_deg),
        spin_inertia_kg_m2=args.spin_inertia,
        carriage_mass_kg=args.carriage_mass,
    )
    print(f"tir            : {args.tir}")
    if is_pac2002:
        pac = parse_pac2002_tir(args.tir)
        model, case = build_pac2002_rig(pac, maneuver)
        radius = pac.radius_m
        print(
            "tir (SI)       : PAC2002  R=%.6f m  WIDTH=%.4f m  k=%.6g N/m  "
            "USE_MODE=%g"
            % (
                pac.radius_m,
                pac.coefficients.get("WIDTH_MM", float("nan")) * 1.0e-3,
                pac.vertical_stiffness_n_per_m,
                pac.coefficients.get("USE_MODE", 14.0),
            )
        )
    else:
        tir = parse_fiala_tir(args.tir)
        tables = parse_fiala_tables(args.tir)
        model, case = build_rig(tir, maneuver, tables=tables)
        radius = tir["UNLOADED_RADIUS"]
        print(
            "tir (SI)       : FIALA  R=%.6f m  WIDTH=%.4f m  CSLIP=%.6g N  "
            "CALPHA=%.6g N/rad  k=%.6g N/m  USE_MODE=%g"
            % (
                tir["UNLOADED_RADIUS"],
                tir["WIDTH"],
                tir["CSLIP"],
                tir["CALPHA"],
                tir["VERTICAL_STIFFNESS"] * 1000.0,
                tir["USE_MODE"],
            )
        )
        if tir.get("DEFLECTION_LOAD_CURVE_POINTS", 0.0):
            curve = tables["deflection_load_curve"]
            print(
                "deflection     : %g 行表取代 VERTICAL_STIFFNESS；Fz=%.6g N 对应贯入 %.6f m"
                % (
                    tir["DEFLECTION_LOAD_CURVE_POINTS"],
                    maneuver.normal_load_n,
                    _curve_penetration_for_load(curve, maneuver.normal_load_n),
                )
            )
    result = run_rig(model, case)
    channels = report(result, "tire", radius, FORWARD, _cross((0.0, 0.0, 1.0), FORWARD))

    print(
        "maneuver       : V=%.4g m/s  Fz=%.6g N  alpha=%.6g rad  gamma=%.6g rad  "
        "omega0=%.6g rad/s  smoothing=%s"
        % (
            maneuver.speed_m_per_s,
            maneuver.normal_load_n,
            maneuver.slip_angle_rad,
            maneuver.camber_rad,
            maneuver.initial_spin_rad_per_s,
            args.startup_smoothing,
        )
    )
    print()
    print(
        "  %6s %10s %8s %11s %11s %11s %11s %10s"
        % (
            "t",
            "kappa%",
            "Fz_N",
            "Fx_N",
            "Fy_N",
            "Mz_Nm",
            "roll_rad_m",
            "omega",
        )
    )
    for index in range(channels["t"].size):
        if index % max(1, channels["t"].size // 12) and index != channels["t"].size - 1:
            continue
        print(
            "  %6.2f %10.5f %8.1f %11.5f %11.3f %11.3f %11.6f %10.4f"
            % (
                channels["t"][index],
                100.0 * channels["kappa"][index],
                channels["normal_force_n"][index],
                channels["longitudinal_force_n"][index],
                channels["lateral_force_n"][index],
                channels["aligning_moment_n_m"][index],
                channels["rolling_radius_m"][index],
                channels["spin_rate_rad_per_s"][index],
            )
        )
    print()
    print(
        "summary: Fz min/max = %.6g / %.6g N   rolling_radius min/max = %.6f / %.6f m"
        % (
            channels["normal_force_n"].min(),
            channels["normal_force_n"].max(),
            channels["rolling_radius_m"].min(),
            channels["rolling_radius_m"].max(),
        )
    )
    # `tir` 只在 Fiala 分支绑定，PAC2002 分支用 `pac`。原先这里无条件访问 `tir`，
    # 使 PAC2002 路径必抛 UnboundLocalError，`--json` 落盘也随之不可达。
    if not is_pac2002:
        print(
            "         kappa(0) = %.5f%%   Fx(0) = %.5f N   CSLIP*kappa(0) = %.5f N"
            % (
                100.0 * channels["kappa"][0],
                channels["longitudinal_force_n"][0],
                tir["CSLIP"] * channels["kappa"][0],
            )
        )
    else:
        print(
            "         kappa(0) = %.5f%%   Fx(0) = %.5f N   k=%.6g N/m"
            % (
                100.0 * channels["kappa"][0],
                channels["longitudinal_force_n"][0],
                pac.vertical_stiffness_n_per_m,
            )
        )
    if args.json is not None:
        np.savez(args.json, **channels)
        print(f"wrote          : {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
