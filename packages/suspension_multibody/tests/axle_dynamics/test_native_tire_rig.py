"""
native 单胎台架（``scripts/run_native_tire_rig.py``）的 Fiala 力律与滚阻矩回归.

台架的对标对象是随装 Adams 台架 ``atire/models/tire_testrig_analysis_1``。这里不读
``artifacts/``（该目录不入库），只断言两侧**已经逐通道对齐过**的解析关系，把它们冻结成
回归：单位换算、贯入/法向力、滑移构造、力律、以及滚阻矩的量级/符号/起动步进。

滚阻矩这一组是缺陷回归：Fiala 路径原先既**没有**把滚阻矩乘上 Adams 的起动步进
``s(t) = u²(3−2u)``（``u = t/0.1``），也**没有**写 ``rolling_resistance_moment`` 输出列。
随装台架在 RR = 0.02（mm）、Fz = 3000 N 时 t = 0.01 s 读数是 −0.001680 N·m，恰好等于
``−0.06·s(0.01)``；未加步进会在 t = 0 就施加满值 −0.06 N·m，使自旋减速的角冲量多出
``RR·Fz·T/2``。修正后该通道与 Adams 逐时刻吻合到 1.8e-16 N·m。
"""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from types import ModuleType

import numpy as np
import pytest

_SCRIPT = (
    Path(__file__).resolve().parents[2] / "scripts" / "run_native_tire_rig.py"
)

#: 随装 Fiala 试验胎（``fiala_235_45R17.tir``，``LENGTH = mm``）换算到 SI。
_TIR: dict[str, float] = {
    "USE_MODE": 2.0,
    "UNLOADED_RADIUS": 0.322,
    "WIDTH": 0.235,
    "VERTICAL_STIFFNESS": 310.0,  # N/mm
    "VERTICAL_DAMPING": 3.1,  # N/(mm/s)
    "MAXIMUM_COMPRESSION": 0.99 * 0.322,
    "CSLIP": 1000.0,
    "CALPHA": 800.0 * 180.0 / math.pi,
    "UMIN": 0.9,
    "UMAX": 1.0,
    "ROLLING_RESISTANCE": 0.0,
    "RELAX_LENGTH_X": 0.05,
    "RELAX_LENGTH_Y": 0.15,
    # `parse_fiala_tir` 的返回值形状：两列表的点数（本胎不带表）。
    "DEFLECTION_LOAD_CURVE_POINTS": 0.0,
}

#: Adams 台架官方机动的初值。
_LOAD_N = 3000.0
_SPEED_M_PER_S = 20.0
_SPIN0_RAD_PER_S = 58.139535

#: 0.02 mm（文件单位）换算到米；乘 Fz 后即 0.06 N·m。
_RR_METRES = 2.0e-5

#: 把台架胎的垂向刚度整体翻倍的表：7 mm→4340 N、14 mm→8680 N（即 620 N/mm）。
#: 同一 Fz = 3000 N 下贯入应减半（9.677 mm → 4.839 mm）。
_CURVE_SECTION = (
    "[DEFLECTION_LOAD_CURVE]\n"
    "{pen      fz}\n"
    "0        0.0\n"
    "7     4340.0\n"
    "14    8680.0\n"
)


def _tir_text(curve: str = "", rolling_resistance: float = 0.0) -> str:
    return (
        "[MDI_HEADER]\n FILE_TYPE = 'tir'\n"
        "[UNITS]\n LENGTH = 'mm'\n ANGLE = 'degree'\n"
        "[MODEL]\n PROPERTY_FILE_FORMAT = 'FIALA'\n USE_MODE = 2.0\n"
        "[DIMENSION]\n UNLOADED_RADIUS = 322.0\n WIDTH = 235.0\n"
        "[PARAMETER]\n VERTICAL_STIFFNESS = 310.0\n VERTICAL_DAMPING = 3.1\n"
        f" ROLLING_RESISTANCE = {rolling_resistance}\n CSLIP = 1000.0\n CALPHA = 800.0\n"
        " UMIN = 0.9\n UMAX = 1.0\n RELAX_LENGTH_X = 0.05\n RELAX_LENGTH_Y = 0.15\n"
        + curve
    )


def _pac2002_tir_text(extra_vertical: str = "", use_mode: int = 3) -> str:
    """随装 PAC2002 胎的最小复刻（LENGTH = meter、NSLIP = N，与真胎同量纲）."""
    return (
        "[MDI_HEADER]\n FILE_TYPE = 'tir'\n FILE_VERSION = 3.0\n"
        "[UNITS]\n LENGTH = 'meter'\n FORCE = 'newton'\n ANGLE = 'radian'\n"
        " MASS = 'kg'\n TIME = 'second'\n"
        f"[MODEL]\n PROPERTY_FILE_FORMAT = 'PAC2002'\n USE_MODE = {use_mode}\n"
        " VXLOW = 1\n LONGVL = 16.6\n TYRESIDE = 'LEFT'\n"
        "[DIMENSION]\n UNLOADED_RADIUS = 0.344\n WIDTH = 0.235\n"
        " ASPECT_RATIO = 0.6\n"
        "[VERTICAL]\n VERTICAL_STIFFNESS = 2.1e+005\n VERTICAL_DAMPING = 50\n"
        " BREFF = 8.4\n DREFF = 0.27\n FREFF = 0.07\n FNOMIN = 4850\n"
        + extra_vertical
        + "[LONGITUDINAL_COEFFICIENTS]\n PDX1 = 1.1739\n"
        "[LATERAL_COEFFICIENTS]\n PDY1 = 1.0489\n"
    )


def _rig() -> ModuleType:
    # 脚本要被登记进 ``sys.modules``，否则 ``@dataclass`` 解析注解时会拿不到模块词典。
    if "native_tire_rig" in sys.modules:
        return sys.modules["native_tire_rig"]
    specification = importlib.util.spec_from_file_location("native_tire_rig", _SCRIPT)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    sys.modules["native_tire_rig"] = module
    specification.loader.exec_module(module)
    return module


def _run(
    tir: dict[str, float],
    *,
    speed_m_per_s: float = _SPEED_M_PER_S,
    spin0_rad_per_s: float = _SPIN0_RAD_PER_S,
    duration_s: float = 0.2,
) -> dict[str, np.ndarray]:
    module = _rig()
    maneuver = module.RigManeuver(
        duration_s=duration_s,
        step_s=0.01,
        speed_m_per_s=speed_m_per_s,
        initial_spin_rad_per_s=spin0_rad_per_s,
    )
    model, case = module.build_rig(tir, maneuver)
    return module.report(
        module.run_rig(model, case),
        "tire",
        tir["UNLOADED_RADIUS"],
        module.FORWARD,
        module._cross((0.0, 0.0, 1.0), module.FORWARD),
    )


def _startup_step(time_s: np.ndarray) -> np.ndarray:
    """Adams 的起动步进：前 0.1 s 上的三次 STEP."""
    u = np.clip(time_s / 0.1, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def test_parsed_tir_carries_the_file_length_unit_into_the_parameters(tmp_path: Path) -> None:
    """``ROLLING_RESISTANCE`` 是长度而非系数，必须按文件的 LENGTH 换算."""
    path = tmp_path / "unit_probe.tir"
    path.write_text(_tir_text(rolling_resistance=0.02), encoding="utf-8")
    tir = _rig().parse_fiala_tir(path)

    assert tir["UNLOADED_RADIUS"] == pytest.approx(0.322)
    assert tir["WIDTH"] == pytest.approx(0.235)
    assert tir["ROLLING_RESISTANCE"] == pytest.approx(_RR_METRES)
    # CALPHA 随文件的 ANGLE 声明走：本胎是 N/deg，内核按 N/rad 用。
    assert tir["CALPHA"] == pytest.approx(800.0 * 180.0 / math.pi)
    # 缺省 MAXIMUM_COMPRESSION 取半径的 0.99。
    assert tir["MAXIMUM_COMPRESSION"] == pytest.approx(0.99 * 0.322)
    assert tir["DEFLECTION_LOAD_CURVE_POINTS"] == pytest.approx(0.0)


def test_parsed_deflection_curve_is_converted_to_si(tmp_path: Path) -> None:
    """``[DEFLECTION_LOAD_CURVE]`` 的两列都按文件单位换算：贯入 mm→m，载荷原样为 N."""
    path = tmp_path / "curve.tir"
    path.write_text(_tir_text(_CURVE_SECTION), encoding="utf-8")
    module = _rig()

    assert module.parse_fiala_tir(path)["DEFLECTION_LOAD_CURVE_POINTS"] == pytest.approx(3.0)
    curve = module.parse_fiala_tables(path)["deflection_load_curve"]
    np.testing.assert_allclose(
        curve, ((0.0, 0.0), (0.007, 4340.0), (0.014, 8680.0)), rtol=1e-12
    )


def test_deflection_load_curve_replaces_the_vertical_stiffness(tmp_path: Path) -> None:
    """带表时 Fz 由表给出，VERTICAL_STIFFNESS 不参与；平衡贯入由表反查."""
    path = tmp_path / "curve.tir"
    path.write_text(_tir_text(_CURVE_SECTION), encoding="utf-8")
    module = _rig()
    tir = module.parse_fiala_tir(path)
    tables = module.parse_fiala_tables(path)
    maneuver = module.RigManeuver(duration_s=0.2, step_s=0.01, speed_m_per_s=_SPEED_M_PER_S)

    model, case = module.build_rig(tir, maneuver, tables=tables)
    channels = module.report(
        module.run_rig(model, case),
        "tire", tir["UNLOADED_RADIUS"], module.FORWARD, module._cross((0.0, 0.0, 1.0), module.FORWARD),
    )

    # 表被翻倍 ⇒ 同一 Fz 下贯入恰好是无表时的一半（9.677 mm → 4.839 mm）。
    assert channels["penetration_m"][0] == pytest.approx(0.004838709677419355, rel=1e-12)
    assert channels["penetration_m"][0] == pytest.approx(
        0.5 * _LOAD_N / 310_000.0, rel=1e-9
    )
    # 垂向自由度自由，表与恒力平衡 ⇒ 整条曲线 Fz = 3000 N。
    np.testing.assert_allclose(channels["normal_force_n"], _LOAD_N, rtol=1e-9)
    assert channels["rolling_radius_m"][0] == pytest.approx(
        0.322 - 0.5 * _LOAD_N / 310_000.0, rel=1e-12
    )


def test_the_rig_refuses_a_curve_tire_without_its_table(tmp_path: Path) -> None:
    """带表却没传表时必须报错，而不是静默退回 VERTICAL_STIFFNESS."""
    path = tmp_path / "curve.tir"
    path.write_text(_tir_text(_CURVE_SECTION), encoding="utf-8")
    module = _rig()
    tir = module.parse_fiala_tir(path)

    with pytest.raises(ValueError, match="DEFLECTION_LOAD_CURVE"):
        module.build_rig(tir, module.RigManeuver())


def test_free_rolling_rig_reproduces_penetration_slip_and_the_fiala_force_law(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.delenv("SUSPENSION_AXLE_FIALA_STARTUP_SMOOTHING", raising=False)
    channels = _run(dict(_TIR), duration_s=1.0)

    loaded_radius = 0.322 - _LOAD_N / (310.0 * 1000.0)
    slip0 = -(_SPEED_M_PER_S - _SPIN0_RAD_PER_S * loaded_radius) / _SPEED_M_PER_S
    free_rolling_spin = _SPEED_M_PER_S / loaded_radius

    # 法向力与贯入：Fz = k·pen，台架自重由 3000 N 恒力平衡，故逐时刻恒定。
    np.testing.assert_allclose(channels["normal_force_n"], _LOAD_N, rtol=1e-12)
    np.testing.assert_allclose(channels["penetration_m"], _LOAD_N / 310_000.0, rtol=1e-12)

    # 滑移构造与纯纵滑力律：κ = (ωR − V)/V、Fx = CSLIP·κ。
    assert channels["kappa"][0] == pytest.approx(slip0, rel=1e-12)
    assert channels["longitudinal_force_n"][0] == pytest.approx(
        _TIR["CSLIP"] * slip0, rel=1e-9
    )
    # 自旋单调升向自由滚动，且不越过。
    spin = channels["spin_rate_rad_per_s"]
    assert np.all(np.diff(spin) > 0.0)
    assert spin[-1] < free_rolling_spin
    assert spin[-1] == pytest.approx(free_rolling_spin, rel=2e-3)
    # 侧向/回正/滚阻在纯纵滑工况下为零。
    np.testing.assert_allclose(channels["lateral_force_n"], 0.0, atol=1e-9)
    np.testing.assert_allclose(channels["aligning_moment_n_m"], 0.0, atol=1e-9)
    np.testing.assert_allclose(channels["rolling_resistance_moment_n_m"], 0.0, atol=1e-15)


def test_rolling_resistance_moment_follows_the_startup_step(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """RR > 0 时输出列必须等于 ``−RR·Fz·s(t)``（Adams 语义），不是 t = 0 就满值."""
    monkeypatch.setenv("SUSPENSION_AXLE_FIALA_STARTUP_SMOOTHING", "1")
    tir = dict(_TIR, ROLLING_RESISTANCE=_RR_METRES)
    channels = _run(tir)
    t = channels["t"]
    moment = channels["rolling_resistance_moment_n_m"]

    full = -_RR_METRES * _LOAD_N
    assert full == pytest.approx(-0.06)
    assert moment[0] == pytest.approx(0.0, abs=1e-15)
    # t = 0.05 处 s = 0.5。
    assert moment[5] == pytest.approx(0.5 * full, rel=1e-12)
    # 0.1 s 起满值。
    np.testing.assert_allclose(moment[t >= 0.1], full, rtol=1e-12)
    np.testing.assert_allclose(moment, full * _startup_step(t), rtol=1e-12, atol=1e-15)


def test_rolling_resistance_moment_is_absent_without_the_startup_step(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """平滑关（USE_MODE 11 或未开平滑）时步进为 1，t = 0 即满值."""
    monkeypatch.delenv("SUSPENSION_AXLE_FIALA_STARTUP_SMOOTHING", raising=False)
    tir = dict(_TIR, ROLLING_RESISTANCE=_RR_METRES, USE_MODE=11.0)
    channels = _run(tir)

    np.testing.assert_allclose(
        channels["rolling_resistance_moment_n_m"], -_RR_METRES * _LOAD_N, rtol=1e-12
    )


def test_rolling_resistance_moment_opposes_the_spin_in_both_directions(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """滚阻矩恒与自旋反向：正向滚动为负，反向滚动为正，且量级相同."""
    monkeypatch.setenv("SUSPENSION_AXLE_FIALA_STARTUP_SMOOTHING", "1")
    tir = dict(_TIR, ROLLING_RESISTANCE=_RR_METRES)
    forward = _run(tir)
    backward = _run(tir, speed_m_per_s=-_SPEED_M_PER_S, spin0_rad_per_s=-_SPIN0_RAD_PER_S)

    full = _RR_METRES * _LOAD_N
    assert forward["spin_rate_rad_per_s"][0] > 0.0
    assert backward["spin_rate_rad_per_s"][0] < 0.0
    np.testing.assert_allclose(
        forward["rolling_resistance_moment_n_m"], -full * _startup_step(forward["t"]),
        rtol=1e-12,
    )
    np.testing.assert_allclose(
        backward["rolling_resistance_moment_n_m"], full * _startup_step(backward["t"]),
        rtol=1e-12,
    )


def test_rolling_resistance_enters_the_spin_dynamics_with_the_startup_ramp(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    滚阻矩必须真的进入自旋方程，且带 0.1 s 的起动斜坡.

    Δω = ω(RR) − ω(RR=0) 就是滚阻矩的累积角冲量效应。t = 0.1 s 处它是斜坡的积分，
    对"是否施加斜坡"最敏感：带斜坡 −2.66e-3 rad/s，不带斜坡 −5.07e-3 rad/s。这两组
    参考值来自随装 Adams 台架（``rr_a0_fwd`` vs ``a0_um2``，Δω 差异 1.9e-4 rad/s）。
    """
    monkeypatch.setenv("SUSPENSION_AXLE_FIALA_STARTUP_SMOOTHING", "1")
    without = _run(dict(_TIR), duration_s=1.0)
    with_rr = _run(dict(_TIR, ROLLING_RESISTANCE=_RR_METRES), duration_s=1.0)

    delta = with_rr["spin_rate_rad_per_s"] - without["spin_rate_rad_per_s"]
    assert delta[0] == pytest.approx(0.0, abs=1e-15)
    assert delta[10] == pytest.approx(-2.6622e-3, rel=0.1)
    assert delta[-1] == pytest.approx(-1.21858e-2, rel=0.03)
    # 滚阻只会让自旋升得更慢，不会反转。
    assert np.all(delta <= 0.0)
    assert np.all(np.diff(delta) <= 0.0)


def test_pac2002_tire_parses_into_si_rig_scalars(tmp_path: Path) -> None:
    """PAC2002 胎的台架标量按文件单位归一到 SI，系数原样交给内核."""
    path = tmp_path / "pac2002.tir"
    path.write_text(_pac2002_tir_text(), encoding="utf-8")
    pac = _rig().parse_pac2002_tir(path)

    assert pac.radius_m == pytest.approx(0.344)
    assert pac.maximum_compression_m == pytest.approx(0.99 * 0.344)
    # 文件是 meter/newton，故 VERTICAL_STIFFNESS = 2.1e5 N/m。
    assert pac.vertical_stiffness_n_per_m == pytest.approx(2.1e5)
    assert pac.vertical_damping_n_s_per_m == pytest.approx(50.0)
    # 系数保持文件单位，内核逐处换算。
    assert pac.coefficients["PDX1"] == pytest.approx(1.1739)
    assert pac.coefficients["FNOMIN"] == pytest.approx(4850.0)
    assert pac.coefficients["USE_MODE"] == pytest.approx(3.0)
    assert pac.parameter_source == "adams_builtin"
    assert pac.tables == {}


def test_pac2002_rig_reproduces_the_elastic_vertical_law(tmp_path: Path) -> None:
    """Fz = k·pen 是这台胎的精确弹性律，故初态一致时 Fz 整条曲线等于 3000 N."""
    path = tmp_path / "pac2002.tir"
    path.write_text(_pac2002_tir_text(), encoding="utf-8")
    module = _rig()
    pac = module.parse_pac2002_tir(path)
    maneuver = module.RigManeuver(duration_s=0.2, step_s=0.01)
    model, case = module.build_pac2002_rig(pac, maneuver)

    tire = model.tires[0]
    assert tire.model_kind == "pac2002_pure_slip"
    assert tire.pac2002_parameter_source == "adams_builtin"
    assert tire.unloaded_radius_m == pytest.approx(0.344)
    # 初态贯入按 Fz/k 反查，故垂向体从平衡位置起步。
    assert model.bodies[1].position_m[2] == pytest.approx(0.344 - _LOAD_N / 2.1e5)

    channels = module.report(
        module.run_rig(model, case),
        "tire",
        pac.radius_m,
        module.FORWARD,
        module._cross((0.0, 0.0, 1.0), module.FORWARD),
    )
    assert channels["penetration_m"][0] == pytest.approx(_LOAD_N / 2.1e5, rel=1e-12)
    np.testing.assert_allclose(channels["normal_force_n"], _LOAD_N, rtol=1e-9)


def test_pac2002_effective_rolling_radius_is_reconstructed_from_the_slip(tmp_path: Path) -> None:
    """``effective_rolling_radius_m`` 必须与 κ 的定义自洽：κ = −(V − ωR_e)/V."""
    path = tmp_path / "pac2002.tir"
    path.write_text(_pac2002_tir_text(), encoding="utf-8")
    module = _rig()
    pac = module.parse_pac2002_tir(path)
    maneuver = module.RigManeuver(duration_s=0.2, step_s=0.01)
    model, case = module.build_pac2002_rig(pac, maneuver)
    channels = module.report(
        module.run_rig(model, case),
        "tire",
        pac.radius_m,
        module.FORWARD,
        module._cross((0.0, 0.0, 1.0), module.FORWARD),
    )

    rolling_speed = channels["rolling_speed_m_per_s"]
    radius = channels["effective_rolling_radius_m"]
    spin = channels["spin_rate_rad_per_s"]
    np.testing.assert_allclose(
        channels["kappa"], -(rolling_speed - spin * radius) / rolling_speed, rtol=1e-9
    )
    # 有载半径与有效滚动半径都在 [R₀−贯入, R₀] 内，且 R_e 不小于几何有载半径。
    loaded = channels["rolling_radius_m"]
    assert np.all(radius >= loaded - 1e-12)
    assert np.all(radius <= pac.radius_m)


def test_the_rig_refuses_a_nonlinear_pac2002_vertical_law(tmp_path: Path) -> None:
    """带 QFZ2 的胎不能用 Fz = k·pen 反查初态，必须报错而不是给出一致性之外的初态."""
    path = tmp_path / "pac2002.tir"
    path.write_text(
        _pac2002_tir_text(extra_vertical=" QFZ2 = 0.05\n"), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="Fz = k\\*pen"):
        _rig().parse_pac2002_tir(path)


def test_the_rig_refuses_an_out_of_scope_pac2002_use_mode(tmp_path: Path) -> None:
    """越出 native 范围的 USE_MODE 交给 ``validate_pac2002_native_scope`` 拒绝."""
    path = tmp_path / "pac2002.tir"
    path.write_text(_pac2002_tir_text(use_mode=99), encoding="utf-8")
    with pytest.raises(ValueError, match="USE_MODE"):
        _rig().parse_pac2002_tir(path)
