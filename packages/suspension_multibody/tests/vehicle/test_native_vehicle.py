from __future__ import annotations

import numpy as np
import pytest

from suspension_multibody.authoring import assemble_generic, migrate_v1_vehicle
from suspension_multibody.authoring.migration import migrate_v1_vehicle_case
from suspension_multibody.modeling.primitives import SE3
from suspension_multibody.schema import (
    AerodynamicDragSpec,
    BumpStop,
    Bushing6x6,
    DrivelineSpec,
    DynamicSolverSettings,
    LinearSpring,
    Pose,
    Quaternion,
    RoadSurfaceSpec,
    StaticDamper,
    SteeringSystemSpec,
    TimeSignal,
    TireModelSpec,
    UnitSystem,
    Vec3,
    VehicleDynamicCase,
)
from suspension_multibody.schema.vehicle import VehicleDeclaration
from tests.vehicle._unified_entry import compile_vehicle
from tests.vehicle._unified_entry import solve_vehicle as vehicle_dynamics_run
from tests.vehicle.vehicle_fixtures import (
    _case as _case,
)
from tests.vehicle.vehicle_fixtures import (
    _pac2002_model as _pac2002_model,
)
from tests.vehicle.vehicle_fixtures import (
    _positioned_vehicle as _positioned_vehicle,
)
from tests.vehicle.vehicle_fixtures import (
    _tire as _tire,
)
from tests.vehicle.vehicle_fixtures import (
    _uniform_velocity_initial_states as _uniform_velocity_initial_states,
)
from tests.vehicle.vehicle_fixtures import (
    _vehicle as _vehicle,
)
from tests.vehicle.vehicle_fixtures import (
    _with_ride_springs as _with_ride_springs,
)


def _torque_tables(model, case):
    _, plan = migrate_v1_vehicle_case(case.model_copy(update={"vehicle": model}))
    return tuple({row["tire"].rsplit(".", 1)[-1]: tuple(row["values"])
        for row in plan.to_payload()["inputs"] if row["role"] == role}
        for role in ("wheel_torque", "brake_torque"))


def test_native_vehicle_runs_two_suspensions_and_four_wheels() -> None:
    model = _vehicle()

    result = vehicle_dynamics_run(model, _case(model))

    # chassis + per axle (rack, rack housing, 8 links, 2 wheel hubs, 2 wheels):
    # the welded rear rack is its own body, and 方式 A's hub is one too -- the weld
    # reaches the kernel as a `fixed` joint instead of being fused away here
    # (A1 decision, 2026-09-22), and the wheel spins on the hub rather than on the
    # upright.
    assert len(result.body_names) == 29
    assert "rear_rack" in result.body_names
    assert result.steering_state("front_rack").shape == (2, 4)
    assert np.all(result.diagnostics.accepted)
    assert np.all(np.isfinite(result.states))



def test_native_fixed_joint_is_what_carries_a_weld() -> None:
    """A fixed mount remains an explicit native joint and preserves its body."""
    model = _vehicle()
    mount = model.wheels[0].model_copy(
        update={
            "mount_joint_kind": "fixed",
            "mass": 20.0,
            "center_local": Vec3(x=35.0, y=-8.0, z=12.0),
            "inertia": ((4.0, 0.2, 0.1), (0.2, 5.0, 0.3), (0.1, 0.3, 6.0)),
        }
    )
    model = model.model_copy(update={"wheels": (mount, *model.wheels[1:])})

    graph = assemble_generic(migrate_v1_vehicle(model))
    compiled = compile_vehicle(model, _case(model))
    fixed = [joint for joint in compiled.model_document["joints"] if joint["type"] == "fixed"]
    assert fixed
    assert "wheel_front_left.wheel_front_left" in graph.bodies
    assert graph.bodies["wheel_front_left.wheel_front_left"].mass == 20
    assert all((joint["body_a"], joint["body_b"]) in {(row["body_a"], row["body_b"]) for row in graph.joints} for joint in fixed)


def test_the_weld_switch_cannot_change_world_mass_properties(monkeypatch) -> None:
    """
    Fusing a weld and sending it to the kernel describe the same vehicle.

    The two routes differ in the *body set* (one fused body against two separate
    ones) and therefore in how the body-level rows are laid out -- but the
    vehicle as a whole must weigh the same and balance at the same point, or the
    production switch would have changed the physics rather than the bookkeeping.
    """
    model = _vehicle()

    def world_mass(assembly):
        total = 0.0
        moment = np.zeros(3)
        for name, body in assembly.bodies.items():
            if body.mass > 0.0:
                total += body.mass
                moment += body.mass * body.pose.transform_point(body.center_of_mass)
        return total, moment / total

    source = migrate_v1_vehicle(model)
    separate = assemble_generic(source)
    monkeypatch.setenv("SUSPENSION_MULTIBODY_CONDENSE_WELDS", "1")
    fused = assemble_generic(source)

    separate_mass, separate_com = world_mass(separate)
    fused_mass, fused_com = world_mass(fused)

    assert np.isclose(separate_mass, fused_mass, rtol=0.0, atol=1e-9)
    assert np.allclose(separate_com, fused_com, rtol=0.0, atol=1e-9)


def test_native_fixed_joint_shape_matches_the_registry() -> None:
    """
    ``fixed`` is a two-run joint: the coincident point, then the full relative
    rotation.  The row count is what the contract advertises, so a report that
    reads constraint rows can rely on it.
    """
    from suspension_multibody.modeling.primitives import WeldJoint

    model = _vehicle()
    mount = model.wheels[0].model_copy(
        update={"mount_joint_kind": "fixed", "mass": 20.0}
    )
    model = model.model_copy(update={"wheels": (mount, *model.wheels[1:])})

    from suspension_multibody.authoring import GenericSubsystemAssembler
    uncondensed = assemble_generic(migrate_v1_vehicle(model))
    welds = [GenericSubsystemAssembler.materialize_joint(row) for row in uncondensed.joints if row["type"] == "fixed"]
    assert all(isinstance(row, WeldJoint) for row in welds)
    assert welds, "the uncondensed assembly must still carry its weld"
    fixed = [joint for joint in compile_vehicle(model, _case(model)).model_document["joints"] if joint["type"] == "fixed"]
    assert len(fixed) == len(welds)
    # The body pair is preserved, not fused: that is the whole point of the
    # native path this test pins.
    for weld, joint in zip(welds, fixed):
        assert joint["body_a"] == weld.body_a
        assert joint["body_b"] == weld.body_b

def test_pac2002_selected_combined_slip_changes_force() -> None:
    def run(combined: bool):
        model = _pac2002_model(combined=combined)
        case = _case(model).model_copy(
            update={
                "road": RoadSurfaceSpec(
                    kind="plane", origin=Vec3(z=1.0)
                ),
                "initial_states": _uniform_velocity_initial_states(model),
            }
        )
        return vehicle_dynamics_run(model, case)

    pure = run(False)
    combined = run(True)
    assert np.all(pure.diagnostics.accepted)
    assert np.all(combined.diagnostics.accepted)
    pure_tire = pure.axle.tire_output[-1]
    combined_tire = combined.axle.tire_output[-1]
    assert np.max(np.abs(combined_tire[:, 5:7] - pure_tire[:, 5:7])) > 1.0e-5
    assert np.all(combined_tire[:, 9] > 0.0)


def test_pac2002_use_mode_gates_native_force_axes() -> None:
    def run(use_mode: int):
        model = _pac2002_model(
            combined=True,
            parameter_source="adams_builtin",
            extra_coefficients={"USE_MODE": float(use_mode)},
        )
        case = _case(model).model_copy(
            update={
                "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
                "initial_states": _uniform_velocity_initial_states(model),
            }
        )
        result = vehicle_dynamics_run(model, case)
        assert np.all(result.diagnostics.accepted)
        return result.axle.tire_output[-1]

    longitudinal_only = run(11)
    lateral_only = run(12)
    combined = run(14)

    assert np.max(np.abs(longitudinal_only[:, 5])) > 1.0e-3
    assert np.max(np.abs(longitudinal_only[:, 6])) < 1.0e-10
    assert np.max(np.abs(lateral_only[:, 6])) > 1.0e-3
    assert np.max(np.abs(lateral_only[:, 5])) < 1.0e-10
    assert np.max(np.abs(combined[:, 5])) > 1.0e-3
    assert np.max(np.abs(combined[:, 6])) > 1.0e-3


def test_pac2002_use_mode_zero_is_vertical_spring_only() -> None:
    """
    USE_MODE 0 must produce no slip forces and no moments.

    Adams documents USE_MODE 0 as "acts as a vertical spring & damper" with the
    output tuple ``0, 0, Fz, 0, 0, 0``.  The tire still carries vertical load, so
    the check is that every slip force and every moment is identically zero while
    the normal force is not.
    """

    def run(use_mode: int):
        model = _pac2002_model(
            combined=True,
            parameter_source="adams_builtin",
            extra_coefficients={"USE_MODE": float(use_mode)},
        )
        case = _case(model).model_copy(
            update={
                "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
                "initial_states": _uniform_velocity_initial_states(model),
            }
        )
        result = vehicle_dynamics_run(model, case)
        assert np.all(result.diagnostics.accepted)
        return result.axle.tire_output[-1]

    vertical_only = run(0)
    reference = run(14)

    # Indices follow TIRE_OUTPUT_COLUMNS: 4 normal force, 5 longitudinal force,
    # 6 lateral force, 12/13/14 overturning, rolling resistance, aligning moment.
    assert np.max(np.abs(vertical_only[:, 4])) > 1.0e-3
    for column in (5, 6, 12, 13, 14):
        assert np.max(np.abs(vertical_only[:, column])) < 1.0e-10, column

    # The same tire in a Magic-Formula mode does produce those quantities, so the
    # assertions above are not vacuous.
    assert np.max(np.abs(reference[:, 5])) > 1.0e-3
    assert np.max(np.abs(reference[:, 6])) > 1.0e-3


def test_pac2002_use_mode_zero_is_accepted_by_scope_checks() -> None:
    """USE_MODE 0 is natively supported and must not be rejected."""
    from suspension_multibody.kernel.capabilities import (
        pac2002_unsupported_native_reasons,
    )
    from suspension_multibody.schema.pac2002_scope import (
        validate_pac2002_native_scope,
    )

    coefficients = {
        "FNOMIN": 4_850.0,
        "PCX1": 1.65,
        "PDX1": 1.0,
        "PKX1": 22.3,
        "PCY1": 1.3,
        "PDY1": 1.0,
        "PKY1": -21.9,
        "USE_MODE": 0.0,
    }
    assert pac2002_unsupported_native_reasons(coefficients) == ()
    validate_pac2002_native_scope(coefficients)


def test_pac2002_validity_ranges_clamp_the_magic_formula_inputs() -> None:
    """
    A narrow declared validity range must change the computed forces.

    Adams clamps the tire model inputs to the ranges declared in the tire
    property file.  Declaring very tight slip and load ranges therefore has to
    produce different forces from a tire with permissive ranges on the same
    maneuver; if it does not, the bounds are being carried but never applied.
    """

    def run(coefficients: dict[str, float]):
        model = _pac2002_model(
            combined=True,
            parameter_source="adams_builtin",
            extra_coefficients=coefficients,
        )
        case = _case(model).model_copy(
            update={
                "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
                "initial_states": _uniform_velocity_initial_states(model),
            }
        )
        result = vehicle_dynamics_run(model, case)
        assert np.all(result.diagnostics.accepted)
        return result.axle.tire_output[-1]

    permissive = {
        "KPUMIN": -1.0e12,
        "KPUMAX": 1.0e12,
        "ALPMIN": -1.0e12,
        "ALPMAX": 1.0e12,
        "CAMMIN": -1.0e12,
        "CAMMAX": 1.0e12,
        "FZMIN": 0.0,
        "FZMAX": 1.0e12,
    }
    # Pin slip to almost nothing and hold the load in a narrow band around the
    # measured value, so both the slip and the load clamps engage.
    narrow = {
        **permissive,
        "KPUMIN": -1.0e-4,
        "KPUMAX": 1.0e-4,
        "ALPMIN": -1.0e-4,
        "ALPMAX": 1.0e-4,
        "FZMIN": 199.0,
        "FZMAX": 201.0,
    }

    unclamped = run(permissive)
    clamped = run(narrow)

    for column, name in ((5, "longitudinal"), (6, "lateral")):
        assert not np.allclose(
            unclamped[:, column], clamped[:, column], atol=1.0e-9
        ), f"{name} force did not respond to the declared validity range"
    # The permissive case must keep producing the normal force it always did.
    assert np.max(np.abs(unclamped[:, 4])) > 1.0e-3


def test_pac2002_deflection_load_curve_replaces_the_stiffness_polynomial() -> None:
    """
    A tire carrying `[DEFLECTION_LOAD_CURVE]` must use the table, not VERTICAL_STIFFNESS.

    The table is the measured load at nominal conditions, so a curve twice as stiff as
    the polynomial has to produce twice the vertical force at the same deflection.
    Measured: 400.173 N against 200.044 N, ratio 2.0004, while the deflection moves by
    4e-6 relative -- this fixture prescribes the wheel position, so the deflection is
    geometry and only the force can discriminate.

    The table is built from the *measured* polynomial response rather than from
    ``vertical_stiffness``: that value is in the file's units (N/mm here, so 200 against
    the kernel's 2e5 N/m), and a table built from it is three orders of magnitude too
    soft -- measured as a vertical load of 0.1 N against 200 N.

    The case runs with `constraint_tolerance` relaxed to 1e-5 (an increment tolerance of
    1e-8 after the engineering-unit scaling): at the default the fixture's Newton reaches
    machine precision on every residual (measured pos=2.2e-16, vel=1.3e-15, dyn=5.3e-12)
    yet stops with an increment of 1.16e-10 and reports "no descent" -- the fixture
    residual floor from step 22, which this case merely tips over.
    """
    extra = {"USE_MODE": 14.0}

    def run(model, relaxed: bool) -> tuple[float, float]:
        case = _case(model)
        if relaxed:
            case = case.model_copy(
                update={
                    "solver": case.solver.model_copy(
                        update={"constraint_tolerance": 1.0e-5}
                    )
                }
            )
        case = case.model_copy(
            update={
                "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
                "initial_states": _uniform_velocity_initial_states(model),
            }
        )
        result = vehicle_dynamics_run(model, case)
        assert np.all(result.diagnostics.accepted)
        tire = result.axle.tire_output[-1]
        return (
            float(np.max(np.abs(tire[:, 2]))),
            float(np.max(np.abs(tire[:, 4]))),
        )

    base = _pac2002_model(
        combined=False, parameter_source="adams_builtin", extra_coefficients=extra
    )
    deflection, load = run(base, relaxed=False)
    # Twice the polynomial stiffness through the measured point, plus a third point so
    # the monotone cubic has an interior slope to work with.
    curve = (
        (0.0, 0.0),
        (deflection, 2.0*load),
        (2.0*deflection, 4.0*load),
    )
    wheels = tuple(
        wheel.model_copy(
            update={
                "tire": wheel.tire.model_copy(
                    update={"pac2002_tables": {"deflection_load_curve": curve}}
                )
            }
        )
        for wheel in base.wheels
    )
    with_curve = base.model_copy(update={"wheels": wheels})
    tabulated, tabulated_load = run(with_curve, relaxed=True)

    # The fixture prescribes the wheel position, so the deflection is geometry rather
    # than a response; it is the *force* at that deflection the table has to change.
    assert 0.9 < tabulated/deflection < 1.1, (deflection, tabulated)
    ratio = tabulated_load/load
    assert 1.6 < ratio < 2.4, (load, tabulated_load, ratio)


def test_pac2002_bottoming_curve_adds_the_rim_force() -> None:
    """
    `[BOTTOMING_CURVE]` adds the rim reaction once the rim reaches the road.

    Adams documents the vertical force as ``Fz = min(0, Fzk + Fzc) + min(0, Fzrim)``:
    the rim contribution is *added* to the tire's own force, engaged once the
    deflection exceeds ``UNLOADED_RADIUS - BOTTOMING_RADIUS``, and the curve is
    tabulated against that rim penetration.  Three runs pin it:

    * no bottoming section: the baseline vertical force;
    * section present but `BOTTOMING_RADIUS = 0`: the documented "no rim contact" case,
      which must leave the force *bit-identical* -- this is what catches a kernel that
      applies the curve without gating on the radius;
    * section with the rim half a millimetre below the unloaded radius: measured
      1200.28 N against 200.04 N, a ratio of 6.000 -- the rim curve's value (1000 N) at
      the rim penetration the fixture's ~1 mm deflection produces, on top of the tire's
      own force.

    The deflection itself moves by 4e-6 relative because this fixture prescribes the
    wheel position, so only the force can discriminate.
    """
    extra = {"USE_MODE": 14.0}

    def run(model) -> tuple[float, float]:
        case = _case(model).model_copy(
            update={
                "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
                "initial_states": _uniform_velocity_initial_states(model),
            }
        )
        result = vehicle_dynamics_run(model, case)
        assert np.all(result.diagnostics.accepted)
        tire = result.axle.tire_output[-1]
        return (
            float(np.max(np.abs(tire[:, 2]))),
            float(np.max(np.abs(tire[:, 4]))),
        )

    def with_tables(tables: dict, radius: float):
        base = _pac2002_model(
            combined=False,
            parameter_source="adams_builtin",
            extra_coefficients=extra,
        )
        wheels = tuple(
            wheel.model_copy(
                update={
                    "tire": wheel.tire.model_copy(
                        update={
                            "pac2002_tables": tables,
                            "pac2002_coefficients": {
                                **wheel.tire.pac2002_coefficients,
                                "BOTTOMING_RADIUS": radius,
                            },
                        }
                    )
                }
            )
            for wheel in base.wheels
        )
        return base.model_copy(update={"wheels": wheels})

    curve = ((0.0, 0.0), (0.0005, 1000.0), (0.001, 2000.0))
    deflection, baseline = run(
        _pac2002_model(
            combined=False, parameter_source="adams_builtin", extra_coefficients=extra
        )
    )
    _, no_rim = run(with_tables({"bottoming_curve": curve}, 0.0))
    assert no_rim == pytest.approx(baseline, rel=1.0e-12)

    unloaded_radius = float(
        _pac2002_model(
            combined=False, parameter_source="adams_builtin", extra_coefficients=extra
        ).wheels[0].tire.unloaded_radius
    )
    tabulated_deflection, with_rim = run(
        with_tables({"bottoming_curve": curve}, unloaded_radius - 0.5)
    )
    assert tabulated_deflection == pytest.approx(deflection, rel=1.0e-4)
    assert with_rim > 4.0*baseline, (baseline, with_rim)


def test_pac2002_vxlow_does_not_floor_the_slip_denominator() -> None:
    """
    VXLOW must not change the slip, only (per the tire file) fade the forces below it.

    The tire file documents VXLOW as "Below this speed forces are scaled down", but the
    kernel used it as a floor on the slip denominator (``max(|Vx|, VXLOW)``), which
    shrinks the slip itself.  Measured on the mode-24 parking maneuver (|Vx| = 1 m/s
    against VXLOW = 2 m/s) the floor gave a lateral-force NRMSE of 73.4 % with the rear
    axle's lateral force and aligning moment anti-phase against Adams (r = -0.78/-0.71);
    dividing by the true |Vx| gives 16.8 % with those correlations at +0.85/+0.89, and
    the handling channels improve with it (lateral acceleration 13.5 -> 6.7 %, yaw rate
    12.8 -> 2.9 %, body roll 24.1 -> 15.5 %).  This test pins the corrected behaviour,
    so reintroducing the floor fails here.

    The force fade itself is deliberately *not* implemented: measured, scaling the tire
    forces by min(1, |Vx|/VXLOW) makes the parking comparison worse (73.4 -> 82.8 % with
    the floor, 16.8 -> 54.5 % without) and breaks standstill cases, so the documented
    half that matters here is the slip reference speed.
    """

    def run(vxlow: float):
        model = _pac2002_model(
            combined=False,
            parameter_source="user",
            extra_coefficients={"USE_MODE": 14.0, "VXLOW": vxlow},
        )
        case = _case(model).model_copy(
            update={
                "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
                "initial_states": _uniform_velocity_initial_states(
                    model, vx_mm_s=100.0, vy_mm_s=50.0
                ),
            }
        )
        result = vehicle_dynamics_run(model, case)
        assert np.all(result.diagnostics.accepted)
        return np.max(np.abs(result.axle.tire_output[-1, :, 10]))

    low_floor_slip = run(0.1)
    high_floor_slip = run(10.0)

    assert high_floor_slip == pytest.approx(low_floor_slip, rel=1.0e-9)


def test_pac2002_vertical_force_uses_speed_and_force_coupling_terms() -> None:
    def run(extra_coefficients: dict[str, float], moving: bool = False):
        model = _pac2002_model(
            combined=False,
            parameter_source="adams_builtin",
            extra_coefficients={"USE_MODE": 14.0, **extra_coefficients},
        )
        case_updates = {
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
        }
        if moving:
            case_updates["initial_states"] = _uniform_velocity_initial_states(model)
        else:
            case_updates["initial_wheel_speeds"] = tuple(
                (wheel.name, 100.0) for wheel in model.wheels
            )
        case = _case(model).model_copy(update=case_updates)
        result = vehicle_dynamics_run(model, case)
        assert np.all(result.diagnostics.accepted)
        return float(np.mean(result.axle.tire_output[-1, :, 4]))

    baseline_spin = run({"QV2": 0.0})
    boosted_spin = run({"QV2": 0.2})
    uncoupled = run({"QFCX1": 0.0}, moving=True)
    force_coupled = run({"QFCX1": 5.0}, moving=True)

    assert boosted_spin > 1.2 * baseline_spin
    assert force_coupled < uncoupled


def test_pac2002_rejects_unsupported_adams_turn_slip_modes() -> None:
    with pytest.raises(ValueError, match="unsupported native PAC2002 scope"):
        TireModelSpec(kind="pac2002", pac2002_coefficients={"USE_MODE": 15.0})
    # The spin/parking coefficient family used to be rejected wholesale.  It is now
    # consumed by USE_MODE 25, so only the two second-order trail coefficients that no
    # documented factor reads still fail closed.
    with pytest.raises(ValueError, match="unsupported native PAC2002 scope"):
        TireModelSpec(kind="pac2002", pac2002_coefficients={"QDTP2": 0.1})
    with pytest.raises(ValueError, match="unsupported native PAC2002 scope"):
        TireModelSpec(kind="pac2002", pac2002_coefficients={"QBRP2": 0.1})
    # USE_MODE 25 and the spin family it needs are inside the native scope now; the
    # comparison that validates them is
    # ``tests/adams/test_pac2002_adams_correlation_gate.py::test_parking_steer_pins_the_turn_slip_and_parking_torque``.
    TireModelSpec(
        kind="pac2002",
        pac2002_coefficients={
            "USE_MODE": 25.0,
            "IC": 0.05, "KP": 11.9, "CP": 2019.0,
            "EP": 1.0, "EP12": 3.0, "BF2": 0.5, "BP1": 0.5, "BP2": 0.667,
            "PECP1": 0.0, "PECP2": 0.0,
            "PDXP1": 0.4, "PDXP2": 0.0, "PDXP3": 0.0,
            "PDYP1": 0.4, "PDYP2": 0.0, "PDYP3": 0.0, "PDYP4": 0.0,
            "PKYP1": 0.0,
            "PHYP1": 0.0, "PHYP2": 0.0, "PHYP3": 0.0, "PHYP4": 0.0,
            "QBRP1": 0.1, "QCRP1": 0.15, "QCRP2": 0.02,
            "QDRP1": 1.0, "QDRP2": 0.0, "QDTP1": 10.0,
        },
    )


def test_pac2002_initial_relaxation_state_matches_current_slip() -> None:
    model = _pac2002_model(combined=False)
    case = _case(model).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(model),
        }
    )

    result = vehicle_dynamics_run(model, case)
    initial_tire = result.axle.tire_output[0]

    assert np.all(result.diagnostics.accepted)
    assert np.max(np.abs(initial_tire[:, 5:7])) > 1.0e-3
    assert np.max(np.abs(initial_tire[:, 10:12])) > 1.0e-6


def test_adams_pac2002_source_uses_local_relaxation_state() -> None:
    base = _pac2002_model(combined=False, parameter_source="adams_builtin")
    model = base.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(update={"axial_inertia": 200_000.0})
                for wheel in base.wheels
            )
        }
    )
    case = _case(model).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(model),
        }
    )

    result = vehicle_dynamics_run(model, case)
    current = result.axle.tire_output[-1]

    assert np.all(result.diagnostics.accepted)
    assert np.all(np.abs(current[:, 7]) > 1.0e-3)
    kinematic = np.clip(-current[:, 7] / 10.0, -1.0, 1.0)
    assert np.max(np.abs(current[:, 10] - kinematic)) > 1.0e-4


def test_adams_pac2002_preserves_source_static_offset() -> None:
    coefficients = {
        "PHX1": 0.01,
        "PVX1": 0.02,
        "PHY1": 0.01,
        "PVY1": 0.02,
    }
    base = _pac2002_model(combined=False)
    user_model = base.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={
                                "pac2002_coefficients": {
                                    **wheel.tire.pac2002_coefficients,
                                    **coefficients,
                                }
                            }
                        )
                    }
                )
                for wheel in base.wheels
            )
        }
    )
    source_model = user_model.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={"parameter_source": "adams_builtin"}
                        )
                    }
                )
                for wheel in user_model.wheels
            )
        }
    )
    user_case = _case(user_model).model_copy(
        update={"road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0))}
    )
    source_case = _case(source_model).model_copy(
        update={"road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0))}
    )

    user_result = vehicle_dynamics_run(user_model, user_case)
    source_result = vehicle_dynamics_run(source_model, source_case)

    assert np.all(user_result.diagnostics.accepted)
    assert np.all(source_result.diagnostics.accepted)
    assert np.max(np.abs(user_result.axle.tire_output[0, :, 5:7])) < 1e-10
    assert np.max(np.abs(source_result.axle.tire_output[0, :, 5:7])) > 1e-8


def test_adams_pac2002_negative_use_mode_mirrors_left_side_coefficients() -> None:
    base = _pac2002_model(combined=False, parameter_source="adams_builtin")

    def with_use_mode(use_mode: float) -> VehicleDeclaration:
        return base.model_copy(
            update={
                "wheels": tuple(
                    wheel.model_copy(
                        update={
                            "tire": wheel.tire.model_copy(
                                update={
                                    "pac2002_coefficients": {
                                        **wheel.tire.pac2002_coefficients,
                                        "PHY1": 0.02,
                                        "PVY1": 0.02,
                                        "USE_MODE": use_mode,
                                    }
                                }
                            )
                        }
                    )
                    if wheel.name == "front_left" else wheel
                    for wheel in base.wheels
                )
            }
        )

    def lateral_force(model: VehicleDeclaration) -> float:
        result = vehicle_dynamics_run(
            model,
            _case(model).model_copy(
                update={"road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0))}
            ),
        )
        assert np.all(result.diagnostics.accepted)
        return float(result.axle.tire_output[0, 0, 6])

    left_side = lateral_force(with_use_mode(14.0))
    mirrored_side = lateral_force(with_use_mode(-14.0))

    assert left_side * mirrored_side < 0.0


def test_adams_pac2002_uses_source_gyroscopic_moment_parameters() -> None:
    base = _pac2002_model(combined=False, parameter_source="adams_builtin")

    def with_gyro(qtz1: float, mbelt: float) -> VehicleDeclaration:
        return base.model_copy(
            update={
                "wheels": tuple(
                    wheel.model_copy(
                        update={
                            "tire": wheel.tire.model_copy(
                                update={
                                    "pac2002_coefficients": {
                                        **wheel.tire.pac2002_coefficients,
                                        "LGYR": 1.0,
                                        "QTZ1": qtz1,
                                        "MBELT": mbelt,
                                    }
                                }
                            )
                        }
                    )
                    for wheel in base.wheels
                )
            }
        )

    disabled = with_gyro(0.0, 0.0)
    enabled = with_gyro(0.2, 5.4)

    def run(model: VehicleDeclaration):
        case = _case(model).model_copy(
            update={
                "solver": _case(model).solver.model_copy(
                    update={
                        "end_time": 0.001,
                        "step_size": 0.001,
                        "internal_step_size": 0.001,
                        "min_internal_step_size": 0.001,
                    }
                ),
                "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
                "initial_states": _uniform_velocity_initial_states(model),
            }
        )
        return vehicle_dynamics_run(model, case)

    disabled_result = run(disabled)
    enabled_result = run(enabled)

    assert np.all(disabled_result.diagnostics.accepted)
    assert np.all(enabled_result.diagnostics.accepted)
    delta = (
        enabled_result.axle.tire_output[-1, :, 14]
        - disabled_result.axle.tire_output[-1, :, 14]
    )
    assert np.max(np.abs(delta)) > 1.0e-8


def test_pac2002_aligning_moment_changes_single_wheel_response() -> None:
    baseline = _pac2002_model(combined=False)
    aligning = baseline.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={
                                "pac2002_coefficients": {
                                    **wheel.tire.pac2002_coefficients,
                                    "QDZ1": 0.08,
                                    "QDZ6": 0.01,
                                    "QCZ1": 1.0,
                                }
                            }
                        )
                    }
                )
                if wheel.name == "front_left" else wheel
                for wheel in baseline.wheels
            )
        }
    )
    baseline_case = _case(baseline).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(baseline),
        }
    )
    aligning_case = _case(aligning).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(aligning),
        }
    )

    baseline_result = vehicle_dynamics_run(baseline, baseline_case)
    aligning_result = vehicle_dynamics_run(aligning, aligning_case)

    assert np.all(baseline_result.diagnostics.accepted)
    assert np.all(aligning_result.diagnostics.accepted)
    assert np.max(
        np.abs(aligning_result.states[-1]-baseline_result.states[-1])
    ) > 1.0e-10


def test_pac2002_chrono_overturning_and_rolling_moments_change_response() -> None:
    # 20 kg、300 mm 轮端使用物理量级的转动惯量，避免附加滚阻矩在
    # 1 ms 回归步长中产生非物理的超大角加速度。
    base = _pac2002_model(combined=False)
    baseline = base.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(update={"axial_inertia": 200_000.0})
                for wheel in base.wheels
            )
        }
    )
    chrono_terms = baseline.model_copy(
        update={
            "wheels": tuple(
                wheel.model_copy(
                    update={
                        "tire": wheel.tire.model_copy(
                            update={
                                "pac2002_coefficients": {
                                    **wheel.tire.pac2002_coefficients,
                                    # Chrono CalcMx：接触坐标系中的载荷相关倾覆力矩。
                                    "QSX1": 0.02,
                                    # Chrono CalcMy：滚动阻力力矩项。
                                    "QSY1": 0.01,
                                }
                            }
                        )
                    }
                )
                if wheel.name == "front_left" else wheel
                for wheel in baseline.wheels
            )
        }
    )
    baseline_case = _case(baseline).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(baseline),
        }
    )
    chrono_case = _case(chrono_terms).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(chrono_terms),
        }
    )

    baseline_result = vehicle_dynamics_run(baseline, baseline_case)
    chrono_result = vehicle_dynamics_run(chrono_terms, chrono_case)

    assert np.all(baseline_result.diagnostics.accepted)
    assert np.all(chrono_result.diagnostics.accepted)
    assert np.max(
        np.abs(chrono_result.states[-1]-baseline_result.states[-1])
    ) > 1.0e-10


def test_reduced_kkt_matches_dense_fallback(monkeypatch: pytest.MonkeyPatch) -> None:
    model = _vehicle()
    case = _case(model)

    monkeypatch.setenv("SUSPENSION_AXLE_DISABLE_REDUCED_KKT", "1")
    dense = vehicle_dynamics_run(model, case)
    monkeypatch.delenv("SUSPENSION_AXLE_DISABLE_REDUCED_KKT")
    reduced = vehicle_dynamics_run(model, case)

    np.testing.assert_allclose(
        reduced.states, dense.states, rtol=2.0e-6, atol=2.0e-8
    )
    np.testing.assert_allclose(
        reduced.axle.tire_output, dense.axle.tire_output,
        rtol=2.0e-6, atol=2.0e-8,
    )
    np.testing.assert_allclose(
        reduced.axle.energy, dense.axle.energy, rtol=2.0e-6, atol=2.0e-8
    )
    assert np.all(reduced.diagnostics.accepted)


def test_native_vehicle_snaps_internal_time_roundoff_at_output_end() -> None:
    model = _vehicle()
    base_case = _case(model)
    case = base_case.model_copy(
        update={
            "solver": base_case.solver.model_copy(
                update={
                    "end_time": 0.37,
                    "step_size": 0.01,
                    "output_step": 0.01,
                    "internal_step_size": 0.001,
                    "min_internal_step_size": 0.001,
                }
            )
        }
    )

    result = vehicle_dynamics_run(model, case)

    assert result.times_s[-1] == pytest.approx(0.37)
    assert np.all(result.diagnostics.accepted)


def test_native_attachment_points_respect_body_origin_and_orientation() -> None:
    angle = np.pi / 3.0
    model = _vehicle()
    upper_arm = next(
        body for body in model.front_axle.bodies if body.name == "upper_arm_L"
    ).model_copy(
        update={
            "pose": Pose(
                translation=Vec3(x=800.0, y=-250.0, z=120.0),
                rotation=Quaternion(
                    w=np.cos(angle / 2.0), z=np.sin(angle / 2.0)
                ),
            ),
            "center_of_mass": Vec3(x=100.0, y=40.0, z=60.0),
        }
    )
    front_bodies = tuple(
        upper_arm if body.name == "upper_arm_L" else body
        for body in model.front_axle.bodies
    )
    model = model.model_copy(
        update={
            "front_axle": model.front_axle.model_copy(
                update={"bodies": front_bodies}
            )
        }
    )
    compiled = compile_vehicle(model, _case(model))
    native_body = next(row for row in compiled.model_document["bodies"]
        if row["name"].endswith(".upper_arm_L"))
    native_joint = next(row for row in compiled.model_document["joints"]
        if row["name"].endswith(".uca_mount_L_inner_front"))
    rotation = SE3(np.zeros(3), np.asarray(upper_arm.pose.rotation.as_tuple())).rotation
    expected_position = (upper_arm.pose.translation.as_array()
        + rotation @ upper_arm.center_of_mass.as_array()) * 1e-3
    np.testing.assert_allclose(native_body["position"], expected_position, atol=1e-12)
    expected = model.front_axle.hardpoints["UPPER_INBOARD_FRONT"].as_array()*1e-3
    reconstructed = np.asarray(native_body["position"]) + rotation @ native_joint["point_b"]
    np.testing.assert_allclose(reconstructed, expected, atol=1e-12)


def test_native_vehicle_passes_spring_and_stop_curves_to_vehicle_abi() -> None:
    base = _positioned_vehicle(_vehicle())
    front = base.front_axle
    spring = LinearSpring(
        name="curve_spring",
        body_a="chassis",
        body_b="lower_arm",
        point_a=front.hardpoints["LOWER_INBOARD_FRONT"],
        point_b=front.hardpoints["LOWER_OUTBOARD"],
        stiffness=100.0,
        free_length=450.0,
        force_curve=((0.0, 0.0), (100.0, 100.0), (200.0, 350.0)),
    )
    stop = BumpStop(
        name="curve_stop",
        body_a="chassis",
        body_b="lower_arm",
        point_a=front.hardpoints["LOWER_INBOARD_FRONT"],
        point_b=front.hardpoints["LOWER_OUTBOARD"],
        clearance=25.0,
        stiffness=1_000.0,
        direction="bump",
        force_curve=((0.0, 0.0), (10.0, 100.0), (20.0, 500.0)),
    )
    model = base.model_copy(
        update={
            "front_axle": front.model_copy(
                update={"springs": (spring,), "stops": (stop,)}
            )
        }
    )
    elements = compile_vehicle(model, _case(model)).model_document["elements"]
    mapped_spring = next(
        item for item in elements if item["name"].endswith(".curve_spring_L")
    )
    mapped_stop = next(item["parameters"] for item in elements if item["name"].endswith(".curve_stop_L"))
    mapped_spring = mapped_spring["parameters"]

    np.testing.assert_allclose(
        np.asarray(mapped_spring["elastic_curve"])[:, 0],
        (-0.2, -0.1, 0.0),
    )
    np.testing.assert_allclose(np.asarray(mapped_spring["elastic_curve"])[:, 1], (-350.0, -100.0, 0.0))
    np.testing.assert_allclose(
        np.asarray(mapped_stop["stop_curve"])[:, 0],
        (0.0, 0.01, 0.02),
    )
    np.testing.assert_allclose(
        np.asarray(mapped_stop["stop_curve"])[:, 1],
        (0.0, 100.0, 500.0),
    )

    result = vehicle_dynamics_run(model, _case(model))

    assert np.all(result.diagnostics.accepted)
    # The elastic and unilateral ledgers are separate now, and each is as wide as
    # the structure it reports.
    assert result.spring_state("front_curve_spring_L").shape == (2, 4)
    assert result.bump_stop_state("front_curve_stop_L").shape == (2, 5)


def test_native_vehicle_passes_bushing_curves_and_coordinates_to_vehicle_abi() -> None:
    base = _positioned_vehicle(_vehicle())
    front = base.front_axle
    point = front.hardpoints["UPPER_INBOARD_FRONT"]
    bushing = Bushing6x6(
        name="curve_bushing",
        body_a="chassis",
        body_b="upper_arm",
        pose_a=Pose(translation=point),
        pose_b=Pose(
            translation=Vec3(x=point.x, y=point.y, z=point.z + 50.0)
        ),
        stiffness=((0.0,) * 6,) * 6,
        damping=(0.0,) * 6,
        rotation_coordinates="cardan_xyz",
        force_curves=(
            (),
            (),
            ((-100.0, -100.0), (0.0, 0.0), (100.0, 100.0)),
            (),
            (),
            (),
        ),
    )
    model = base.model_copy(
        update={
            "front_axle": front.model_copy(update={"bushings": (bushing,)})
        }
    )

    result = vehicle_dynamics_run(model, _case(model))

    assert np.all(result.diagnostics.accepted)
    bushing_state = result.bushing_state("front_curve_bushing_L")
    assert bushing_state.shape == (2, 12)
    assert bushing_state[0, 2] == pytest.approx(0.05, abs=1e-8)
    assert bushing_state[0, 8] == pytest.approx(-50.0, abs=1e-6)


def test_native_vehicle_static_trim_balances_gravity_with_four_tire_contacts() -> None:
    model = _with_ride_springs(_positioned_vehicle(_vehicle()))
    base_case = _case(model)
    case = base_case.model_copy(
        update={
            "static_equilibrium": True,
            "solver": base_case.solver.model_copy(
                update={
                    "gravity": Vec3(x=0.0, y=0.0, z=-9810.0),
                    "projection_max_iterations": 60,
                    "projection_backtracking": 20,
                }
            ),
        }
    )

    result = vehicle_dynamics_run(model, case)

    assert np.all(result.diagnostics.accepted)
    assert np.all(result.diagnostics.active_contacts == 4)
    assert np.max(result.diagnostics.position_residual) < 1.0e-8
    assert np.max(result.diagnostics.dynamics_residual) < 1.0e-7
    assert np.max(result.diagnostics.pinned_null_directions) >= 1


def test_static_trim_accepts_zero_speed_drag_and_roundoff_brake_torque() -> None:
    model = _with_ride_springs(_positioned_vehicle(_vehicle())).model_copy(
        update={
            "aerodynamic_drag": AerodynamicDragSpec(
                air_density=1.225,
                drag_coefficient=0.32,
                frontal_area=2.2,
            ),
            "steering": SteeringSystemSpec(
                ratio=16.0,
                rack_damping=0.0,
                actuator_mode="prescribed_translation",
                # The rack is prescribed relative to the body it slides in -- the
                # steering housing, which is welded to the chassis, so naming it is
                # the same statement about where the rack goes as naming the chassis
                # (and it is the only body the rack has a guide joint with).
                actuator_reaction_body="front_rack_housing",
                actuator_axis_local=Vec3(y=1.0),
            ),
        }
    )
    base_case = _case(model, steering=TimeSignal(constant=2.0))
    case = base_case.model_copy(
        update={
            "static_equilibrium": True,
            "wheel_brake_torque": tuple(
                (wheel.name, TimeSignal(constant=1.0e-14))
                for wheel in model.wheels
            ),
            "solver": base_case.solver.model_copy(
                update={
                    "gravity": Vec3(x=0.0, y=0.0, z=-9810.0),
                    "projection_max_iterations": 60,
                    "projection_backtracking": 20,
                }
            ),
        }
    )

    result = vehicle_dynamics_run(model, case)

    assert np.all(result.diagnostics.accepted)
    assert np.all(result.diagnostics.active_contacts == 4)


def test_native_vehicle_combines_trim_road_steering_and_drive() -> None:
    model = _with_ride_springs(_positioned_vehicle(_vehicle())).model_copy(
        update={
            "driveline": DrivelineSpec(
                driven_wheels=("front_left", "front_right"),
                maximum_drive_torque=800.0,
                maximum_brake_torque=1_000.0,
                front_brake_bias=0.6,
                drive_split=(0.5, 0.5, 0.0, 0.0),
            ),
            "steering": SteeringSystemSpec(ratio=16.0, rack_damping=200.0),
        }
    )
    base_case = _case(
        model,
        steering=TimeSignal(
            times=(0.0, 0.003, 0.006), values=(0.0, 0.0, 2.0)
        ),
    )
    case = base_case.model_copy(
        update={
            "drive_input": TimeSignal(
                times=(0.0, 0.003, 0.006), values=(0.0, 0.0, 0.2)
            ),
            "road": RoadSurfaceSpec(kind="sine", amplitude=3.0, wavelength=2_000.0),
            "static_equilibrium": True,
            "solver": base_case.solver.model_copy(
                update={
                    "end_time": 0.006,
                    "step_size": 0.001,
                    "internal_step_size": 0.001,
                    "min_internal_step_size": 0.001,
                    "adaptive_substepping": False,
                    "gravity": Vec3(x=0.0, y=0.0, z=-9810.0),
                }
            ),
        }
    )

    result = vehicle_dynamics_run(model, case)

    assert np.all(result.diagnostics.accepted)
    assert np.all(result.diagnostics.active_contacts == 4)
    assert np.max(result.diagnostics.position_residual) < 1.0e-7
    assert np.max(result.diagnostics.dynamics_residual) < 1.0e-6
    assert result.steering_state("front_rack")[-1, 2] == 0.002
    assert np.all(np.isfinite(result.body_state("wheel_front_left")))


def test_four_post_sampled_signals_do_not_duplicate_analytic_profile() -> None:
    signals = tuple(
        TimeSignal(times=(0.0, 0.001), values=(0.0, 10.0))
        for _ in range(4)
    )
    road = RoadSurfaceSpec(
        kind="four_post",
        origin=Vec3(z=2.0),
        amplitude=10.0,
        bump_length=100.0,
        corner_height_signals=signals,  # type: ignore[arg-type]
    )

    _, plan = migrate_v1_vehicle_case(_case(_vehicle()).model_copy(update={"road": road}))
    payload = plan.to_payload()
    assert payload["excitation"]["inputs"]["road"]["kind"] == "plane"
    height = next(row for row in payload["inputs"] if row["role"] == "road_height"
        and row["tire"] == "wheel_front_left.front_left")
    assert tuple(height["values"]) == (0.002, 0.012)


def test_engineering_damper_preload_is_converted_before_si_scaling() -> None:
    damper = StaticDamper(
        name="gas_damper",
        body_a="chassis",
        body_b="lower_arm",
        point_a=Vec3(x=0, y=-500, z=100),
        point_b=Vec3(x=0, y=-700, z=100),
        gas_stiffness=10.0,
        gas_reference_length=100.0,
        gas_reference_force=50.0,
        preload=20.0,
        friction=5.0,
    )
    model = _vehicle(front_dampers=(damper,))
    dampers = compile_vehicle(model, _case(model)).model_document["elements"]

    # The gas law, the preload and the friction are their own fields on the
    # damper record now, so the record states the source model's declarations
    # rather than an equivalent free length derived from them.  The old fold was
    # `free_length = reference - offset/gas_k`; this asserts the record can be
    # folded back to exactly that, which is what "no physics was lost" means.
    mapped = next(row["parameters"] for row in dampers if row["name"].endswith(".gas_damper_L"))
    assert mapped["gas_reference_length"] == pytest.approx(0.1, abs=1e-12)
    assert mapped["gas_stiffness"] == pytest.approx(10.0 * 1000.0)
    assert mapped["gas_reference_force"] == pytest.approx(50.0)
    assert mapped["preload"] == pytest.approx(20.0)
    assert mapped["friction"] == pytest.approx(5.0)
    offset = mapped["gas_reference_force"] + mapped["preload"] + mapped["friction"]
    folded_free_length = (
        mapped["gas_reference_length"] - offset / mapped["gas_stiffness"]
    )
    assert folded_free_length == pytest.approx(0.0925, abs=1e-12)


def test_brake_signal_is_a_nonnegative_magnitude() -> None:
    model = _vehicle().model_copy(
        update={
            "driveline": DrivelineSpec(
                maximum_brake_torque=1000.0,
                front_brake_bias=0.6,
            )
        }
    )
    case = _case(
        model,
        brake=1.0,
        wheel_speeds=(("front_left", -10.0),),
    )

    drive, brake = _torque_tables(model, case)

    assert drive["front_left"] == (0.0, 0.0)
    assert brake["front_left"] == (0.3, 0.3)


def test_direct_wheel_torque_signals_override_global_distribution() -> None:
    model = _vehicle().model_copy(
        update={
            "driveline": DrivelineSpec(
                driven_wheels=("front_left", "front_right"),
                maximum_drive_torque=1_000.0,
                maximum_brake_torque=1_000.0,
                front_brake_bias=0.6,
                drive_split=(0.5, 0.5, 0.0, 0.0),
            )
        }
    )
    case = _case(model).model_copy(
        update={
            "wheel_drive_torque": (
                ("rear_left", TimeSignal(constant=250.0)),
            ),
            "wheel_brake_torque": (
                ("front_right", TimeSignal(constant=40.0)),
            ),
        }
    )

    drive, brake = _torque_tables(model, case)

    assert drive == {
        "front_left": (0.0, 0.0),
        "front_right": (0.0, 0.0),
        "rear_left": (0.25, 0.25),
        "rear_right": (0.0, 0.0),
    }
    assert brake == {
        "front_left": (0.0, 0.0),
        "front_right": (0.04, 0.04),
        "rear_left": (0.0, 0.0),
        "rear_right": (0.0, 0.0),
    }

    mixed_drive = _case(model, brake=1.0).model_copy(
        update={
            "wheel_drive_torque": (
                ("rear_left", TimeSignal(constant=250.0)),
            )
        }
    )
    drive, brake = _torque_tables(model, mixed_drive)
    assert drive["rear_left"] == (0.25, 0.25)
    assert brake["front_left"] == (0.3, 0.3)

    mixed_brake = _case(model).model_copy(
        update={
            "drive_input": TimeSignal(constant=0.5),
            "wheel_brake_torque": (
                ("rear_right", TimeSignal(constant=40.0)),
            ),
        }
    )
    drive, brake = _torque_tables(model, mixed_brake)
    assert drive["front_left"] == (0.25, 0.25)
    assert brake["rear_right"] == (0.04, 0.04)


@pytest.mark.xfail(
    strict=True,
    reason=(
        "this fixture is degenerate: ideal joints only (no springs, no bushings), "
        "gravity=0, the wheel centre sits exactly one unloaded radius above the "
        "road so the tire carries no load, and the solver gets a single 1 ms step "
        "with min_internal_step_size == internal_step_size, so a rejected step has "
        "nowhere to go.  The kernel now applies the caliper couple's reaction on "
        "the knuckle (required for the braking load transfer), and this fixture "
        "cannot satisfy it: the line search finds no descent, and where it does "
        "converge the blocker is the raw-max increment criterion "
        "(axle_kernel.cpp:14298) against increment_tolerance = "
        "constraint_tolerance * scale.  The residual floor scales as "
        "1/gas_stiffness of the suspension, i.e. it is the zero-stiffness "
        "mechanism that cannot react the couple.  Giving the fixture a gas spring "
        "does not rescue it either: it then trips 'contact event localization "
        "failed' on the negative-spin half, because the single non-subdividable "
        "step cannot resolve the contact switching.  So this needs a fixture "
        "redesign (loaded tire and/or substeppable solver), not a tweak.  "
        "Reproduce with the raw/brake_fixture_* probes under "
        ".codex-tasks/20260912-native-fiala-parity/tasks/08-adams-braking-case/"
    ),
)
def test_native_brake_opposes_the_instantaneous_wheel_spin() -> None:
    model = _vehicle().model_copy(
        update={
            "driveline": DrivelineSpec(
                maximum_brake_torque=0.01,
                front_brake_bias=0.6,
            )
        }
    )
    positive = vehicle_dynamics_run(
        model,
        _case(model, brake=1.0, wheel_speeds=(("front_left", 10.0),)),
    )
    negative = vehicle_dynamics_run(
        model,
        _case(model, brake=1.0, wheel_speeds=(("front_left", -10.0),)),
    )

    assert positive.body_state("wheel_front_left")[-1, 11] < 10.0
    assert negative.body_state("wheel_front_left")[-1, 11] > -10.0


def test_native_vehicle_applies_a_rack_displacement_target() -> None:
    model = _vehicle()
    result = vehicle_dynamics_run(
        model,
        _case(
            model,
            steering=TimeSignal(times=(0.0, 0.001), values=(0.0, 1.0)),
        ),
    )

    steering = result.steering_state("front_rack")
    assert steering[-1, 2] == 0.001
    assert np.isfinite(steering[-1, 0])


def test_si_vehicle_requires_explicit_gravity() -> None:
    model = _vehicle().model_copy(
        update={
            "units": UnitSystem.SI,
            "front_axle": _vehicle().front_axle.model_copy(
                update={"units": UnitSystem.SI}
            ),
            "rear_axle": _vehicle().rear_axle.model_copy(
                update={"units": UnitSystem.SI}
            ),
        }
    )

    case = VehicleDynamicCase(
        solver=DynamicSolverSettings(
            end_time=0.001,
            step_size=0.001,
            internal_step_size=0.001,
            min_internal_step_size=0.001,
            adaptive_substepping=False,
            integrator="generalized_alpha",
        ),
        vehicle=model,
    )
    with pytest.raises(ValueError, match="gravity.*explicitly"):
        vehicle_dynamics_run(model, case)


def test_analytic_jacobian_stays_consistent_across_steps(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """
    The Newton retry path must rebuild the Jacobian at the current iterate.

    ``newton_step`` reuses the cached constraint Jacobians and the interpolated
    states held in ``primary_workspace``.  A line search that fails leaves that
    workspace describing its last *rejected trial*, so the retry used to assemble
    analytic columns for a point whose residual was not the one being solved.  The
    run then died on its second step with "Newton solve did not converge", because
    the direction was not a descent direction for the true residual.

    With ``SUSPENSION_AXLE_VALIDATE_JACOBIAN`` the kernel compares every analytic
    column against a finite difference of the same residual and aborts on a
    mismatch, so *completing three steps* is the assertion.  Before the fix this
    failed at ``t = 0.001`` with the mismatch reported at the first velocity
    column.
    """
    monkeypatch.setenv("SUSPENSION_AXLE_VALIDATE_JACOBIAN", "1")
    model = _pac2002_model(combined=True, parameter_source="adams_builtin")
    step = 0.001
    case = _case(model).model_copy(
        update={
            "road": RoadSurfaceSpec(kind="plane", origin=Vec3(z=1.0)),
            "initial_states": _uniform_velocity_initial_states(model),
            "solver": DynamicSolverSettings(
                end_time=3.0 * step,
                step_size=step,
                internal_step_size=step,
                min_internal_step_size=step,
                adaptive_substepping=False,
                integrator="generalized_alpha",
                gravity=Vec3(x=0, y=0, z=0),
            ),
        }
    )
    result = vehicle_dynamics_run(model, case)
    assert np.all(result.diagnostics.accepted)
    assert result.axle.tire_output.shape[0] >= 3
