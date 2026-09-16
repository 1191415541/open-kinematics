from __future__ import annotations

import ctypes
import json
import math
import re
from pathlib import Path

import numpy as np
import pytest

from suspension_multibody.axle_dynamics import (
    BODY_STATE_COLUMNS,
    DIAGNOSTIC_COLUMNS,
    ENERGY_COLUMNS,
    TIRE_OUTPUT_COLUMNS,
    AxleAntiRollBar,
    AxleBody,
    AxleBushing,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleJoint,
    AxleSolverSettings,
    AxleSpringDamper,
    AxleTire,
    load_axle_dynamics_case,
    load_axle_dynamics_model,
    run_axle_dynamics,
    write_axle_dynamics_artifact,
)
from suspension_multibody.axle_dynamics.schema import (
    PAC2002_PARAMETER_DEFAULTS,
    PAC2002_PARAMETER_NAMES,
)


def test_pac2002_parameter_abi_matches_native_header() -> None:
    # K1 moved the kernel sources out of `cpp/` at the repository root and into
    # the generic kernel package.  Resolve the header from there; the parameter
    # count it declares is a fixed part of the shared C ABI.
    root = next(
        parent
        for parent in Path(__file__).resolve().parents
        if (
            parent
            / "packages/suspension_kernel/cpp/axle_dynamics/axle_kernel.hpp"
        ).is_file()
    )
    cpp = root / "packages/suspension_kernel/cpp/axle_dynamics"
    # The count lives in the dependency-free enumeration header, and the public
    # ABI header pulls that in.  Read both and treat the pair as the declaration
    # surface, so relocating a declaration between the two does not read as a
    # contract change while removing it altogether still fails.
    header = (
        (cpp / "axle_kernel.hpp").read_text(encoding="utf-8")
        + (
            root
            / "packages/suspension_kernel/cpp/include/mb_model/enums.hpp"
        ).read_text(encoding="utf-8")
    )
    match = re.search(r"VEHICLE_PAC2002_PARAMETER_COUNT\s*=\s*(\d+)", header)

    assert match is not None
    assert len(PAC2002_PARAMETER_NAMES) == int(match.group(1))
    assert set(PAC2002_PARAMETER_NAMES) <= set(PAC2002_PARAMETER_DEFAULTS)
    # The layout is append-only: earlier positions must never move.  Anchor on
    # the boundaries that were appended in turn, rather than on the tail, so that
    # a future append does not require rewriting this assertion.
    assert PAC2002_PARAMETER_NAMES[0] == "FNOMIN"
    use_mode_index = PAC2002_PARAMETER_NAMES.index("USE_MODE")
    assert use_mode_index == 167
    assert PAC2002_PARAMETER_NAMES[use_mode_index + 1] == "KPUMIN"
    assert PAC2002_PARAMETER_NAMES.index("FZMAX") == use_mode_index + 8
    assert PAC2002_PARAMETER_NAMES[use_mode_index + 9] == "MC"


def _vertical_slider_model() -> AxleDynamicsModel:
    mass = 10.0
    stiffness = 10_000.0
    equilibrium_length = 0.25 - mass * 9.80665 / stiffness
    return AxleDynamicsModel(
        name="vertical-slider",
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                fixed=True,
            ),
            AxleBody(
                name="slider",
                mass_kg=mass,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                position_m=(0.0, 0.0, equilibrium_length),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
        ),
        springs=(
            AxleSpringDamper(
                name="spring",
                body_a="ground",
                body_b="slider",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                stiffness_n_per_m=stiffness,
                compression_damping_n_s_per_m=100.0,
                rebound_damping_n_s_per_m=100.0,
                free_length_m=0.25,
            ),
        ),
    )


def test_axle_input_rejects_a_struct_size_mismatch() -> None:
    """
    The axle ABI must report a layout mismatch instead of reading past the caller.

    ``axle_run`` is called directly rather than through the product wrapper: this
    is a statement about the kernel's own gate, and the kernel has to reject the
    bad structure before it looks at any payload field.
    """
    from suspension_multibody.axle_dynamics import native

    library = native._load_library()
    buffer = ctypes.create_string_buffer(4096)

    def call(input_value: object, output_value: object) -> tuple[int, str]:
        code = library.axle_run(
            ctypes.byref(input_value),  # type: ignore[arg-type]
            ctypes.byref(output_value),  # type: ignore[arg-type]
            buffer,
            len(buffer),
        )
        return int(code), buffer.value.decode("utf-8", "replace")

    truncated = native._AxleInput()
    truncated.struct_size = 8
    truncated.abi_version = native._NATIVE_KERNEL_ABI_VERSION
    code, message = call(truncated, native._AxleOutput())
    assert code == 4, message
    assert "axle ABI mismatch" in message

    current = native._AxleInput()
    current.struct_size = ctypes.sizeof(native._AxleInput)
    current.abi_version = native._NATIVE_KERNEL_ABI_VERSION
    version_skewed_output = native._AxleOutput()
    version_skewed_output.struct_size = ctypes.sizeof(native._AxleOutput)
    version_skewed_output.abi_version = native._NATIVE_KERNEL_ABI_VERSION + 1
    code, message = call(current, version_skewed_output)
    assert code == 4, message
    assert "axle ABI mismatch" in message


def test_element_block_spring_matches_the_parallel_array_spring() -> None:
    """
    The same spring, spelled as a block and as parallel arrays, must agree.

    This is the real test of the generic element surface: one model, two spellings,
    and an exact comparison.  Anything the block reader gets wrong about offsets,
    body indices or validity rules shows up as a difference in the solved state,
    which the array spelling does not share.
    """
    from suspension_multibody.axle_dynamics import native

    model = _vertical_slider_model()
    case = AxleDynamicsCase(
        name="block-equivalence",
        times_s=(0.0, 0.001, 0.002),
        solver=AxleSolverSettings(internal_step_s=0.00025),
    )

    by_array = native._run_native(model, case).result
    block = native._spring_element_block(
        body_a=0,
        body_b=1,
        stiffness=10_000.0,
        compression_damping=100.0,
        rebound_damping=100.0,
        free_length=0.25,
        point_a=(0.0, 0.0, 0.0),
        point_b=(0.0, 0.0, 0.0),
    )
    by_block = native._run_native(model, case, element_blocks=(block,)).result

    np.testing.assert_array_equal(
        by_block.body_state("slider"), by_array.body_state("slider")
    )
    # The spring channel is deliberately not compared here.  With blocks the solver
    # builds one spring, while the array-driven run in this fixture also builds
    # one, so the two buffers have the same shape -- but the block run's buffer is
    # sized from the block count and the *tail* of the array run's is padding.  The
    # comparison that matters is the integrated state above; comparing observer
    # output would be measuring the allocator, not the reader.
    np.testing.assert_array_equal(
        by_block.spring_state("spring")[:1], by_array.spring_state("spring")[:1]
    )
    assert np.all(by_block.diagnostics.accepted)

    # Negative control: an agreement between the two spellings would also appear
    # if the block were ignored and both runs used the arrays.  Changing one block
    # parameter has to change the solved state, which rules that out.
    softer = native._spring_element_block(
        body_a=0,
        body_b=1,
        stiffness=5_000.0,
        compression_damping=100.0,
        rebound_damping=100.0,
        free_length=0.25,
        point_a=(0.0, 0.0, 0.0),
        point_b=(0.0, 0.0, 0.0),
    )
    by_softer_block = native._run_native(
        model, case, element_blocks=(softer,)
    ).result
    assert not np.array_equal(
        by_softer_block.body_state("slider"), by_array.body_state("slider")
    ), "the element block was ignored: a halved stiffness changed nothing"


def test_element_blocks_reach_the_vehicle_entry_point() -> None:
    """
    The element surface must be read by `vehicle_run`, not only by `axle_run`.

    `_VehicleInput` embeds `_AxleInput` by value, so the element fields set on the
    axle structure do not travel with the copy.  A driven coordinate is the
    lightest vehicle-only feature to declare, and declaring one is what makes
    `_run_native` take the `vehicle_run` path at all.

    Two things must hold for this to pass: the vehicle structure must receive the
    block arrays, and the component output buffers must be sized for the elements
    the kernel will build.  An earlier version satisfied neither, and the two
    failures were a spring-curve count mismatch and then an undersized buffer --
    both because the vehicle path read zero springs while the block declared one.

    The evidence is deliberately a *rejection* rather than a motion difference: the
    driven row holds the carrier at the origin, so a block's force has nothing to
    move and a state comparison would pass even if the block were dropped.  A tire
    element, on the other hand, is one the block reader refuses by name -- so if
    the vehicle path answers with that refusal, the reader ran there.
    """
    from suspension_multibody.axle_dynamics import native
    from suspension_multibody.axle_dynamics.schema import AxleDrivenCoordinate

    model = AxleDynamicsModel(
        name="vehicle-route",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                fixed=True,
            ),
            AxleBody(
                name="carrier",
                mass_kg=10.0,
                inertia_kg_m2=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
            ),
        ),
        joints=(),
        driven_coordinates=(
            AxleDrivenCoordinate(
                name="prescribed",
                kind="translation",
                body="carrier",
                reaction_body="ground",
                axis_local=(1.0, 0.0, 0.0),
                reaction_point_local_m=(0.0, 0.0, 0.0),
                point_local_m=(0.0, 0.0, 0.0),
            ),
        ),
    )
    case = AxleDynamicsCase(
        name="vehicle-route",
        times_s=(0.0, 0.001, 0.002),
        solver=AxleSolverSettings(internal_step_s=0.00025),
        driven_target_m={"prescribed": (0.0, 0.0, 0.0)},
    )

    block = native._spring_element_block(
        body_a=0,
        body_b=1,
        stiffness=10_000.0,
        compression_damping=100.0,
        rebound_damping=100.0,
        free_length=0.1,
        point_a=(0.0, 0.0, 0.0),
        point_b=(0.0, 0.0, 0.0),
    )
    # A supported block runs to completion through the vehicle entry point.
    with_blocks = native._run_native(model, case, element_blocks=(block,)).result
    assert np.all(np.isfinite(with_blocks.body_state("carrier")))

    # A tire block without its PAC2002 table is one the reader refuses by name.
    # That refusal can only come from the vehicle path having read the block, which
    # is what makes it evidence: if the element list had arrived empty, the model
    # would have built without complaint instead.
    unsupported = native._ElementBlock()
    unsupported.kind = native.ELEMENT_TIRE
    unsupported.body_a = 1
    unsupported.body_b = -1
    unsupported.ints[0] = 1  # VEHICLE_TIRE_PAC2002_PURE_SLIP
    # Give the scalars valid values so the reader reaches the table check rather
    # than stopping at the parameter sanity check first.
    unsupported.parameters[157] = 0.3   # ELEMENT_TIRE_RADIUS
    unsupported.parameters[154] = 200_000.0  # ELEMENT_TIRE_STIFFNESS
    unsupported.parameters[155] = 1_000.0    # ELEMENT_TIRE_DAMPING
    unsupported.parameters[156] = 0.2        # ELEMENT_TIRE_MAXIMUM_COMPRESSION
    with pytest.raises(Exception) as captured:  # noqa: B017 - the message is the evidence
        native._run_native(model, case, element_blocks=(unsupported,))
    assert "parameter table" in str(captured.value), str(captured.value)


def test_native_solver_preserves_static_equilibrium() -> None:
    model = _vertical_slider_model()
    case = AxleDynamicsCase(
        name="static-equilibrium",
        times_s=(0.0, 0.001, 0.002),
        solver=AxleSolverSettings(internal_step_s=0.00025),
    )

    result = run_axle_dynamics(model, case)
    slider = result.body_state("slider")

    expected_z = 0.25 - 10.0 * 9.80665 / 10_000.0
    np.testing.assert_allclose(slider[:, 2], expected_z, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(slider[:, 7:], 0.0, atol=1e-9, rtol=0.0)
    spring = result.spring_state("spring")
    np.testing.assert_allclose(spring[:, 0], expected_z, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(spring[:, 1], 0.0, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(
        spring[:, 2], 10.0 * 9.80665, atol=1e-6, rtol=1e-10
    )
    np.testing.assert_allclose(spring[:, 3:6], 0.0, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(
        spring[:, 6], 10.0 * 9.80665, atol=1e-6, rtol=1e-10
    )
    assert np.all(result.diagnostics.accepted)
    assert np.max(result.diagnostics.position_residual) <= 1e-8
    assert np.max(result.diagnostics.velocity_residual) <= 1e-7


def test_tire_contact_is_at_patch_and_reports_physical_loads() -> None:
    model = AxleDynamicsModel(
        name="tire-slider",
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                fixed=True,
            ),
            AxleBody(
                name="wheel",
                mass_kg=10.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                position_m=(0.0, 0.0, 0.31),
            ),
        ),
        joints=(
            AxleJoint(
                name="guide",
                kind="prismatic",
                body_a="ground",
                body_b="wheel",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
            ),
        ),
        tires=(
            AxleTire(
                name="wheel_tire",
                body="wheel",
                unloaded_radius_m=0.3,
                maximum_compression_m=0.05,
                vertical_stiffness_n_per_m=10_000.0,
                vertical_damping_n_s_per_m=100.0,
                longitudinal_friction_coefficient=1.0,
                lateral_friction_coefficient=1.0,
                longitudinal_brush_stiffness_n_per_m=100_000.0,
                lateral_brush_stiffness_n_per_m=100_000.0,
                longitudinal_relaxation_length_m=0.2,
                lateral_relaxation_length_m=0.2,
                detached_relaxation_s=0.02,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="tire-static",
            times_s=(0.0, 0.001),
            solver=AxleSolverSettings(internal_step_s=0.00025),
        ),
    )

    tire = result.tire_output[:, 0, :]
    np.testing.assert_allclose(tire[:, 4], 10.0 * 9.80665, atol=1e-5, rtol=1e-8)
    assert np.all(tire[:, 0] == 1.0)
    assert np.all(tire[:, 9] <= 1.0 + 1e-12)
    assert np.all(np.isfinite(result.energy))


def test_bushing_reference_and_static_force_balance() -> None:
    model = AxleDynamicsModel(
        name="bushing-slider",
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                fixed=True,
            ),
            AxleBody(
                name="body",
                mass_kg=10.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                position_m=(0.0, 0.0, 0.3),
            ),
        ),
        joints=(),
        bushings=(
            AxleBushing(
                name="mount",
                body_a="ground",
                body_b="body",
                point_a_m=(0.0, 0.0, 0.0),
                point_b_m=(0.0, 0.0, 0.0),
                reference_translation_in_frame_a_m=(0.0, 0.0, 0.3),
                reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                stiffness=(
                    (1000.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                    (0.0, 1000.0, 0.0, 0.0, 0.0, 0.0),
                    (0.0, 0.0, 10_000.0, 0.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 1000.0, 0.0, 0.0),
                    (0.0, 0.0, 0.0, 0.0, 1000.0, 0.0),
                    (0.0, 0.0, 0.0, 0.0, 0.0, 1000.0),
                ),
                damping=((0.0,) * 6,) * 6,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="bushing-static",
            times_s=(0.0, 0.001),
            solver=AxleSolverSettings(internal_step_s=0.00025),
        ),
    )

    expected_z = 0.3 - 10.0 * 9.80665 / 10_000.0
    np.testing.assert_allclose(
        result.body_state("body")[:, 2], expected_z, atol=1e-6, rtol=0.0
    )
    bushing = result.bushing_state("mount")
    np.testing.assert_allclose(
        bushing[:, 2], -10.0 * 9.80665 / 10_000.0, atol=1e-6, rtol=0.0
    )
    np.testing.assert_allclose(bushing[:, :2], 0.0, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(bushing[:, 3:6], 0.0, atol=1e-9, rtol=0.0)
    np.testing.assert_allclose(
        bushing[:, 8], 10.0 * 9.80665, atol=1e-5, rtol=1e-8
    )
    np.testing.assert_allclose(bushing[:, [6, 7, 9, 10, 11]], 0.0, atol=1e-9)


def test_anti_roll_bar_reports_physical_angle_rate_and_torque() -> None:
    angle = 0.04
    angular_rate = 0.2
    stiffness = 5000.0
    damping = 50.0
    model = AxleDynamicsModel(
        name="anti-roll-output",
        gravity_m_per_s2=(0.0, 0.0, 0.0),
        bodies=(
            AxleBody(
                name="ground",
                mass_kg=0.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                fixed=True,
            ),
            AxleBody(
                name="arm",
                mass_kg=10.0,
                inertia_kg_m2=(
                    (1.0, 0.0, 0.0),
                    (0.0, 1.0, 0.0),
                    (0.0, 0.0, 1.0),
                ),
                quaternion_body_to_world=(
                    math.cos(0.5 * angle),
                    0.0,
                    0.0,
                    math.sin(0.5 * angle),
                ),
                angular_velocity_rad_per_s=(0.0, 0.0, angular_rate),
            ),
        ),
        joints=(),
        anti_roll_bars=(
            AxleAntiRollBar(
                name="bar",
                body_a="ground",
                body_b="arm",
                axis_a=(0.0, 0.0, 1.0),
                reference_quaternion_a_to_b=(1.0, 0.0, 0.0, 0.0),
                stiffness_n_m_per_rad=stiffness,
                damping_n_m_s_per_rad=damping,
            ),
        ),
    )
    result = run_axle_dynamics(
        model,
        AxleDynamicsCase(
            name="anti-roll-output",
            times_s=(0.0, 0.0001),
            solver=AxleSolverSettings(
                initialization_mode="provided_consistent_state",
                adaptive_step=False,
                internal_step_s=0.0001,
            ),
        ),
    )

    output = result.anti_roll_bar_state("bar")[0]
    np.testing.assert_allclose(output[0], angle, atol=1e-12, rtol=0.0)
    np.testing.assert_allclose(output[1], angular_rate, atol=1e-12, rtol=0.0)
    np.testing.assert_allclose(
        output[2], -stiffness * angle - damping * angular_rate, atol=1e-10
    )


def test_axle_schema_loader_and_result_artifact_are_self_describing(
    tmp_path: Path,
) -> None:
    model = _vertical_slider_model()
    case = AxleDynamicsCase(
        name="artifact",
        times_s=(0.0, 0.001),
        solver=AxleSolverSettings(internal_step_s=0.00025),
    )
    model_path = tmp_path / "model.json"
    case_path = tmp_path / "case.yaml"
    model_path.write_text(
        json.dumps(model.model_dump(mode="json")),
        encoding="utf-8",
    )
    import yaml

    case_path.write_text(
        yaml.safe_dump(case.model_dump(mode="json")),
        encoding="utf-8",
    )
    loaded_model = load_axle_dynamics_model(model_path)
    loaded_case = load_axle_dynamics_case(case_path)
    result = run_axle_dynamics(loaded_model, loaded_case)
    manifest_path = write_axle_dynamics_artifact(
        result, loaded_model, loaded_case, tmp_path / "artifact"
    )

    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "success"
    assert len(manifest["model_sha256"]) == 64
    assert len(manifest["case_sha256"]) == 64
    assert manifest["layouts"]["body_state"] == list(BODY_STATE_COLUMNS)
    assert manifest["layouts"]["diagnostics"] == list(DIAGNOSTIC_COLUMNS)
    assert manifest["layouts"]["energy"] == list(ENERGY_COLUMNS)
    assert manifest["layouts"]["tire_output"] == list(TIRE_OUTPUT_COLUMNS)
    # The layout is a published contract, so it is pinned here rather than only
    # derived: the first fifteen columns are the forces, moments and contact
    # kinematics Adams can also report, and the last six are the contact-body states
    # of the advanced transient modes (USE_MODE 21-25), which Adams has no request
    # channel for and which therefore have to be appended.
    assert TIRE_OUTPUT_COLUMNS[:3] == (
        "active",
        "gap_m",
        "penetration_m",
    )
    assert TIRE_OUTPUT_COLUMNS[12:15] == (
        "overturning_moment_n_m",
        "rolling_resistance_moment_n_m",
        "aligning_moment_n_m",
    )
    assert TIRE_OUTPUT_COLUMNS[15:21] == (
        "contact_body_longitudinal_m",
        "contact_body_longitudinal_rate_m_per_s",
        "contact_body_lateral_m",
        "contact_body_lateral_rate_m_per_s",
        "contact_body_yaw_rad",
        "contact_body_yaw_rate_rad_per_s",
    )
    # The next four are the turn-slip relaxation states of USE_MODE 25.
    assert TIRE_OUTPUT_COLUMNS[21:25] == (
        "turn_slip_phi_c_rad_per_m",
        "turn_slip_phi_f2_rad_per_m",
        "turn_slip_phi_1_rad_per_m",
        "turn_slip_phi_2_rad_per_m",
    )
    assert TIRE_OUTPUT_COLUMNS[25:] == (
        "rolling_speed_m_per_s",
        "slip_reference_speed_m_per_s",
        "lateral_slip_base_rad",
        "lateral_slip_beta_term_rad",
        "lateral_slip_beta_st_term_rad",
        "lateral_slip_target_rad",
        "lateral_slip_target_clamped_rad",
        "turn_slip_force_rad_per_m",
        "turn_slip_moment_rad_per_m",
        "turn_slip_drive_rad_per_s",
        "turn_slip_yaw_rate_rad_per_s",
        "turn_slip_camber_term_rad_per_s",
        "turn_slip_total_spin_rate_rad_per_s",
        "lateral_slip_relaxation_length_m",
        "longitudinal_relaxed_slip",
        "lateral_relaxed_slip",
    )
    assert len(TIRE_OUTPUT_COLUMNS) == 41
    with np.load(manifest_path.parent / "arrays.npz") as arrays:
        np.testing.assert_allclose(arrays["states"], result.states)
        np.testing.assert_allclose(arrays["energy"], result.energy)
