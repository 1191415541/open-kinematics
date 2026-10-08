"""Every public run exposes the same result contract and native submission."""
import numpy as np
import pytest

from suspension_multibody.api import simulate
from suspension_multibody.results import ResultEnvelope
from suspension_multibody.simulation import SimulationRun

from ._documents import PROTOCOLS, documents


@pytest.mark.parametrize("protocol", PROTOCOLS)
def test_public_run_returns_the_uniform_envelope(protocol):
    run = simulate(*documents(protocol))
    assert isinstance(run, SimulationRun)
    assert isinstance(run.result, ResultEnvelope)
    assert run.result.raw is run.raw
    assert run.compiled.request.model is run.result.model
    assert run.status == run.raw.status == run.result.status == "success"
    assert run.request is run.compiled.request
    assert run.result.times_s.size >= 2
    assert run.result.body_ids == run.raw.body_names
    for name, block in run.result.named_blocks.items():
        np.testing.assert_array_equal(block, run.raw.block(name))


def test_fields_are_read_explicitly_from_the_result():
    run = simulate(*documents())
    assert run.result.body_state("wheel.wheel").shape == (3, 19)
    assert run.result.tire_state("wheel.tire").shape == (3, 41)
    assert run.result.element_state("servo.actuator").shape == (3, 4)
    with pytest.raises(AttributeError):
        _ = run.states
    with pytest.raises(AttributeError):
        _ = run.not_a_field


def test_prescribed_motion_is_a_native_submission_with_the_same_envelope():
    from ..physics.test_wheel_spin_boundary import plan, wheel_assembly

    case = plan("prescribed_angle", samples=[0, .001, .002],
                values=[0, .001, .002], rates=[1, 1, 1]).to_document()
    case["initial_state"] = {"wheel.wheel": {"omega": [0, 1, 0]}}
    case["solver"] = {"initialization_mode": "provided_consistent_state"}
    run = simulate(wheel_assembly(), case)
    assert isinstance(run.result, ResultEnvelope)
    assert run.compiled is not None and run.raw is not None
    np.testing.assert_allclose(run.result.body_state("wheel.wheel")[:, 11], 1, atol=1e-7)
