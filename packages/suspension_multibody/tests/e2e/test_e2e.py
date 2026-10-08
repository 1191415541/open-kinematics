"""Ordinary document submission and complete channel artifact round trip."""
import numpy as np

from suspension_multibody.api import simulate
from suspension_multibody.authoring.migration import migrate_v1_kc_case
from suspension_multibody.io import read_artifact, write_artifact

from ..api.test_api import kc_model


def test_end_to_end_result_has_stable_state_table(tmp_path):
    source, case = migrate_v1_kc_case(kc_model(), mode="K")
    run = simulate(source, case)
    stored = read_artifact(write_artifact(run.result, tmp_path))
    assert run.status == stored["manifest"]["status"] == "success"
    assert len(run.result.cases) == 1
    for name, values in run.result.named_blocks.items():
        np.testing.assert_array_equal(stored["arrays"][name], values)
