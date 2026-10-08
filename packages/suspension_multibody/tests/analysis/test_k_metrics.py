"""K/C metrics query explicit SI frames with stable sign conventions."""

import numpy as np
import pytest

from suspension_multibody.modeling.primitives.spatial import quaternion_to_matrix
from suspension_multibody.report.metrics import compute_k_metrics


class Frames:
    def __init__(self, quaternion=(1, 0, 0, 0)):
        self.rotation = quaternion_to_matrix(np.asarray(quaternion, dtype=float))

    def frame_pose(self, key):
        assert key in {"left", "right"}
        value = np.eye(4)
        value[:3, :3] = self.rotation
        value[:3, 3] = [0, -.003 if key == "left" else .003, .003]
        return value[None]


def test_static_symmetric_metrics_have_zero_differences():
    metrics = compute_k_metrics(Frames(), {"L": "left", "R": "right"})
    assert metrics["camber_deg_difference"] == metrics["toe_deg_difference"] == 0
    assert metrics["track_mm"] == 6


def test_wheel_angles_follow_lateral_axis_and_side_sign():
    result = Frames([.9997546608565223, -.00010381443267525098, .003797996815780008, .021821607145468765])
    metrics = compute_k_metrics(result, {"L": "left", "R": "right"})
    assert metrics["left_camber_deg"] == pytest.approx(-.0023984589007616836)
    assert metrics["right_camber_deg"] == pytest.approx(.0023984589007616836)
    assert metrics["left_toe_deg"] == pytest.approx(-2.500797637638747)
    assert metrics["right_toe_deg"] == pytest.approx(2.500797637638747)
