"""The offline built-in declaration table preserves frozen activation semantics."""

import pytest

from suspension_multibody.templates import DOUBLE_WISHBONE
from tests.subsystems._generic import axle


@pytest.mark.parametrize("mode,joints,bushings", [("K", 16, 0), ("C", 12, 8)])
def test_builtin_activation_reaches_the_common_graph(mode, joints, bushings):
    graph = axle(mode)
    assert len(graph.joints) == joints
    assert sum(row["type"] == "bushing" for row in graph.elements) == bushings
    names = {row["name"].rsplit(".", 1)[-1]: row["type"] for row in graph.joints}
    elements = {row["name"].rsplit(".", 1)[-1] for row in graph.elements}
    for connection in DOUBLE_WISHBONE.connections:
        if connection.active_column(mode) == "joint":
            assert names[connection.name] == connection.joint_kind(mode)
        elif connection.active_column(mode) == "bushing":
            assert connection.bushing in elements
            assert connection.name not in names
        else:
            assert connection.name not in names
    assert names["rack_guide"] == "prismatic"
    assert names["housing_mount"] == "fixed"


def test_inboard_rear_points_remain_axis_locators_in_k_mode():
    names = {row["name"].rsplit(".", 1)[-1] for row in axle("K").joints}
    for side in ("L", "R"):
        for arm in ("uca", "lca"):
            assert f"{arm}_mount_{side}_inner_front" in names
            assert f"{arm}_mount_{side}_inner_rear" not in names
