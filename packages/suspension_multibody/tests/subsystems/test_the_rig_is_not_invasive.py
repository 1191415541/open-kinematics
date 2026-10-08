"""Adding an ordinary rig preserves every pre-existing physical declaration."""

import pytest

from suspension_multibody.authoring import assemble_generic
from suspension_multibody.rigs import RIGS
from tests.rigs._generic import wheel_specimen


@pytest.mark.parametrize("name", tuple(RIGS))
def test_every_rig_preserves_the_specimen_subgraph(name):
    before = assemble_generic(wheel_specimen()).resolved_model()
    after = assemble_generic(wheel_specimen(name)).resolved_model()
    original, bound = before.to_document(), after.to_document()
    selected = [row["name"] for row in original["bodies"]]
    assert before.subgraph_fingerprint(selected) == after.subgraph_fingerprint(selected)
    for section in ("joints", "elements", "tires", "coordinates"):
        assert original[section] == bound[section]
    assert {row["name"] for row in bound["bodies"]} - set(selected) == {"rig.fixture"}
    assert len(bound["tires"]) == 2
    assert {row["body"] for row in bound["tires"]} == {"left.wheel", "right.wheel"}
