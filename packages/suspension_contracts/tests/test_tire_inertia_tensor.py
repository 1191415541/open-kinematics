"""Products of inertia have a sign; physical tensors remain validated."""
import copy

import pytest

from suspension_contracts import ContractError, validate_model

MODEL = {
    "contract": "multibody-model", "contract_version": 1, "kind": "model", "name": "inertia",
    "units": {"length": "m", "mass": "kg", "time": "s", "angle": "rad"}, "bodies": [],
    "tires": [{"name": "wheel", "model": "fiala", "body": "wheel", "inertia": [[2.0, -0.5, 0.0], [-0.5, 2.0, 0.0], [0.0, 0.0, 2.0]]}],
}


def test_negative_products_of_inertia_are_valid():
    validate_model(MODEL)


@pytest.mark.parametrize("tensor", [
    [[-1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]],
    [[6.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]],
    [[2.0, -0.5, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, 2.0]],
])
def test_nonphysical_or_asymmetric_tensor_is_rejected(tensor):
    payload = copy.deepcopy(MODEL)
    payload["tires"][0]["inertia"] = tensor
    with pytest.raises(ContractError, match="inertia"):
        validate_model(payload)
