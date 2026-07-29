"""Common schema validation tests."""

import pytest
from pydantic import ValidationError

from suspension_mbd.schema import Pose, Quaternion, Vec3


def test_vectors_accept_yaml_sequences() -> None:
    assert Vec3.model_validate([1, 2, 3]).as_tuple() == (1.0, 2.0, 3.0)
    assert Quaternion.model_validate([1, 0, 0, 0]).as_tuple() == (1.0, 0.0, 0.0, 0.0)
    assert Pose.model_validate({"translation": [1, 2, 3]}).translation.z == 3


def test_quaternion_must_be_unit() -> None:
    with pytest.raises(ValidationError, match="unit norm"):
        Quaternion(w=2)


def test_common_models_reject_unknown_fields() -> None:
    with pytest.raises(ValidationError):
        Vec3(x=1, y=2, z=3, extra=4)
