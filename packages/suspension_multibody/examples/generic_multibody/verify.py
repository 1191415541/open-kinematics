"""Run the file examples and check their physical readings."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from suspension_multibody import simulate
from suspension_multibody.results.envelope import ResultEnvelope


def main() -> None:
    """Validate native file-based tire inertia and signed torque feedback."""
    root = Path(__file__).parent
    case = json.loads((root / "dynamic.case.json").read_text(encoding="utf-8"))
    tire = simulate(root / "tire.assembly.json", case)
    assert tire.status == "success"
    assert tire.raw is not None and tire.compiled is not None
    assert isinstance(tire.result, ResultEnvelope)
    contact = tire.result.tire_state(tire.result.tire_ids[0])
    np.testing.assert_allclose(contact[:, 4], 21 * 9.80665, atol=1e-8)
    assert len(tire.compiled.model_document["tires"]) == 1
    model = tire.compiled.model_document
    assert sum(body.get("mass", 0) for body in model["bodies"]) == 21
    assert model["tires"][0]["mass"] == 0
    case = json.loads((root / "rotor.case.json").read_text(encoding="utf-8"))
    rotor = simulate(root / "rotor.assembly.json", case)
    assert rotor.status == "success"
    assert rotor.raw is not None
    assert isinstance(rotor.result, ResultEnvelope)
    omega = rotor.result.body_state("rotor.sub.json.rotor")[-1, 12]
    assert -0.06 < omega < 0
    print(f"tire load: {contact[-1, 4]:.8f} N; total mass: 21 kg")
    print(f"signed rotor feedback: omega={omega:.10f} rad/s")


if __name__ == "__main__":
    main()
