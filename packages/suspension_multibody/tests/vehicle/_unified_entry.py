"""Test-only labels over the production ResultEnvelope for frozen assertions."""

from types import SimpleNamespace

import numpy as np

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.migration import migrate_v1_vehicle_case
from suspension_multibody.report.metrics import compute_case_metrics

from ..axle_dynamics._frozen_result_projection import DIAGNOSTIC_FIELDS


def compile_vehicle(model, case):
    return validate(*migrate_v1_vehicle_case(case.model_copy(update={"vehicle": model})))


def solve_vehicle(model, case):
    run = simulate(*migrate_v1_vehicle_case(case.model_copy(update={"vehicle": model})))
    return VehicleEvidence(run)


class VehicleEvidence:
    def __init__(self, run):
        self.run = run
        self.envelope = run.result
        self.times_s = run.result.times_s
        self._bodies = {self._body_alias(name): name for name in run.result.body_ids}
        self.body_names = tuple(self._bodies)
        self.states = np.stack([run.result.body_state(name) for name in self._bodies.values()], axis=1)
        self._elements = {}
        for key in ("elements", "tires"):
            for row in run.compiled.model_document.get(key, ()):
                ref, name = row["name"].rsplit(".", 1)
                alias = ref.split("_", 1)[0]+"_"+name if ref.startswith(("front_", "rear_")) else name
                self._elements[alias] = row["name"]
                if row.get("type") == "steering_actuator":
                    self._elements[ref.removeprefix("steering_")] = row["name"]
        tires = run.result.tire_ids
        tire_output = np.stack([run.result.tire_state(name) for name in tires], axis=1) if tires else np.zeros((len(self.times_s), 0, 41))
        self.axle = SimpleNamespace(tire_output=tire_output, energy=run.result.energy)
        rows = run.raw.diagnostics
        self.diagnostics = SimpleNamespace(**{field: rows[:, index].astype(bool) if field == "accepted" else rows[:, index]
            for index, field in enumerate(DIAGNOSTIC_FIELDS)})
        self.metrics = compute_case_metrics("vehicle_dynamic", run.result)

    @staticmethod
    def _body_alias(name):
        ref, local = name.rsplit(".", 1)
        return ref.split("_", 1)[0]+"_"+local if ref.startswith(("front_", "rear_")) else local

    def body_state(self, name):
        return self.envelope.body_state(self._bodies[name])

    def steering_state(self, name):
        return self.envelope.element_state(self._elements[name])

    def spring_state(self, name):
        return self.envelope.element_state(self._elements[name])

    damper_state = spring_state
    bump_stop_state = spring_state
    bushing_state = spring_state
