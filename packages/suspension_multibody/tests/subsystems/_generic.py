"""Ordinary document fixtures for the retired assembly surface's invariants."""

from suspension_multibody.authoring import AssemblyDocument, assemble_generic
from suspension_multibody.authoring.loader import DocumentLoader
from suspension_multibody.authoring.migration import migrate_v1_axle, migrate_v1_vehicle
from tests.authoring.test_generic_multibody import _case
from tests.benchmark_fixture import benchmark_model


def axle_source(mode="K", model=None):
    return migrate_v1_axle(benchmark_model() if model is None else model, mode=mode)


def resolved(source):
    return DocumentLoader().load(source, _case()).resolve()


def axle(mode="K", model=None):
    return assemble_generic(axle_source(mode, model))


def vehicle_source(model, mode="K"):
    return migrate_v1_vehicle(model, mode=mode)


def reordered(source):
    payload = source.to_payload()
    payload["subsystems"] = list(reversed(payload["subsystems"]))
    return AssemblyDocument.from_payload(payload, subsystems={row.ref: row.subsystem for row in source.entries})
