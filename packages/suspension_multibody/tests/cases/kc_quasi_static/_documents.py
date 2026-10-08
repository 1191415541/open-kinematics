"""K/C declarations with explicit rig IDs and one production execution entry."""

from suspension_multibody.api import simulate, validate
from suspension_multibody.authoring.loader import CaseDocument
from suspension_multibody.authoring.migration import migrate_v1_case, migrate_v1_kc_case
from suspension_multibody.cases.kc_quasi_static.settings import (
    DEFAULT_SETTINGS,
    DEFAULT_TIMES,
)


def documents(model, mode, *, driven=True, **kwargs):
    return migrate_v1_kc_case(model, mode=mode,
        **({"wheel_values_mm": (0.,)} if driven else {"paths": ("fz",), "levels": 3, "maximum": 0}),
        times_s=DEFAULT_TIMES, settings=DEFAULT_SETTINGS, **kwargs)


def native_case(pair, native):
    assembly, base = pair
    compiled = validate(assembly, base)
    known = {}
    for key in ("bodies", "joints", "markers"):
        for row in compiled.model_document[key]:
            known[row["name"].rsplit(".", 1)[-1]] = row["name"]
            if "target" in row:
                known[row["target"].rsplit(".", 1)[-1]] = row["target"]
    mapped = migrate_v1_case({"solver": base.to_payload()["solver"], **native}, entity_ids=known).to_payload()
    original = base.to_payload()
    for key in ("boundaries", "element_activation"):
        mapped[key] = original[key]
    return CaseDocument(mapped)


def run(pair, native):
    return simulate(pair[0], native_case(pair, native)).raw


def body_id(raw, local):
    return next(name for name in raw.document["manifest"]["bodies"] if name.endswith("."+local))
