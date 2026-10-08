"""Tire inertia evidence through the shared document compiler."""

from suspension_multibody.api import validate
from suspension_multibody.authoring import migrate_v1_dynamic_axle
from suspension_multibody.axle_dynamics.schema import (
    AxleBody,
    AxleDynamicsCase,
    AxleDynamicsModel,
    AxleTire,
)


def tire_document(tire: AxleTire):
    model = AxleDynamicsModel(name="tire-inertia", bodies=(AxleBody(
        name=tire.body, mass_kg=1, inertia_kg_m2=((1, 0, 0), (0, 1, 0), (0, 0, 1)), fixed=True),),
        joints=(), tires=(tire,))
    case = AxleDynamicsCase(name="tire-inertia", times_s=(0, .001))
    compiled = validate(*migrate_v1_dynamic_axle(model, case))
    return compiled.model_document["tires"][0]
