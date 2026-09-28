"""File based authoring API."""
from .documents import (
    FUNCTIONAL_ROLES,
    OVERRIDE_KEYS,
    PLACEMENT_ROLES,
    SUBSYSTEM_VALUE_KEYS,
    TOPOLOGY_KEYS,
    WHEEL_ENDS,
    AssemblyDocument,
    AssemblyEntry,
    EffectiveSubsystem,
    RigDocument,
    SimulationAssembly,
    SubsystemDocument,
    TemplateDocument,
)
from .errors import AuthoringError, ElementPropertyError, TemplateAuthoringError
from .project import DOCUMENT_KINDS, Project, ProjectError
from .properties import ELEMENT_MODELS, ElementPropertyDocument
from .security import (
    EXPERT_FIELDS,
    USER_ASSEMBLY_FIELDS,
    USER_FIELDS,
    AuthoringPermissionError,
    ExpertAuthoring,
    Revision,
    UserAuthoring,
)
from .vehicle import vehicle_document_from, vehicle_model_from

__all__ = [
    "DOCUMENT_KINDS",
    "ELEMENT_MODELS",
    "EXPERT_FIELDS",
    "USER_ASSEMBLY_FIELDS",
    "USER_FIELDS",
    "FUNCTIONAL_ROLES",
    "OVERRIDE_KEYS",
    "PLACEMENT_ROLES",
    "SUBSYSTEM_VALUE_KEYS",
    "TOPOLOGY_KEYS",
    "WHEEL_ENDS",
    "AssemblyDocument",
    "AssemblyEntry",
    "AuthoringError",
    "AuthoringPermissionError",
    "ExpertAuthoring",
    "EffectiveSubsystem",
    "ElementPropertyDocument",
    "ElementPropertyError",
    "Project",
    "ProjectError",
    "Revision",
    "RigDocument",
    "SimulationAssembly",
    "SubsystemDocument",
    "TemplateAuthoringError",
    "TemplateDocument",
    "UserAuthoring",
    "vehicle_document_from",
    "vehicle_model_from",
]
