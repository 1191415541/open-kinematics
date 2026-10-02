"""
FMI 2.0 Co-Simulation export for a solved suspension_multibody run.

This package has one job: take the two contract documents of a run -- the model
and the case a caller would have submitted -- and frame them as an FMU that an
outside co-simulation tool can load, step, and read outputs from.

Its scope is the epic's D4 ruling, and the ruling is narrow on purpose:

* **FMU 2.0 Co-Simulation**, and nothing else.  No Model Exchange, no real-time
  guarantee, no hardware-in-the-loop claim -- the artifact is for offline
  co-simulation.
* **the model plus its input/output variables**.  There is no Python-side
  evaluation in the artifact: the wrapper binary shipped inside the FMU loads the
  native kernel and calls its one entry point, so a run performed through the FMU
  and a run performed through this package are the same solve.

What is where
-------------

``export``
    The Python half: authoring the two containers, deriving the variable list
    from declarations that already exist, writing ``modelDescription.xml`` and
    packing the archive.  See :func:`export.export_fmu`.
``fmu_wrapper.c``
    The C half: the FMI 2.0 Co-Simulation symbol set, built into the binary the
    archive carries.  It is source, not a checked-in artifact --
    ``scripts/build_fmu_binary.py`` compiles it.

Layering
--------

This package is a *side channel over the public API*: it authors documents and
writes a file, and it reaches the kernel only through the same submission point
every other path uses (``simulation.compiler``).  It therefore sits above
``simulation`` and beside ``api``, and nothing in the existing run path imports
it -- an export cannot change a run that was not exported.
"""

from __future__ import annotations

from .export import (
    FMI_MODEL_IDENTIFIER,
    FMI_VERSION,
    FmiBinding,
    FmiExportError,
    FmiVariable,
    FmuExport,
    bindings_resource,
    description_json,
    export_fmu,
    fmi_binary_name,
    fmi_platform,
    model_description_document,
    read_description,
    variable_declarations,
)

__all__ = [
    "FMI_MODEL_IDENTIFIER",
    "FMI_VERSION",
    "FmiBinding",
    "FmiExportError",
    "FmiVariable",
    "FmuExport",
    "bindings_resource",
    "description_json",
    "export_fmu",
    "fmi_binary_name",
    "fmi_platform",
    "model_description_document",
    "read_description",
    "variable_declarations",
]
