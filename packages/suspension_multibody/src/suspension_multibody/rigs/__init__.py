"""
Rigs: the test bench, independent of the assembly it drives.

A run is one assembly plus one rig, and the two are registered separately so a new
combination is a registration rather than a new code path.  The rig declares what
it drives, how it solves, and the outputs only it produces; the assembly reports
what it can offer, and the rig's interface **shrinks** to fit -- an assembly with
no steering has no rack axis, and the rig runs without it rather than failing or
padding the axis with zeros.

* `rig.py` -- the registered benches and what each drives.
* `bench.py` -- the physical bench template: the entities a bench contributes.
* `compose.py` -- pairing an assembly with a rig, and the shrinkage.
"""


from .bench import (
    BENCH_CAPABILITIES,
    BENCH_TEMPLATE_NAME,
    bench_capability,
    build_rig_assembly,
    build_rig_fragment,
    rig_ports,
)
from .compose import (
    ASSEMBLIES,
    Composition,
    CompositionError,
    check_assembly,
    combinations,
    compose,
    resolve_combination,
)
from .rig import RIG_NAMES, RIGS, DriveSpec, RigError, RigSpec, get_rig, rig_names

__all__ = [
    "ASSEMBLIES",
    "BENCH_CAPABILITIES",
    "BENCH_TEMPLATE_NAME",
    "RIGS",
    "RIG_NAMES",
    "Composition",
    "CompositionError",
    "DriveSpec",
    "RigError",
    "RigSpec",
    "bench_capability",
    "build_rig_assembly",
    "build_rig_fragment",
    "check_assembly",
    "combinations",
    "compose",
    "get_rig",
    "resolve_combination",
    "rig_names",
    "rig_ports",
]
