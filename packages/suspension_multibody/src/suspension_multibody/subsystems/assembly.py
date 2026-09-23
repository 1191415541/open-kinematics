"""
The role assembly path: a template instance in, a subsystem contribution out.

This is the one place a template becomes an assembly contribution, and it is
written so that it *cannot* tell one implementation of a role from another.  The
only thing it reads is `SubsystemInstance.bodies` -- the part names the template
declared -- and the only thing it asks the context is whether the model happens
to carry a mass spec for a name.  There is no count, no template name, no role,
and no "does this have bodies" test anywhere in the entry point.

That is what makes requirement 20 / D11 real rather than rhetorical: a detailed
brake template that adds calipers and rotors is assembled by this same function,
unchanged, because the function never learns that the simplified template had no
parts.  A template with zero parts and a template with four parts differ in their
*data*, and this path is data-driven.
"""

from __future__ import annotations

from ..preparation.assembly.types import RigidBody
from ..templates import SubsystemInstance
from .geometry import body_from_spec, body_without_spec
from .types import SubsystemContext, SubsystemOutput

__all__ = ["assemble_from_template"]


def assemble_from_template(
    instance: SubsystemInstance, context: SubsystemContext
) -> SubsystemOutput:
    """
    Turn a template instance into the subsystem contribution it describes.

    Every body the instance declares becomes a runtime body, in declaration
    order: the model's mass spec when it has one, a bare identity body otherwise.
    Zero declared parts yields no bodies, which is exactly what a simplified
    torque-only template asks for -- and it is the same code path a template with
    calipers takes.
    """
    specs = context.body_specs
    return SubsystemOutput(
        bodies={name: _body_for(name, specs.get(name)) for name in instance.bodies}
    )


def _body_for(name: str, spec: object | None) -> RigidBody:
    """
    Return the runtime body for `name`, from its spec when the model has one.

    The presence of a mass spec is a property of the *model*, not of the
    template's implementation: the same template assembled against a model that
    declares the body and against one that does not must both succeed.  This is
    the only conditional on the path, and it is not a judgement about which
    template is in use.
    """
    if spec is None:
        return body_without_spec(name)
    return body_from_spec(name, spec)
