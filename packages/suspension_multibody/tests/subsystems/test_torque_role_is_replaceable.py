"""
The hard gate: a detailed template replaces the simplified one with no code change.

Requirement 20 / D11 says the simplified brake and drive must be *providers*, not
special cases.  The proof is here, and it has two halves that have to hold
together:

* **behavioural** -- register a second template under the same `brake` role that
  declares one rigid body (the stand-in for a caliper/rotor template), and run it
  through *the same* `subsystems.brake.build` object.  The body appears.  Switch
  back to the simplified template through the same `build` object and nothing
  appears.  Same function, two templates, two answers: the answer came from the
  template data.
* **structural** -- the assembly path's own source contains no decision that
  could distinguish the two: no `len(...)` count, no name/role comparison, no
  "is it simplified" flag.  The one conditional it *does* have is "does the model
  carry a mass spec for this name", which is a property of the model and not of
  the template's implementation.

The structural half is asserted by walking the AST rather than by substring
matching, because the module's prose legitimately says "simplified" and a
substring check would either forbid the explanation or miss a real branch.
"""

from __future__ import annotations

import ast
import inspect

from suspension_multibody.subsystems import (
    AssemblyRequest,
    SubsystemContext,
    assemble_from_template,
    brake,
    drive,
)
from suspension_multibody.templates import (
    ConnectionDefinition,
    PartDefinition,
    PropertySlot,
    Template,
    instantiate,
    register,
)
from tests.benchmark_fixture import benchmark_model

#: A caliper-like body name the benchmark model does not declare a mass spec for,
#: so the assembly path has to fall back to a bare body -- which it must.
STUB_PART = "caliper_L"
STUB_BRAKE_NAME = "brake_4wdisk_calipers_stub"
STUB_DRIVE_NAME = "powertrain_stub"

#: Names that would mean the assembly path is inspecting *which* implementation it
#: got.  None may be read by the entry point, which maps over the declared bodies
#: and delegates the per-body choice without making a test of its own.
FORBIDDEN_NAMES = frozenset(
    {"simplified", "is_simplified", "template_name", "has_bodies", "body_count"}
)

#: The helper it delegates to makes exactly one, and it compares the *model's*
#: mass spec against `None` -- never anything about the template.  The test is
#: kept as source text so the assertion reads as the statement it makes.
ALLOWED_TESTS = frozenset({"spec is None"})


def _stub(role: str, name: str, part: str) -> Template:
    """Return a one-body template that satisfies `role`'s contract."""
    spec = brake.SIMPLIFIED_BRAKE.role_spec if role == "brake" else drive.SIMPLIFIED_DRIVE.role_spec
    return Template(
        name=name,
        role=role,
        parts=(PartDefinition(part),),
        connections=tuple(
            ConnectionDefinition(f"{mount}_{side}", mount)
            for mount in spec.required_mounts
            for side in ("L", "R")
        ),
        property_slots=tuple(
            PropertySlot(slot, "-", default=0.0) for slot in spec.required_slots
        ),
    )


def _context() -> SubsystemContext:
    return SubsystemContext(
        model=benchmark_model(), request=AssemblyRequest(mode="K")
    )


def _without_docstrings(function_node: ast.FunctionDef) -> ast.FunctionDef:
    body = [
        node
        for node in function_node.body
        if not (
            isinstance(node, ast.Expr)
            and isinstance(node.value, ast.Constant)
            and isinstance(node.value.value, str)
        )
    ]
    return ast.FunctionDef(
        name=function_node.name,
        args=function_node.args,
        body=body or [ast.Pass()],
        decorator_list=function_node.decorator_list,
        returns=function_node.returns,
    )


def _vocabulary_from_source(source: str) -> tuple[set[str], set[str], bool]:
    """
    Return the names a function body reads, the tests it makes, and whether it counts.

    Every form a branch can take is collected -- an `if`, a conditional
    expression, and a comprehension's `if` clause -- because a filter clause is
    exactly where "only keep the bodies that ..." would hide.  Docstrings are
    removed first: the explanation is allowed to say "simplified", the code is
    not allowed to branch on it.
    """
    module = ast.parse(source)
    node = _without_docstrings(module.body[0])  # type: ignore[arg-type]
    names: set[str] = set()
    tests: set[str] = set()
    counts = False
    for child in ast.walk(node):
        if isinstance(child, ast.Name):
            names.add(child.id)
        elif isinstance(child, ast.Attribute):
            names.add(child.attr)
        elif isinstance(child, (ast.If, ast.IfExp)):
            tests.add(ast.unparse(child.test))
        elif isinstance(child, ast.comprehension) and child.ifs:
            tests.update(ast.unparse(condition) for condition in child.ifs)
        elif isinstance(child, ast.Call) and isinstance(child.func, ast.Name):
            if child.func.id == "len":
                counts = True
    return names, tests, counts


def _decision_vocabulary(function) -> tuple[set[str], set[str], bool]:
    """Return the vocabulary of a live function, read from its own source."""
    return _vocabulary_from_source(inspect.getsource(function).lstrip())


def test_the_brake_role_is_a_replaceable_provider() -> None:
    """One `build` object, two templates, two different body sets."""
    context = _context()
    build_function = brake.build
    assemble_function = assemble_from_template

    # The simplified template: no bodies.
    simplified = instantiate(brake.SIMPLIFIED_BRAKE, mode="K", properties={})
    assert build_function(simplified, context).bodies == {}

    # A same-role template that declares a body: the body appears, with no
    # change to `build` and none to the assembly path.
    register(_stub("brake", STUB_BRAKE_NAME, STUB_PART), replace=True)
    detailed = instantiate(
        register(_stub("brake", STUB_BRAKE_NAME, STUB_PART), replace=True),
        mode="K",
        properties={},
    )
    output = build_function(detailed, context)
    assert list(output.bodies) == [STUB_PART]

    # Switched back: the same call site produces no body again.  If the path had
    # remembered "the brake is the simplified one", this is where it would show.
    assert build_function(simplified, context).bodies == {}

    # Same function objects on every call, i.e. nothing rebound itself.
    assert brake.build is build_function
    assert assemble_from_template is assemble_function


def test_the_branch_detector_would_catch_a_branch() -> None:
    """
    A gate that cannot fail is not a gate, so the checker is shown failing.

    Each hostile sample is a plausible wrong implementation of the same path: a
    part count, a template-name suffix, a role comparison, and a comprehension
    filter.  Each must register a test or a count; the live implementation must
    register none.
    """
    hostile = {
        "count": (
            "def f(instance, context):\n"
            "    if len(instance.bodies) == 0:\n"
            "        return None\n"
            "    return instance.bodies\n"
        ),
        "template name": (
            "def f(instance, context):\n"
            "    if instance.template.name.endswith('simplified'):\n"
            "        return None\n"
            "    return instance.bodies\n"
        ),
        "role": (
            "def f(instance, context):\n"
            "    if instance.template.role == 'brake':\n"
            "        return None\n"
            "    return instance.bodies\n"
        ),
        "filter": (
            "def f(instance, context):\n"
            "    return [n for n in instance.bodies if n.startswith('caliper')]\n"
        ),
    }
    for label, source in hostile.items():
        names, tests, counts = _vocabulary_from_source(source)
        assert tests or counts, label
        del names


def test_the_drive_role_is_a_replaceable_provider() -> None:
    """The same substitution, under the `drive` role."""
    context = _context()
    build_function = drive.build

    simplified = instantiate(drive.SIMPLIFIED_DRIVE, mode="K", properties={})
    assert build_function(simplified, context).bodies == {}

    detailed = instantiate(
        register(_stub("drive", STUB_DRIVE_NAME, "differential_housing"), replace=True),
        mode="K",
        properties={},
    )
    output = build_function(detailed, context)
    assert list(output.bodies) == ["differential_housing"]

    assert build_function(simplified, context).bodies == {}
    assert drive.build is build_function


def test_the_assembly_path_has_no_simplified_decision() -> None:
    """
    The entry point cannot tell one role implementation from another.

    Read as: it never counts anything, it never reads a "which template" name,
    and it makes no test at all -- the per-body choice is delegated to a helper
    whose single conditional compares the *model's* mass spec against `None`.
    """
    names, tests, counts = _decision_vocabulary(assemble_from_template)
    assert not counts, "the assembly path must not count parts"
    assert names & FORBIDDEN_NAMES == set(), sorted(names & FORBIDDEN_NAMES)
    assert tests == set(), sorted(tests)

    # The helper makes exactly the one comparison, and it is about the model.
    helper_names, helper_tests, helper_counts = _decision_vocabulary(
        assemble_from_template.__globals__["_body_for"]
    )
    assert not helper_counts
    assert helper_names & FORBIDDEN_NAMES == set()
    assert helper_tests == ALLOWED_TESTS, sorted(helper_tests)


def test_the_role_interface_does_not_carry_a_simplified_flag() -> None:
    """
    The role declaration itself must not leak implementation state (G9).

    `RoleSpec` is the interface both templates are assembled through, so a
    `has_bodies`/`is_simple` field there would re-introduce the branch one level
    up.  This is the same check `templates.roles` performs on itself, asserted
    from the consumer side.
    """
    for role_name in ("brake", "drive"):
        spec = brake.SIMPLIFIED_BRAKE.role_spec if role_name == "brake" else drive.SIMPLIFIED_DRIVE.role_spec
        fields = set(vars(spec))
        assert not fields & {"body_count", "has_bodies", "is_simple", "bodies"}
        assert spec.has_torque_channel
