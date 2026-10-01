"""
p3-03 evidence run.  Every number quoted in ``raw/`` comes out of this script.

Sections
--------
0. the pre-change implementation, imported from ``git show HEAD:...``
1. before / after on the same double-wishbone fixture
2. the force-to-generalized-displacement derivative matrix, element by element
3. the independent numerical criterion (a): finite displacement, no ``twist_of``
4. the independent numerical criterion (b): lateral-force increment and the moment
5. drive independence
6. the pre-change construction on all four topologies
7. the three non-wishbone topologies, after
8. the textual checks (name sniffing, removed paths)
"""

from __future__ import annotations

import pathlib
import subprocess
import sys

import numpy as np

HERE = pathlib.Path(__file__).parent
REPO = pathlib.Path(".")
sys.path.insert(0, str(REPO / "packages/suspension_multibody/src"))
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REPO / "packages/suspension_multibody"))

import roll_centers_before as before  # noqa: E402
from rc_cmp import vehicle  # noqa: E402

from suspension_multibody.subsystems.entry import compose_vehicle  # noqa: E402
from suspension_multibody.vehicle import roll_centers as rc  # noqa: E402
from suspension_multibody.vehicle import screw_kinematics as sk  # noqa: E402
from tests.physics.test_roll_centres_by_topology import (  # noqa: E402
    _TOPOLOGIES,
    _vehicle,
)

np.set_printoptions(precision=12, suppress=False)


def section(title: str) -> None:
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)


v = vehicle()
runtime = compose_vehicle(v, "K")


# --------------------------------------------------------------------------- #
section("1. before / after, double wishbone")

b = before.compute_vehicle_roll_centers(v)
a = rc.compute_vehicle_roll_centers(v)

print(f"{'axle':6} {'quantity':28} {'before':>18} {'after':>20} {'diff':>14}")
for axle in ("front", "rear"):
    rows = [
        ("center[0] (y, mm)", float(b[axle].center[0]), float(a[axle].center[0])),
        ("center[1] (z, mm)", float(b[axle].center[1]), float(a[axle].center[1])),
        ("left IC y (before path)", float(b[axle].left_instant_center[0]), np.nan),
        ("left IC z (before path)", float(b[axle].left_instant_center[1]), np.nan),
        ("left patch y", -750.0, float(a[axle].left_contact_patch[1])),
        ("right patch y", 750.0, float(a[axle].right_contact_patch[1])),
        ("left patch slope r", np.nan, float(a[axle].left_contact_patch_slope)),
        ("right patch slope r", np.nan, float(a[axle].right_contact_patch_slope)),
    ]
    for name, bv, av in rows:
        diff = av - bv if np.isfinite(bv) and np.isfinite(av) else np.nan
        print(f"{axle:6} {name:28} {bv:18.12f} {av:20.12f} {diff:14.6e}")

print()
print("The two constructions describe the same thing -- the line from the patch to")
print("where the suspension's lateral force comes from.  Expressed the one way they")
print("can be compared, as that line's d z / d y through the patch:")
for axle in ("front", "rear"):
    pairs = (
        ("left", b[axle].left_instant_center, -750.0, a[axle].left_contact_patch_slope),
        ("right", b[axle].right_instant_center, 750.0, a[axle].right_contact_patch_slope),
    )
    for side, ic, patch_y, patch_slope in pairs:
        # before: the line from the patch to the arm-line intersection
        before_dzdy = (ic[1] - 0.0) / (ic[0] - patch_y)
        # after: the force line is perpendicular to the patch's own path, whose
        # d y / d z is the reported slope; so d z / d y == -r
        after_dzdy = -patch_slope
        z_before = 0.0 + (0.0 - patch_y) * before_dzdy
        z_after = 0.0 + (0.0 - patch_y) * after_dzdy
        print(f"  {axle:6} {side:5} IC=({ic[0]:.9f}, {ic[1]:.9f})  "
              f"d z/d y: before={before_dzdy:.12f}  after={after_dzdy:.12f}  "
              f"diff={before_dzdy - after_dzdy:.3e}")
        print(f"          line crosses y = 0 at: before {z_before:.12f}   "
              f"after {z_after:.12f}   diff {z_before - z_after:.3e}")


# --------------------------------------------------------------------------- #
section("2. force-to-generalized-displacement derivative matrix B (double wishbone)")

for placement, axle_runtime in runtime.axle_assemblies.items():
    print(f"\n--- axle {placement!r} ---")
    for body, label in sorted(axle_runtime.points):
        if label != "wheel_center":
            continue
        local = np.asarray(axle_runtime.points[(body, label)], dtype=float)
        centre_world = axle_runtime.state.point_world(body, local)
        patch = np.array([centre_world[0], centre_world[1], 0.0])
        motion = sk.solve_rigid_motion(
            axle_runtime.constraints,
            axle_runtime.state,
            [sk.PointDrive(body, label, np.array([0.0, 0.0, 1.0]))],
            points=axle_runtime.points,
            pivot_body=body,
            pivot_point=(body, label),
        )
        velocity = motion.twist_of(body).transform_point(patch)
        ratio = float(velocity[1]) / float(velocity[2])
        print(f"  body {body!r}: patch world = "
              f"{np.array2string(patch, precision=6)}  v = "
              f"{np.array2string(velocity, precision=12)}")
        print(f"      r = v_y / v_z            = {ratio!r}")
        print("      B[u_y] = -1               (mm per mm of lateral sprung-mass travel)")
        print(f"      B[phi] = y_patch * r       = {patch[1] * ratio!r}  (mm per rad)")

for placement, res in a.items():
    print(f"\n--- axle {placement!r}: assembled generalized loads ---")
    print(f"  patches        = {res.left_contact_patch[1]!r}, {res.right_contact_patch[1]!r}")
    print(f"  slopes         = {res.left_contact_patch_slope!r}, {res.right_contact_patch_slope!r}")
    print(f"  Q_uy = sum(-1)         = {res.lateral_force!r}  N per mm of lateral travel")
    print(f"  Q_phi = sum(y_patch*r) = {res.roll_moment!r}  N.mm per rad")
    print(f"  h = -Q_phi / Q_uy      = {res.center[1]!r}  mm")
    print(f"  h rebuilt here         = {-res.roll_moment / res.lateral_force!r}  mm")


# --------------------------------------------------------------------------- #
section("3. criterion (a): finite displacement, no twist_of")


def patch_travel(axle_runtime, body: str, delta: float) -> tuple[float, float]:
    """Move the mechanism a finite step and read the contact patch's own travel.

    ``state.retract`` + ``state.point_world`` only: no ``twist_of``, no Jacobian read.
    The step is the solved motion scaled by ``delta`` in each body's own local tangent
    coordinates, so the wheel end rises by ``delta``.  The patch is carried as a
    *material* point of the wheel end -- stored in that body's local frame first, then
    re-read in world after the step.
    """
    local = np.asarray(axle_runtime.points[(body, "wheel_center")], dtype=float)
    centre_world = axle_runtime.state.point_world(body, local)
    patch_world = np.array([centre_world[0], centre_world[1], 0.0])
    patch_local = axle_runtime.state.pose(body).inverse().transform_point(patch_world)
    motion = sk.solve_rigid_motion(
        axle_runtime.constraints,
        axle_runtime.state,
        [sk.PointDrive(body, "wheel_center", np.array([0.0, 0.0, 1.0]))],
        points=axle_runtime.points,
        pivot_body=body,
        pivot_point=(body, "wheel_center"),
    )
    increments = {
        name: motion.velocity[6 * index : 6 * index + 6] * delta
        for index, name in enumerate(motion.bodies)
    }
    moved = axle_runtime.state.retract(increments)
    now = moved.point_world(body, patch_local)
    return float(now[1] - patch_world[1]), float(now[2] - patch_world[2])


print("the step carries every body along its own part of the solved motion, so the")
print("patch's finite travel is its tangent direction to first order in delta; the")
print("ratio below is read from those two displacements alone.")
print()
for placement, axle_runtime in runtime.axle_assemblies.items():
    for body, label in sorted(axle_runtime.points):
        if label != "wheel_center":
            continue
        first = None
        for delta in (1e-4, 1e-5, 1e-6):
            dy, dz = patch_travel(axle_runtime, body, delta)
            ratio = dy / dz
            if first is None:
                first = ratio
                print(f"{placement:6} {body:14} delta={delta:.0e}  dy={dy:.12e} "
                      f"dz={dz:.12e}  ratio={ratio!r}")
            else:
                print(f"{placement:6} {body:14} delta={delta:.0e}  dy={dy:.12e} "
                      f"dz={dz:.12e}  ratio={ratio!r}  rel.err="
                      f"{abs(ratio - first) / abs(first):.3e}")
        local = np.asarray(axle_runtime.points[(body, "wheel_center")], dtype=float)
        patch_y = float(axle_runtime.state.point_world(body, local)[1])
        module_h = float(a[placement].center[1])
        print(f"       h from this independent ratio alone = {patch_y * first!r}")
        print(f"       h the module reports                 = {module_h!r}")
        print(f"       difference                           = "
              f"{patch_y * first - module_h:.3e} mm")


# --------------------------------------------------------------------------- #
section("4. criterion (b): lateral-force increment and the roll moment")

# The force each wheel end passes to the sprung mass runs along the tangent to the
# patch's own path in the front view, i.e. perpendicular to (v_y, v_z): the direction
# (v_z, -v_y).  Normalised so the lateral component of the force is the increment dF,
# that is the direction (1, -r) with r = v_y / v_z.
#
# The moment of those forces about a longitudinal axis at height h:
#     M(h) = sum_i [ y_i * F_z,i - (z_i - h) * F_y,i ] ,  z_i = 0, F_y = dF, F_z = -r*dF
#          = -sum_i (y_i * r_i * dF_i) + h * sum_i dF_i
#          = M(0) - h * dF_total
# so M is linear in h with slope -dF_total, and h is pinned by M(h) == 0.
# An asymmetric pair is used so nothing cancels by symmetry: with dF_L == dF_R the
# first term cancels to roundoff and the criterion would carry no information.

dF = {"L": 1.0, "R": 0.4}


def force_directions(axle_runtime) -> dict[str, tuple[float, float, float]]:
    """Per side: (patch y, r = v_y / v_z, from the finite-displacement direction)."""
    out = {}
    for body, label in sorted(axle_runtime.points):
        if label != "wheel_center":
            continue
        local = np.asarray(axle_runtime.points[(body, label)], dtype=float)
        centre_world = axle_runtime.state.point_world(body, local)
        dy, dz = patch_travel(axle_runtime, body, 1e-7)
        side = "L" if centre_world[1] < 0.0 else "R"
        out[side] = (float(centre_world[1]), dy / dz, 0.0)
    return out


for placement, axle_runtime in runtime.axle_assemblies.items():
    info = force_directions(axle_runtime)
    m0 = -sum(y * r * dF[side] for side, (y, r, _) in info.items())
    d_total = sum(dF[side] for side in info)

    def moment(h: float) -> float:
        return m0 + h * d_total

    print(f"\n--- axle {placement!r} ---")
    for side, (y, r, _) in sorted(info.items()):
        print(f"  side {side}: patch y = {y!r}  r = {r!r}  "
              f"F = ({dF[side]!r}, {-r * dF[side]!r})")
    print(f"  dF_total = sum dF_i        = {d_total!r}  N")
    print(f"  M(0)     = -sum y*r*dF_i   = {m0!r}  N.mm")
    print(f"  M(h) = M(0) + h * dF_total, so the moment is linear in h with slope "
          f"{d_total!r}")

    # the zero crossing, found by bisection with no reference to the module's h
    lo, hi = -1000.0, 1000.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        if moment(lo) * moment(mid) <= 0.0:
            hi = mid
        else:
            lo = mid
    h_zero = 0.5 * (lo + hi)
    print(f"  h from the zero crossing   = {h_zero!r}  mm")
    print(f"  module h                   = {a[placement].center[1]!r}  mm")
    print(f"  difference                 = "
          f"{h_zero - float(a[placement].center[1]):.3e} mm")

    # the product relation itself, at the module's h and at two other heights
    module_h = float(a[placement].center[1])
    for h in (module_h, module_h + 10.0, module_h - 25.0):
        print(f"    M({h!r:>22}) = {moment(h):.12e}")
    print(f"    M(module h) = {moment(module_h):.6e} N.mm "
          f"(the linear relation's zero, to the accuracy of the independently read r)")


# --------------------------------------------------------------------------- #
section("5. drive independence")


def slopes_under(kind: str) -> list[float]:
    slopes: list[float] = []
    for _placement, axle_runtime in runtime.axle_assemblies.items():
        left = sorted(
            body
            for (body, label) in axle_runtime.points
            if label == "wheel_center" and axle_runtime.state.point_world(
                body, np.asarray(axle_runtime.points[(body, label)], dtype=float)
            )[1]
            < 0.0
        )
        other = sorted(
            body
            for (body, label) in axle_runtime.points
            if label == "wheel_center"
        )
        body = left[0]
        local = np.asarray(axle_runtime.points[(body, "wheel_center")], dtype=float)
        centre_world = axle_runtime.state.point_world(body, local)
        patch = np.array([centre_world[0], centre_world[1], 0.0])
        if kind == "this wheel end rises":
            drives = [sk.PointDrive(body, "wheel_center", np.array([0.0, 0.0, 1.0]))]
        elif kind == "the wheel end's own local x":
            drives = [sk.TangentDrive(body, 0, 1.0)]
        elif kind == "the wheel end rolls":
            drives = [sk.TangentDrive(body, 3, 1.0)]
        elif kind == "the other side rises":
            partner = [name for name in other if name != body][0]
            drives = [sk.PointDrive(partner, "wheel_center", np.array([0.0, 0.0, 1.0]))]
        else:
            raise ValueError(kind)
        motion = sk.solve_rigid_motion(
            axle_runtime.constraints,
            axle_runtime.state,
            drives,
            points=axle_runtime.points,
            pivot_body=body,
            pivot_point=(body, "wheel_center"),
        )
        velocity = motion.twist_of(body).transform_point(patch)
        if abs(float(velocity[2])) > 1e-12:
            slopes.append(float(velocity[1]) / float(velocity[2]))
    return slopes


reference = slopes_under("this wheel end rises")
print(f"{'drive':32} {'slopes (left wheel ends)':46} max |diff|")
print(f"{'this wheel end rises':32} {str(np.round(reference, 12).tolist()):46} -")
for kind in ("the wheel end's own local x", "the wheel end rolls", "the other side rises"):
    got = slopes_under(kind)
    if len(got) == len(reference):
        delta = max(abs(np.array(got) - np.array(reference)))
        shown = f"{delta:.3e}"
    else:
        shown = "different length"
    print(f"{kind:32} {str(np.round(got, 12).tolist()):46} {shown}")


# --------------------------------------------------------------------------- #
section("6. the pre-change construction on all four topologies")

print("the pre-change construction looks up hard-point roles by name, so it can only")
print("answer where those names exist.  Each topology below is run through it:")
topologies = {
    "double wishbone": lambda: v,
    "five-link": lambda: _vehicle(_TOPOLOGIES["five-link"]()),
    "macpherson": lambda: _vehicle(_TOPOLOGIES["macpherson"]()),
    "twist-beam": lambda: _vehicle(_TOPOLOGIES["twist-beam"]()),
}
for name, build in topologies.items():
    try:
        got = before.compute_vehicle_roll_centers(build())
        print(f"  {name:16} before: answered "
              f"{ {k: round(float(r.center[1]), 6) for k, r in got.items()} }")
    except Exception as exc:  # noqa: BLE001 - the refusal is the evidence
        print(f"  {name:16} before: refused -> {type(exc).__name__}: {exc}")
    after_got = rc.compute_vehicle_roll_centers(build())
    print(f"  {name:16} after : answered "
          f"{ {k: round(float(r.center[1]), 6) for k, r in after_got.items()} }")


# --------------------------------------------------------------------------- #
section("7. the three non-wishbone topologies, after")

for topology in sorted(_TOPOLOGIES):
    centers = rc.compute_vehicle_roll_centers(_vehicle(_TOPOLOGIES[topology]()))
    print(f"\n--- {topology} ---")
    for placement, res in centers.items():
        print(f"  {placement}: center = "
              f"{np.array2string(np.asarray(res.center), precision=12)}")
        print(f"           left  slope = {res.left_contact_patch_slope!r}   "
              f"patch = {np.array2string(np.asarray(res.left_contact_patch), precision=6)}")
        print(f"           right slope = {res.right_contact_patch_slope!r}   "
              f"patch = {np.array2string(np.asarray(res.right_contact_patch), precision=6)}")
        print(f"           Q_uy = {res.lateral_force!r}   Q_phi = {res.roll_moment!r}")


# --------------------------------------------------------------------------- #
section("8. textual checks")

repo = pathlib.Path(".")
vehicle_dir = repo / "packages/suspension_multibody/src/suspension_multibody/vehicle"
module = vehicle_dir / "roll_centers.py"

grep_alias = subprocess.run(
    ["grep", "-rn", "-e", "UPPER_INBOARD", "-e", "LOWER_INBOARD", "-e", "UCA_", str(vehicle_dir)],
    capture_output=True,
    text=True,
)
print(f"grep -rn -e UPPER_INBOARD -e LOWER_INBOARD -e UCA_ {vehicle_dir}")
print(f"  exit code = {grep_alias.returncode}   (1 == no match)")
print(f"  stdout    = {grep_alias.stdout!r}")
print(f"  stderr    = {grep_alias.stderr!r}")

grep_paths = subprocess.run(
    ["grep", "-n", "-e", "_line_intersection", "-e", "_instant_center", "-e", "_POINT_ALIASES",
     "-e", "_hardpoint", "-e", "side_hardpoints", str(module)],
    capture_output=True,
    text=True,
)
print(f"\ngrep -n -e _line_intersection -e _instant_center -e _POINT_ALIASES -e _hardpoint "
      f"-e side_hardpoints {module}")
print(f"  exit code = {grep_paths.returncode}   (1 == no match)")
print(f"  stdout    = {grep_paths.stdout!r}")

source = module.read_text(encoding="utf-8")
print(f"\nroll_centers.py: {len(source.splitlines())} lines")
for token in ("side_hardpoints", "UPPER_", "LOWER_", "UCA_", "_line_intersection"):
    print(f"  {token!r} in source: {token in source}")
