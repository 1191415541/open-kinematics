import numpy as np
from scipy.spatial.transform import Rotation

from suspension_mbd.analysis.metrics import compute_k_metrics
from suspension_mbd.core import CoordinateDrive, RigidBody, RigidBodyState, SE3
from suspension_mbd.core.constraints import ConstraintSystem
from suspension_mbd.elements import BushingElement
from suspension_mbd.model import build_front_axle
from suspension_mbd.schema import FrontAxleModel, MassSpec
from suspension_mbd.solver.equilibrium import EquilibriumSolver, evaluate_generalized_forces

hp = {
    "uca_front": [-367, -450, 555], "uca_rear": [-517, -490, 560],
    "uca_outer": [-307, -675, 555], "lca_front": [-67, -400, 180],
    "lca_rear": [-467, -450, 185], "lca_outer": [-267, -750, 130],
    "tierod_inner": [-467, -400, 330], "tierod_outer": [-417, -750, 330],
    "wheel_center": [-267, -760, 330], "rack_center": [-467, 0, 330],
}
stiffness = {
    "uca": np.diag([1000, 2600, 1100, 332315, 401070, 21772]),
    "lca": np.diag([4400, 4400, 800, 1489690, 1489690, 57296]),
}
orientation = {
    "uca_front": ([0.0333148302, 0, -0.999444907], [0.96573417, -0.257529112, 0.032191139]),
    "uca_rear": ([-0.0333148302, 0, 0.999444907], [-0.96573417, 0.257529112, -0.032191139]),
    "lca_front": ([0.0124990236, 0, -0.9999218842], [0.9922015565, -0.1240251946, 0.0124025195]),
    "lca_rear": ([0.0124990236, 0, -0.9999218842], [0.9922015565, -0.1240251946, 0.0124025195]),
}


def pose(point, xp, zp, right=False):
    x = np.asarray(xp, float)
    z = np.asarray(zp, float)
    y = np.cross(z, x)
    matrix = np.column_stack((x, y, z))
    translation = np.asarray(point, float)
    if right:
        mirror = np.diag([1.0, -1.0, 1.0])
        matrix = mirror @ matrix @ mirror
        translation = mirror @ translation
    qx, qy, qz, qw = Rotation.from_matrix(matrix).as_quat()
    return SE3(translation, np.array([qw, qx, qy, qz]))


assembly = build_front_axle(
    FrontAxleModel(name="c", hardpoints=hp, mass=MassSpec(sprung_mass=1200)), "C"
)
state = RigidBodyState({**assembly.state.bodies, "subframe": RigidBody("subframe")})
elements = list(assembly.elements)
for side, right in (("L", False), ("R", True)):
    for arm in ("uca", "lca"):
        for location in ("front", "rear"):
            name = f"{arm}_{location}"
            frame = pose(hp[name], *orientation[name], right=right)
            elements.append(
                BushingElement(
                    name=f"{name}_{side}", body_a="chassis" if arm == "uca" else "subframe", body_b=f"{'upper' if arm == 'uca' else 'lower'}_arm_{side}",
                    local_pose_a=frame, local_pose_b=frame, stiffness=stiffness[arm],
                )
            )

subframe_stiffness = np.diag([500, 500, 1000 / 3, 1500 * 180 / np.pi, 1500 * 180 / np.pi, 1000 * 180 / np.pi])
for side, sign in (("L", -1.0), ("R", 1.0)):
    for location, x in (("front", 133.0), ("rear", -667.0)):
        point = np.array([x, sign * 450.0, 180.0])
        frame = SE3(point, np.array([1.0, 0.0, 0.0, 0.0]))
        elements.append(
            BushingElement(
                name=f"subframe_{location}_{side}", body_a="chassis", body_b="subframe",
                local_pose_a=frame, local_pose_b=frame, stiffness=subframe_stiffness,
            )
        )

constraints = list(assembly.constraints)
constraints.extend(
    [
        CoordinateDrive("upright_L", assembly.point("upright_L", "wheel_center"), np.array([0.0, 0.0, 1.0]), 330.0),
        CoordinateDrive("upright_R", assembly.point("upright_R", "wheel_center"), np.array([0.0, 0.0, 1.0]), 330.0),
        CoordinateDrive("rack", assembly.point("rack", "center"), np.array([0.0, 1.0, 0.0]), 0.0),
    ]
)


def response(external):
    order = tuple(name for name, body in state.bodies.items() if not body.fixed)
    solver = EquilibriumSolver()
    force, _ = evaluate_generalized_forces(state, elements, external, order)
    tangent = solver._force_tangent(state, tuple(elements), external, order)
    jacobian = ConstraintSystem(tuple(constraints)).jacobian(state, order)
    kkt = np.block([[tangent, jacobian.T], [jacobian, np.zeros((len(jacobian), len(jacobian)))]])
    increment = np.linalg.lstsq(kkt, -np.r_[force, np.zeros(len(jacobian))], rcond=1e-12)[0][: len(order) * 6]
    solved_state = state.retract(
        {name: increment[index * 6 : (index + 1) * 6] for index, name in enumerate(order)}
    )
    base = compute_k_metrics(state, assembly)
    result = compute_k_metrics(solved_state, assembly)
    return {key: result[key] - base[key] for key in result}


print(
    "longitudinal",
    response(
        {
            "upright_L": np.array([500, 0, 0, 0, 0, 0], float),
            "upright_R": np.array([500, 0, 0, 0, 0, 0], float),
        }
    ),
)
