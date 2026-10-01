import sys
import numpy as np
sys.path.insert(0, "packages/suspension_multibody/src")
sys.path.insert(0, "packages/suspension_multibody")
from suspension_multibody.subsystems.entry import compose_vehicle
from suspension_multibody.vehicle import screw_kinematics as sk
from tests.physics.test_roll_centres_by_topology import _TOPOLOGIES, _vehicle
for name in sorted(_TOPOLOGIES):
    v = _vehicle(_TOPOLOGIES[name]())
    rt = compose_vehicle(v, "K")
    for placement, ax in rt.axle_assemblies.items():
        res = sk.constraint_residual(ax.constraints, ax.state)
        J, bodies = sk.constraint_jacobian(ax.constraints, ax.state)
        print(f"{name:12} {placement:6} max|C|={np.max(np.abs(res)):.3e}  rows={len(res)}  cols={J.shape[1]}  bodies={len(bodies)}")
