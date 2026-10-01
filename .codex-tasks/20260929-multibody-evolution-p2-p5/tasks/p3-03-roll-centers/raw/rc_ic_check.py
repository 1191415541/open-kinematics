from __future__ import annotations
import sys
import numpy as np
sys.path.insert(0, "packages/suspension_multibody/src")
sys.path.insert(0, r"C:/Users/zzy11/.pi-desktop/scratch/ba123231-caeb-4061-b2dc-c970cf431e16/p303")
from rc_cmp import vehicle
from suspension_multibody.subsystems.entry import compose_vehicle
from suspension_multibody.vehicle import screw_kinematics as sk

v = vehicle()
rt = compose_vehicle(v, "K")
for placement, ax in rt.axle_assemblies.items():
    print(f"--- {placement} ---")
    for body, label in sorted(ax.points):
        if label != "wheel_center":
            continue
        local = np.asarray(ax.points[(body, label)], dtype=float)
        cw = ax.state.point_world(body, local)
        patch = np.array([cw[0], cw[1], 0.0])
        m = sk.solve_rigid_motion(ax.constraints, ax.state,
            [sk.PointDrive(body, label, np.array([0.0, 0.0, 1.0]))],
            points=ax.points, pivot_body=body, pivot_point=(body, label))
        tw = m.twist_of(body)
        sc = m.screw
        vel = tw.transform_point(patch)
        r = float(vel[1]) / float(vel[2])
        w = np.asarray(tw.omega)
        vv = np.asarray(tw.velocity)
        print(f"  {body}: screw point={np.array2string(sc.point, precision=9)} dir={np.array2string(sc.direction, precision=9)} pitch={sc.pitch:.3e}")
        # front-view IC: the point where omega x (x - p) + v = 0 in the y,z plane
        # omega is along x here: solve for the (y,z) point with zero (y,z) velocity
        om = w
        print(f"      omega = {np.array2string(om, precision=12)}  velocity = {np.array2string(vv, precision=9)}")
        if abs(om[0]) > 1e-12:
            # v(q) = v + omega x q = 0  -> q = -(omega x v)/|omega|^2 ... only valid if omega.v = 0
            q = -np.cross(om, vv) / (om @ om)
            print(f"      zero-velocity point = {np.array2string(q, precision=9)}")
        print(f"      patch velocity = {np.array2string(vel, precision=12)}  r = v_y/v_z = {r!r}")
        # crossing of the line through patch and the zero-velocity point, at y=0
        if abs(om[0]) > 1e-12:
            q = -np.cross(om, vv) / (om @ om)
            if abs(q[1] - patch[1]) > 1e-9:
                dzdy = (q[2] - patch[2]) / (q[1] - patch[1])
                z0 = patch[2] + (0.0 - patch[1]) * dzdy
                print(f"      dz/dy (patch -> zero-velocity point) = {dzdy!r}")
                print(f"      crossing at y=0 = {z0!r}     y_patch * r = {patch[1] * r!r}     -r = {-r!r}")
