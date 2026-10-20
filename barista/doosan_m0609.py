"""
Doosan M0609 cobot: a standard DH model, drawn with Doosan's link meshes.

The kinematics are the DH table below (6 kg payload, 900 mm reach). It was
derived from Doosan's joint geometry with the common-normal procedure and
checked against Doosan's own URDF: see Doosan-M0609/derive_dh.py and
Doosan-M0609/README.md.

The meshes (Doosan's, BSD-3) only make it look right. Each one belongs to a
link, so it is fixed to that link's DH frame by a constant transform, worked
out once from Doosan's link frames at the zero pose.

At q = 0 the arm stands straight up, like the real robot's home.
"""
import importlib.util
from math import pi
from pathlib import Path

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as geometry
from spatialmath import SE3

ROOT = Path(__file__).resolve().parents[1] / "Doosan-M0609"
MESHES = ROOT / "meshes"

# ---------------------------------------------------------------- the model
# Standard DH, metres and radians. offset is the joint angle at which the DH
# frame lines up with Doosan's zero (so q here equals the robot's joint angle).
D = [0.1345, 0.0, 0.0062, 0.368, 0.0, 0.121]
A = [0.0, 0.411, 0.0, 0.0, 0.0, 0.0]
ALPHA = [pi / 2, 0.0, pi / 2, pi / 2, pi / 2, 0.0]
OFFSET = [pi, pi / 2, pi / 2, -pi, pi, 0.0]
# Joint limits from Doosan's specification: J3 (elbow) +-150 deg, the rest +-360.
QLIM = [(-2 * pi, 2 * pi)] * 2 + [(-5 * pi / 6, 5 * pi / 6)] + [(-2 * pi, 2 * pi)] * 3
HOME_Q = [0.0] * 6

# Doosan's mesh files, by the link (0 = base) each belongs to.
LINK_MESHES = {
    0: ["MF0609_0_0"],
    1: ["MF0609_1_0"],
    2: ["MF0609_2_0", "MF0609_2_1", "MF0609_2_2"],
    3: ["MF0609_3_0"],
    4: ["MF0609_4_0", "MF0609_4_1"],
    5: ["MF0609_5_0"],
    6: ["MF0609_6_0"],
}
MESH_SCALE = 0.001          # the meshes are in millimetres


def _doosan_link_frames():
    """Doosan's link frames at q = 0, from its URDF (used to place the meshes)."""
    spec = importlib.util.spec_from_file_location("derive_dh", ROOT / "derive_dh.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return [np.eye(4)] + mod.urdf_fk(np.zeros(6), mod.urdf_joints())


class DoosanM0609(rtb.DHRobot):
    def __init__(self):
        links = [rtb.RevoluteDH(d=d, a=a, alpha=alpha, offset=off, qlim=lim)
                 for d, a, alpha, off, lim in zip(D, A, ALPHA, OFFSET, QLIM)]
        super().__init__(links, name="M0609", manufacturer="Doosan Robotics")

        # Fix each mesh to its link's DH frame: offset = (DH frame)^-1 * (Doosan frame),
        # both at q = 0. It is constant because both frames move with the link.
        dh0 = [T.A for T in self.fkine_all(np.zeros(6))]       # base, then frames 1..6
        doosan0 = _doosan_link_frames()
        self._meshes = []
        for link, names in LINK_MESHES.items():
            offset = SE3(np.linalg.inv(dh0[link]) @ doosan0[link], check=False)
            for name in names:
                mesh = geometry.Mesh(filename=str(MESHES / f"{name}.dae"), scale=[MESH_SCALE] * 3)
                self._meshes.append((link, offset, mesh))
        self._ready = True
        self.q = HOME_Q

    def _update_shapes(self):
        frames = self.fkine_all(self.q)        # includes the base transform
        for link, offset, mesh in self._meshes:
            mesh.T = (frames[link] * offset).A

    def add_to_env(self, env):
        self._update_shapes()
        for _, _, mesh in self._meshes:
            env.add(mesh)

    def __setattr__(self, name, value):
        """Keep the meshes in sync whenever q or base is assigned."""
        super().__setattr__(name, value)
        if name in ("q", "base") and getattr(self, "_ready", False):
            self._update_shapes()
