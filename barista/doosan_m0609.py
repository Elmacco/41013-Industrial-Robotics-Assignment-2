"""
Doosan M0609 cobot: a standard DH model, drawn with Doosan's link meshes.

The kinematics are the DH table below (6 kg payload, 900 mm reach). It was
derived from Doosan's joint geometry with the common-normal procedure and
checked against Doosan's own URDF: see Doosan-M0609/derive_dh.py and
Doosan-M0609/README.md.

The meshes (Doosan's, BSD-3) only make it look right. They are placed the way
the subject's DHRobot3D places the UR3's: QTEST_TRANSFORMS says where each
link's mesh sits with the arm at QTEST, and each mesh is then fixed to its
link's DH frame by a constant relation matrix.

At q = 0 the arm stands straight up, like the real robot's home.
"""
from math import pi, radians
from pathlib import Path

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as geometry
import spatialmath.base as spb
from spatialmath import SE3

ROOT = Path(__file__).resolve().parents[1] / "Doosan-M0609"
MESHES = ROOT / "meshes"

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

# A joint config and the 3D object transforms to match that config (as in the
# subject's UR3.py). Doosan draws each mesh in its own link frame, so these are
# Doosan's link frames with the arm standing straight up: the joint offsets
# along the arm (0.1345, 0.411, 0.368, 0.121 and the 6.2 mm elbow step) and
# the way each frame is turned. Within 0.21 mm of Doosan's URDF, which rounds
# pi/2 to 1.571.
QTEST = [0.0] * 6
QTEST_TRANSFORMS = [
    spb.transl(0, 0, 0),                                                    # base
    spb.transl(0, 0, 0.1345),                                               # shoulder
    spb.transl(0, 0.0062, 0.1345) @ spb.rpy2tr(-pi / 2, 0, -pi / 2, order="xyz"),   # upper arm
    spb.transl(0, 0.0062, 0.5455) @ spb.trotx(-pi / 2),                     # elbow
    spb.transl(0, 0.0062, 0.9135),                                          # forearm
    spb.transl(0, 0.0062, 0.9135) @ spb.trotx(-pi / 2),                     # wrist
    spb.transl(0, 0.0062, 1.0345),                                          # flange
]


class DoosanM0609(rtb.DHRobot):
    def __init__(self):
        # DH links
        links = self._create_DH()
        super().__init__(links, name="M0609", manufacturer="Doosan Robotics")

        # Relation between each link's DH frame and its mesh, as in DHRobot3D:
        # relation = (DH frame at QTEST)^-1 * (mesh pose at QTEST). It is
        # constant because both move with the link. Several meshes can share a
        # link (the upper arm has three), which DHRobot3D does not allow.
        dh_test = [T.A for T in self.fkine_all(QTEST)]       # base, then frames 1..6
        self._meshes = []
        for link, names in LINK_MESHES.items():
            relation = SE3(np.linalg.inv(dh_test[link]) @ QTEST_TRANSFORMS[link], check=False)
            for name in names:
                mesh = geometry.Mesh(filename=str(MESHES / f"{name}.dae"), scale=[MESH_SCALE] * 3)
                self._meshes.append((link, relation, mesh))
        self._ready = True
        self.q = HOME_Q

    def _create_DH(self):
        """
        Create robot's standard DH model

        Lengths from Doosan's joint geometry (Doosan-M0609/derive_dh.py):
        d1 0.1345 base to shoulder, a2 0.411 upper arm, d3 0.0062 elbow side
        step, d4 0.368 forearm (along z, so alpha3 turns z towards it), d6 0.121
        wrist centre to flange. a4 = a5 = d5 = 0: a spherical wrist, singular
        when q5 = 0.

        offset is the joint angle at which each DH frame lines up with Doosan's
        own zero, so q is the real robot's joint angle (q = 0: arm straight up).
        Limits from Doosan's specification: +-360 deg, except J3 +-150 deg.
        """
        a = [0, 0.411, 0, 0, 0, 0]
        d = [0.1345, 0, 0.0062, 0.368, 0, 0.121]
        alpha = [pi/2, 0, pi/2, pi/2, pi/2, 0]
        offset = [pi, pi/2, pi/2, -pi, pi, 0]
        qlim = [[-2*pi, 2*pi] for _ in range(6)]
        qlim[2] = [radians(-150), radians(150)]
        links = []
        for i in range(6):
            link = rtb.RevoluteDH(d=d[i], a=a[i], alpha=alpha[i], offset=offset[i], qlim=qlim[i])
            links.append(link)
        return links

    def _update_shapes(self):
        frames = self.fkine_all(self.q)        # includes the base transform
        for link, relation, mesh in self._meshes:
            mesh.T = (frames[link] * relation).A

    def add_to_env(self, env):
        self._update_shapes()
        for _, _, mesh in self._meshes:
            env.add(mesh)

    def __setattr__(self, name, value):
        """Keep the meshes in sync whenever q or base is assigned."""
        super().__setattr__(name, value)
        if name in ("q", "base") and getattr(self, "_ready", False):
            self._update_shapes()


if __name__ == "__main__":
    import sys

    sys.stdout.reconfigure(encoding="utf-8")      # the DH table uses box-drawing characters
    print(DoosanM0609())
