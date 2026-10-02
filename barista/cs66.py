"""
Elite Robots CS66 cobot as a 6-DOF DH robot.

ir_support has no CS66 mesh, so the arm is drawn with simple primitives: one
cylinder between each pair of joint frames plus a sphere at each joint. DH values
come from the CS66 dimension drawing (UR-style kinematics), reach ~0.91 m.
"""
from math import pi

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as geometry

# Standard DH, metres. d4 = 0.141 - 0.128 + 0.1345 (net sideways offset).
D = [0.1625, 0, 0, 0.1475, 0.0965, 0.095]
A = [0, -0.427, -0.3905, 0, 0, 0]
ALPHA = [pi / 2, 0, 0, pi / 2, -pi / 2, 0]
HOME_Q = [0, -pi / 2, 0, 0, 0, 0]  # Upright, as in the drawing

BASE_RADIUS = 0.075  # 150 mm base flange
LINK_RADIUS = 0.045
JOINT_RADIUS = 0.055
LINK_COLOR = (0.85, 0.86, 0.88, 1)
JOINT_COLOR = (0.20, 0.20, 0.22, 1)


def _segment_pose(p0, p1):
    """Pose of a cylinder whose axis (local z) runs from p0 to p1."""
    z = (p1 - p0) / np.linalg.norm(p1 - p0)
    x = np.cross([0, 0, 1], z) if abs(z[2]) < 0.99 else np.array([1.0, 0, 0])
    x /= np.linalg.norm(x)
    T = np.eye(4)
    T[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    T[:3, 3] = (p0 + p1) / 2
    return T


class CS66(rtb.DHRobot):
    def __init__(self):
        links = [rtb.RevoluteDH(d=d, a=a, alpha=alpha, qlim=[-2 * pi, 2 * pi])
                 for d, a, alpha in zip(D, A, ALPHA)]
        super().__init__(links, name="CS66", manufacturer="Elite Robots")

        # Distance between consecutive DH frame origins is fixed: sqrt(a^2 + d^2).
        self._segments = [
            geometry.Cylinder(radius=BASE_RADIUS if i == 0 else LINK_RADIUS,
                              length=float(np.hypot(a, d)), color=LINK_COLOR)
            for i, (a, d) in enumerate(zip(A, D))
        ]
        self._joints = [geometry.Sphere(radius=JOINT_RADIUS, color=JOINT_COLOR) for _ in range(self.n)]
        self._ready = True
        self.q = HOME_Q

    def _update_shapes(self):
        frames = [self.base.A]
        for link, qi in zip(self.links, self.q):
            frames.append(frames[-1] @ link.A(qi).A)
        for i, segment in enumerate(self._segments):
            segment.T = _segment_pose(frames[i][:3, 3], frames[i + 1][:3, 3])
        for joint, frame in zip(self._joints, frames[1:]):
            joint.T = frame

    def add_to_env(self, env):
        self._update_shapes()
        for shape in (*self._segments, *self._joints):
            env.add(shape)

    def __setattr__(self, name, value):
        """Keep the drawn shapes in sync whenever q or base is assigned."""
        super().__setattr__(name, value)
        if name in ("q", "base") and getattr(self, "_ready", False):
            self._update_shapes()
