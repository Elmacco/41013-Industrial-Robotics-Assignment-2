"""
Derive the Doosan M0609's standard DH parameters from Doosan's joint geometry,
and check the result against Doosan's own kinematics.

    python Doosan-M0609/derive_dh.py

Doosan's URDF (m0609.urdf, BSD-3, see LICENSE-doosan) gives each joint as a
fixed transform from the previous link followed by a rotation about its local
z axis. From it we take only the joint axes at the zero pose, then build DH
frames with the usual procedure:

    z(i-1) is the axis of joint i
    x(i) lies along the common normal from z(i-1) to z(i)
    (if the axes intersect, x(i) = z(i-1) x z(i); if parallel, the normal
     through the previous origin)

    d(i)     distance along z(i-1) to the common normal
    theta(i) angle from x(i-1) to x(i) about z(i-1)  (the joint offset at q = 0)
    a(i)     length of the common normal
    alpha(i) angle from z(i-1) to z(i) about x(i)

The last frame is put at the flange, along the joint 6 axis. The table is then
checked by comparing its forward kinematics with the URDF's at random poses.
"""
import xml.etree.ElementTree as ET
from math import atan2, cos, pi, sin
from pathlib import Path

import numpy as np

URDF = Path(__file__).with_name("m0609.urdf")
EPS = 1e-9


# ---------------------------------------------------------------- the URDF chain
def _rot(r, p, y):
    """URDF rpy: R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = cos(r), sin(r), cos(p), sin(p), cos(y), sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def _tf(R=np.eye(3), t=(0, 0, 0)):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


def rotz(q):
    return _tf(_rot(0, 0, q))


def urdf_joints():
    """[(origin transform, joint limits)] for the six revolute joints, base to tip."""
    joints = []
    for j in ET.parse(URDF).getroot().iter("joint"):
        if j.get("type") != "revolute":
            continue
        o = j.find("origin")
        xyz = [float(v) for v in o.get("xyz", "0 0 0").split()]
        rpy = [float(v) for v in o.get("rpy", "0 0 0").split()]
        axis = [float(v) for v in j.find("axis").get("xyz").split()]
        assert np.allclose(axis, [0, 0, 1]), "every M0609 joint turns about its local z"
        lim = j.find("limit")
        joints.append((_tf(_rot(*rpy), xyz), (float(lim.get("lower")), float(lim.get("upper")))))
    return joints


def urdf_fk(q, joints):
    """World frame of each link 1..6 (the flange is link 6)."""
    T, out = np.eye(4), []
    for (origin, _), qi in zip(joints, q):
        T = T @ origin @ rotz(qi)
        out.append(T)
    return out


# ---------------------------------------------------------------- DH derivation
def _angle(a, b, axis):
    """Signed angle from vector a to vector b about `axis`."""
    return atan2(np.dot(np.cross(a, b), axis), np.dot(a, b))


def derive(joints):
    """Standard DH (d, a, alpha, theta offset) per joint, plus the tool
    transform from DH frame 6 to the flange."""
    frames = urdf_fk(np.zeros(6), joints)
    # Axis of joint i at q = 0 is the z axis of the frame it rotates (before its
    # own rotation, which is zero here), through that frame's origin.
    axes = []
    T = np.eye(4)
    for origin, _ in joints:
        T = T @ origin
        axes.append((T[:3, 3].copy(), T[:3, 2].copy()))
    flange = frames[-1]
    lines = axes + [(flange[:3, 3], flange[:3, 2])]   # z0..z5 joints, z6 the tool axis

    o_prev, x_prev = np.zeros(3), np.array([1.0, 0, 0])  # base frame = world
    z_prev = lines[0][1]
    assert np.allclose(z_prev, [0, 0, 1]) and np.allclose(lines[0][0][:2], 0), "joint 1 is the world z axis"
    params = []
    for i in range(1, 7):
        p, z = lines[i]
        w = np.cross(z_prev, z)
        if np.linalg.norm(w) < EPS:
            # Parallel (or the same line): normal through the previous origin.
            foot = o_prev
            perp = (p - o_prev) - np.dot(p - o_prev, z_prev) * z_prev
            if np.linalg.norm(perp) < EPS:          # same line: keep x, slide along z
                x = x_prev if i < 6 else flange[:3, 0]
                o = o_prev + np.dot(p - o_prev, z_prev) * z_prev
            else:
                x = perp / np.linalg.norm(perp)
                o = foot + perp
        else:
            # Closest points between the two axes (lines o_prev + s z_prev, p + t z).
            A = np.array([[np.dot(z_prev, z_prev), -np.dot(z_prev, z)],
                          [np.dot(z_prev, z), -np.dot(z, z)]])
            b = np.array([np.dot(p - o_prev, z_prev), np.dot(p - o_prev, z)])
            s, t = np.linalg.solve(A, b)
            foot, o = o_prev + s * z_prev, p + t * z
            gap = o - foot
            x = gap / np.linalg.norm(gap) if np.linalg.norm(gap) > EPS else w / np.linalg.norm(w)
        d = np.dot(o - o_prev, z_prev) if np.linalg.norm(w) < EPS else np.dot(foot - o_prev, z_prev)
        theta = _angle(x_prev, x, z_prev)
        a = np.dot(o - (o_prev + d * z_prev), x)
        alpha = _angle(z_prev, z, x)
        params.append((d, a, alpha, theta))
        o_prev, x_prev, z_prev = o, x, z

    # What is left between DH frame 6 and the flange frame (a rotation about z
    # at most, if x6 was taken from the flange).
    T = np.eye(4)
    for d, a, alpha, theta in params:
        T = T @ dh(theta, d, a, alpha)
    tool = np.linalg.inv(T) @ flange
    return params, tool


def dh(theta, d, a, alpha):
    ct, st, ca, sa = cos(theta), sin(theta), cos(alpha), sin(alpha)
    return np.array([[ct, -st * ca, st * sa, a * ct],
                     [st, ct * ca, -ct * sa, a * st],
                     [0, sa, ca, d],
                     [0, 0, 0, 1]])


def dh_fk(q, params, tool):
    T = np.eye(4)
    for (d, a, alpha, theta), qi in zip(params, q):
        T = T @ dh(qi + theta, d, a, alpha)
    return T @ tool


def _nice(v):
    """Round to the nearest tenth of a millimetre / multiple of pi/2 for printing."""
    for k in (-2, -1, 0, 1, 2):
        if abs(v - k * pi / 2) < 1e-3:
            return {-2: "-pi", -1: "-pi/2", 0: "0", 1: "pi/2", 2: "pi"}[k]
    return f"{v:+.4f}"


if __name__ == "__main__":
    joints = urdf_joints()
    params, tool = derive(joints)
    print("Doosan M0609 - standard DH, derived from the URDF joint axes\n")
    print(f"  {'joint':5s} {'d (m)':>9s} {'a (m)':>9s} {'alpha':>7s} {'offset':>7s}   limits (rad)")
    for i, ((d, a, alpha, theta), (_, lim)) in enumerate(zip(params, joints), 1):
        print(f"  {i:5d} {d:+9.4f} {a:+9.4f} {_nice(alpha):>7s} {_nice(theta):>7s}   [{lim[0]:+.4f}, {lim[1]:+.4f}]")
    print("\n  tool (DH frame 6 -> flange):")
    print("  " + str(np.round(tool, 6)).replace("\n", "\n  "))

    rng = np.random.default_rng(0)
    worst_p = worst_r = 0.0
    for _ in range(2000):
        q = rng.uniform(-pi, pi, 6)
        A, B = dh_fk(q, params, tool), urdf_fk(q, joints)[-1]
        worst_p = max(worst_p, np.linalg.norm(A[:3, 3] - B[:3, 3]))
        worst_r = max(worst_r, np.linalg.norm(A[:3, :3] - B[:3, :3]))
    print(f"\n  check against Doosan's URDF, 2000 random poses:")
    print(f"    worst position error  {worst_p * 1000:.6f} mm")
    print(f"    worst rotation error  {worst_r:.2e} (matrix norm)")
