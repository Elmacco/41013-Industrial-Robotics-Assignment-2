"""
Collision checking for a planned path: every link, every step (Lab 5).

Lab 5 treats each link as a line segment and tests it against an obstacle's
faces with line_plane_intersection. That has two gaps for real hardware: a
segment has no thickness, and a segment lying wholly inside a box crosses no
face. So here each link is a capsule (a segment with a radius), and obstacles
are boxes tested by point-to-box distance, which handles both:

    clearance = distance(points along the link, box) - link radius

A capsule is clear when its clearance is above the margin. The same distance
test between two capsules covers self-collision (Lab 6 does this with
ellipsoids; capsules fit long thin links better and need no point cloud).

Bodies checked: the arm links, the gripper, and the pitcher (with the arm while
held, as an obstacle when it is standing somewhere).
"""
from dataclasses import dataclass, field

import numpy as np
from spatialmath import SE3

from barista import build


# ---------------------------------------------------------------- geometry
@dataclass
class Box:
    name: str
    T: np.ndarray            # 4x4 pose of the box centre (any yaw)
    half: np.ndarray         # half extents in the box frame

    def distance(self, pts):
        """Signed distance from each point to the box (negative inside)."""
        local = (pts - self.T[:3, 3]) @ self.T[:3, :3]
        q = np.abs(local) - self.half
        outside = np.linalg.norm(np.maximum(q, 0.0), axis=1)
        inside = np.minimum(np.max(q, axis=1), 0.0)
        return outside + inside


def box_from_spec(name, spec):
    """Bounding box of a layout placeholder (a cylinder becomes its square box)."""
    x, y, z, yaw = spec["pose"]
    if spec["shape"] == "box":
        sx, sy, sz = spec["size"]
    else:
        r, sz = spec["size"]
        sx = sy = 2 * r
    T = build.pose(x, y, z + sz / 2, yaw).A
    return Box(name, T, np.array([sx, sy, sz]) / 2)


def sample(p0, p1, spacing=0.02):
    n = max(int(np.ceil(np.linalg.norm(p1 - p0) / spacing)), 1) + 1
    return p0 + np.linspace(0.0, 1.0, n)[:, None] * (p1 - p0)


@dataclass
class Capsule:
    name: str
    pts: np.ndarray          # points along the axis
    radius: float


def capsule_gap(a: Capsule, b: Capsule):
    d = np.min(np.linalg.norm(a.pts[:, None, :] - b.pts[None, :, :], axis=2))
    return d - a.radius - b.radius


# ---------------------------------------------------------------- the checker
@dataclass
class ArmModel:
    """How the arm is drawn as capsules.

    links: (name, start link, end link, radius) between frame origins.
    """
    links: list
    gripper_len: float
    gripper_radius: float
    self_pairs: list         # capsule name pairs to test against each other
    skip_obstacles: dict = field(default_factory=dict)   # body -> obstacle names to ignore


AUBO_I5 = ArmModel(
    links=[
        ("shoulder", "shoulder_Link", "upperArm_Link", 0.065),
        ("upper_arm", "upperArm_Link", "foreArm_Link", 0.060),
        ("forearm", "foreArm_Link", "wrist1_Link", 0.050),
        ("wrist1", "wrist1_Link", "wrist2_Link", 0.045),
        ("wrist2", "wrist2_Link", "wrist3_Link", 0.045),
    ],
    gripper_len=0.05,         # + the radius = fingertips at 0.095
    gripper_radius=0.045,
    # Only pairs at least two joints apart: neighbours always touch at the joint.
    self_pairs=[(a, b) for a in ("base", "shoulder")
                for b in ("forearm", "wrist1", "wrist2", "gripper", "pitcher")]
               + [("upper_arm", b) for b in ("wrist2", "gripper", "pitcher")]
               + [("forearm", "pitcher")],
)
BASE_COLUMN = ("base", 0.075)    # fixed column from the base up to the shoulder


class CollisionChecker:
    """Clearance of the arm, gripper and pitcher against boxes and each other.

    :param obstacles: list of Box.
    :param margin: clearance required between the arm and anything else.
    :param pitcher: (radius, z0, z1) capsule along the pitcher axis, in its frame.
    :param pitcher_skip: obstacles the pitcher may touch by design (the wand).
    """

    def __init__(self, robot, obstacles, pitcher, pitcher_skip=(), margin=0.01,
                 model=AUBO_I5):
        self.robot = robot
        self.obstacles = list(obstacles)
        self.pitcher = pitcher
        self.pitcher_skip = set(pitcher_skip)
        self.margin = margin
        self.model = model

    def capsules(self, q, holding, pitcher_T):
        r = self.robot
        frames = {name: r.fkine(q, end=name) for name in
                  {lk for _, a, b, _ in self.model.links for lk in (a, b)}}
        base = r.fkine(q, end="base_link").t
        caps = [Capsule(BASE_COLUMN[0], sample(base, frames["shoulder_Link"].t), BASE_COLUMN[1])]
        for name, a, b, rad in self.model.links:
            caps.append(Capsule(name, sample(frames[a].t, frames[b].t), rad))
        flange = r.fkine(q)
        tip = (flange * SE3.Tz(self.model.gripper_len)).t
        caps.append(Capsule("gripper", sample(flange.t, tip), self.model.gripper_radius))
        if holding:
            rad, z0, z1 = self.pitcher
            caps.append(Capsule("pitcher", sample((pitcher_T * SE3.Tz(z0)).t,
                                                  (pitcher_T * SE3.Tz(z1)).t), rad))
        return caps

    def worst(self, q, holding, pitcher_T):
        """(clearance, what) for the tightest spot at configuration q."""
        caps = self.capsules(q, holding, pitcher_T)
        obstacles = list(self.obstacles)
        if not holding and pitcher_T is not None:
            rad, z0, z1 = self.pitcher
            obstacles.append(Box("pitcher", (pitcher_T * SE3.Tz((z0 + z1) / 2)).A,
                                 np.array([rad, rad, (z1 - z0) / 2 + rad])))
        best = (np.inf, "")
        for cap in caps:
            if cap.name == "base":
                continue                      # bolted to the table
            for box in obstacles:
                if cap.name == "pitcher" and box.name in self.pitcher_skip:
                    continue
                if cap.name == "gripper" and box.name == "pitcher":
                    continue                  # it grips it
                margin = 0.0 if cap.name == "pitcher" else self.margin
                c = float(np.min(box.distance(cap.pts))) - cap.radius - margin
                if c < best[0]:
                    best = (c, f"{cap.name} / {box.name}")
        by_name = {c.name: c for c in caps}
        for a, b in self.model.self_pairs:
            if a in by_name and b in by_name:
                c = capsule_gap(by_name[a], by_name[b])
                if c < best[0]:
                    best = (c, f"{a} / {b}")
        return best

    def first_collision(self, qs, holding, pitcher_T, fk_pitcher=None):
        """Index and description of the first colliding row of qs, or None.

        fk_pitcher(q) gives the pitcher pose while it is held.
        """
        for k, q in enumerate(qs):
            P = fk_pitcher(q) if (holding and fk_pitcher) else pitcher_T
            c, what = self.worst(q, holding, P)
            if c < 0.0:
                return k, what, c
        return None
