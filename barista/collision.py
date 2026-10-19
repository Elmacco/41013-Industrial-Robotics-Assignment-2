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

Bodies checked: the arm links, the gripper, and the object it carries (with
the arm while held, as an obstacle when it is standing somewhere). Other robots
can be added as capsules frozen where they wait.
"""
from dataclasses import dataclass

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


# ---------------------------------------------------------------- arm models
def _named_frames(names, base="base_link"):
    """Frame origins of a URDF robot, looked up by link name."""
    def frames(robot, q):
        out = {n: robot.fkine(q, end=n).t for n in names}
        out["base"] = robot.fkine(q, end=base).t
        return out
    return frames


def _dh_frames(names):
    """Frame origins of a DH robot: the base, then one frame per joint."""
    def frames(robot, q):
        return dict(zip(names, (T.t for T in robot.fkine_all(q))))
    return frames


@dataclass
class ArmModel:
    """How an arm is drawn as capsules.

    links: (name, start frame, end frame, radius). The one named "base" is the
        fixed column, never tested against the table it is bolted to.
    self_pairs: capsule names to test against each other. "held" is whatever
        the gripper is carrying.
    """
    frames: object           # (robot, q) -> {frame name: xyz}
    links: list
    gripper_len: float       # gripper capsule axis length (its ends add the radius)
    gripper_radius: float
    self_pairs: list


AUBO_I5 = ArmModel(
    frames=_named_frames(["shoulder_Link", "upperArm_Link", "foreArm_Link",
                          "wrist1_Link", "wrist2_Link", "wrist3_Link"]),
    links=[
        ("base", "base", "shoulder_Link", 0.075),
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
                for b in ("forearm", "wrist1", "wrist2", "gripper", "held")]
               + [("upper_arm", b) for b in ("wrist2", "gripper", "held")]
               + [("forearm", "held")],
)

# UR3 (standard DH). Its DH frames fold the shoulder and elbow side offsets into
# the wrist, so the capsules run between DH frame origins with UR3 link radii.
UR3 = ArmModel(
    frames=_dh_frames(["base", "j1", "j2", "j3", "j4", "j5", "j6"]),
    links=[
        ("base", "base", "j1", 0.065),
        ("upper_arm", "j1", "j2", 0.050),
        ("forearm", "j2", "j3", 0.045),
        ("wrist1", "j3", "j4", 0.040),
        ("wrist2", "j4", "j5", 0.040),
        ("wrist3", "j5", "j6", 0.040),
    ],
    gripper_len=0.03,
    gripper_radius=0.035,
    self_pairs=[("base", b) for b in ("forearm", "wrist1", "wrist2", "wrist3", "gripper", "held")]
               + [("upper_arm", b) for b in ("wrist2", "wrist3", "gripper", "held")]
               + [("forearm", "held")],
)


# CS66 (standard DH, see cs66.py). Drawn as 45 mm cylinders with 55 mm joint
# spheres, so the capsules use the sphere radius.
CS66 = ArmModel(
    frames=_dh_frames(["base", "j1", "j2", "j3", "j4", "j5", "j6"]),
    links=[
        ("base", "base", "j1", 0.075),
        ("upper_arm", "j1", "j2", 0.055),
        ("forearm", "j2", "j3", 0.055),
        ("wrist1", "j3", "j4", 0.055),
        ("wrist2", "j4", "j5", 0.055),
        ("wrist3", "j5", "j6", 0.050),
    ],
    gripper_len=0.03,
    gripper_radius=0.035,
    self_pairs=[("base", b) for b in ("forearm", "wrist1", "wrist2", "wrist3", "gripper", "held")]
               + [("upper_arm", b) for b in ("wrist2", "wrist3", "gripper", "held")]
               + [("forearm", "held")],
)


# ---------------------------------------------------------------- the checker
def held_segments(held):
    """Normalise a held-object shape to a list of (p0, p1, radius) in its frame.

    Either one capsule up the object's axis, (radius, z0, z1), or a list of
    ((x, y, z), (x, y, z), radius) capsules for an odd shape (the portafilter).
    """
    if isinstance(held, tuple) and len(held) == 3 and np.isscalar(held[0]):
        r, z0, z1 = held
        return [((0.0, 0.0, z0), (0.0, 0.0, z1), r)]
    return [(np.asarray(a, float), np.asarray(b, float), r) for a, b, r in held]


class CollisionChecker:
    """Clearance of an arm, its gripper and what it holds against boxes, other
    robots and itself.

    :param obstacles: list of Box.
    :param held: shape of the held object in its own frame (see held_segments).
        While it is not held it is an obstacle (except to the gripper, which
        closes on it).
    :param held_name: what to call the held object in messages.
    :param held_skip: obstacles the held object may touch by design (the wand).
    :param others: Capsules of other robots, frozen where they wait.
    :param margin: clearance required between the arm and anything else.
    """

    def __init__(self, robot, obstacles, held, held_name="pitcher", held_skip=(),
                 others=(), margin=0.01, model=AUBO_I5):
        self.robot = robot
        self.obstacles = list(obstacles)
        self.held = held_segments(held)
        self.held_name = held_name
        self.held_skip = set(held_skip)
        self.others = list(others)
        self.margin = margin
        self.model = model
        # Box round the held object, in its frame, for when it stands somewhere.
        pts = np.array([p for a, b, _ in self.held for p in (a, b)])
        r = max(rad for _, _, rad in self.held)
        self._held_lo, self._held_hi = pts.min(0) - r, pts.max(0) + r

    def capsules(self, q, holding=False, held_T=None):
        frames = self.model.frames(self.robot, q)
        caps = [Capsule(name, sample(frames[a], frames[b]), rad)
                for name, a, b, rad in self.model.links]
        flange = self.robot.fkine(q)
        tip = (flange * SE3.Tz(self.model.gripper_len)).t
        caps.append(Capsule("gripper", sample(flange.t, tip), self.model.gripper_radius))
        if holding:
            for a, b, rad in self.held:
                caps.append(Capsule("held", sample((held_T * SE3(*a)).t, (held_T * SE3(*b)).t), rad))
        return caps

    def resting_box(self, held_T):
        centre = (self._held_lo + self._held_hi) / 2
        return Box(self.held_name, (held_T * SE3(*centre)).A, (self._held_hi - self._held_lo) / 2)

    def _label(self, name):
        return self.held_name if name == "held" else name

    def worst(self, q, holding, held_T):
        """(clearance, what) for the tightest spot at configuration q."""
        caps = self.capsules(q, holding, held_T)
        obstacles = list(self.obstacles)
        if not holding and held_T is not None:
            obstacles.append(self.resting_box(held_T))
        best = (np.inf, "")
        for cap in caps:
            if cap.name == "base":
                continue                      # bolted to the table
            margin = 0.0 if cap.name == "held" else self.margin
            for box in obstacles:
                if cap.name == "held" and box.name in self.held_skip:
                    continue
                if cap.name == "gripper" and box.name == self.held_name:
                    continue                  # it grips it
                c = float(np.min(box.distance(cap.pts))) - cap.radius - margin
                if c < best[0]:
                    best = (c, f"{self._label(cap.name)} / {box.name}")
            for other in self.others:
                c = capsule_gap(cap, other) - margin
                if c < best[0]:
                    best = (c, f"{self._label(cap.name)} / {other.name}")
        by_name = {}
        for c in caps:
            by_name.setdefault(c.name, []).append(c)
        for a, b in self.model.self_pairs:
            for ca in by_name.get(a, []):
                for cb in by_name.get(b, []):
                    c = capsule_gap(ca, cb)
                    if c < best[0]:
                        best = (c, f"{self._label(a)} / {self._label(b)}")
        return best

    def first_collision(self, qs, holding, held_T, fk_held=None):
        """Index and description of the first colliding row of qs, or None.

        fk_held(q) gives the held object's pose while it is held.
        """
        for k, q in enumerate(qs):
            P = fk_held(q) if (holding and fk_held) else held_T
            c, what = self.worst(q, holding, P)
            if c < 0.0:
                return k, what, c
        return None


def frozen(checker, q, prefix):
    """Capsules of checker's robot at q, renamed with prefix, for use as `others`."""
    return [Capsule(f"{prefix} {c.name}", c.pts, c.radius) for c in checker.capsules(q)]
