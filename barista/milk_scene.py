"""
AUBO i5 milk demo. Each cycle:

    take the full pitcher from the dispenser -> steam it -> pour into the cup
    -> rinse it upside down at the wash station -> put it back in the
    dispenser, which refills it while the arm goes home

    python -m barista.milk_scene                     # latte, in Swift
    python -m barista.milk_scene --profile swirl     # another steam style
    python -m barista.milk_scene --list              # list the steam styles
    python -m barista.milk_scene --check             # plan every style, no Swift

The whole task is planned up front as a list of Segments (joint paths) and only
then played back. Every segment is collision checked as it is planned (arm,
gripper and pitcher against the station and against the arm itself, see
collision.py), and IK picks the clearest arm configuration rather than whatever
the solver lands on first.
"""
import argparse
import sys
from dataclasses import dataclass, field
from math import atan2, ceil, cos, hypot, pi, radians, sin
from pathlib import Path

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from barista import build  # noqa: E402
from barista import milk_layout as layout  # noqa: E402
from barista.collision import CollisionChecker, box_from_spec  # noqa: E402
from barista.steam_profiles import DEFAULT_PROFILE, PROFILES, WandPlacement  # noqa: E402

DT = 0.05
HIDDEN = SE3(0.0, 0.0, -5.0)          # parking spot for effects that are off
MAX_JOINT_STEP = radians(2)           # finest joint step a path is checked at

# Pitcher frame expressed in the flange frame while it is held: the flange's +Z
# points at the pitcher axis, the pitcher's +Z (up) is the flange's -X.
GRASP = SE3(layout.GRIP_HEIGHT, 0.0, layout.GRIP_DEPTH) * SE3.Ry(-pi / 2)
BACK_OFF = SE3.Tz(-0.08)    # pre-grasp / release: flange pulled back from the handle
CARRY_Z = layout.TABLE_TOP_Z + 0.18   # pitcher floor height while carried over the cup/basin
PULL_OUT = 0.12             # how far the pitcher is drawn out of the dispenser bay
CARRY_SPEED = 0.25          # m/s along the carry arcs

# Pouring: the pitcher tilts about its spout lip, which starts high over the cup
# (pitcher floor just clear of the rim) and drops towards the coffee as it tips.
# Keyframes over pour progress u: (u, tilt in degrees, lip height above rim).
POUR_KEYS = [(0.0, 0.0, 0.13), (0.3, 40.0, 0.08), (1.0, 80.0, 0.035)]
POUR_DURATION = 5.0
FLOW_TILT = radians(30)     # milk starts to leave the spout past this tilt
LIP_INSIDE_RIM = 0.015      # spout sits this far inside the cup's near rim

# Rinsing: the pitcher rolls upside down about the gripper axis over the spray.
WASH_HIGH = layout.TABLE_TOP_Z + 0.24   # floor height while it turns over
WASH_LOW = layout.SPRAY_TOP_Z + 0.01    # floor height when rinsing (mouth just over the spray)
RINSE_TIME = 2.5
REFILL_TIME = 2.0

# Pitcher capsule for collision checks: (radius, z0, z1) along its axis. Its
# rounded ends reach z0 - r and z1 + r, i.e. just inside the real pitcher.
PITCHER_CAPSULE = (0.050, 0.055, 0.060)


# ---------------------------------------------------------------- pitcher poses
def yaw_to(x, y):
    """Yaw that points the spout (and the gripper's approach) away from the base."""
    bx, by = layout.ROBOTS["milk"]["base"][:2]
    return atan2(y - by, x - bx)


def on_arc(angle_deg, r, z):
    """Upright pitcher with its floor centre r out along angle_deg, at height z."""
    x, y = layout.polar(angle_deg, r)
    return SE3(x, y, z) * SE3.Rz(radians(angle_deg))


def arc_coords(P):
    """(angle_deg, r, z) of a pitcher pose's floor centre."""
    x, y, z = P.t
    return np.degrees(atan2(y, x)), hypot(x, y), z


def at_wand(p: WandPlacement):
    """Pitcher pose that puts the (fixed) wand tip at placement p inside it."""
    wx, wy, _ = layout.WAND_TIP
    return SE3(*layout.WAND_TIP) * SE3.Rz(yaw_to(wx, wy)) * SE3.Rx(p.tilt) * SE3(-p.x, -p.y, -p.z)


def _ease(u):
    return u * u * (3.0 - 2.0 * u)


def pour_state(u):
    """(tilt, lip height above rim) at pour progress u in [0, 1]."""
    keys = np.array(POUR_KEYS)
    s = _ease(u)
    return radians(np.interp(s, keys[:, 0], keys[:, 1])), float(np.interp(s, keys[:, 0], keys[:, 2]))


def pour_pose(u):
    """Pitcher pose at pour progress u: tipped about the spout lip over the cup."""
    tilt, height = pour_state(u)
    cx, cy = layout.CUP_AT
    yaw = yaw_to(cx, cy)
    r = layout.CUP_R - LIP_INSIDE_RIM                     # lip sits on the near side
    lip = SE3(cx - r * cos(yaw), cy - r * sin(yaw),
              layout.TABLE_TOP_Z + layout.CUP_H + height) * SE3.Rz(yaw)
    return lip * SE3.Ry(tilt) * SE3(*(-np.array(layout.PITCHER_LIP)))


def rolled(P, angle):
    """P rolled by `angle` about the gripper axis (the pitcher's +X at grip height)."""
    g = SE3.Tz(layout.GRIP_HEIGHT)
    return P * g * SE3.Rx(angle) * g.inv()


def cup_level(tilt):
    """Milk height in the cup for a given pour tilt (monotonic, so no state)."""
    full = layout.CUP_H - 0.008
    w = np.clip((tilt - FLOW_TILT) / (radians(POUR_KEYS[-1][1]) - FLOW_TILT), 0.0, 1.0)
    return layout.ESPRESSO_H + w * (full - layout.ESPRESSO_H)


def flange(pitcher_pose):
    """Flange pose that holds the pitcher at pitcher_pose."""
    return pitcher_pose * GRASP.inv()


# ---------------------------------------------------------------- planning
class PlanningError(RuntimeError):
    pass


@dataclass
class Segment:
    name: str
    qs: np.ndarray                         # (N, n) joint path; row 0 is where it starts
    holding: bool = False                  # pitcher travels with the gripper
    cup_level: np.ndarray = None           # per row milk height in the cup
    effects: dict = field(default_factory=dict)   # effect name -> per row on/off

    @property
    def duration(self):
        return (len(self.qs) - 1) * DT


class Planner:
    """Turns targets into collision-checked joint paths, one Segment at a time.

    * ``ik`` solves from the current joints first and, when asked, from a spread
      of other seeds too, and keeps the clearest collision-free answer.
    * ``free_move`` (empty-handed, only the end pose matters) tries the direct
      joint path to each IK answer, then via random clear configurations (the
      Lab 5 primitive RRT) if every direct path is blocked.
    * ``follow`` / ``line_move`` are constrained Cartesian moves; they keep the
      arm's configuration and are rejected if any step collides.
    """

    def __init__(self, robot, q0, checker=None, pitcher_T=None, dt=DT, seed=1):
        self.robot = robot
        self.q = np.array(q0, dtype=float)
        self.dt = dt
        self.checker = checker
        self.pitcher_T = pitcher_T          # where the pitcher is when not held
        self.holding = False
        self.segments = []
        self.rng = np.random.default_rng(seed)

    # ---- helpers
    def _steps(self, duration):
        return max(int(round(duration / self.dt)), 1)

    def _fk_pitcher(self, q):
        return self.robot.fkine(q) * GRASP

    def clearance(self, q):
        if self.checker is None:
            return np.inf, ""
        P = self._fk_pitcher(q) if self.holding else self.pitcher_T
        return self.checker.worst(q, self.holding, P)

    def collision(self, qs):
        if self.checker is None:
            return None
        return self.checker.first_collision(qs, self.holding, self.pitcher_T, self._fk_pitcher)

    def _solve(self, T, seed):
        # A URDF robot's ikine solves in its base frame (fkine does include the
        # base), so hand it the target relative to the base.
        sol = self.robot.ikine_LM(self.robot.base.inv() * T, q0=seed, joint_limits=True)
        if not sol.success or np.linalg.norm(self.robot.fkine(sol.q).t - T.t) > 1e-3:
            return None
        # Every joint turns +-2 pi, so the solver may return the same pose a
        # full turn away; take the turn nearest the seed (if within limits).
        q = np.array(sol.q)
        near = q - 2 * pi * np.round((q - seed) / (2 * pi))
        lo, hi = self.robot.qlim
        return np.where((near >= lo) & (near <= hi), near, q)

    def ik(self, T, seed, label, search=0):
        """IK for flange pose T. With search > 0, also try that many random
        seeds and return the clear solutions, clearest first."""
        found = []
        for s in [seed] + [self.rng.uniform(-pi, pi, self.robot.n) for _ in range(search)]:
            q = self._solve(T, s)
            if q is not None and all(np.max(np.abs(q - f)) > 1e-2 for f in found):
                found.append(q)
        if not found:
            raise PlanningError(f"{label}: no IK solution")
        if not search:
            return found[0]
        scored = sorted(((self.clearance(q)[0], i) for i, q in enumerate(found)), reverse=True)
        clear = [found[i] for c, i in scored if c >= 0.0]
        if not clear:
            c, i = scored[0]
            raise PlanningError(f"{label}: every IK solution collides "
                                f"(best {c * 1000:.0f} mm, {self.clearance(found[i])[1]})")
        return clear

    def _add(self, name, qs, **fx):
        qs = np.asarray(qs)
        hit = self.collision(qs)
        if hit is not None:
            k, what, c = hit
            raise PlanningError(f"{name}: collision at step {k}/{len(qs) - 1}: "
                                f"{what} ({c * 1000:.0f} mm)")
        seg = Segment(name, qs, holding=self.holding, **fx)
        self.segments.append(seg)
        self.q = seg.qs[-1]
        return seg

    def _joint_path(self, q0, q1, duration=0.0):
        n = max(self._steps(duration), int(ceil(np.max(np.abs(q1 - q0)) / MAX_JOINT_STEP)))
        return rtb.jtraj(q0, q1, n + 1).q

    # ---- gripper
    def grip(self, closed):
        if not closed:
            self.pitcher_T = self._fk_pitcher(self.q)
        self.holding = closed

    # ---- moves
    def free_move(self, name, target, duration, search=40, tries=200):
        """Joint-space move to a flange pose (SE3) or a joint vector."""
        goals = self.ik(target, self.q, name, search) if isinstance(target, SE3) else [np.asarray(target, float)]
        for q1 in goals:
            path = self._joint_path(self.q, q1, duration)
            if self.collision(path) is None:
                return self._add(name, path)
        # Primitive RRT (Lab 5, 3.3): random clear configuration as a via point.
        q1 = goals[0]
        for _ in range(tries):
            via = self.q + self.rng.uniform(-pi / 2, pi / 2, self.robot.n)
            if self.clearance(via)[0] < 0.0:
                continue
            a, b = self._joint_path(self.q, via, duration / 2), self._joint_path(via, q1, duration / 2)
            if self.collision(a) is None and self.collision(b) is None:
                return self._add(name, np.vstack([a, b[1:]]))
        raise PlanningError(f"{name}: no collision-free joint path found")

    def follow(self, name, flange_poses, max_jump=0.2, **fx):
        """Track sampled flange poses, each IK seeded by the one before.

        A big jump between samples means the solver flipped configuration (or
        the path passes near a singularity), so it is refused instead of
        letting the arm whip round.
        """
        qs = [self.q]
        for i, T in enumerate(flange_poses):
            q = self.ik(T, qs[-1], f"{name}, sample {i}")
            jump = np.max(np.abs(q - qs[-1]))
            if jump > max_jump:
                raise PlanningError(f"{name}, sample {i}: joint jump of {jump:.2f} rad")
            qs.append(q)
        return self._add(name, qs, **fx)

    def line_move(self, name, T_flange, duration, **fx):
        """Straight-line Cartesian move of the flange (orientation slerped)."""
        T0 = self.robot.fkine(self.q)
        return self.follow(name, list(rtb.ctraj(T0, T_flange, self._steps(duration)))[1:], **fx)

    def carry(self, name, *waypoints):
        """Carry the pitcher upright through (angle_deg, r, z) waypoints, sweeping
        round the base on arcs rather than cutting across it in straight lines."""
        poses = []
        a0, r0, z0 = arc_coords(self._fk_pitcher(self.q))
        for a1, r1, z1 in waypoints:
            length = hypot(radians(a1 - a0) * (r0 + r1) / 2, r1 - r0, z1 - z0)
            n = max(self._steps(length / CARRY_SPEED), 10)
            for u in np.linspace(0.0, 1.0, n + 1)[1:]:
                s = _ease(u)
                poses.append(flange(on_arc(a0 + s * (a1 - a0), r0 + s * (r1 - r0), z0 + s * (z1 - z0))))
            a0, r0, z0 = a1, r1, z1
        return self.follow(name, poses)

    def wait(self, name, duration, **fx):
        return self._add(name, np.repeat(self.q[None], self._steps(duration) + 1, axis=0), **fx)


def make_checker(robot):
    obstacles = [box_from_spec(n, s) for n, s in layout.PLACEHOLDERS.items()]
    return CollisionChecker(robot, obstacles, PITCHER_CAPSULE, pitcher_skip={"steam_wand"})


def pitcher_home():
    return on_arc(*layout.PITCHER_HOME)


def plan_task(robot, profile=DEFAULT_PROFILE, check=True, q0=None):
    """Plan one full cycle for a steam profile; returns the Segments."""
    steam = PROFILES[profile]
    q0 = HOME_Q if q0 is None else q0
    home = pitcher_home()
    pl = Planner(robot, q0, make_checker(robot) if check else None, pitcher_T=home)

    a_disp, r, z_bay = layout.PITCHER_HOME
    a_steam, a_wash = layout.ANGLE["steamer"], layout.ANGLE["wash"]
    lifted = on_arc(a_disp, r, z_bay + 0.06)
    below_wand = at_wand(WandPlacement(0.0, 0.0, layout.PITCHER_H + 0.03))   # rim 3 cm under the tip
    _, _, z_below_wand = arc_coords(below_wand)

    # Take the full pitcher out of the dispenser
    pl.free_move("approach dispenser", flange(home) * BACK_OFF, 3.0)
    pl.line_move("reach handle", flange(home), 1.0)
    pl.grip(True)
    pl.line_move("lift pitcher", flange(lifted), 1.0)
    pl.carry("draw out of bay", (a_disp, r - PULL_OUT, z_bay + 0.06))

    # Steam: over the basin and cup, drop down beside the steamer, slide under the wand
    pl.carry("carry to steamer", (a_disp, r - PULL_OUT, CARRY_Z), (a_steam + 25, r, CARRY_Z),
             (a_steam + 25, r, z_below_wand), (a_steam, r, z_below_wand))
    pl.line_move("dip wand in", flange(at_wand(steam.placement(0.0))), 1.5)
    n = pl._steps(steam.duration)
    pl.follow(f"steam: {profile}", [flange(at_wand(steam.placement(i / n))) for i in range(1, n + 1)])
    pl.line_move("lower off wand", flange(below_wand), 1.5)

    # Pour
    pl.carry("carry to cup", arc_coords(pour_pose(0.0)))
    n = pl._steps(POUR_DURATION)
    us = np.linspace(0.0, 1.0, n + 1)
    tilts = np.array([pour_state(u)[0] for u in us])
    pl.follow("pour", [flange(pour_pose(u)) for u in us[1:]],
              cup_level=np.array([cup_level(t) for t in tilts]),
              effects={"milk_stream": tilts > FLOW_TILT})
    pl.line_move("level pitcher", flange(pour_pose(0.0)), 2.0)

    # Rinse upside down over the spray
    over_basin = on_arc(a_wash, r, WASH_HIGH)
    pl.carry("carry to wash", (a_wash, r, WASH_HIGH))
    n = pl._steps(1.5)
    pl.follow("turn over", [flange(rolled(over_basin, pi * _ease(i / n))) for i in range(1, n + 1)])
    pl.line_move("lower over spray", flange(rolled(on_arc(a_wash, r, WASH_LOW), pi)), 1.0)
    pl.wait("rinse", RINSE_TIME, effects={"rinse_spray": True})
    pl.line_move("raise off spray", flange(rolled(over_basin, pi)), 1.0)
    pl.follow("turn upright", [flange(rolled(over_basin, pi * (1 - _ease(i / n)))) for i in range(1, n + 1)])

    # Back into the dispenser, which refills it
    pl.carry("carry to dispenser", (a_disp, r - PULL_OUT, CARRY_Z), (a_disp, r - PULL_OUT, z_bay + 0.06))
    pl.carry("slide into bay", (a_disp, r, z_bay + 0.06))
    pl.line_move("set down", flange(home), 1.0)
    pl.grip(False)
    pl.line_move("release", flange(home) * BACK_OFF, 1.0)
    pl.free_move("home", q0, 2.5)
    pl.wait("refill", REFILL_TIME, effects={"dispenser_stream": True})
    return pl.segments


# Elbow up, gripper level and pointing out over the cup. Clear of everything.
HOME_Q = np.array([0.0, 0.0, 1.6, 0.0, 1.57, 0.0])


# ---------------------------------------------------------------- scene
def _place(obj, spec, parent):
    """Put a part given in a parent frame (spec pose z = its bottom face)."""
    x, y, z, yaw = spec["pose"]
    obj.T = (parent * build.pose(x, y, z + spec["size"][-1] / 2, yaw)).A


MILK_WHITE, COFFEE, WATER = (0.97, 0.95, 0.90, 1), (0.40, 0.24, 0.12, 1), (0.55, 0.75, 0.95, 0.6)
EFFECTS = {
    # The coffee and milk are full-height cylinders rising out of the table
    # (which hides the part below the cup), so their top face is the level.
    "espresso": dict(shape="cylinder", size=(layout.CUP_R - 0.003, layout.CUP_H), color=COFFEE),
    "cup_milk": dict(shape="cylinder", size=(layout.CUP_R - 0.006, layout.CUP_H), color=MILK_WHITE),
    "milk_stream": dict(shape="cylinder", size=(0.005, 0.15), color=MILK_WHITE),
    "dispenser_stream": dict(shape="cylinder", size=(0.004, 0.09), color=MILK_WHITE),
    "rinse_spray": dict(shape="cylinder", size=(0.006, 0.05), color=WATER),
}


def build_scene(env=None):
    scene = build.build_scene(layout, env)
    scene.robots["milk"].q = HOME_Q

    scene.pitcher = {n: build.make_placeholder(s) for n, s in layout.PITCHER_PARTS.items()}
    scene.gripper = {n: build.make_placeholder(s) for n, s in layout.GRIPPER_PARTS.items()}
    scene.effects = {n: build.make_placeholder(dict(s, pose=(0, 0, 0, 0))) for n, s in EFFECTS.items()}
    scene.pitcher_pose = pitcher_home()

    _set_level(scene, "espresso", layout.ESPRESSO_H)
    _set_level(scene, "cup_milk", 0.0)
    _set_effects(scene, {})
    _update_attached(scene, holding=False)

    if env is not None:
        for obj in (*scene.pitcher.values(), *scene.gripper.values(), *scene.effects.values()):
            env.add(obj)
    return scene


def _set_level(scene, name, level):
    """Raise a full-height liquid cylinder so its top sits `level` above the cup floor."""
    cx, cy = layout.CUP_AT
    scene.effects[name].T = SE3(cx, cy, layout.TABLE_TOP_Z + level - layout.CUP_H / 2).A


def _update_attached(scene, holding):
    T_flange = scene.robots["milk"].fkine(scene.robots["milk"].q)
    for name, spec in layout.GRIPPER_PARTS.items():
        _place(scene.gripper[name], spec, T_flange)
    if holding:
        scene.pitcher_pose = T_flange * GRASP
    for name, spec in layout.PITCHER_PARTS.items():
        _place(scene.pitcher[name], spec, scene.pitcher_pose)


def _set_effects(scene, on):
    """Show the streams named in `on`, hide the rest."""
    fx = scene.effects
    for name in ("milk_stream", "dispenser_stream", "rinse_spray"):
        fx[name].T = HIDDEN.A
    if on.get("milk_stream"):
        lip = (scene.pitcher_pose * SE3(*layout.PITCHER_LIP)).t
        fx["milk_stream"].T = SE3(lip[0], lip[1], lip[2] - 0.075).A      # hangs from the lip
    if on.get("dispenser_stream"):
        x, y = layout.polar(layout.ANGLE["dispenser"], layout.STATION_R)
        fx["dispenser_stream"].T = SE3(x, y, layout.NOZZLE_Z - 0.045).A  # nozzle into the pitcher
    if on.get("rinse_spray"):
        x, y = layout.polar(layout.ANGLE["wash"], layout.STATION_R)
        fx["rinse_spray"].T = SE3(x, y, layout.SPRAY_TOP_Z + 0.025).A    # up into the pitcher


def play(scene, env, segments, dt=DT, verbose=True):
    robot = scene.robots["milk"]
    for seg in segments:
        if verbose:
            print(f"    {seg.name:22s} {seg.duration:4.1f} s")
        for k, q in enumerate(seg.qs):
            robot.q = q
            _update_attached(scene, seg.holding)
            if seg.cup_level is not None:
                _set_level(scene, "cup_milk", seg.cup_level[k])
            _set_effects(scene, {name: bool(np.atleast_1d(v)[min(k, np.size(v) - 1)])
                                 for name, v in seg.effects.items()})
            env.step(dt)


def animate(scene, env, cycles=1, profile=DEFAULT_PROFILE):
    """Hook used by barista.launch: run the milk task `cycles` times."""
    segments = plan_task(scene.robots["milk"], profile)
    print(f"  steam profile '{profile}': {PROFILES[profile].description}")
    print(f"  {len(segments)} segments, {sum(s.duration for s in segments):.1f} s per cycle")
    for _ in range(max(int(cycles), 0)):
        _set_level(scene, "cup_milk", 0.0)        # a fresh cup each cycle
        play(scene, env, segments)


def check_reach(scene):
    return build.check_reach(scene, layout)


# ---------------------------------------------------------------- CLI
def clearance_report(robot, segments):
    """Tightest clearance per segment, for --check."""
    checker = make_checker(robot)
    pitcher_T = pitcher_home()
    rows = []
    for seg in segments:
        worst = (np.inf, "")
        for q in seg.qs[::2]:
            P = robot.fkine(q) * GRASP if seg.holding else pitcher_T
            worst = min(worst, checker.worst(q, seg.holding, P))
        if seg.holding:
            pitcher_T = robot.fkine(seg.qs[-1]) * GRASP
        rows.append((seg.name, *worst))
    return rows


def _check_all(profiles, verbose=False):
    """Plan every profile without Swift; returns True if all succeed."""
    ok = True
    for name in profiles:
        robot = build_scene().robots["milk"]
        try:
            segs = plan_task(robot, name)
            total = sum(s.duration for s in segs)
            print(f"OK    {name:12s} {len(segs)} segments, {total:.1f} s")
            if verbose:
                for seg, c, what in clearance_report(robot, segs):
                    print(f"        {seg:22s} tightest {c * 1000:5.0f} mm  {what}")
        except PlanningError as e:
            ok = False
            print(f"FAIL  {name:12s} {e}")
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("-p", "--profile", choices=sorted(PROFILES), default=DEFAULT_PROFILE,
                        help=f"steam style (default: {DEFAULT_PROFILE})")
    parser.add_argument("-c", "--cycles", type=int, default=1, help="times to run the task")
    parser.add_argument("--list", action="store_true", help="list the steam styles and exit")
    parser.add_argument("--check", action="store_true",
                        help="plan every style without Swift, print clearances and exit")
    args = parser.parse_args(argv)

    if args.list:
        for name, prof in PROFILES.items():
            print(f"  {name:12s} {prof.duration:4.1f} s  {prof.description}")
        return
    if args.check:
        sys.exit(0 if _check_all(PROFILES, verbose=True) else 1)

    import swift

    env = swift.Swift()
    env.launch(realtime=True)
    scene = build_scene(env)
    env.set_camera_pose(list(layout.CAMERA["position"]), list(layout.CAMERA["look_at"]))
    env.step(DT)
    animate(scene, env, cycles=args.cycles, profile=args.profile)
    input("Press ENTER to close")
    env.close()


if __name__ == "__main__":
    main()
