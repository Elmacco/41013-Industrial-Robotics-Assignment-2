"""
Doosan M0609 milk demo. Each cycle:

    take the full pitcher from the dispenser -> steam it -> pour into the cup
    -> rinse it upside down at the wash station -> put it back in the
    dispenser, which refills it while the arm goes home

    python -m barista.milk_scene                     # latte, in Swift
    python -m barista.milk_scene --profile swirl     # another steam style
    python -m barista.milk_scene --list              # list the steam styles
    python -m barista.milk_scene --check             # plan every style, no Swift

The whole task is planned up front as a list of Segments (joint paths) and only
then played back (planner.py). Every segment is collision checked as it is
planned (arm, gripper and pitcher against the station and against the arm
itself, see collision.py), and IK picks the clearest arm configuration rather
than whatever the solver lands on first.

full_scene.py runs this same task inside the whole cell, with the UR3 bringing
the cup.
"""
import argparse
import sys
from math import atan2, cos, hypot, pi, radians, sin
from pathlib import Path

import numpy as np
from spatialmath import SE3

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from barista import build  # noqa: E402
from barista import milk_layout as layout  # noqa: E402
from barista.collision import M0609, CollisionChecker, box_from_spec  # noqa: E402
from barista.cup import Cup  # noqa: E402
from barista.planner import DT, Planner, PlanningError  # noqa: E402
from barista.steam_profiles import DEFAULT_PROFILE, PROFILES, WandPlacement  # noqa: E402

HIDDEN = SE3(0.0, 0.0, -5.0)          # parking spot for effects that are off

# Pitcher frame expressed in the flange frame while it is held: the flange's +Z
# points at the pitcher axis, the pitcher's +Z (up) is the flange's -X.
GRASP = SE3(layout.GRIP_HEIGHT, 0.0, layout.GRIP_DEPTH) * SE3.Ry(-pi / 2)
BACK_OFF = SE3.Tz(-0.08)    # pre-grasp / release: flange pulled back from the handle
CARRY_Z = layout.TABLE_TOP_Z + 0.18   # pitcher floor height while carried over the cup/basin
PULL_OUT = 0.07             # how far the pitcher is drawn out of the dispenser bay (clear of the nozzle)
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


def pour_pose(u, cup_at=None):
    """Pitcher pose at pour progress u: tipped about the spout lip over the cup
    (at cup_at, default layout.CUP_AT)."""
    tilt, height = pour_state(u)
    cx, cy = layout.CUP_AT if cup_at is None else cup_at
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
def carry(pl, name, *waypoints):
    """Carry the pitcher upright through (angle_deg, r, z) waypoints, sweeping
    round the base on arcs rather than cutting across it in straight lines."""
    poses = []
    a0, r0, z0 = arc_coords(pl.fk_held(pl.q))
    for a1, r1, z1 in waypoints:
        length = hypot(radians(a1 - a0) * (r0 + r1) / 2, r1 - r0, z1 - z0)
        n = max(pl._steps(length / CARRY_SPEED), 10)
        for u in np.linspace(0.0, 1.0, n + 1)[1:]:
            s = _ease(u)
            poses.append(flange(on_arc(a0 + s * (a1 - a0), r0 + s * (r1 - r0), z0 + s * (z1 - z0))))
        a0, r0, z0 = a1, r1, z1
    return pl.follow(name, poses)


def make_checker(robot, obstacles=None, others=(), cup_at=None):
    """Checker for the M0609. obstacles defaults to the milk station plus the
    cup (at cup_at, default layout.CUP_AT)."""
    if obstacles is None:
        obstacles = [box_from_spec(n, s) for n, s in layout.PLACEHOLDERS.items()]
        cup = layout.CUP_SPEC if cup_at is None else dict(layout.CUP_SPEC, pose=(*cup_at, layout.TABLE_TOP_Z, 0.0))
        obstacles.append(box_from_spec("cup", cup))
    return CollisionChecker(robot, obstacles, PITCHER_CAPSULE, held_name="pitcher",
                            held_skip={"steam_wand"}, others=others, model=M0609)


def pitcher_home():
    return on_arc(*layout.PITCHER_HOME)


def plan_task(robot, profile=DEFAULT_PROFILE, check=True, q0=None, checker=None, name="milk",
              cup_at=None):
    """Plan one full cycle for a steam profile; returns the Segments.

    checker defaults to make_checker(robot); pass one to check against a bigger
    scene. check=False plans with no collision checks at all. cup_at is where
    to pour (default layout.CUP_AT), e.g. where visual servoing found the cup.
    """
    steam = PROFILES[profile]
    q0 = HOME_Q if q0 is None else q0
    home = pitcher_home()
    if check and checker is None:
        checker = make_checker(robot, cup_at=cup_at)
    # RMRC for the Cartesian moves: the pour passes close to the M0609's wrist
    # singularity, where damped least squares keeps the motion smooth.
    pl = Planner(robot, q0, checker if check else None, grasp=GRASP, held_T=home, name=name,
                 tracker="rmrc")

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
    carry(pl, "draw out of bay", (a_disp, r - PULL_OUT, z_bay + 0.06))

    # Steam: over the basin and cup, drop down beside the steamer, slide under the wand
    carry(pl, "carry to steamer", (a_disp, r - PULL_OUT, CARRY_Z), (a_steam + 25, r, CARRY_Z),
             (a_steam + 25, r, z_below_wand), (a_steam, r, z_below_wand))
    pl.line_move("dip wand in", flange(at_wand(steam.placement(0.0))), 1.5)
    n = pl._steps(steam.duration)
    pl.follow(f"steam: {profile}", [flange(at_wand(steam.placement(i / n))) for i in range(1, n + 1)])
    pl.line_move("lower off wand", flange(below_wand), 1.5)

    # Pour
    carry(pl, "carry to cup", arc_coords(pour_pose(0.0, cup_at)))
    n = pl._steps(POUR_DURATION)
    us = np.linspace(0.0, 1.0, n + 1)
    tilts = np.array([pour_state(u)[0] for u in us])
    pl.follow("pour", [flange(pour_pose(u, cup_at)) for u in us[1:]],
              cup_level=np.array([cup_level(t) for t in tilts]),
              effects={"milk_stream": tilts > FLOW_TILT})
    pl.line_move("level pitcher", flange(pour_pose(0.0, cup_at)), 2.0)

    # Rinse upside down over the spray
    over_basin = on_arc(a_wash, r, WASH_HIGH)
    carry(pl, "carry to wash", (a_wash, r, WASH_HIGH))
    n = pl._steps(1.5)
    pl.follow("turn over", [flange(rolled(over_basin, pi * _ease(i / n))) for i in range(1, n + 1)])
    pl.line_move("lower over spray", flange(rolled(on_arc(a_wash, r, WASH_LOW), pi)), 1.0)
    pl.wait("rinse", RINSE_TIME, effects={"rinse_spray": True})
    pl.line_move("raise off spray", flange(rolled(over_basin, pi)), 1.0)
    pl.follow("turn upright", [flange(rolled(over_basin, pi * (1 - _ease(i / n)))) for i in range(1, n + 1)])

    # Back into the dispenser, which refills it
    carry(pl, "carry to dispenser", (a_disp, r - PULL_OUT, CARRY_Z), (a_disp, r - PULL_OUT, z_bay + 0.06))
    carry(pl, "slide into bay", (a_disp, r, z_bay + 0.06))
    pl.line_move("set down", flange(home), 1.0)
    pl.grip(False)
    pl.line_move("release", flange(home) * BACK_OFF, 1.0)
    pl.free_move("home", q0, 2.5)
    pl.wait("refill", REFILL_TIME, effects={"dispenser_stream": True})
    return pl.segments


# Elbow up, gripper level and pointing out over the cup. Clear of everything.
HOME_Q = np.array([0.0, 0.0, 1.9, 0.0, 1.2, 0.0])


# ---------------------------------------------------------------- scene
def _place(obj, spec, parent):
    """Put a part given in a parent frame (spec pose z = its bottom face)."""
    x, y, z, yaw = spec["pose"]
    obj.T = (parent * build.pose(x, y, z + spec["size"][-1] / 2, yaw)).A


MILK_WHITE, WATER = (0.97, 0.95, 0.90, 1), (0.55, 0.75, 0.95, 0.6)
EFFECTS = {
    "milk_stream": dict(shape="cylinder", size=(0.005, 0.15), color=MILK_WHITE),
    "dispenser_stream": dict(shape="cylinder", size=(0.004, 0.09), color=MILK_WHITE),
    "rinse_spray": dict(shape="cylinder", size=(0.006, 0.05), color=WATER),
}


def build_scene(env=None):
    scene = build.build_scene(layout, env)
    add_station_parts(scene, env)
    scene.cup = Cup(layout.CUP_R, layout.CUP_H, SE3(*layout.CUP_AT, layout.TABLE_TOP_Z))
    reset_cup(scene)
    if env is not None:
        scene.cup.add_to_env(env)
    return scene


def add_station_parts(scene, env=None):
    """Add the M0609's moving parts (pitcher, gripper, streams) to a built scene."""
    scene.robots["milk"].q = HOME_Q
    scene.pitcher = {n: build.make_placeholder(s) for n, s in layout.PITCHER_PARTS.items()}
    scene.gripper = {n: build.make_placeholder(s) for n, s in layout.GRIPPER_PARTS.items()}
    scene.effects = {n: build.make_placeholder(dict(s, pose=(0, 0, 0, 0))) for n, s in EFFECTS.items()}
    scene.pitcher_pose = pitcher_home()
    set_effects(scene, {})
    update_attached(scene, holding=False)
    if env is not None:
        for obj in (*scene.pitcher.values(), *scene.gripper.values(), *scene.effects.values()):
            env.add(obj)


def reset_cup(scene):
    """A fresh cup with just the espresso in it."""
    scene.cup.set(espresso=layout.ESPRESSO_H, level=layout.ESPRESSO_H)


def update_attached(scene, holding):
    T_flange = scene.robots["milk"].fkine(scene.robots["milk"].q)
    for name, spec in layout.GRIPPER_PARTS.items():
        _place(scene.gripper[name], spec, T_flange)
    if holding:
        scene.pitcher_pose = T_flange * GRASP
    for name, spec in layout.PITCHER_PARTS.items():
        _place(scene.pitcher[name], spec, scene.pitcher_pose)


def set_effects(scene, on):
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
            update_attached(scene, seg.holding)
            if seg.cup_level is not None:
                scene.cup.set(level=seg.cup_level[k])
            set_effects(scene, {name: bool(np.atleast_1d(v)[min(k, np.size(v) - 1)])
                                 for name, v in seg.effects.items()})
            env.step(dt)


def animate(scene, env, cycles=1, profile=DEFAULT_PROFILE):
    """Hook used by barista.launch: run the milk task `cycles` times."""
    segments = plan_task(scene.robots["milk"], profile)
    print(f"  steam profile '{profile}': {PROFILES[profile].description}")
    print(f"  {len(segments)} segments, {sum(s.duration for s in segments):.1f} s per cycle")
    for _ in range(max(int(cycles), 0)):
        reset_cup(scene)                          # a fresh cup each cycle
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
