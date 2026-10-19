"""
The full barista cell: order on the tablet, the robots make the drink, and it
comes out through the collection drawer. Nobody ever reaches the robots.

    python -m barista.full_scene                       # kiosk mode in Swift
    python -m barista.full_scene --order latte --order espresso   # pre-queue orders
    python -m barista.full_scene --check               # plan every drink, no Swift

One order:

    1. the cup dispenser drops a cup; the UR3 puts it under the group head
    2. CS66: takes the portafilter from its dock, fills it at the grinder,
       presses it in the tamping machine, locks it into the group head and the
       shot pulls into the cup; then knocks the puck out, rinses the
       portafilter and puts it back on its dock
    3. UR3 carries the cup to the hand-off spot          (skipped: espresso)
    4. AUBO i5 fills, steams and pours the milk           (skipped: espresso)
    5. UR3 carries the drink to the collection drawer
    6. the drawer slides out through the glass, the customer takes the cup,
       and the drawer comes back

In Swift, the tablet's buttons queue orders. Either e-stop button (the one
inside the cell or the one outside) freezes every robot where it is until
"Reset e-stop" is pressed. The beacon on its mast shows green (ready), amber
(making a drink) or red (e-stop).

The robots work at the same time (see OrderPlanner): each robot's moves are
planned and collision checked against the cell, then schedule.py runs them
together, checking every pair of robots against each other at every frame and
pausing one when two would come too close. Orders queued together are made
together, overlapped.
"""
import argparse
import hashlib
import pickle
import sys
import tempfile
import threading
from collections import deque
from dataclasses import dataclass
from math import atan2, degrees, hypot, pi, radians
from pathlib import Path

import numpy as np
import spatialgeometry as geometry
from spatialmath import SE3

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from barista import build  # noqa: E402
from barista import cs66  # noqa: E402
from barista import full_layout as layout  # noqa: E402
from barista import labels  # noqa: E402
from barista import milk_scene  # noqa: E402
from barista.collision import CS66 as CS66_MODEL, UR3 as UR3_MODEL  # noqa: E402
from barista.collision import Box, CollisionChecker, box_from_spec, frozen  # noqa: E402
from barista.cup import GLASS, Cup  # noqa: E402
from barista.planner import DT, Planner, PlanningError, Segment  # noqa: E402
from barista.schedule import Lane, Signal, Step, Wait, schedule  # noqa: E402

HIDDEN = SE3(0.0, 0.0, -5.0)

# ---------------------------------------------------------------- menu
# Drink -> steam profile (None = no milk: the cup goes straight to the drawer).
MENU = {
    "latte": "latte",
    "cappuccino": "cappuccino",
    "flat_white": "flat_white",
    "swirl_latte": "swirl",
    "espresso": None,
}

# ---------------------------------------------------------------- UR3 runner
UR3_HOME_Q = np.array([0.0, -pi / 2, 0.0, -pi / 2, 0.0, 0.0])   # upright, wrist folded
CUP_GRIP_DEPTH = 0.10        # flange to cup axis
CUP_GRIP_HEIGHT = 0.06       # how far up the cup the fingers close
GRASP_CUP = SE3(CUP_GRIP_HEIGHT, 0.0, CUP_GRIP_DEPTH) * SE3.Ry(-pi / 2)
CUP_CAPSULE = (0.042, 0.045, 0.055)   # (radius, z0, z1) along the cup axis
# Approach / retreat: back towards the UR3 and up at the same time (the flange's
# -X is up while gripping), so the low arm never folds in on its own base.
# Under the group head or the dispenser tube there is no room above, so those
# are approached level.
BACK_OFF = SE3(-0.05, 0.0, -0.06)
BACK_OFF_LEVEL = SE3.Tz(-0.07)
OVERHEAD = {"dispenser", "espresso"}   # spots with something just above the cup
LIFT = 0.08
CARRY_LIFT = 0.15            # cup floor height above the table while carried
CARRY_SPEED = 0.25
DRAW_OUT = 0.12              # slide the cup out from under things this far first

UR3_GRIPPER_COLOR = (0.65, 0.35, 0.15, 1)
UR3_GRIPPER_PARTS = {
    "body": dict(shape="box", size=(0.06, 0.06, 0.04), pose=(0.0, 0.0, 0.0, 0.0), color=UR3_GRIPPER_COLOR),
    "finger_l": dict(shape="box", size=(0.02, 0.01, 0.08), pose=(0.0, 0.047, 0.04, 0.0), color=UR3_GRIPPER_COLOR),
    "finger_r": dict(shape="box", size=(0.02, 0.01, 0.08), pose=(0.0, -0.047, 0.04, 0.0), color=UR3_GRIPPER_COLOR),
}

# ---------------------------------------------------------------- CS66 barista
CS66_HOME_Q = np.array(cs66.HOME_Q, dtype=float)
# The gripper closes on the portafilter handle from above: flange +Z points
# down, +X along the handle towards the basket.
PF_HANDLE_Z = -0.015
PF_FINGER = 0.09             # flange to handle centre (long fingers keep the wrist clear)
GRASP_PF = (SE3(layout.PF_GRIP_X, 0.0, PF_HANDLE_Z + PF_FINGER) * SE3.Rx(pi)).inv()
ABOVE = SE3.Tz(-0.07)        # flange pulled straight up (its +Z is down)
PF_CARRY_Z = layout.TABLE_TOP_Z + 0.44   # rim height between stations, over every machine
PF_FRONT = 0.15              # stations are entered radially from this far in
LOCK_TWIST = -40.0           # degrees: the handle swings this far to lock in
CS66_KEEP_CLEAR = 180.0      # direction (about the CS66) the portafilter never swings through
LOCK_RISE = 0.03             # rim rises this much into the group head
KNOCK_DROP = 0.04

CS66_GRIPPER_COLOR = (0.20, 0.55, 0.30, 1)
CS66_GRIPPER_PARTS = {
    "body": dict(shape="box", size=(0.05, 0.06, 0.03), pose=(0.0, 0.0, 0.0, 0.0), color=CS66_GRIPPER_COLOR),
    "finger_l": dict(shape="box", size=(0.03, 0.008, 0.075), pose=(0.0, 0.021, 0.03, 0.0), color=CS66_GRIPPER_COLOR),
    "finger_r": dict(shape="box", size=(0.03, 0.008, 0.075), pose=(0.0, -0.021, 0.03, 0.0), color=CS66_GRIPPER_COLOR),
}

# ---------------------------------------------------------------- timings
GRIND_TIME = 3.0
TAMP_TIME = 2.0
SHOT_TIME = 4.0
RINSE_TIME = 2.0
DRAWER_TIME = 2.5
COLLECT_TIME = 2.0

GROUNDS_NONE, GROUNDS_LOOSE, GROUNDS_TAMPED = 0, 1, 2


# ---------------------------------------------------------------- shared helpers
def _ease(u):
    return u * u * (3.0 - 2.0 * u)


def _place(obj, spec, parent):
    """Put a part given in a parent frame (spec pose z = its bottom face)."""
    x, y, z, yaw = spec["pose"]
    obj.T = (parent * build.pose(x, y, z + spec["size"][-1] / 2, yaw)).A


def arc_coords(centre, P):
    """(angle_deg, r, z, yaw offset from radial) of pose P about `centre`."""
    x, y, z = P.t
    a = degrees(atan2(y - centre[1], x - centre[0]))
    yaw = degrees(atan2(P.R[1, 0], P.R[0, 0]))
    return a, hypot(x - centre[0], y - centre[1]), z, (yaw - a + 180.0) % 360.0 - 180.0


def arc_pose(centre, a, r, z, off=0.0):
    """Held object at angle a, r out from `centre`, height z, yawed radial + off."""
    x, y = layout.around(centre, a, r)
    return SE3(x, y, z) * SE3.Rz(radians(a + off))


def carry_around(pl, name, centre, flange_of, *waypoints, speed=CARRY_SPEED, avoid=None):
    """Carry the held object round `centre` through (angle_deg, r, z[, yaw offset])
    waypoints on arcs. The yaw offset blends too.

    Turns the shortest way round, or, given `avoid` (degrees), whichever way
    does not sweep through that direction. Keeping one side of a robot always
    clear stops its base joint winding up past its limit over a long routine.
    """
    poses = []
    a0, r0, z0, o0 = arc_coords(centre, pl.fk_held(pl.q))
    for wp in waypoints:
        a1, r1, z1 = wp[:3]
        o1 = wp[3] if len(wp) > 3 else 0.0
        da = (a1 - a0 + 180.0) % 360.0 - 180.0
        if avoid is not None and 0.0 < ((avoid - a0) % 360.0) * np.sign(da) % 360.0 < abs(da):
            da -= np.sign(da) * 360.0                  # the other way round
        length = hypot(radians(da) * (r0 + r1) / 2, r1 - r0, z1 - z0)
        n = max(pl._steps(length / speed), 10)
        for u in np.linspace(0.0, 1.0, n + 1)[1:]:
            s = _ease(u)
            poses.append(flange_of(arc_pose(centre, a0 + s * da, r0 + s * (r1 - r0),
                                            z0 + s * (z1 - z0), o0 + s * (o1 - o0))))
        a0, r0, z0, o0 = a0 + da, r1, z1, o1
    return pl.follow(name, poses)


# ---------------------------------------------------------------- UR3: cup moves
def cup_pose(x, y, z):
    """Cup standing upright at (x, y, z), turned so the UR3 grips it from its base side."""
    bx, by = layout.UR3_BASE
    return SE3(x, y, z) * SE3.Rz(atan2(y - by, x - bx))


def cup_flange(cup_T):
    return cup_T * GRASP_CUP.inv()


SPOTS = {
    "dispenser": cup_pose(*layout.DISPENSER_CUP),
    "espresso": cup_pose(*layout.ESPRESSO_CUP),
    "handoff": cup_pose(*layout.HANDOFF_CUP),
    "drawer": cup_pose(layout.DRAWER_X, layout.DRAWER_Y_IN, layout.DRAWER_TOP_Z),
}


def plan_runner(scene, checker, start, end, label="UR3"):
    """UR3 takes the cup from SPOTS[start] to SPOTS[end] and goes home."""
    a, b = SPOTS[start], SPOTS[end]
    U = layout.UR3_BASE
    pl = Planner(scene.robots["runner"], UR3_HOME_Q, checker, grasp=GRASP_CUP, held_T=a, name="runner")
    z_carry = layout.TABLE_TOP_Z + CARRY_LIFT
    back_a = BACK_OFF_LEVEL if start in OVERHEAD else BACK_OFF
    back_b = BACK_OFF_LEVEL if end in OVERHEAD else BACK_OFF
    a_ang, a_r, a_z, _ = arc_coords(U, a)
    b_ang, b_r, b_z, _ = arc_coords(U, b)

    pl.free_move(f"reach for cup ({start})", cup_flange(a) * back_a, 2.5)
    pl.line_move("grip cup", cup_flange(a), 0.8)
    pl.grip(True)
    if start in OVERHEAD:
        pl.line_move("lift cup", cup_flange(SE3.Tz(0.01) * a), 0.4)
        carry_around(pl, f"slide cup out ({start})", U, cup_flange, (a_ang, a_r - DRAW_OUT, a_z + 0.01))
    else:
        pl.line_move("lift cup", cup_flange(SE3.Tz(LIFT) * a), 0.8)
    if end in OVERHEAD:
        carry_around(pl, f"carry cup to {end}", U, cup_flange, (b_ang, b_r - DRAW_OUT, z_carry),
                     (b_ang, b_r - DRAW_OUT, b_z + 0.01), (b_ang, b_r, b_z + 0.01))
    else:
        carry_around(pl, f"carry cup to {end}", U, cup_flange, (b_ang, b_r, z_carry), (b_ang, b_r, b_z + LIFT))
    pl.line_move("set cup down", cup_flange(b), 0.8)
    pl.grip(False)
    pl.line_move("let go", cup_flange(b) * back_b, 0.6)
    pl.free_move("UR3 home", UR3_HOME_Q, 2.5)
    for seg in pl.segments:
        seg.name = f"{label}: {seg.name}"
    return pl.segments


# ---------------------------------------------------------------- CS66: the espresso
def pf_at(station, dr=0.0, dz=0.0, off=0.0):
    """Portafilter pose at a CS66 station, handle pointing at the CS66."""
    a, r = layout.cs66_station(station)
    return arc_pose(layout.CS66_BASE, a, r + dr, layout.PF_RIM_Z[station] + dz, off)


def pf_flange(P):
    return P * GRASP_PF.inv()


def plan_barista(scene, checker):
    """CS66: dock -> grinder -> tamper -> group head (shot) -> knock -> rinse -> dock."""
    C = layout.CS66_BASE
    pl = Planner(scene.robots["espresso"], CS66_HOME_Q, checker, grasp=GRASP_PF,
                 held_T=pf_at("pf_dock"), name="espresso")

    def go(name, *wps):
        # Never swing the portafilter round through the UR3's side of the CS66.
        return carry_around(pl, name, C, pf_flange, *wps, avoid=CS66_KEEP_CLEAR)

    def enter(station, dz=0.0, off=0.0):
        """Arc round at carry height, drop in front of the station, slide in."""
        a, r = layout.cs66_station(station)
        z = layout.PF_RIM_Z[station] + dz
        go(f"to the {station.replace('_', ' ')}", (a, r - PF_FRONT, PF_CARRY_Z, off),
           (a, r - PF_FRONT, z, off), (a, r, z, off))

    def leave(station, dz=0.0, off=0.0):
        a, r = layout.cs66_station(station)
        go(f"out of the {station.replace('_', ' ')}", (a, r - PF_FRONT, layout.PF_RIM_Z[station] + dz, off),
           (a, r - PF_FRONT, PF_CARRY_Z, off))

    def rows(duration):
        return pl._steps(duration) + 1

    # Pick the portafilter off its dock
    dock = pf_at("pf_dock")
    pl.free_move("reach for portafilter", pf_flange(dock) * ABOVE, 3.0)
    pl.line_move("grip portafilter", pf_flange(dock), 1.0)
    pl.grip(True)
    pl.line_move("lift off dock", pf_flange(SE3.Tz(0.08) * dock), 1.0)

    # Grind: grounds appear a third of the way in
    enter("grinder")
    n = rows(GRIND_TIME)
    pl.wait("grind coffee", GRIND_TIME, effects={
        "grind_stream": np.ones(n, bool),
        "grounds": np.where(np.arange(n) > n // 3, GROUNDS_LOOSE, GROUNDS_NONE)})
    leave("grinder")

    # Tamp: piston down into the basket and back up
    enter("tamper")
    n = rows(TAMP_TIME)
    stroke = np.sin(np.linspace(0.0, pi, n))
    pl.wait("tamp", TAMP_TIME, effects={
        "tamp": stroke, "grounds": np.where(np.arange(n) >= n // 2, GROUNDS_TAMPED, GROUNDS_LOOSE)})
    leave("tamper")

    # Lock into the group head: in level and turned, rise, twist the handle home
    enter("group_head", dz=-LOCK_RISE, off=LOCK_TWIST)
    pl.line_move("into the group head", pf_flange(pf_at("group_head", off=LOCK_TWIST)), 0.8)
    nt = pl._steps(1.2)
    pl.follow("twist to lock", [pf_flange(pf_at("group_head", off=LOCK_TWIST * (1 - _ease(i / nt))))
                                for i in range(1, nt + 1)])
    n = rows(SHOT_TIME)
    pl.wait("pull espresso shot", SHOT_TIME, effects={
        "shot_stream": np.ones(n, bool),
        "espresso_level": np.linspace(0.0, milk_scene.layout.ESPRESSO_H, n)})
    pl.follow("twist to unlock", [pf_flange(pf_at("group_head", off=LOCK_TWIST * _ease(i / nt)))
                                  for i in range(1, nt + 1)])
    pl.line_move("down out of the group head",
                 pf_flange(pf_at("group_head", dz=-LOCK_RISE, off=LOCK_TWIST)), 0.8)
    leave("group_head", dz=-LOCK_RISE, off=LOCK_TWIST)

    # Knock the spent puck out: two sharp taps over the knock box
    a, r = layout.cs66_station("knock")
    go("to the knock box", (a, r, PF_CARRY_Z), (a, r, layout.PF_RIM_Z["knock"]))
    top, low = pf_at("knock"), pf_at("knock", dz=-KNOCK_DROP)
    taps = [low, top, low, top]
    poses = []
    for P0, P1 in zip([top] + taps[:-1], taps):
        poses += [pf_flange(P0.interp(P1, s)) for s in (0.5, 1.0)]
    n = len(poses) + 1
    pl.follow("knock out the puck", poses, effects={
        "grounds": np.where(np.arange(n) >= 2, GROUNDS_NONE, GROUNDS_TAMPED)})
    go("up from the knock box", (a, r, PF_CARRY_Z))

    # Rinse under the spray
    enter("pf_wash")
    pl.wait("rinse portafilter", RINSE_TIME, effects={"pf_rinse": np.ones(rows(RINSE_TIME), bool)})
    leave("pf_wash")

    # Back on the dock
    a, r = layout.cs66_station("pf_dock")
    go("to the dock", (a, r, PF_CARRY_Z), (a, r, dock.t[2] + 0.08))
    pl.line_move("set on dock", pf_flange(dock), 1.0)
    pl.grip(False)
    pl.line_move("let go of portafilter", pf_flange(dock) * ABOVE, 0.8)
    pl.free_move("CS66 home", CS66_HOME_Q, 3.0)
    for seg in pl.segments:
        seg.name = f"CS66: {seg.name}"
    return pl.segments


# ---------------------------------------------------------------- collision checkers
def label_obstacles():
    """The upright station signs, as boxes (flat ones lie on the table)."""
    cam = layout.CAMERA["position"]
    out = []
    for text, at, flat in layout.LABELS:
        if not flat:
            _, _, T, size = labels.layout_sign(text, at, cam, layout.LABEL_HEIGHT)
            out.append(Box(f"{text.lower()} sign", T.A, size / 2))
    return out


def static_obstacles():
    piston = layout.TAMPER_PISTON
    a, r = layout.cs66_station("tamper")
    return [box_from_spec(n, s) for n, s in layout.PLACEHOLDERS.items()
            if not n.startswith(layout.NOT_OBSTACLES)] + [
        box_from_spec("tamper_piston", dict(shape="cylinder", size=(piston["radius"], piston["height"]),
                                            pose=(*layout.around(layout.CS66_BASE, a, r), piston["up_z"], 0.0)))
    ] + label_obstacles()


def _waiting(scene):
    """Each robot's capsules frozen at home, and what it leaves standing about."""
    aubo = milk_scene.make_checker(scene.robots["milk"], obstacles=[])
    ur3 = CollisionChecker(scene.robots["runner"], [], CUP_CAPSULE, model=UR3_MODEL)
    barista = CollisionChecker(scene.robots["espresso"], [], layout.PF_CAPSULES, held_name="portafilter",
                               model=CS66_MODEL)
    return dict(
        milk=frozen(aubo, milk_scene.HOME_Q, "AUBO"),
        runner=frozen(ur3, UR3_HOME_Q, "UR3"),
        espresso=frozen(barista, CS66_HOME_Q, "CS66"),
        pitcher=aubo.resting_box(milk_scene.pitcher_home()),
        portafilter=barista.resting_box(pf_at("pf_dock")),
    )


def make_checkers(scene):
    w = _waiting(scene)
    cup_at = lambda spot: box_from_spec("cup", dict(shape="cylinder", size=(layout.CUP_R, layout.CUP_H),
                                                    pose=(*SPOTS[spot].t, 0.0)))
    base = static_obstacles()
    return dict(
        # UR3: the AUBO and CS66 wait at home, pitcher and portafilter on their docks.
        runner=CollisionChecker(scene.robots["runner"], base + [w["pitcher"], w["portafilter"]],
                                CUP_CAPSULE, held_name="cup", others=w["milk"] + w["espresso"],
                                model=UR3_MODEL),
        # AUBO: the cup waits at the hand-off.
        milk=milk_scene.make_checker(scene.robots["milk"], obstacles=base + [cup_at("handoff"), w["portafilter"]],
                                     others=w["runner"] + w["espresso"]),
        # CS66: the cup waits under the group head; the portafilter seats in
        # the group head and dock by design.
        espresso=CollisionChecker(scene.robots["espresso"], base + [cup_at("espresso"), w["pitcher"]],
                                  layout.PF_CAPSULES, held_name="portafilter",
                                  held_skip={"group_head", "pf_dock"},
                                  others=w["milk"] + w["runner"], model=CS66_MODEL),
    )


# ---------------------------------------------------------------- planning orders
GRASPS = {"runner": GRASP_CUP, "milk": milk_scene.GRASP, "espresso": GRASP_PF}
HOMES = {"runner": UR3_HOME_Q, "milk": milk_scene.HOME_Q, "espresso": CS66_HOME_Q}
LABELS = {"runner": "UR3", "milk": "AUBO", "espresso": "CS66"}
# Who keeps moving when two robots would clash: the CS66 holds the most
# delicate thing (a locked-in portafilter), the AUBO's steaming can always wait.
PRIORITY = {"espresso": 3, "runner": 2, "milk": 1}
CUP_POOL = 5                 # cups that can be in flight at once (one per order)


def event(name, duration, **effects):
    """A segment where no robot moves, only scene effects play."""
    n = max(int(round(duration / DT)), 1)
    return Segment(name, np.zeros((n + 1, 0)), robot="", effects=effects)


def _cut(segs, name, keep=True):
    """Split segs at the segment called `name`: (up to and including it, the
    rest), or with keep=False (before it, from it on)."""
    for i, seg in enumerate(segs):
        if seg.name == name:
            i += 1 if keep else 0
            return segs[:i], segs[i:]
    raise KeyError(f"no segment called {name!r}")


def _drawer_segments():
    n = int(DRAWER_TIME / DT)
    out = np.array([_ease(u) for u in np.linspace(0, 1, n + 1)])
    return [event("drawer out to customer", DRAWER_TIME, drawer=out),
            event("customer takes the drink", COLLECT_TIME, drawer=1.0),
            event("drawer back in", DRAWER_TIME, drawer=out[::-1], cup_gone=True)]


# Plans are saved here between runs, in a file named after a hash of the
# barista code: change any of it and the old plans are simply not used.
PLAN_CACHE = Path(tempfile.gettempdir()) / "barista_plans"


def _code_hash():
    h = hashlib.md5()
    for f in sorted(Path(__file__).parent.glob("*.py")):
        h.update(f.read_bytes())
    return h.hexdigest()[:12]


class OrderPlanner:
    """Plans every robot's moves once per drink, then schedules a batch of
    orders so the robots work at the same time.

    Who waits for what, for order i:

        dispenser   drops cup i once cup i-1 has left it
        UR3         cup i: dispenser -> group head            -> "cup at group head"
        CS66        grinds and tamps straight away, waits for "cup at group head",
                    locks in, pulls the shot, unlocks          -> "portafilter out"
                    then knocks, rinses and docks while the others carry on
        AUBO        steams straight away, waits for "cup at hand-off",
                    pours                                     -> "pour done"
                    then rinses and refills while the others carry on
        UR3         waits for "portafilter out": cup i -> hand-off (or, for an
                    espresso, -> drawer); waits for "pour done": -> drawer
        drawer      waits for the drink, slides out and back -> "drawer back"

    The UR3 does its trips for consecutive orders interleaved (fetch cup i+1
    before taking drink i to the drawer), so the next order starts while the
    last one is still being finished.
    """

    def __init__(self, scene, use_cache=True):
        self.scene = scene
        self.checkers = make_checkers(scene)
        self.parts = {}
        self.timelines = {}
        self.lock = threading.RLock()     # the warm-up and an order may plan at once
        self.cache_file = PLAN_CACHE / f"plans_{_code_hash()}.pkl" if use_cache else None
        self._load()

    # ---- plans saved between runs
    def _load(self):
        if self.cache_file is None or not self.cache_file.exists():
            return
        try:
            with open(self.cache_file, "rb") as f:
                self.parts, self.timelines = pickle.load(f)
        except Exception as e:                  # a stale or broken file: just re-plan
            print(f"  ignoring saved plans ({e})")
            self.parts, self.timelines = {}, {}

    def _save(self):
        if self.cache_file is None:
            return
        self.cache_file.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.cache_file.with_suffix(".tmp")
        with open(tmp, "wb") as f:
            pickle.dump((self.parts, self.timelines), f, protocol=pickle.HIGHEST_PROTOCOL)
        tmp.replace(self.cache_file)

    def _part(self, key, make):
        if key not in self.parts:
            self.parts[key] = make()
        return self.parts[key]

    def runner(self, a, b):
        return self._part(("runner", a, b), lambda: plan_runner(self.scene, self.checkers["runner"], a, b))

    def barista(self):
        return self._part("barista", lambda: plan_barista(self.scene, self.checkers["espresso"]))

    def milk(self, profile):
        return self._part(("milk", profile), lambda: milk_scene.plan_task(
            self.scene.robots["milk"], profile, checker=self.checkers["milk"]))

    def _capsules(self, robot):
        checker, grasp, arm = self.checkers[robot], GRASPS[robot], self.scene.robots[robot]

        def caps(seg, k):
            q = seg.qs[k]
            return checker.capsules(q, seg.holding, arm.fkine(q) * grasp if seg.holding else None)
        return caps

    def sequential_time(self, drinks):
        """How long the batch takes with one robot moving at a time."""
        total = 0.0
        for drink in drinks:
            profile = MENU[drink]
            segs = self.runner("dispenser", "espresso") + self.barista()
            if profile is None:
                segs += self.runner("espresso", "drawer")
            else:
                segs += self.runner("espresso", "handoff") + self.milk(profile) + self.runner("handoff", "drawer")
            total += sum(s.duration for s in segs) + 0.5 + 2 * DRAWER_TIME + COLLECT_TIME
        return total

    def lanes(self, drinks):
        dispense = event("dispense a cup", 0.5, dispense_cup=True)
        drawer_segs = _drawer_segments()
        has_milk = [MENU[d] is not None for d in drinks]
        steps = lambda segs, i: [Step(seg, i) for seg in segs]
        disp, cs, aubo, drawer = [], [], [], []

        for i, drink in enumerate(drinks):
            if i:
                disp.append(Wait(f"{i - 1}:cup at group head"))
            disp += [Step(dispense, i), Signal(f"{i}:cup ready")]

            pre, rest = _cut(self.barista(), "CS66: to the group head", keep=False)
            locked, post = _cut(rest, "CS66: out of the group head")
            cs += steps(pre, i) + [Wait(f"{i}:cup at group head")] + steps(locked, i) \
                + [Signal(f"{i}:portafilter out")] + steps(post, i)

            if has_milk[i]:
                pre, rest = _cut(self.milk(MENU[drink]), "carry to cup", keep=False)
                pour, post = _cut(rest, "carry to wash")
                aubo += steps(pre, i) + [Wait(f"{i}:cup at hand-off")] + steps(pour, i) \
                    + [Signal(f"{i}:pour done")] + steps(post, i)

            drawer += [Wait(f"{i}:drink in drawer")] + steps(drawer_segs, i) + [Signal(f"{i}:drawer back")]

        def trip(a, b, i, waits, signal):
            head, tail = _cut(self.runner(a, b), "UR3: let go")
            return [Wait(w) for w in waits] + steps(head, i) + [Signal(signal)] + steps(tail, i)

        def drawer_free(i):
            return [f"{i - 1}:drawer back"] if i else []

        def fetch(i):
            return trip("dispenser", "espresso", i, [f"{i}:cup ready"], f"{i}:cup at group head")

        def onward(i):
            if has_milk[i]:
                return trip("espresso", "handoff", i, [f"{i}:portafilter out"], f"{i}:cup at hand-off")
            return trip("espresso", "drawer", i, [f"{i}:portafilter out"] + drawer_free(i), f"{i}:drink in drawer")

        def deliver(i):
            return trip("handoff", "drawer", i, [f"{i}:pour done"] + drawer_free(i), f"{i}:drink in drawer")

        ur3 = fetch(0) + onward(0)
        for i in range(1, len(drinks)):
            ur3 += fetch(i)
            if has_milk[i - 1]:
                ur3 += deliver(i - 1)
            ur3 += onward(i)
        if has_milk[-1]:
            ur3 += deliver(len(drinks) - 1)

        def robot(name, program):
            home = Segment(f"{LABELS[name]} home", np.asarray(HOMES[name], float)[None], robot=name)
            return Lane(name, program, home=home, capsules=self._capsules(name), priority=PRIORITY[name])

        return [Lane("dispenser", disp), Lane("drawer", drawer),
                robot("runner", ur3), robot("espresso", cs), robot("milk", aubo)]

    def __call__(self, drinks):
        key = tuple(drinks)
        with self.lock:
            if key not in self.timelines:
                self.timelines[key] = schedule(self.lanes(list(drinks)))
                self._save()
            return self.timelines[key]

    def ready(self, drinks):
        """True if this batch is already planned (no wait to start it)."""
        return tuple(drinks) in self.timelines


# ---------------------------------------------------------------- scene
def _effect(radius, height, color):
    return build.make_placeholder(dict(shape="cylinder", size=(radius, height), pose=(0, 0, 0, 0), color=color))


def build_scene(env=None):
    scene = build.build_scene(layout, env)
    scene.robots["runner"].q = UR3_HOME_Q
    scene.robots["espresso"].q = CS66_HOME_Q
    milk_scene.add_station_parts(scene, env)

    scene.runner_gripper = {n: build.make_placeholder(s) for n, s in UR3_GRIPPER_PARTS.items()}
    scene.barista_gripper = {n: build.make_placeholder(s) for n, s in CS66_GRIPPER_PARTS.items()}
    scene.portafilter = {n: build.make_placeholder(s) for n, s in layout.PORTAFILTER_PARTS.items()}
    scene.pf_pose = pf_at("pf_dock")
    # One cup per order in flight; order i uses cups[i % CUP_POOL].
    scene.cups = [Cup(layout.CUP_R, layout.CUP_H, SPOTS["dispenser"]) for _ in range(CUP_POOL)]
    for cup in scene.cups:
        cup.set(visible=False)
    scene.drawer_s = 0.0
    scene.grounds = GROUNDS_NONE

    coffee = (0.30, 0.18, 0.08, 1)
    piston = layout.TAMPER_PISTON
    scene.fx = dict(
        grounds_loose=_effect(0.032, 0.014, (0.50, 0.33, 0.20, 1)),
        grounds_tamped=_effect(0.032, 0.010, (0.28, 0.17, 0.09, 1)),
        grind_stream=_effect(0.006, layout.GRINDER_CHUTE_Z - layout.PF_RIM_Z["grinder"], (0.50, 0.33, 0.20, 1)),
        tamper_piston=_effect(piston["radius"], piston["height"], (0.55, 0.55, 0.58, 1)),
        shot_stream=_effect(0.003, 0.16, coffee),
        pf_rinse=_effect(0.008, layout.PF_SHOWER_Z - layout.PF_RIM_Z["pf_wash"], (0.55, 0.75, 0.95, 0.6)),
    )
    # Nested cups waiting in the dispenser tube.
    scene.stack = layout.CUP_STACK
    scene.stack_cups = [geometry.Cylinder(radius=layout.CUP_R, length=layout.CUP_H, color=GLASS)
                        for _ in range(layout.CUP_STACK)]
    lamp = dict(shape="cylinder", size=(0.035, 0.06), pose=(0, 0, 0, 0))
    scene.beacon = {c: build.make_placeholder(dict(lamp, color=rgba)) for c, rgba in
                    (("green", (0.1, 0.9, 0.2, 1)), ("amber", (1.0, 0.65, 0.0, 1)), ("red", (1.0, 0.1, 0.1, 1)))}
    scene.labels = [shape for text, at, flat in layout.LABELS
                    for shape in labels.make_sign(text, at, layout.CAMERA["position"],
                                                  layout.LABEL_HEIGHT, flat)]

    _update_runner(scene, holding=False)
    _update_barista(scene, holding=False)
    _set_drawer(scene, 0.0)
    _set_tamper(scene, 0.0)
    _update_stack(scene)
    for name in STREAMS:
        scene.fx[name].T = HIDDEN.A
    set_beacon(scene, "green")

    if env is not None:
        for obj in (*scene.runner_gripper.values(), *scene.barista_gripper.values(),
                    *scene.portafilter.values(), *scene.fx.values(), *scene.stack_cups,
                    *scene.beacon.values(), *scene.labels):
            env.add(obj)
        for cup in scene.cups:
            cup.add_to_env(env)
    return scene


def _drawer_y(s):
    return layout.DRAWER_Y_IN + s * (layout.DRAWER_Y_OUT - layout.DRAWER_Y_IN)


def _set_drawer(scene, s):
    scene.drawer_s = s
    scene.props["drawer_tray"].T = build.pose(layout.DRAWER_X, _drawer_y(s), layout.DRAWER_Z, 0.0).A


def _set_tamper(scene, s):
    """Piston travel: 0 up, 1 pressed into the basket."""
    piston = layout.TAMPER_PISTON
    a, r = layout.cs66_station("tamper")
    x, y = layout.around(layout.CS66_BASE, a, r)
    z = piston["up_z"] - s * layout.TAMPER_STROKE
    scene.fx["tamper_piston"].T = SE3(x, y, z + piston["height"] / 2).A


def _update_stack(scene):
    x, y = layout.DISPENSER_AT
    for i, cup in enumerate(scene.stack_cups):
        z = layout.DISPENSER_TUBE_Z + 0.005 + i * layout.CUP_NEST + layout.CUP_H / 2
        cup.T = SE3(x, y, z).A if i < scene.stack else HIDDEN.A


def _update_runner(scene, holding, cup=None):
    T = scene.robots["runner"].fkine(scene.robots["runner"].q)
    for name, spec in UR3_GRIPPER_PARTS.items():
        _place(scene.runner_gripper[name], spec, T)
    if holding and cup is not None:
        cup.set(T=T * GRASP_CUP)


def _draw_grounds(scene):
    P = scene.pf_pose
    scene.fx["grounds_loose"].T = ((P * SE3.Tz(-0.007)).A if scene.grounds == GROUNDS_LOOSE else HIDDEN.A)
    scene.fx["grounds_tamped"].T = ((P * SE3.Tz(-0.017)).A if scene.grounds == GROUNDS_TAMPED else HIDDEN.A)


def _update_barista(scene, holding):
    T = scene.robots["espresso"].fkine(scene.robots["espresso"].q)
    for name, spec in CS66_GRIPPER_PARTS.items():
        _place(scene.barista_gripper[name], spec, T)
    if holding:
        scene.pf_pose = T * GRASP_PF
    for name, spec in layout.PORTAFILTER_PARTS.items():
        _place(scene.portafilter[name], spec, scene.pf_pose)
    _draw_grounds(scene)


def set_beacon(scene, colour):
    x, y, z = layout.BEACON_AT
    for c, lamp in scene.beacon.items():
        lamp.T = SE3(x, y, z + 0.03).A if c == colour else HIDDEN.A


def _row(value, k):
    """Value of a per-row effect at row k (scalars apply to every row)."""
    v = np.atleast_1d(value)
    return v[min(k, len(v) - 1)]


def _stream(scene, name, on, top, length):
    """Show a vertical stream hanging down from point `top`."""
    scene.fx[name].T = SE3(top[0], top[1], top[2] - length / 2).A if on else HIDDEN.A


STREAMS = ("grind_stream", "shot_stream", "pf_rinse")                  # espresso station
MILK_STREAMS = ("milk_stream", "dispenser_stream", "rinse_spray")      # milk station


def apply_frame(scene, frame):
    """Put the scene in the state of one Timeline frame: move the robots that
    are moving, then show every effect the moving lanes ask for."""
    fx = {}                                   # effect -> (value, order)
    for lane, (step, k, active) in frame.items():
        if not active:
            continue
        seg, cup = step.seg, scene.cups[step.order % CUP_POOL]
        if seg.robot:
            scene.robots[seg.robot].q = seg.qs[k]
        if seg.robot == "milk":
            milk_scene.update_attached(scene, seg.holding)
            if seg.cup_level is not None:
                fx["cup_level"] = (seg.cup_level[k], step.order)
        elif seg.robot == "runner":
            _update_runner(scene, seg.holding, cup)
        elif seg.robot == "espresso":
            _update_barista(scene, seg.holding)
        for name, value in seg.effects.items():
            if name == "dispense_cup" and k:
                continue                      # only on the first row
            if name == "cup_gone" and not k:
                continue
            fx[name] = (_row(value, k), step.order)
    apply_effects(scene, fx)


def apply_effects(scene, fx):
    # A stream only shows while a moving segment asks for it this frame.
    for name in STREAMS:
        if name not in fx:
            scene.fx[name].T = HIDDEN.A
    milk_scene.set_effects(scene, {n: bool(v) for n, (v, _) in fx.items() if n in MILK_STREAMS})
    cup = lambda name: scene.cups[fx[name][1] % CUP_POOL]
    val = lambda name: fx[name][0]

    # ---- espresso station
    if "grounds" in fx:
        scene.grounds = int(val("grounds"))
        _draw_grounds(scene)
    if "grind_stream" in fx:
        a, r = layout.cs66_station("grinder")
        _stream(scene, "grind_stream", bool(val("grind_stream")),
                (*layout.around(layout.CS66_BASE, a, r), layout.GRINDER_CHUTE_Z),
                layout.GRINDER_CHUTE_Z - layout.PF_RIM_Z["grinder"])
    if "tamp" in fx:
        _set_tamper(scene, float(val("tamp")))
    if "shot_stream" in fx:
        spout = (scene.pf_pose * SE3.Tz(-layout.PF_BASKET_H - layout.PF_SPOUT_H)).t
        _stream(scene, "shot_stream", bool(val("shot_stream")), spout, 0.16)
    if "pf_rinse" in fx:
        a, r = layout.cs66_station("pf_wash")
        _stream(scene, "pf_rinse", bool(val("pf_rinse")),
                (*layout.around(layout.CS66_BASE, a, r), layout.PF_SHOWER_Z),
                layout.PF_SHOWER_Z - layout.PF_RIM_Z["pf_wash"])

    # ---- cups and the drawer
    if "dispense_cup" in fx:
        scene.stack -= 1
        if scene.stack < 0:                    # tube empty: staff reload it
            scene.stack = layout.CUP_STACK - 1
            print("  cup dispenser refilled")
        _update_stack(scene)
        cup("dispense_cup").set(T=SPOTS["dispenser"], espresso=0.0, level=0.0, visible=True)
    if "espresso_level" in fx:
        lvl = float(val("espresso_level"))
        cup("espresso_level").set(espresso=lvl, level=lvl)
    if "cup_level" in fx:
        cup("cup_level").set(level=float(val("cup_level")))
    if "drawer" in fx:
        _set_drawer(scene, float(val("drawer")))
        if "cup_gone" in fx:
            cup("drawer").set(visible=False)
        else:
            cup("drawer").set(T=SE3(layout.DRAWER_X, _drawer_y(scene.drawer_s), layout.DRAWER_TOP_Z))


def describe_frame(frame):
    """What each robot is doing in a frame, for the status label."""
    doing = [step.seg.name if step.seg.name.startswith(LABELS[lane]) else f"{LABELS[lane]}: {step.seg.name}"
             for lane, (step, k, active) in frame.items() if active and lane in LABELS]
    return " | ".join(doing) or "waiting"


# ---------------------------------------------------------------- the cell (kiosk + e-stops)
# Rough times until a drink can be collected, used for estimates until the real
# schedule for that batch exists. Replaced by planned figures as they come in.
TYPICAL_READY = {"milk": 72.0, "espresso": 54.0}
EXTRA_PER_DRINK = 62.0       # each further drink in a batch: about one CS66 cycle


@dataclass
class Order:
    id: int
    drink: str
    status: str = "queued"
    eta: float = None         # seconds until it can be collected
    estimate: bool = True     # eta is a rough guess, not from a schedule


def ready_frames(timeline):
    """For each order index in a batch: (frame the drawer is fully out, frame it
    starts back in) - when the drink can be collected, and when it was."""
    out = {}
    for f, frame in enumerate(timeline.frames):
        entry = frame.get("drawer")
        if entry and entry[2]:
            step = entry[0]
            ready, gone = out.get(step.order, (None, None))
            if step.seg.name == "customer takes the drink" and ready is None:
                ready = f
            if step.seg.name == "drawer back in" and gone is None:
                gone = f
            out[step.order] = (ready, gone)
    return out


class Cell:
    """Order queue, e-stop latch and the order screens around a built scene.

    Orders waiting when the robots are free are made together as one batch,
    overlapped (see OrderPlanner); orders placed while a batch is being made go
    into the next batch. Screens (the Swift tablet buttons, the desktop
    KioskWindow) read snapshot() and call order() / press_estop().
    """

    def __init__(self, scene, env, planner=None):
        self.scene, self.env = scene, env
        self.plan = planner or OrderPlanner(scene)
        self.queue = deque()          # Orders not yet started
        self.active = []              # Orders in the batch being made
        self.done = deque(maxlen=4)   # recently collected, still shown
        self.next_id = 1
        self.estop = None             # which e-stop is latched, or None
        self.state = "ready"
        self.batch_left = 0.0         # seconds until the current batch finishes
        self.windows = []             # things to update every step
        self.warming = False          # warm_up() still planning drinks
        self.status = self.queue_label = None
        self._shown = None

    @staticmethod
    def menu():
        return list(MENU)

    # ---- screens
    def add_ui(self):
        """Tablet buttons and status inside the Swift page."""
        import swift

        self.status = swift.Label("Status: ready")
        self.queue_label = swift.Label("Queue: empty")
        self.env.add(swift.Label("Tablet - order a drink:"))
        for drink in MENU:
            self.env.add(swift.Button(lambda _e, d=drink: self.order(d), desc=drink.replace("_", " ").title()))
        self.env.add(self.status)
        self.env.add(self.queue_label)
        self.env.add(swift.Button(lambda _e: self.press_estop("inside the cell"), desc="E-STOP (inside cell)"))
        self.env.add(swift.Button(lambda _e: self.press_estop("outside the cell"), desc="E-STOP (outside)"))
        self.env.add(swift.Button(lambda _e: self.reset_estop(), desc="Reset e-stop"))

    def warm_up(self):
        """Plan every drink on its own in the background, so the first order of
        each is ready to go (and its typical time is known)."""
        def work():
            for drink in MENU:
                try:
                    self.plan([drink])
                except PlanningError as e:
                    print(f"  could not plan {drink}: {e}")
            self.warming = False
        todo = [d for d in MENU if not self.plan.ready([d])]
        if todo:
            self.warming = True
            print(f"  planning {len(todo)} drinks in the background (saved for next time)...")
            threading.Thread(target=work, daemon=True).start()

    def add_window(self):
        """The desktop ordering window (kiosk_window.py)."""
        from barista.kiosk_window import KioskWindow
        window = KioskWindow(self)
        self.windows.append(window)
        return window

    def beacon(self):
        return "red" if self.estop else ("green" if self.state == "ready" else "amber")

    def status_text(self):
        if self.estop:
            return f"E-STOP pressed ({self.estop}) - press Reset to resume"
        if self.warming and self.state == "ready":
            return "Ready (still planning some drinks in the background)"
        return self.state[:1].upper() + self.state[1:]

    def _show(self):
        text = f"Status: {self.status_text()}"
        names = ", ".join(o.drink.replace("_", " ") for o in self.queue) or "empty"
        if (text, names) == self._shown:
            return
        self._shown = (text, names)
        if self.status is not None:
            self.status.desc = text
            self.queue_label.desc = f"Queue: {names}"
        set_beacon(self.scene, self.beacon())

    # ---- estimates
    def typical(self, drink):
        """Seconds from ordering to collection for one drink on its own."""
        tl = self.plan.timelines.get((drink,))
        if tl is not None:
            ready, _ = ready_frames(tl)[0]
            return ready * DT
        return TYPICAL_READY["espresso" if MENU[drink] is None else "milk"]

    def _estimate_queue(self):
        """Rough ready times for queued orders: after the current batch, the
        next batch takes the first drink's own time plus about one CS66 cycle
        for each drink after it."""
        extra = sum(seg.duration for seg in self.plan.parts["barista"]) if "barista" in self.plan.parts \
            else EXTRA_PER_DRINK
        for i, o in enumerate(self.queue):
            o.eta = self.batch_left + self.typical(o.drink) + i * extra
            o.estimate = True

    def snapshot(self):
        """Everything a screen shows, as plain data."""
        self._estimate_queue()
        rows = [dict(id=o.id, drink=o.drink, status=o.status, eta=o.eta, estimate=o.estimate)
                for o in [*self.done, *self.active, *self.queue]]
        return dict(orders=rows, status=self.status_text(), beacon=self.beacon(),
                    typical={d: self.typical(d) for d in MENU})

    # ---- controls
    def order(self, drink):
        o = Order(self.next_id, drink)
        self.next_id += 1
        self.queue.append(o)
        print(f"  order #{o.id}: {drink}")
        self._show()
        return o

    def press_estop(self, where):
        if self.estop is None:
            print(f"  E-STOP pressed {where}: all robots halted")
        self.estop = where
        self._show()

    def reset_estop(self):
        if self.estop is not None:
            print("  e-stop reset: resuming")
        self.estop = None
        self._show()

    # ---- running
    def _tick(self):
        self.env.step(DT)
        for w in self.windows:
            w.update()

    def step(self):
        self._tick()
        while self.estop is not None and not self.closed():   # frozen: keep the screens alive
            self._tick()

    def closed(self):
        return any(w.closed for w in self.windows)

    def make(self, orders):
        drinks = [o.drink for o in orders]
        names = ", ".join(d.replace("_", " ") for d in drinks)
        self.active = list(orders)
        for o in orders:
            o.status = "planning"
        self.state = f"planning {names}..."
        self._show()

        # Plan in the background so the screens stay live meanwhile.
        result = {}

        def work():
            try:
                result["timeline"] = self.plan(drinks)
            except PlanningError as e:
                result["error"] = e
        worker = threading.Thread(target=work, daemon=True)
        worker.start()
        while worker.is_alive():
            if self.closed():
                return                        # window closed: give up (the thread is a daemon)
            self.step()
            worker.join(timeout=DT)           # give the planner the CPU between frames
        if "error" in result:
            print(f"  cannot make {names}: {result['error']}")
            for o in orders:
                o.status, o.eta = "could not be made", None
            self.done.extend(orders)
            self.active = []
            self.state = "ready"
            self._show()
            return

        timeline = result["timeline"]
        marks = ready_frames(timeline)
        total = len(timeline.frames)
        print(f"  making {names}: {timeline.duration:.0f} s")
        for f, frame in enumerate(timeline.frames):
            if self.closed():
                return
            self.batch_left = (total - f) * DT
            for i, o in enumerate(orders):
                ready, gone = marks[i]
                o.estimate = False
                if f < ready:
                    o.status, o.eta = "being made", (ready - f) * DT
                elif f < gone:
                    o.status, o.eta = "ready - collect from the drawer", 0.0
                else:
                    o.status, o.eta = "collected", None
            for o in [o for o in self.active if o.status == "collected"]:
                self.active.remove(o)
                self.done.append(o)
            self.state = f"making {names}: {describe_frame(frame)}"
            self._show()
            apply_frame(self.scene, frame)
            self.step()
        self.done.extend(self.active)
        self.active = []
        self.batch_left = 0.0
        self.state = "ready"
        self._show()

    def run(self, stop=None):
        """Serve orders until `stop` is set or a window closes (or, with
        neither, until the queue is empty)."""
        def keep_going():
            if self.closed():
                return False
            return not stop.is_set() if stop is not None else (bool(self.queue) or bool(self.windows))
        while keep_going():
            if self.queue:
                orders = list(self.queue)
                self.queue.clear()
                self.make(orders)
            else:
                self.step()


def animate(scene, env, cycles=1):
    """Hook used by barista.launch: make `cycles` lattes in one batch, no tablet."""
    cell = Cell(scene, env)
    for _ in range(max(int(cycles), 0)):
        cell.order("latte")
    cell.run()


def check_reach(scene):
    return build.check_reach(scene, layout)


# ---------------------------------------------------------------- CLI
def _report(planner, drinks):
    tl = planner(drinks)
    seq = planner.sequential_time(drinks)
    waits = ", ".join(f"{LABELS.get(n, n)} paused {v['paused']} s" for n, v in tl.stats.items()
                      if v["paused"] > 0) or "no robot had to pause"
    print(f"OK    {' + '.join(drinks):34s} {tl.duration:6.1f} s  (one at a time: {seq:5.1f} s)  "
          f"closest robots {tl.min_gap * 1000:.0f} mm")
    print(f"        {waits}")


def _check_all(batch=("latte", "espresso", "cappuccino")):
    scene = build_scene()
    planner = OrderPlanner(scene, use_cache=False)   # always plan fresh: this is the check
    ok = True
    for drinks in [[d] for d in MENU] + [list(batch)]:
        try:
            _report(planner, drinks)
        except PlanningError as e:
            ok = False
            print(f"FAIL  {' + '.join(drinks):34s} {e}")
    return ok


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("-o", "--order", action="append", choices=sorted(MENU), default=[],
                        help="queue an order at start-up (repeatable; queued orders are made together)")
    parser.add_argument("--no-window", action="store_true",
                        help="no desktop order window (order from the Swift tablet buttons instead)")
    parser.add_argument("--check", action="store_true",
                        help="plan and schedule every drink, and a batch, without Swift and exit")
    args = parser.parse_args(argv)
    if args.check:
        sys.exit(0 if _check_all() else 1)

    import swift

    env = swift.Swift()
    env.launch(realtime=True)
    scene = build_scene(env)
    env.set_camera_pose(list(layout.CAMERA["position"]), list(layout.CAMERA["look_at"]))
    cell = Cell(scene, env)
    cell.add_ui()
    for drink in args.order:
        cell.order(drink)
    env.step(DT)

    stop = threading.Event()
    if args.no_window:
        threading.Thread(target=lambda: (input("Kiosk running - order on the tablet in Swift. "
                                               "Press ENTER here to quit.\n"), stop.set()),
                         daemon=True).start()
    else:
        cell.add_window()
        print("Kiosk running - order in the 'Barista robots' window. Close it to quit.")
    cell.warm_up()
    cell.run(stop)
    env.close()


if __name__ == "__main__":
    main()
