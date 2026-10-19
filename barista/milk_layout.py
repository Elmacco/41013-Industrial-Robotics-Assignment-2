"""
Milk station layout (AUBO i5): fill the pitcher, steam it, pour it into a cup,
then rinse the pitcher and put it back to be refilled.

World frame: origin on the floor under the AUBO base, +X straight out from the
robot, +Y to its left, +Z up. Units are metres and radians.

The stations sit on an arc STATION_R out from the base, so the arm works with
its elbow open rather than folded up near its own base. Going round the arc:

    steamer (-50 deg) -> cup (0) -> wash (+50) -> milk dispenser (+100)

The two tall machines are at the ends and the low cup and wash basin in the
middle, so the full pitcher can be carried from the dispenser to the steamer
over the low ones.

Everything here is a placeholder primitive (box or cylinder) until real meshes
are made. The static station goes in PLACEHOLDERS like the other layouts; the
parts that move (the pitcher and the gripper) are described in their own local
frames in PITCHER_PARTS / GRIPPER_PARTS, and milk_scene.py builds them.

Run ``python -m barista.milk_layout`` to print where every object ends up.
"""
from math import cos, radians, sin

from barista import build

TABLE_TOP_Z = 0.75
TABLE_SIZE = (1.55, 2.05)
TABLE_CENTRE = (0.30, 0.05)

# ---------------------------------------------------------------- robots
ROBOTS = {
    "milk": dict(model="AuboI5", base=(0.0, 0.0, TABLE_TOP_Z, 0.0),
                 role="Fill, steam and pour milk; rinse the pitcher"),
}

PROPS = {}

# ---------------------------------------------------------------- stations on the arc
STATION_R = 0.60
ANGLE = dict(steamer=-50.0, cup=0.0, wash=50.0, dispenser=100.0)   # degrees


def polar(angle_deg, r, tangential=0.0):
    """(x, y) at angle_deg round the base, r out from it, shifted sideways."""
    a = radians(angle_deg)
    return (r * cos(a) - tangential * sin(a), r * sin(a) + tangential * cos(a))


def _facing(angle_deg, r, z, size, shape="box", color=(0.25, 0.25, 0.27, 1)):
    """Placeholder centred r out along angle_deg, turned to face the robot."""
    return dict(shape=shape, size=size, pose=(*polar(angle_deg, r), z, radians(angle_deg)),
                color=color)


STEEL = (0.78, 0.80, 0.83, 1)
DARK = (0.25, 0.25, 0.27, 1)

# Steam wand: a vertical tube hanging from an arm off the steamer body.
WAND_TIP = (*polar(ANGLE["steamer"], STATION_R), TABLE_TOP_Z + 0.26)
WAND_LEN = 0.16
STEAMER_FRONT_R = STATION_R + 0.09     # body front face, clear of the pitcher spout

# Milk dispenser: the pitcher stands in a bay under a nozzle and is filled there.
# This bay is also where the pitcher lives between drinks.
DISPENSER_FRONT_R = STATION_R + 0.09
BAY_Z = TABLE_TOP_Z + 0.01             # top of the bay's drip grate
NOZZLE_Z = TABLE_TOP_Z + 0.21          # bottom of the milk nozzle

# Wash station: a basin with a spray nozzle pointing up; the pitcher is held
# upside down over it.
BASIN_TOP_Z = TABLE_TOP_Z + 0.08
SPRAY_TOP_Z = BASIN_TOP_Z + 0.04

CUP_AT = polar(ANGLE["cup"], STATION_R)

# ---------------------------------------------------------------- pitcher
# Pitcher frame: origin at the centre of its floor, +Z up its axis, +X out
# through the spout. The handle is on -X, which is the side the robot grips.
PITCHER_R, PITCHER_H = 0.045, 0.11
PITCHER_LIP = (PITCHER_R + 0.012, 0.0, PITCHER_H)   # spout tip, the pour pivot
MILK_FILL = 0.045                                   # cold milk depth before steaming

PITCHER_PARTS = {
    "pitcher_body": dict(shape="cylinder", size=(PITCHER_R, PITCHER_H),
                         pose=(0.0, 0.0, 0.0, 0.0), color=STEEL),
    "pitcher_spout": dict(shape="box", size=(0.03, 0.03, 0.012),
                          pose=(PITCHER_R, 0.0, PITCHER_H - 0.012, 0.0), color=STEEL),
    "pitcher_handle": dict(shape="box", size=(0.03, 0.015, 0.07),
                           pose=(-PITCHER_R - 0.012, 0.0, 0.025, 0.0), color=(0.20, 0.20, 0.22, 1)),
}
PITCHER_HOME = (ANGLE["dispenser"], STATION_R, BAY_Z)    # (deg, r, floor z)

# ---------------------------------------------------------------- gripper
# Gripper frame = the robot flange: +Z points out of the tool. GRIP_DEPTH is the
# distance from the flange to the pitcher axis when it is held by the handle,
# GRIP_HEIGHT how far up the pitcher the fingers close (the handle's middle).
GRIP_DEPTH = 0.14
GRIP_HEIGHT = 0.06
GRIPPER_COLOR = (0.15, 0.35, 0.65, 1)
GRIPPER_PARTS = {
    "gripper_body": dict(shape="box", size=(0.07, 0.07, 0.06),
                         pose=(0.0, 0.0, 0.0, 0.0), color=GRIPPER_COLOR),
    "gripper_finger_l": dict(shape="box", size=(0.02, 0.012, 0.035),
                             pose=(0.0, 0.022, 0.06, 0.0), color=GRIPPER_COLOR),
    "gripper_finger_r": dict(shape="box", size=(0.02, 0.012, 0.035),
                             pose=(0.0, -0.022, 0.06, 0.0), color=GRIPPER_COLOR),
}

# ---------------------------------------------------------------- cup
# The cup moves between scenes' robots, so it is not a static placeholder: it is
# a barista.cup.Cup standing at CUP_AT while the milk is poured. CUP_SPEC is its
# stand-in for collision checks while it stands there.
CUP_R, CUP_H = 0.04, 0.10
ESPRESSO_H = 0.02          # coffee already in the cup when the pour starts
CUP_SPEC = dict(shape="cylinder", size=(CUP_R, CUP_H), pose=(*CUP_AT, TABLE_TOP_Z, 0.0))

# ---------------------------------------------------------------- placeholders
# box: (sx, sy, sz); cylinder: (radius, height). Pose z is the bottom face.
# Machine boxes are given (radial depth, width, height) and face the robot.
_s, _d, _w = ANGLE["steamer"], ANGLE["dispenser"], ANGLE["wash"]
_arm = STEAMER_FRONT_R - STATION_R + 0.01
_noz = DISPENSER_FRONT_R - STATION_R + 0.02
PLACEHOLDERS = {
    "table": dict(shape="box", size=(*TABLE_SIZE, TABLE_TOP_Z),
                  pose=(*TABLE_CENTRE, 0.0, 0.0), color=(0.55, 0.40, 0.28, 1)),

    # Steamer
    "steamer_body": _facing(_s, STEAMER_FRONT_R + 0.11, TABLE_TOP_Z, (0.22, 0.30, 0.55)),
    "steamer_drip_tray": _facing(_s, STATION_R + 0.02, TABLE_TOP_Z, (0.22, 0.26, 0.02),
                                 color=(0.35, 0.35, 0.37, 1)),
    "wand_arm": _facing(_s, STATION_R - 0.01 + _arm / 2, WAND_TIP[2] + WAND_LEN,
                        (_arm, 0.025, 0.025), color=STEEL),
    "steam_wand": dict(shape="cylinder", size=(0.006, WAND_LEN), pose=(*WAND_TIP, 0.0), color=STEEL),

    # Wash station
    "wash_basin": _facing(_w, STATION_R, TABLE_TOP_Z, (0.22, 0.22, BASIN_TOP_Z - TABLE_TOP_Z),
                          color=(0.70, 0.72, 0.75, 1)),
    "wash_spray": dict(shape="cylinder", size=(0.01, SPRAY_TOP_Z - BASIN_TOP_Z),
                       pose=(*polar(_w, STATION_R), BASIN_TOP_Z, 0.0), color=STEEL),

    # Milk dispenser
    "dispenser_body": _facing(_d, DISPENSER_FRONT_R + 0.12, TABLE_TOP_Z, (0.24, 0.28, 0.50),
                              color=(0.92, 0.92, 0.95, 1)),
    "dispenser_grate": _facing(_d, STATION_R + 0.01, TABLE_TOP_Z, (0.18, 0.18, BAY_Z - TABLE_TOP_Z),
                               color=DARK),
    "dispenser_arm": _facing(_d, STATION_R - 0.02 + _noz / 2, NOZZLE_Z + 0.03,
                             (_noz, 0.04, 0.04), color=(0.92, 0.92, 0.95, 1)),
    "dispenser_nozzle": dict(shape="cylinder", size=(0.008, 0.03),
                             pose=(*polar(_d, STATION_R), NOZZLE_Z, 0.0), color=STEEL),
}

# ---------------------------------------------------------------- stations
# Points the flange must at least reach (position-only), for build.check_reach.
STATIONS = {
    "milk": {
        "dispenser_bay": (*polar(_d, STATION_R - GRIP_DEPTH), BAY_Z + GRIP_HEIGHT),
        "steam_wand": (*polar(_s, STATION_R - GRIP_DEPTH), WAND_TIP[2] - 0.05),
        "cup": (*polar(ANGLE["cup"], STATION_R - GRIP_DEPTH - 0.08), TABLE_TOP_Z + CUP_H + 0.15),
        "wash": (*polar(_w, STATION_R - GRIP_DEPTH), SPRAY_TOP_Z + 0.08),
    },
}

CAMERA = dict(position=(1.9, -1.4, 1.65), look_at=(0.30, 0.10, TABLE_TOP_Z + 0.15))


if __name__ == "__main__":
    import sys

    build.describe(sys.modules[__name__])
