"""
Full barista cell layout: every robot, station and safety feature, as data.

World frame: origin on the floor under the AUBO base (the same frame as
milk_layout.py, so the milk station drops in unchanged), +X along the counter
towards the espresso side, +Y towards the customer, +Z up. Metres and radians.

    -X  milk station (AUBO i5)  |  UR3 runner  |  espresso station (CS66)  +X
                       [ collection drawer ]
    ======================= glass, front (+Y) =======================
          outside e-stop      customer      tablet kiosk

How a drink flows:
    cup dispenser -> UR3 puts a cup under the group head
    -> CS66: portafilter from its dock -> grinder -> tamper -> locks it in the
       group head -> the shot pulls into the cup -> knock box -> rinse -> dock
    -> UR3 takes the cup to the hand-off spot (AUBO pours the milk)
    -> UR3 takes it to the collection drawer -> drawer slides out through the
       glass to the customer.

Run ``python -m barista.full_layout`` to print where every object ends up.
"""
from math import atan2, cos, degrees, hypot, pi, radians, sin

from barista import build
from barista import milk_layout as milk

TABLE_TOP_Z = milk.TABLE_TOP_Z
TABLE_X = (-0.50, 2.40)
TABLE_Y = (-0.95, 1.05)
TABLE_CENTRE = (sum(TABLE_X) / 2, sum(TABLE_Y) / 2)
TABLE_SIZE = (TABLE_X[1] - TABLE_X[0], TABLE_Y[1] - TABLE_Y[0])
STEEL = (0.75, 0.75, 0.78, 1)
DARK = (0.20, 0.20, 0.22, 1)

# ---------------------------------------------------------------- robots
UR3_BASE = (1.00, 0.05)
CS66_BASE = (1.80, 0.00)
ROBOTS = {
    "milk": milk.ROBOTS["milk"],
    "runner": dict(model="UR3", base=(*UR3_BASE, TABLE_TOP_Z, 0.0),
                   role="Move the cup: dispenser -> espresso -> AUBO -> drawer"),
    "espresso": dict(model="CS66", base=(*CS66_BASE, TABLE_TOP_Z, pi),
                     role="Grind, tamp, lock the portafilter, pull the shot, clean up"),
}


def around(centre, angle_deg, r):
    """(x, y) at angle_deg round `centre`, r out from it."""
    a = radians(angle_deg)
    return (centre[0] + r * cos(a), centre[1] + r * sin(a))


def polar_of(centre, xy):
    """(angle_deg, r) of xy about `centre`."""
    return degrees(atan2(xy[1] - centre[1], xy[0] - centre[0])), hypot(xy[0] - centre[0], xy[1] - centre[1])


def _box(size, at, color):
    return dict(shape="box", size=size, pose=at, color=color)


def _cyl(radius, height, at, color):
    return dict(shape="cylinder", size=(radius, height), pose=at, color=color)


def _facing(centre, angle_deg, r, z, size, color):
    """Box r out from `centre` along angle_deg, turned to face it."""
    return _box(size, (*around(centre, angle_deg, r), z, radians(angle_deg)), color)


# ---------------------------------------------------------------- cups
CUP_R, CUP_H = milk.CUP_R, milk.CUP_H
ESPRESSO_CUP = (1.35, 0.38, TABLE_TOP_Z + 0.01)          # cup floor under the group head
HANDOFF_CUP = (*milk.CUP_AT, TABLE_TOP_Z)                # where the AUBO pours

# Cup dispenser: a clear tube holding a nested stack of cups. The bottom cup
# drops onto a raised platform (so the UR3 is not reaching low and close in
# to its own base), where the UR3 slides it out sideways.
DISPENSER_AT = around(UR3_BASE, -60.0, 0.44)
DISPENSER_PLATFORM_Z = TABLE_TOP_Z + 0.10
DISPENSER_CUP = (*DISPENSER_AT, DISPENSER_PLATFORM_Z)
DISPENSER_TUBE_Z = DISPENSER_CUP[2] + CUP_H + 0.03     # tube bottom, clear of the cup below
DISPENSER_TUBE_R, DISPENSER_TUBE_H = 0.05, 0.40
CUP_STACK = 6                                           # cups loaded in the tube
CUP_NEST = 0.03                                         # height step between nested cups

# ---------------------------------------------------------------- espresso station (CS66)
# The portafilter frame has its origin at the centre of the basket rim, +Z up,
# and the handle along -X. Stations point the handle at the CS66.
CS66_R = 0.50                                           # station distance from the CS66
CS66_ANGLE = dict(grinder=90.0, tamper=40.0, knock=-30.0, pf_wash=-80.0, pf_dock=-125.0)
GROUP_HEAD_AT = ESPRESSO_CUP[:2]
GROUP_HEAD_Z = TABLE_TOP_Z + 0.24                       # underside, where the basket rim locks

PF_BASKET_R, PF_BASKET_H = 0.035, 0.035
PF_SPOUT_H = 0.015
PF_HANDLE_LEN = 0.13
PF_GRIP_X = -0.14                                       # where on the handle the gripper closes
PORTAFILTER_PARTS = {
    "pf_basket": _cyl(PF_BASKET_R, PF_BASKET_H, (0.0, 0.0, -PF_BASKET_H, 0.0), (0.55, 0.55, 0.58, 1)),
    "pf_spout": _cyl(0.012, PF_SPOUT_H, (0.0, 0.0, -PF_BASKET_H - PF_SPOUT_H, 0.0), (0.55, 0.55, 0.58, 1)),
    "pf_handle": _box((PF_HANDLE_LEN, 0.025, 0.025),
                      (-PF_BASKET_R - PF_HANDLE_LEN / 2, 0.0, -0.0275, 0.0), (0.08, 0.08, 0.08, 1)),
}
# Collision shape, in the portafilter frame: two short capsules across the
# basket (a flat disc is badly fitted by one capsule), the spout, the handle.
PF_CAPSULES = [
    ((-0.02, 0.0, -0.018), (0.02, 0.0, -0.018), 0.018),
    ((0.0, -0.02, -0.018), (0.0, 0.02, -0.018), 0.018),
    ((0.0, 0.0, -0.038), (0.0, 0.0, -0.040), 0.012),
    ((-0.05, 0.0, -0.015), (-0.155, 0.0, -0.015), 0.0135),
]

# Rim heights at each station, and what sits there.
PF_DOCK_TOP = TABLE_TOP_Z + 0.06
PF_RIM_Z = dict(
    pf_dock=PF_DOCK_TOP + PF_BASKET_H,       # basket resting on the dock ring
    grinder=TABLE_TOP_Z + 0.16,
    tamper=TABLE_TOP_Z + 0.14,
    knock=TABLE_TOP_Z + 0.22,
    pf_wash=TABLE_TOP_Z + 0.20,
    group_head=GROUP_HEAD_Z,
)
GRINDER_CHUTE_Z = TABLE_TOP_Z + 0.22           # chute outlet underside
TAMPER_PISTON = dict(radius=0.030, height=0.07, up_z=TABLE_TOP_Z + 0.23)   # piston underside when up
TAMPER_STROKE = TAMPER_PISTON["up_z"] - (PF_RIM_Z["tamper"] - 0.012)       # down into the basket
PF_SHOWER_Z = TABLE_TOP_Z + 0.31               # rinse nozzle underside


def cs66_station(name):
    """(angle_deg, r) of a CS66 station about its base."""
    if name == "group_head":
        return polar_of(CS66_BASE, GROUP_HEAD_AT)
    return CS66_ANGLE[name], CS66_R


def _cs(name, dr, z, size, color):
    """Placeholder at a CS66 station, dr further out than the basket, facing the CS66."""
    a, r = cs66_station(name)
    return _facing(CS66_BASE, a, r + dr, z, size, color)


def _cs_cyl(name, dr, z, radius, height, color):
    a, r = cs66_station(name)
    return _cyl(radius, height, (*around(CS66_BASE, a, r + dr), z, 0.0), color)


# ---------------------------------------------------------------- collection drawer
# A tray slides along +Y on a rail, through a slot in the front glass. The UR3
# loads it at DRAWER_Y_IN; the customer takes the cup at DRAWER_Y_OUT. The
# customer never reaches into the cell: the slot only fits the tray and cup,
# and the shroud stops the cell being reached through it.
DRAWER_X = TABLE_CENTRE[0]
DRAWER_SIZE = (0.38, 0.25, 0.04)          # Tray part; origin at its bottom centre
RAIL_H = 0.02
DRAWER_Z = TABLE_TOP_Z + RAIL_H           # tray underside
DRAWER_TOP_Z = DRAWER_Z + DRAWER_SIZE[2]  # cup stands here
DRAWER_Y_IN = 0.42

# ---------------------------------------------------------------- enclosure
GLASS_T = 0.010
GLASS_H = 2.00
GLASS_GAP = 0.40                          # walk-round gap between table and glass
GLASS_COLOR = (0.60, 0.80, 0.85, 0.22)
ENCLOSURE_CENTRE = TABLE_CENTRE
INNER = (TABLE_SIZE[0] / 2 + GLASS_GAP, TABLE_SIZE[1] / 2 + GLASS_GAP)   # half extents
FRONT_Y = ENCLOSURE_CENTRE[1] + INNER[1]  # inner face of the front glass

DRAWER_Y_OUT = FRONT_Y + GLASS_T + 0.03 + DRAWER_SIZE[1] / 2
SLOT_W = DRAWER_SIZE[0] + 0.04
SLOT_Z0 = DRAWER_Z - 0.01
SLOT_Z1 = DRAWER_TOP_Z + CUP_H + 0.03
SHROUD_WALL = 0.02
SHROUD_Y0 = DRAWER_Y_IN + DRAWER_SIZE[1] / 2 + 0.04

# ---------------------------------------------------------------- safety tape
TAPE_OFFSET = 0.15        # from the outer face of the glass
TAPE_W = 0.05
TAPE_DASH = 0.10
YELLOW, BLACK = (0.95, 0.80, 0.05, 1), (0.08, 0.08, 0.08, 1)

# ---------------------------------------------------------------- controls
KIOSK_AT = (DRAWER_X + 0.70, FRONT_Y + 0.45)   # tablet the customer orders on
ESTOP_OUT_AT = (DRAWER_X - 0.75, FRONT_Y + 0.45)
ESTOP_IN_AT = (TABLE_X[1] - 0.10, TABLE_Y[0] + 0.10)   # table corner, inside the cell
# Status light on a mast in the front corner of the cell; BEACON_AT is the mast top.
BEACON_AT = (ENCLOSURE_CENTRE[0] + INNER[0] - 0.10, FRONT_Y - 0.10, GLASS_H)


# ---------------------------------------------------------------- helpers
def _shift(entries, dx, dy):
    """Move layout helper output (built about the origin) to (dx, dy)."""
    out = {}
    for name, spec in entries.items():
        x, y, z, yaw = spec["pose"]
        out[name] = dict(spec, pose=(x + dx, y + dy, z, yaw))
    return out


def hazard_tape(x_range, y_range, skip=(), prefix="tape"):
    """Yellow/black floor tape round a rectangle, as alternating dashes.

    skip: (x0, x1, y0, y1) areas the tape stops short of (the collection shelf).
    """
    x0, x1 = x_range
    y0, y1 = y_range
    out = {}

    def run(name, start, end, fixed, along_x):
        n = max(int(round((end - start) / TAPE_DASH)), 1)
        step = (end - start) / n
        for i in range(n):
            c = start + (i + 0.5) * step
            x, y = (c, fixed) if along_x else (fixed, c)
            if any(a <= x <= b and c0 <= y <= c1 for a, b, c0, c1 in skip):
                continue
            size = (step, TAPE_W, 0.002) if along_x else (TAPE_W, step, 0.002)
            out[f"{prefix}_{name}_{i}"] = _box(size, (x, y, 0.0, 0.0), YELLOW if i % 2 else BLACK)

    run("front", x0, x1, y1, True)
    run("back", x0, x1, y0, True)
    run("left", y0, y1, x0, False)
    run("right", y0, y1, x1, False)
    return out


# ---------------------------------------------------------------- props (support-package parts)
PROPS = {
    "drawer_tray": ("Tray", (DRAWER_X, DRAWER_Y_IN, DRAWER_Z, 0.0), None),   # full_scene slides it
    "estop_outside": ("emergencyStopButton", (*ESTOP_OUT_AT, 0.80, 0.0), None),
    "customer": ("personFemaleBusiness", (DRAWER_X, FRONT_Y + 0.90, 0.0, -pi / 2), None),
}

# ---------------------------------------------------------------- placeholders
# box: (sx, sy, sz); cylinder: (radius, height). Pose z is the bottom face.
# Station boxes facing a robot are given (radial depth, width, height).
_ox = INNER[0] + GLASS_T
_oy = INNER[1] + GLASS_T
_cx, _cy = ENCLOSURE_CENTRE
_T = TABLE_TOP_Z
_disp_a, _ = polar_of(UR3_BASE, DISPENSER_AT)

PLACEHOLDERS = {
    # The milk station, minus its own small table.
    **{k: v for k, v in milk.PLACEHOLDERS.items() if k != "table"},

    "table": _box((*TABLE_SIZE, _T), (*TABLE_CENTRE, 0.0, 0.0), (0.55, 0.40, 0.28, 1)),

    # ---- espresso machine; the portafilter locks into the group head
    "espresso_machine": _box((0.40, 0.30, 0.45), (1.40, 0.62, _T, 0.0), STEEL),
    # Round, like a real group head, with a neck back to the machine body.
    "group_head": _cyl(0.05, 0.06, (*GROUP_HEAD_AT, GROUP_HEAD_Z, 0.0), (0.30, 0.30, 0.32, 1)),
    "group_head_neck": _box((0.06, 0.05, 0.06), (GROUP_HEAD_AT[0], 0.455, GROUP_HEAD_Z, 0.0),
                            (0.30, 0.30, 0.32, 1)),
    "espresso_drip_tray": _box((0.20, 0.18, 0.01), (*GROUP_HEAD_AT, _T, 0.0), (0.35, 0.35, 0.37, 1)),

    # ---- grinder: body behind the basket, a chute out over it, hopper on top
    "grinder_body": _cs("grinder", 0.155, _T, (0.18, 0.20, 0.42), DARK),
    "grinder_chute": _cs("grinder", 0.03, GRINDER_CHUTE_Z, (0.11, 0.05, 0.04), DARK),
    "grinder_hopper": _cs_cyl("grinder", 0.155, _T + 0.42, 0.07, 0.15, (0.55, 0.40, 0.25, 0.7)),

    # ---- tamping machine: base plate under the basket, column, head arm.
    # The piston itself moves, so full_scene builds it (TAMPER_PISTON).
    "tamper_base": _cs("tamper", 0.0, _T, (0.12, 0.12, 0.07), DARK),
    "tamper_column": _cs("tamper", 0.115, _T, (0.06, 0.08, 0.34), DARK),
    "tamper_head": _cs("tamper", 0.06, TAMPER_PISTON["up_z"] + TAMPER_PISTON["height"],
                       (0.17, 0.08, 0.04), DARK),

    # ---- knock box (spent pucks) and portafilter rinse
    "knock_box": _cs("knock", 0.0, _T, (0.16, 0.16, 0.12), (0.15, 0.15, 0.17, 1)),
    "pf_wash_sink": _cs("pf_wash", 0.0, _T, (0.20, 0.20, 0.08), (0.70, 0.72, 0.75, 1)),
    "pf_wash_column": _cs("pf_wash", 0.13, _T, (0.05, 0.05, 0.40), STEEL),
    "pf_wash_arm": _cs("pf_wash", 0.065, PF_SHOWER_Z + 0.03, (0.15, 0.04, 0.03), STEEL),
    "pf_wash_nozzle": _cs_cyl("pf_wash", 0.0, PF_SHOWER_Z, 0.012, 0.03, STEEL),

    # ---- portafilter dock: a ring the basket sits in between drinks
    "pf_dock": _cs("pf_dock", 0.0, _T, (0.09, 0.09, PF_DOCK_TOP - _T), DARK),

    # ---- cup dispenser: platform, clear tube above it, support column
    "dispenser_platform": _cyl(0.06, DISPENSER_PLATFORM_Z - _T, (*DISPENSER_AT, _T, 0.0), DARK),
    "dispenser_tube": _cyl(DISPENSER_TUBE_R, DISPENSER_TUBE_H, (*DISPENSER_AT, DISPENSER_TUBE_Z, 0.0),
                           (0.85, 0.90, 0.95, 0.25)),
    "dispenser_column": _facing(UR3_BASE, _disp_a, 0.44 + DISPENSER_TUBE_R + 0.03, _T,
                                (0.04, 0.05, DISPENSER_TUBE_Z + DISPENSER_TUBE_H - _T), DARK),

    # ---- collection drawer: rail on the table, a bridge over the walk-round
    # gap, and a shelf outside the glass where the customer collects.
    "drawer_rail": _box((SLOT_W, TABLE_Y[1] - (DRAWER_Y_IN - 0.15), RAIL_H),
                        (DRAWER_X, (TABLE_Y[1] + DRAWER_Y_IN - 0.15) / 2, _T, 0.0), STEEL),
    "drawer_bridge": _box((SLOT_W, FRONT_Y - TABLE_Y[1], DRAWER_Z),
                          (DRAWER_X, (FRONT_Y + TABLE_Y[1]) / 2, 0.0, 0.0), STEEL),
    "collection_shelf": _box((SLOT_W + 0.10, 0.40, DRAWER_Z),
                             (DRAWER_X, FRONT_Y + GLASS_T + 0.20, 0.0, 0.0), STEEL),
    **{k: v for k, v in _shift(build.tunnel_walls(
        half_width=SLOT_W / 2, y_range=(SHROUD_Y0 - _cy, FRONT_Y - _cy),
        z_range=(SLOT_Z0, SLOT_Z1), wall=SHROUD_WALL, color=GLASS_COLOR, prefix="shroud"),
        DRAWER_X, _cy).items() if k != "shroud_floor"},

    # ---- clear enclosure: four walls on the floor and a slot for the drawer. No
    # roof (it blocked the camera); the walls stand well above anything a robot
    # can reach.
    **_shift(build.guard_walls(INNER, GLASS_H, GLASS_T, GLASS_COLOR,
                               opening=(SLOT_W, SLOT_Z0, SLOT_Z1), prefix="glass"), _cx, _cy),

    # ---- safety tape on the floor just outside the glass
    **hazard_tape((_cx - _ox - TAPE_OFFSET, _cx + _ox + TAPE_OFFSET),
                  (_cy - _oy - TAPE_OFFSET, _cy + _oy + TAPE_OFFSET),
                  skip=[(DRAWER_X - SLOT_W / 2 - 0.05, DRAWER_X + SLOT_W / 2 + 0.05,
                         FRONT_Y, FRONT_Y + 0.5)]),

    # ---- controls
    "kiosk_post": _cyl(0.03, 1.05, (*KIOSK_AT, 0.0, 0.0), DARK),
    "kiosk_foot": _cyl(0.15, 0.02, (*KIOSK_AT, 0.0, 0.0), DARK),
    "kiosk_tablet": _box((0.28, 0.03, 0.20), (*KIOSK_AT, 1.05, 0.0), (0.05, 0.05, 0.07, 1)),
    "estop_out_post": _box((0.30, 0.30, 0.80), (*ESTOP_OUT_AT, 0.0, 0.0), YELLOW),
    "estop_in_box": _box((0.09, 0.09, 0.07), (*ESTOP_IN_AT, _T, 0.0), YELLOW),
    "estop_in_button": _cyl(0.03, 0.03, (*ESTOP_IN_AT, _T + 0.07, 0.0), (0.85, 0.05, 0.05, 1)),
    "beacon_post": _cyl(0.02, GLASS_H, (*BEACON_AT[:2], 0.0, 0.0), DARK),
}

# Placeholders that are outside the cell or flat on the floor; collision checks
# skip them (nothing inside can reach them).
NOT_OBSTACLES = ("tape_", "kiosk_", "estop_out", "collection_shelf", "beacon_")

# ---------------------------------------------------------------- stations
# Points each robot's flange must at least reach (position-only), for check_reach.
STATIONS = {
    **milk.STATIONS,
    "runner": {
        "dispenser": (*DISPENSER_AT, DISPENSER_CUP[2] + 0.06),
        "espresso_cup": (*ESPRESSO_CUP[:2], ESPRESSO_CUP[2] + 0.06),
        "handoff": (*HANDOFF_CUP[:2], HANDOFF_CUP[2] + 0.06),
        "drawer": (DRAWER_X, DRAWER_Y_IN, DRAWER_TOP_Z + 0.06),
    },
    "espresso": {name: (*around(CS66_BASE, *cs66_station(name)), z + 0.06)
                 for name, z in PF_RIM_Z.items()},
}

CAMERA = dict(position=(4.3, 3.6, 2.7), look_at=(0.95, 0.05, 0.8))

# ---------------------------------------------------------------- station labels
# (text, (x, y, z), flat). Upright signs stand on top of or behind a machine,
# out of the robots' way, turned to face CAMERA; z is the sign's bottom. Busy
# spots get a flat label lying on the table instead; z is the table top.
# Upright signs are collision obstacles too (see full_scene.static_obstacles).
LABEL_HEIGHT = 0.03
LABELS = [
    # Milk station (AUBO)
    ("AUBO i5", (-0.28, -0.12, _T), True),
    ("STEAMER", (*milk.polar(milk.ANGLE["steamer"], milk.STEAMER_FRONT_R + 0.11), _T + 0.55), False),
    ("MILK DISPENSER", (*milk.polar(milk.ANGLE["dispenser"], milk.DISPENSER_FRONT_R + 0.12), _T + 0.50), False),
    ("PITCHER RINSE", (*milk.polar(milk.ANGLE["wash"], milk.STATION_R + 0.16), _T), False),
    ("MILK POUR", (HANDOFF_CUP[0], HANDOFF_CUP[1] - 0.10, _T), True),
    # UR3
    ("UR3", (UR3_BASE[0], UR3_BASE[1] - 0.12, _T), True),
    ("CUP DISPENSER", (*DISPENSER_AT, DISPENSER_TUBE_Z + DISPENSER_TUBE_H), False),
    ("COLLECTION DRAWER", (DRAWER_X, 1.00, SLOT_Z1 + SHROUD_WALL), False),
    # Espresso station (CS66)
    ("CS66", (*around(CS66_BASE, 5.0, 0.28), _T), True),
    ("ESPRESSO MACHINE", (1.40, 0.68, _T + 0.45), False),
    ("GRINDER", (*around(CS66_BASE, CS66_ANGLE["grinder"], CS66_R + 0.155), _T + 0.57), False),
    ("TAMPER", (*around(CS66_BASE, CS66_ANGLE["tamper"], CS66_R + 0.115), _T + 0.34), False),
    ("KNOCK BOX", (*around(CS66_BASE, CS66_ANGLE["knock"], CS66_R + 0.16), _T), False),
    ("PORTAFILTER RINSE", (*around(CS66_BASE, CS66_ANGLE["pf_wash"], CS66_R + 0.13), _T + 0.40), False),
    ("PORTAFILTER DOCK", (*around(CS66_BASE, CS66_ANGLE["pf_dock"], CS66_R + 0.12), _T), False),
    # Safety and ordering
    ("E-STOP", (*ESTOP_OUT_AT, 1.10), False),
    ("E-STOP", (ESTOP_IN_AT[0], ESTOP_IN_AT[1] + 0.10, _T), True),
    ("ORDER HERE", (*KIOSK_AT, 1.27), False),
]


if __name__ == "__main__":
    import sys

    build.describe(sys.modules[__name__])
