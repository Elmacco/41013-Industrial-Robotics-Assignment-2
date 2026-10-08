"""
Barista cell layout: every pose and dimension in the scene, as plain data.

World frame: origin on the floor under the centre of the counter, +X along the
counter (cup supply -> espresso -> milk -> pickup), +Y towards the customer,
+Z up. Units are metres and radians.

Run ``python -m barista.layout`` to print where every object actually ends up.
"""
from math import pi

COUNTER_TOP_Z = 0.93  # Workbench part height

# ---------------------------------------------------------------- robots
# base = (x, y, z, yaw). LinearUR3's rail mesh spans x in [base-0.91, base+0.21]
# and is 0.5 m wide; its carriage travels 0.8 m (q0 in [-0.8, 0]).
ROBOTS = {
    "espresso": dict(model="UR3", base=(-0.65, -0.35, COUNTER_TOP_Z, 0.0),
                     role="Grind, tamp, lock portafilter, pull shot"),
    "milk": dict(model="UR3e", base=(0.70, -0.30, COUNTER_TOP_Z, 0.0),
                 role="Pour milk, steam, latte pour, place at pickup"),
    "runner": dict(model="LinearUR3", base=(0.25, 0.45, COUNTER_TOP_Z, 0.0),
                   role="Move cups: supply -> espresso -> milk"),
    "cs66": dict(model="CS66", base=(1.00, 0.45, COUNTER_TOP_Z, 0.0),
                 role="Placed only, job not decided yet"),
}

# ---------------------------------------------------------------- support-package parts
# (part_name, (x, y, z, yaw), rgba or None). Names are from ir_support_extra_parts.
PROPS = {
    # Furniture
    "counter_back": ("Workbench", (0.0, -0.375, 0.0, 0.0), None),
    "counter_front": ("Workbench", (0.0, 0.375, 0.0, 0.0), None),

    # Cups and milk (PintCup ~76 mm dia x 149 mm, closest to a real coffee cup)
    "cup_1": ("PintCup", (-0.85, 0.30, COUNTER_TOP_Z, 0.0), None),
    "cup_2": ("PintCup", (-0.95, 0.30, COUNTER_TOP_Z, 0.0), None),
    "cup_3": ("PintCup", (-1.05, 0.30, COUNTER_TOP_Z, 0.0), None),
    "milk_carton": ("MilkCarton", (1.00, -0.55, COUNTER_TOP_Z, 0.0), None),
    "milk_pitcher": ("MilkPitcher", (1.05, -0.15, COUNTER_TOP_Z, 0.0), None),
    "syrup": ("BlueSyrupBottle", (1.10, -0.65, COUNTER_TOP_Z, 0.0), None),
    "pickup_tray": ("Tray", (0.95, 0.10, COUNTER_TOP_Z, 0.0), None),

    # Controls / sensing
    "light_tower": ("LightTower", (-0.10, -0.68, COUNTER_TOP_Z, 0.0), None),
    "camera": ("Camera", (0.10, -0.66, COUNTER_TOP_Z, 0.0), None),

    # Safety
    "estop": ("emergencyStopButton", (-1.40, 1.00, 0.0, 0.0), None),
    "light_curtain_l": ("SafetyLightCurtain", (0.70, 0.70, COUNTER_TOP_Z, 0.0), None),
    "light_curtain_r": ("SafetyLightCurtain", (1.15, 0.70, COUNTER_TOP_Z, 0.0), None),
    "railing_back": ("SafetyRailing", (0.0, -1.40, 0.0, 0.0), None),
    "railing_left": ("SafetyRailing", (-1.90, 0.0, 0.0, pi / 2), None),
    "railing_right": ("SafetyRailing", (1.90, 0.0, 0.0, pi / 2), None),
    "fire_extinguisher": ("FireExtinguisher", (-1.40, -0.95, 0.0, 0.0), None),
    "warning_sign": ("WarningSign", (1.50, 0.95, 0.0, 0.0), None),
    "customer": ("personFemaleBusiness", (0.95, 1.40, 0.0, -pi / 2), None),
}

# ---------------------------------------------------------------- placeholders
# The support package has no coffee machine, grinder, portafilter or steam wand,
# so these are simple primitives until custom meshes are made.
# box: (sx, sy, sz); cylinder: (radius, height). Pose z is the bottom face.
PLACEHOLDERS = {
    "espresso_machine": dict(shape="box", size=(0.40, 0.30, 0.40),
                             pose=(-1.00, -0.55, COUNTER_TOP_Z, 0.0), color=(0.75, 0.75, 0.78, 1)),
    "grinder": dict(shape="cylinder", size=(0.08, 0.40),
                    pose=(-0.25, -0.55, COUNTER_TOP_Z, 0.0), color=(0.15, 0.15, 0.15, 1)),
    "tamp_station": dict(shape="cylinder", size=(0.06, 0.08),
                         pose=(-0.30, -0.15, COUNTER_TOP_Z, 0.0), color=(0.40, 0.25, 0.15, 1)),
    "portafilter": dict(shape="cylinder", size=(0.035, 0.05),
                        pose=(-0.85, -0.70, COUNTER_TOP_Z + 0.40, 0.0), color=(0.30, 0.30, 0.30, 1)),
    "steamer": dict(shape="box", size=(0.20, 0.20, 0.45),
                    pose=(0.35, -0.58, COUNTER_TOP_Z, 0.0), color=(0.75, 0.75, 0.78, 1)),
}

# ---------------------------------------------------------------- stations
# Named target points (x, y, z) each robot must reach. Used by scene.check_reach().
STATIONS = {
    "espresso": {
        "grinder": (-0.25, -0.55, COUNTER_TOP_Z + 0.25),
        "tamp": (-0.30, -0.15, COUNTER_TOP_Z + 0.10),
        "group_head": (-1.00, -0.42, COUNTER_TOP_Z + 0.25),
        "cup_handoff": (-0.65, 0.05, COUNTER_TOP_Z + 0.08),
    },
    "milk": {
        "milk_carton": (1.00, -0.55, COUNTER_TOP_Z + 0.08),
        "pitcher_rest": (1.05, -0.15, COUNTER_TOP_Z + 0.08),
        "steam_wand": (0.35, -0.45, COUNTER_TOP_Z + 0.20),
        "cup_handoff": (0.45, 0.05, COUNTER_TOP_Z + 0.08),
        "pickup": (0.95, 0.10, COUNTER_TOP_Z + 0.08),
    },
    "runner": {
        "cup_supply": (-0.85, 0.30, COUNTER_TOP_Z + 0.08),
        "espresso_handoff": (-0.65, 0.05, COUNTER_TOP_Z + 0.08),
        "milk_handoff": (0.45, 0.05, COUNTER_TOP_Z + 0.08),
    },
    "cs66": {
        "above_tray": (0.95, 0.10, COUNTER_TOP_Z + 0.20),
    },
}

CAMERA = dict(position=(2.6, 2.4, 2.2), look_at=(0.0, 0.0, COUNTER_TOP_Z))


if __name__ == "__main__":
    import sys

    from barista import build

    build.describe(sys.modules[__name__])
