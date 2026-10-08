"""
Scene 2 layout: every pose and dimension in the scene, as plain data.

World frame: origin on the floor under the centre of the table, +X along the
long edge of the table, +Y towards the customer side, +Z up. Units are metres
and radians.

So far: the table, a plexiglass guard standing on the floor around it, and a
sliding collection drawer through the front glass. The robots and stations
sections are empty placeholders to be filled in as the cell is designed.

Run ``python -m barista.layout2`` to print where every object actually ends up.
"""
from barista import build

# tableBrown2.1x1.4x0.5m spans x in [-1.06, 1.06], y in [-0.71, 0.71] about its
# own origin, which sits on the floor. Its top face is 0.5 m up.
TABLE_PART = "tableBrown2.1x1.4x0.5m"
TABLE_SIZE = (2.12, 1.41)  # (x, y) extents of the table top
TABLE_TOP_Z = 0.50

# ---------------------------------------------------------------- plexiglass guard
# A translucent enclosure standing on the FLOOR, set back GLASS_GAP from every
# table edge so there is walk-around clearance. Swift renders alpha < 1 as
# see-through; keep rgba in the 0-1 range (any component above 1.0 makes the
# colour setter divide all four by 255, which silently zeroes the alpha) and
# never use alpha exactly 0.0, which renders fully opaque.
GLASS_T = 0.010          # panel thickness
GLASS_H = 1.30           # panel height, measured from the floor
GLASS_GAP = 0.50         # clear gap between the table edge and the glass
GLASS_COLOR = (0.60, 0.80, 0.85, 0.25)   # faint acrylic tint

# ---------------------------------------------------------------- collection drawer
# A tray shuttles along +Y through a slot in the front glass: a robot loads it
# at DRAWER_Y_IN (inside the guard, within UR3 reach of the table) and the
# customer takes the cup at DRAWER_Y_OUT (clear of the glass). The slot is the
# only opening left in the guard, but it is NOT sealed: the tray plugs it while
# extended and the shroud blocks line of sight, yet the slot stands open while
# the tray is retracted. Closing it properly needs an interlocked shutter.
DRAWER_PART = "Tray"
DRAWER_SIZE = (0.38, 0.25, 0.04)   # Tray mesh extents; its origin is bottom-centre
DRAWER_TOP_Z = 0.90                # cup stands here - comfortable standing collection
DRAWER_Z = DRAWER_TOP_Z - DRAWER_SIZE[2]   # tray underside
DRAWER_Y_IN = 0.65                 # loading position, reachable from the table
DRAWER_Y_OUT = 1.33                # collection position, outside the glass
CUP_H = 0.149                      # PintCup height - what has to clear the slot

# Where a robot must place a cup to load the drawer. Add this to STATIONS once
# there is a robot in ROBOTS to check it against.
DRAWER_LOAD = (0.0, DRAWER_Y_IN, DRAWER_TOP_Z)

SLOT_W = DRAWER_SIZE[0] + 0.04           # tray width plus clearance
SLOT_Z0 = DRAWER_Z - 0.02                # sill, just under the tray
SLOT_Z1 = DRAWER_TOP_Z + CUP_H + 0.03    # lintel, just over a standing cup

# Shroud: a tunnel around the drawer path from the loading zone out to the
# glass, so the slot does not look straight into the cell. It stops short of the
# retracted tray to leave the loading zone open from above for the robot.
SHROUD_WALL = 0.02
SHROUD_Y0 = DRAWER_Y_IN + DRAWER_SIZE[1] / 2 + 0.03
SHROUD_Y1 = TABLE_SIZE[1] / 2 + GLASS_GAP    # the inner face of the glass

# ---------------------------------------------------------------- robots
# base = (x, y, z, yaw).
ROBOTS = {}

# ---------------------------------------------------------------- support-package parts
# (part_name, (x, y, z, yaw), rgba or None). Names are from ir_support_extra_parts.
PROPS = {
    "table": (TABLE_PART, (0.0, 0.0, 0.0, 0.0), None),
    # Starts retracted; scene2.set_drawer() slides it along +Y.
    "drawer_tray": (DRAWER_PART, (0.0, DRAWER_Y_IN, DRAWER_Z, 0.0), None),
}

# ---------------------------------------------------------------- placeholders
# box: (sx, sy, sz); cylinder: (radius, height). Pose z is the bottom face.
# The guard and shroud are generated from the figures above rather than written
# out panel by panel, so their corners cannot drift apart when one is edited.
PLACEHOLDERS = {
    **build.guard_walls(
        inner=(TABLE_SIZE[0] / 2 + GLASS_GAP, TABLE_SIZE[1] / 2 + GLASS_GAP),
        height=GLASS_H, thickness=GLASS_T, color=GLASS_COLOR,
        opening=(SLOT_W, SLOT_Z0, SLOT_Z1), prefix="glass"),
    **build.tunnel_walls(
        half_width=SLOT_W / 2, y_range=(SHROUD_Y0, SHROUD_Y1),
        z_range=(SLOT_Z0, SLOT_Z1), wall=SHROUD_WALL,
        color=GLASS_COLOR, prefix="shroud"),
}

# ---------------------------------------------------------------- stations
# Named target points (x, y, z) each robot must reach. Used by check_reach().
STATIONS = {}

CAMERA = dict(position=(3.4, 3.1, 2.3), look_at=(0.0, 0.0, TABLE_TOP_Z))


if __name__ == "__main__":
    import sys

    build.describe(sys.modules[__name__])
