"""
Works out the joint states for the UR3e pick-and-place demo in
ur3_real.py (STEPS), which jointstatesim.py also animates.
Run `python UR3/pick_place.py` and paste its output over STEPS in ur3_real.py after
changing anything below.

The gripper reaches straight down onto a small block on the bench, lifts it,
carries it over and puts it down at a second spot, then goes home:

    home -> above pick -> pick -> CLOSE -> above pick
         -> above place -> place -> OPEN -> above place -> home

Positions are in the UR3e's base frame (the frame of ir_support's UR3e
model, which is what the joint angles are worked out against), in metres,
with z = 0 on the surface the robot is mounted on. Edit the constants below to
suit the real bench.
"""
from math import pi

import numpy as np
from spatialmath import SE3

HOME = [0.0, -pi / 2, 0.0, -pi / 2, 0.0, 0.0]   # upright, wrist folded
TOOL_LEN = 0.30      # m from the flange to the middle of the finger pads: set long to start, shorten to suit the real gripper
BLOCK = 0.05         # m, side of the block being moved
PICK = (0.30, -0.15)   # (x, y) of the block's centre at the start
PLACE = (0.15, -0.35)  # (x, y) where it is put down
PICK_Z = BLOCK / 2 - 0.05   # finger height at the pick: 5 cm below halfway up the block
PLACE_Z = PICK_Z + 0.025    # let go 2.5 cm higher than the pick, so the block drops onto the bench
LIFT = 0.10          # m above the grip height for the approach / lift

MAX_JOINT_SPEED = 0.5  # rad/s: each move's duration is set so no joint goes faster
MIN_MOVE_TIME = 2.0    # s: shortest move sent

# Gripper pointing straight down (flange z axis to -z of the base).
DOWN = SE3.Rx(pi)


def tcp_pose(x, y, z):
    """Flange pose that puts the middle of the finger pads at (x, y, z), pointing down."""
    return SE3(x, y, z) * DOWN * SE3.Tz(-TOOL_LEN)


def plan():
    """The demo as a list of steps, each ("move", name, joints, duration) or
    ("grip", "close"/"open"). Joint angles come from the UR3e model's IK,
    limited to one elbow-up, wrist-down configuration and each seeded from the
    one before, so the arm never flips between poses."""
    from ir_support import UR3e
    robot = UR3e()
    qlim = robot.qlim.copy()
    qlim[:, 0] = [-pi, pi]          # base
    qlim[:, 1] = [-pi, 0]           # shoulder lifts the arm up off the bench
    qlim[:, 2] = [0, pi]            # elbow up
    qlim[:, 3] = [-2 * pi, pi / 2]  # wrist 1
    qlim[:, 4] = [-pi, 0]           # wrist 2
    qlim[:, 5] = [-pi, pi]          # wrist 3
    robot.qlim = qlim

    q = np.array(HOME)
    joints = {}
    # Seed the first solve with an elbow-up, wrist-down pose facing the pick spot
    # (the base angle is pi off the direction, as the UR's arm reaches along -x at zero).
    seed = np.array([np.arctan2(PICK[1], PICK[0]) + pi, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0.0])
    for name, (x, y), z in [("above pick", PICK, PICK_Z + LIFT), ("pick", PICK, PICK_Z),
                            ("above place", PLACE, PLACE_Z + LIFT), ("place", PLACE, PLACE_Z)]:
        sol = robot.ikine_LM(tcp_pose(x, y, z), q0=seed, joint_limits=True, slimit=200)
        if not sol.success:
            raise RuntimeError(f"no IK solution for '{name}' at ({x}, {y}, {z}): move it closer to the robot")
        joints[name] = np.round(sol.q, 5)
        seed = sol.q

    steps = []
    for item in ["open", "above pick", "pick", "close", "above pick", "above place", "place", "open",
                 "above place", "home"]:
        if item in ("close", "open"):
            steps.append(("grip", item))
            continue
        target = np.array(HOME) if item == "home" else joints[item]
        duration = max(np.max(np.abs(target - q)) / MAX_JOINT_SPEED, MIN_MOVE_TIME)
        steps.append(("move", item, [float(v) for v in target], round(float(duration), 1)))
        q = target
    return steps


if __name__ == "__main__":
    # Print the steps ready to paste over STEPS in ur3_real.py.
    print("STEPS = [")
    for s in plan():
        if s[0] == "grip":
            print(f"    ('grip', '{s[1]}'),")
        else:
            joints = "HOME" if s[1] == "home" else s[2]
            print(f"    ('move', '{s[1]}', {joints}, {s[3]}),")
    print("]")
