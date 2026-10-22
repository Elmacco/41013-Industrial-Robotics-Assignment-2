"""
Simulated rehearsal of ur3_real.py: the UR3e pick-and-place demo, in Swift.

    python UR3/jointstatesim.py
    python UR3/jointstatesim.py --speed 2      # play back twice as fast

Animates exactly the joint states and move durations in the control script's
STEPS list (imported from it), so the motion can be checked by eye before it
goes to the real arm. Between targets the arm follows a smooth joint-space
path, as the real controller does. A rough two-finger gripper is drawn on the
flange; it closes and opens where the control script calls the OnRobot
gripper, taking the same GRIP_TIME as simple_gripper_control.py. Translucent
markers show where the block starts and where it ends up.
"""
import argparse

import numpy as np
import spatialgeometry as geometry
import swift
from ir_support import UR3e
from roboticstoolbox import jtraj
from spatialmath import SE3

import pick_place as pp
from ur3_real import HOME, STEPS

DT = 0.05                           # s per animation frame
GRIP_TIME = 2.0                     # s, as in simple_gripper_control.py
GAP_OPEN, GAP_CLOSED = 0.085, pp.BLOCK   # m between the finger pads
FINGER = (0.02, 0.012, 0.06)        # finger size along flange x, y, z
GRIPPER_COLOR = (0.25, 0.25, 0.28, 1)
SPOT_COLOR = (0.2, 0.6, 1.0, 0.3)


class Gripper:
    """A body and two fingers that follow the flange, pointing out along its z
    axis. The fingers close along the flange y axis on a point TOOL_LEN out."""

    def __init__(self, env):
        self.gap = GAP_OPEN
        body_len = pp.TOOL_LEN - 0.035
        self.body_offset = SE3.Tz(body_len / 2)
        self.body = geometry.Cuboid(scale=[0.05, 0.09, body_len], color=GRIPPER_COLOR)
        self.fingers = [geometry.Cuboid(scale=list(FINGER), color=GRIPPER_COLOR) for _ in range(2)]
        for g in [self.body, *self.fingers]:
            env.add(g)

    def update(self, flange):
        self.body.T = (flange * self.body_offset).A
        y = self.gap / 2 + FINGER[1] / 2
        for side, f in zip((1, -1), self.fingers):
            f.T = (flange * SE3(0, side * y, pp.TOOL_LEN - 0.005)).A

    def move(self, close, flange, env, speed):
        """Animate the fingers to closed or open over GRIP_TIME, scaled by the playback speed."""
        start, end = self.gap, GAP_CLOSED if close else GAP_OPEN
        steps = max(int(round(GRIP_TIME / DT)), 1)
        for i in range(1, steps + 1):
            self.gap = start + (end - start) * i / steps
            self.update(flange)
            env.step(DT / speed)


def drop(block, env, speed):
    """Let a released block fall straight down onto the bench."""
    T = SE3(block.T)
    v = 0.0
    while T.t[2] > pp.BLOCK / 2:
        v += 9.81 * DT / 4
        T = SE3(0, 0, -min(v * DT / 4, T.t[2] - pp.BLOCK / 2)) * T
        block.T = T.A
        env.step(DT / 4 / speed)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument("--speed", type=float, default=1.0, help="playback speed, multiple of real time")
    args = p.parse_args(argv)

    moves = {s[1]: s[2] for s in STEPS if s[0] == "move"}

    env = swift.Swift()
    env.launch(realtime=True)
    ur3e = UR3e()
    ur3e.q = HOME
    ur3e.add_to_env(env)
    env.add(geometry.Cuboid(scale=[1.2, 1.2, 0.02], pose=SE3(0, 0, -0.011), color=(0.55, 0.40, 0.28, 1)))
    # Where the finger pads are at the pick and place poses.
    pick_tcp, place_tcp = (ur3e.fkine(moves[k]) * SE3.Tz(pp.TOOL_LEN) for k in ("pick", "place"))
    for T in (pick_tcp, place_tcp):
        env.add(geometry.Cylinder(radius=pp.BLOCK, length=0.004, color=SPOT_COLOR, pose=SE3(T.t[0], T.t[1], 0.002)))
    # The block starts at the pick spot, turned to line up with the fingers.
    block = geometry.Cuboid(scale=[pp.BLOCK] * 3, color=(0.9, 0.3, 0.2, 1),
                            pose=SE3(pick_tcp.t[0], pick_tcp.t[1], pp.BLOCK / 2) * SE3.Rz(pick_tcp.rpy()[2]))
    env.add(block)
    gripper = Gripper(env)
    gripper.update(ur3e.fkine(ur3e.q))
    env.set_camera_pose([1.0, -1.2, 0.8], [0.15, -0.2, 0.1])

    print("UR3e pick-and-place, from ur3_real.py STEPS")
    print(f"  block at ({pick_tcp.t[0]:.3f}, {pick_tcp.t[1]:.3f}), put down at "
          f"({place_tcp.t[0]:.3f}, {place_tcp.t[1]:.3f}) (m, robot base frame); tool length {pp.TOOL_LEN} m")
    input("Press ENTER to start")
    held = None   # the block's pose in the flange frame while gripped
    for s in STEPS:
        flange = ur3e.fkine(ur3e.q)
        if s[0] == "grip":
            print(f"  GRIPPER: {s[1]}_gripper(client)")
            gripper.move(s[1] == "close", flange, env, args.speed)
            if s[1] == "close":
                held = flange.inv() * SE3(block.T)
            else:
                held = None
                drop(block, env, args.speed)
            continue
        _, name, target, duration = s
        print(f"  {name} ({duration:.1f} s)")
        for q in jtraj(ur3e.q, target, max(int(round(duration / DT)), 2)).q:
            ur3e.q = q
            flange = ur3e.fkine(q)
            gripper.update(flange)
            if held is not None:
                block.T = (flange * held).A
            env.step(DT / args.speed)

    print("Done: back at home.")
    input("Press ENTER to close")
    env.close()


if __name__ == "__main__":
    main()
