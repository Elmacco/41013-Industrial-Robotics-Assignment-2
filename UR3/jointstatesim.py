"""
Simulated rehearsal of ur3_real.py: the same UR3 joint angles, in Swift.

    python UR3/jointstatesim.py                          # every milk-drink trip
    python UR3/jointstatesim.py --drink espresso
    python UR3/jointstatesim.py --trip espresso_to_handoff

Animates exactly the joint paths ur3_real.py sends to the real robot (both
read ur3_trajectories.json, exported from the full simulation), so the motion
can be checked by eye before it goes to the real arm. The UR3 stands on a
bench with translucent markers at the cup spots, and a cup rides in the
gripper while it is held, so the real props can be set out to match.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import spatialgeometry as geometry
import swift
from ir_support import UR3
from spatialmath import SE3

DATA = Path(__file__).with_name("ur3_trajectories.json")
CUP_R, CUP_H = 0.04, 0.10
SPOT_COLOR = (0.2, 0.6, 1.0, 0.25)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument("--trip", help="animate one trip (see ur3_real.py --list)")
    p.add_argument("--drink", default="milk", help="animate every trip for a drink: 'milk' or 'espresso'")
    p.add_argument("--speed", type=float, default=1.0, help="playback speed, fraction of real time")
    args = p.parse_args(argv)

    data = json.loads(DATA.read_text())
    if args.trip:
        trips = [args.trip]
    else:
        match = [k for k in data["sequence"] if k.startswith(args.drink)]
        if not match:
            sys.exit(f"unknown drink {args.drink!r}")
        trips = data["sequence"][match[0]]
    cup_in_flange = SE3(np.array(data["cup_in_flange"]), check=False)

    env = swift.Swift()
    env.launch(realtime=True)
    ur3 = UR3()
    ur3.q = data["home"]
    ur3.add_to_env(env)
    env.add(geometry.Cuboid(scale=[1.4, 1.4, 0.02], pose=SE3(0, 0, -0.011), color=(0.55, 0.40, 0.28, 1)))
    for s in data["spots"].values():
        env.add(geometry.Cylinder(radius=CUP_R + 0.01, length=0.004, color=SPOT_COLOR,
                                  pose=SE3(s["x"], s["y"], s["z"] + 0.002)))
    # The cup starts where the first trip picks it up, and stays wherever it is put down.
    start = data["spots"][data["trips"][trips[0]]["start"]]
    cup = geometry.Cylinder(radius=CUP_R, length=CUP_H, color=(0.85, 0.90, 0.95, 0.5),
                            pose=SE3(start["x"], start["y"], start["z"] + CUP_H / 2))
    env.add(cup)
    env.set_camera_pose([1.4, -1.4, 1.1], [0, 0, 0.2])

    dt = data["dt"] / args.speed
    print(f"Joint angles from {DATA.name} ({data['source']})")
    for key in trips:
        input(f"\nPress ENTER to animate {key}")
        for seg in data["trips"][key]["segments"]:
            print(f"  {seg['name']}")
            for q in seg["points"]:
                ur3.q = q
                if seg["holding"]:
                    cup.T = (ur3.fkine(ur3.q) * cup_in_flange * SE3.Tz(CUP_H / 2)).A
                env.step(dt)
        print(f"Done: {key}")

    input("\nPress ENTER to close")
    env.close()


if __name__ == "__main__":
    main()
