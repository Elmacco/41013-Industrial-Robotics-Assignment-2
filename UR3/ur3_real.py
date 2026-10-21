"""
Run the simulation's UR3 cup moves on the real UR3, using the lab's control
script (new_ur_control_script.py, unchanged).

    python UR3/ur3_real.py --list                          # what can be run
    python UR3/ur3_real.py --trip dispenser_to_espresso    # one trip
    python UR3/ur3_real.py --drink espresso                # every trip for a drink
    python UR3/ur3_real.py --trip espresso_to_handoff --dry-run   # print, send nothing

The joint angles come from ur3_trajectories.json, exported from the full
simulation by export_trajectories.py, so the real arm goes through the same
joint states as the simulated UR3. Rehearse them first with jointstatesim.py.

Each move is one call to the lab script's move_ur_joint_positions(client,
joint_positions, duration): it reads the arm's current joints, sends a move
from there to the target taking `duration` seconds, and waits for it. Between
two targets the controller interpolates in joint space, so to stay close to
the simulated (collision-checked) route each segment is sent as its end pose
plus intermediate waypoints every WAYPOINT_EVERY seconds of motion. The arm
pauses briefly at each waypoint, because the lab script waits after every
move.

Durations are the simulated ones divided by --speed (default 0.5, half
speed), stretched so no joint moves faster than MAX_JOINT_SPEED, and never
shorter than MIN_MOVE_TIME.

SAFETY: the real bench is not the simulated cell. The paths reach down to
about 6 cm above the robot's mounting plane and out to the cup spots listed by
--list. Clear that space (or set out matching props), keep the physical
e-stop in reach, start with a low --speed and --step, and follow the lab's
risk assessment and SWMS. The robot's e-stop is the way to stop it: Ctrl+C
only stops the script sending further moves.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

DATA = Path(__file__).with_name("ur3_trajectories.json")
ROS_HOST = "192.168.27.1"      # the lab's ROS bridge (as in new_ur_control_script.py)
ROS_PORT = 9090
WAYPOINT_EVERY = 1.0           # s of simulated motion between waypoints within a segment
MAX_JOINT_SPEED = 0.5          # rad/s: a move's duration is stretched to respect this
MIN_MOVE_TIME = 1.0            # s: shortest move sent


def load():
    return json.loads(DATA.read_text())


def describe(data):
    print(f"Trajectories from: {data['source']}")
    print(f"Home (start and end of every trip): {data['home']}")
    print(f"Lowest flange height: {data['lowest_flange_height'] * 1000:.0f} mm above the mounting plane\n")
    print("Cup spots (from the UR3 base centre, metres):")
    for name, s in data["spots"].items():
        print(f"  {name:10s} x {s['x']:+.3f}  y {s['y']:+.3f}  z {s['z']:+.3f}")
    print("\nTrips:")
    for key, trip in data["trips"].items():
        segs = trip["segments"]
        secs = sum(len(s["points"]) - 1 for s in segs) * data["dt"]
        print(f"  {key:24s} {trip['start']} -> {trip['end']}: {len(segs)} segments, "
              f"{secs:.1f} s simulated")
    print("\nDrinks:")
    for drink, trips in data["sequence"].items():
        print(f"  {drink}: {' -> '.join(trips)}")


def waypoints(seg, dt_sim):
    """The joint states to send for one segment: every WAYPOINT_EVERY seconds of
    its motion, always ending on its last point. Returns [(joints, sim seconds)]."""
    points = np.array(seg["points"])
    every = max(int(round(WAYPOINT_EVERY / dt_sim)), 1)
    idx = list(range(every, len(points) - 1, every)) + [len(points) - 1]
    out, prev = [], 0
    for i in idx:
        if i > 0:
            out.append((points[i], (i - prev) * dt_sim))
            prev = i
    return out


def move_time(q_from, q_to, sim_seconds, speed):
    """Duration for one move: the simulated time at --speed, stretched so no
    joint beats MAX_JOINT_SPEED, and at least MIN_MOVE_TIME."""
    slowest = np.max(np.abs(np.asarray(q_to) - np.asarray(q_from))) / MAX_JOINT_SPEED
    return max(sim_seconds / speed, slowest, MIN_MOVE_TIME)


def run(trips, data, speed, step, send, ask):
    """Send every waypoint of every trip, in order, through `send(joints, duration)`."""
    q = np.array(data["home"])
    for key in trips:
        print(f"\n=== {key}")
        holding = False
        for seg in data["trips"][key]["segments"]:
            if seg["holding"] != holding:
                ask("GRIPPER: close it on the cup now, then press ENTER" if seg["holding"]
                    else "GRIPPER: open it now, then press ENTER")
                holding = seg["holding"]
            wps = waypoints(seg, data["dt"])
            if not wps:
                continue
            if step:
                ask(f"  {seg['name']} ({len(wps)} moves): press ENTER to send")
            else:
                print(f"  {seg['name']} ({len(wps)} moves)")
            for target, sim_s in wps:
                duration = move_time(q, target, sim_s, speed)
                send([round(float(v), 5) for v in target], duration)
                q = target


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument("--list", action="store_true", help="show the trips, drinks and cup spots, then exit")
    p.add_argument("--trip", help="run one trip (see --list)")
    p.add_argument("--drink", help="run every trip for a drink: 'espresso' or 'milk'")
    p.add_argument("--speed", type=float, default=0.5, help="fraction of the simulated speed, 0-1 (default 0.5)")
    p.add_argument("--step", action="store_true", help="wait for ENTER before every segment")
    p.add_argument("--host", default=ROS_HOST)
    p.add_argument("--port", type=int, default=ROS_PORT)
    p.add_argument("--dry-run", action="store_true", help="print what would be sent; no robot")
    args = p.parse_args(argv)

    data = load()
    if args.list or not (args.trip or args.drink):
        describe(data)
        return
    if not 0 < args.speed <= 1.0:
        sys.exit("--speed must be between 0 and 1")
    if args.trip:
        if args.trip not in data["trips"]:
            sys.exit(f"unknown trip {args.trip!r}; see --list")
        trips = [args.trip]
    else:
        match = [k for k in data["sequence"] if k.startswith(args.drink)]
        if not match:
            sys.exit(f"unknown drink {args.drink!r}; see --list")
        trips = data["sequence"][match[0]]

    if args.dry_run:
        def send(joints, duration):
            print(f"    move_ur_joint_positions(client, {joints}, duration={duration:.1f})")
        run(trips, data, args.speed, args.step, send, ask=print)
        return

    import roslibpy
    from new_ur_control_script import move_ur_joint_positions   # the lab's script, unchanged

    client = roslibpy.Ros(host=args.host, port=args.port)
    client.run()
    try:
        home = data["home"]
        input(f"Press ENTER to move to the start pose {home} (slowly), Ctrl+C to quit")
        move_ur_joint_positions(client, home, duration=8.0)
        run(trips, data, args.speed, args.step,
            send=lambda joints, duration: move_ur_joint_positions(client, joints, duration=duration),
            ask=input)
        print("\nDone: the arm is back at home.")
    except KeyboardInterrupt:
        print("\n[APP] interrupted: no further moves sent (use the e-stop to stop the arm)")
    finally:
        client.terminate()


if __name__ == "__main__":
    main()
