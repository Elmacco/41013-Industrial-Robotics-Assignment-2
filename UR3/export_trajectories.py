"""
Export the UR3's moves from the full barista simulation for the real robot.

    python UR3/export_trajectories.py

Plans the UR3 runner's cup moves exactly as barista.full_scene does (the same
collision-checked joint paths), and writes them to UR3/ur3_trajectories.json:

    trips      each cup move (e.g. dispenser -> espresso): its start and end
               spots, and segments of joint angles sampled every DT_EXPORT
               seconds, with the gripper state
    sequence   which trips make each drink, in order
    spots      where each cup position is relative to the UR3 base, so the
               real props can be set out in the same places

ur3_real.py sends these to the real UR3, and jointstatesim.py animates them in
Swift, so all three use the very same joint angles. Re-run this whenever the
simulation's UR3 moves change.
"""
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from barista import full_layout as layout  # noqa: E402
from barista import full_scene  # noqa: E402

OUT = Path(__file__).with_name("ur3_trajectories.json")
DT_EXPORT = 0.10                 # seconds between exported trajectory points
TRIPS = {                        # file key -> (from spot, to spot)
    "dispenser_to_espresso": ("dispenser", "espresso"),
    "espresso_to_handoff": ("espresso", "handoff"),
    "handoff_to_drawer": ("handoff", "drawer"),
    "espresso_to_drawer": ("espresso", "drawer"),
}
SEQUENCE = {
    "milk drink (latte, cappuccino, flat white)": ["dispenser_to_espresso", "espresso_to_handoff",
                                                   "handoff_to_drawer"],
    "espresso": ["dispenser_to_espresso", "espresso_to_drawer"],
}


def export():
    scene = full_scene.build_scene()
    planner = full_scene.OrderPlanner(scene)
    robot = scene.robots["runner"]
    base = np.array([*layout.UR3_BASE, layout.TABLE_TOP_Z])
    step = int(round(DT_EXPORT / full_scene.DT))

    trips, lowest = {}, np.inf
    for key, (a, b) in TRIPS.items():
        segments = []
        for seg in planner.runner(a, b):
            rows = seg.qs[::step]
            if not np.allclose(rows[-1], seg.qs[-1]):
                rows = np.vstack([rows, seg.qs[-1]])          # always end exactly on target
            segments.append(dict(name=seg.name.removeprefix("UR3: "), holding=bool(seg.holding),
                                 points=np.round(rows, 5).tolist()))
            lowest = min(lowest, min(robot.fkine(q).t[2] for q in seg.qs) - layout.TABLE_TOP_Z)
        trips[key] = dict(start=a, end=b, segments=segments)

    spots = {}
    for name, T in full_scene.SPOTS.items():
        x, y, z = np.round(T.t - base, 4)
        spots[name] = dict(x=float(x), y=float(y), z=float(z),
                           note="cup floor centre, metres from the UR3 base centre "
                                "(x, y on the mounting plane, z up)")

    data = dict(
        source="barista.full_scene: the UR3 runner's planned, collision-checked cup moves",
        joint_names=["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        units="radians",
        dt=DT_EXPORT,
        home=np.round(full_scene.UR3_HOME_Q, 5).tolist(),
        lowest_flange_height=round(float(lowest), 3),
        cup_in_flange=np.round(full_scene.GRASP_CUP.A, 6).tolist(),
        spots=spots,
        sequence=SEQUENCE,
        trips=trips,
    )
    OUT.write_text(json.dumps(data, indent=1))
    n = sum(len(s["points"]) for t in trips.values() for s in t["segments"])
    print(f"wrote {OUT.name}: {len(trips)} trips, {n} points, "
          f"lowest flange {lowest * 1000:.0f} mm above the mounting plane")


if __name__ == "__main__":
    export()
