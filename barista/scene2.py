"""Builds the scene described in layout2.py into a Swift environment.

The shared machinery lives in build.py; what is specific to this scene is the
collection drawer motion below.
"""
import sys
from pathlib import Path

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from barista import build  # noqa: E402
from barista import layout2 as layout  # noqa: E402


def build_scene(env=None):
    return build.build_scene(layout, env)


def check_reach(scene):
    return build.check_reach(scene, layout)


# ---------------------------------------------------------------- collection drawer
def drawer_pose(s):
    """Pose of the collection drawer at travel fraction s (0 = loaded, 1 = out)."""
    s = min(max(float(s), 0.0), 1.0)
    y = layout.DRAWER_Y_IN + s * (layout.DRAWER_Y_OUT - layout.DRAWER_Y_IN)
    return build.pose(0.0, y, layout.DRAWER_Z, 0.0)


def set_drawer(scene, s):
    """Move the drawer to travel fraction s. No-op if the scene has no drawer."""
    tray = scene.props.get("drawer_tray")
    if tray is not None:
        tray.T = drawer_pose(s).A
    return s


def _smoothstep(u):
    return u * u * (3.0 - 2.0 * u)


def animate(scene, env, cycles=1, travel=2.0, dwell=1.0, dt=0.02):
    """Run the collection drawer out and back `cycles` times.

    Swift re-sends every shape pose on each env.step(), so moving the tray is
    just a matter of setting its transform between steps.
    """
    for _ in range(max(int(cycles), 0)):
        for start, end in ((0.0, 1.0), (1.0, 0.0)):
            steps = max(int(travel / dt), 1)
            for i in range(steps + 1):
                set_drawer(scene, start + (end - start) * _smoothstep(i / steps))
                env.step(dt)
            for _ in range(max(int(dwell / dt), 1)):
                env.step(dt)


if __name__ == "__main__":
    for (robot, station), ok in check_reach(build_scene()).items():
        print(f"{'OK ' if ok else 'FAIL'}  {robot:9s} -> {station}")
