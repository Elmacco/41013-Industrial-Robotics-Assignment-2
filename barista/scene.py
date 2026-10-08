"""Builds the barista cell described in layout.py into a Swift environment.

All the machinery lives in build.py; this module only says which layout to use.
"""
import sys
from pathlib import Path

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from barista import build, layout  # noqa: E402


def build_scene(env=None):
    return build.build_scene(layout, env)


def check_reach(scene):
    return build.check_reach(scene, layout)


if __name__ == "__main__":
    for (robot, station), ok in check_reach(build_scene()).items():
        print(f"{'OK ' if ok else 'FAIL'}  {robot:9s} -> {station}")
