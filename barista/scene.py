"""Builds the barista cell described in layout.py into a Swift environment."""
import sys
from dataclasses import dataclass, field
from pathlib import Path

import ir_support
import numpy as np
import spatialgeometry as geometry
from ir_support_extra_parts import part_mesh
from spatialmath import SE3

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from barista import layout  # noqa: E402


def _pose(x, y, z, yaw):
    return SE3(x, y, z) * SE3.Rz(yaw)


@dataclass
class BaristaScene:
    robots: dict = field(default_factory=dict)
    props: dict = field(default_factory=dict)
    placeholders: dict = field(default_factory=dict)


def make_robot(spec):
    robot = getattr(ir_support, spec["model"])()
    # Pre-multiply so models with a built-in base rotation (LinearUR3) keep it.
    robot.base = _pose(*spec["base"]) * robot.base
    return robot


def make_placeholder(spec):
    x, y, z, yaw = spec["pose"]
    if spec["shape"] == "box":
        sx, sy, sz = spec["size"]
        return geometry.Cuboid(scale=[sx, sy, sz], pose=_pose(x, y, z + sz / 2, yaw), color=spec["color"])
    radius, height = spec["size"]
    return geometry.Cylinder(radius=radius, length=height, pose=_pose(x, y, z + height / 2, yaw), color=spec["color"])


def build_scene(env=None):
    """Create all robots and objects; add them to env if one is given."""
    scene = BaristaScene()
    for name, spec in layout.ROBOTS.items():
        scene.robots[name] = make_robot(spec)
    for name, (part, pose, color) in layout.PROPS.items():
        scene.props[name] = part_mesh(part, pose=_pose(*pose), color=color)
    for name, spec in layout.PLACEHOLDERS.items():
        scene.placeholders[name] = make_placeholder(spec)

    if env is not None:
        for robot in scene.robots.values():
            robot.add_to_env(env)
        for obj in (*scene.props.values(), *scene.placeholders.values()):
            env.add(obj)
    return scene


def check_reach(scene):
    """Position-only IK for every station; returns {(robot, station): ok}."""
    results = {}
    for robot_name, stations in layout.STATIONS.items():
        robot = scene.robots[robot_name]
        for station, xyz in stations.items():
            sol = robot.ikine_LM(SE3(*xyz), q0=robot.q, mask=[1, 1, 1, 0, 0, 0],
                                 joint_limits=True, slimit=200)
            err = np.linalg.norm(robot.fkine(sol.q).t - np.array(xyz)) if sol.success else np.inf
            results[(robot_name, station)] = bool(sol.success and err < 1e-3)
    return results


if __name__ == "__main__":
    for (robot, station), ok in check_reach(build_scene()).items():
        print(f"{'OK ' if ok else 'FAIL'}  {robot:9s} -> {station}")
