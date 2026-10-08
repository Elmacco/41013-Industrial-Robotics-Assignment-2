"""
Shared machinery for the barista scenes.

Three things live here, in this order:

1. Layout authoring helpers (``guard_walls``, ``tunnel_walls``) that return
   ready-made placeholder specs, so a layout says what a structure *is* instead
   of spelling out every panel's arithmetic.
2. The Swift scene builder (``build_scene``) and the reach check.
3. ``describe``, a geometry dump that prints where every object actually ended
   up, so a layout can be checked by eye without opening Swift.

Every function takes the layout module to work from as an argument, so the
per-scene modules differ only in which layout they hand over.
"""
import re
import sys
from dataclasses import dataclass, field
from functools import lru_cache
from pathlib import Path

import ir_support
import numpy as np
import spatialgeometry as geometry
from ir_support_extra_parts import part_mesh, part_path
from spatialmath import SE3

# Allow running a scene file directly (e.g. VS Code's Run button), not just -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from barista.cs66 import CS66  # noqa: E402

# Robots that are not in ir_support, by their layout model name.
CUSTOM_ROBOTS = {"CS66": CS66}


def pose(x, y, z, yaw):
    """SE3 for a layout (x, y, z, yaw) tuple."""
    return SE3(x, y, z) * SE3.Rz(yaw)


# ------------------------------------------------- layout authoring helpers
def _box(size, at, color):
    return dict(shape="box", size=size, pose=at, color=color)


def guard_walls(inner, height, thickness, color, opening=None, prefix="glass"):
    """Four walls standing on the floor around a rectangular clear area.

    :param inner: (x, y) half-extents of the clear area. The INNER face of each
        wall sits exactly on these, so the enclosed space is what you asked for
        and the walls grow outwards.
    :param height: wall height above the floor.
    :param thickness: wall thickness.
    :param color: rgba; alpha below 1 renders translucent.
    :param opening: ``None``, or ``(width, z0, z1)`` cutting a rectangular hole
        centred in the +y wall, which is then built as two full-height side
        panels plus a sill below the hole and a lintel above it.
    :param prefix: name prefix for the returned entries.
    :returns: ``{name: placeholder spec}`` to merge into ``PLACEHOLDERS``.

    Corners overlap by design: the -y/+y walls span the full outer width so the
    run reads as sealed rather than as four separate rectangles.
    """
    ix, iy = inner
    cx, cy = ix + thickness / 2, iy + thickness / 2      # wall centrelines
    ox, oy = ix + thickness, iy + thickness              # outer half-extents

    walls = {
        f"{prefix}_back": _box((2 * ox, thickness, height), (0.0, -cy, 0.0, 0.0), color),
        f"{prefix}_left": _box((thickness, 2 * oy, height), (-cx, 0.0, 0.0, 0.0), color),
        f"{prefix}_right": _box((thickness, 2 * oy, height), (cx, 0.0, 0.0, 0.0), color),
    }

    if opening is None:
        walls[f"{prefix}_front"] = _box((2 * ox, thickness, height), (0.0, cy, 0.0, 0.0), color)
        return walls

    width, z0, z1 = opening
    side = ox - width / 2                                # width of each side panel
    walls[f"{prefix}_front_l"] = _box((side, thickness, height), (-(ox - side / 2), cy, 0.0, 0.0), color)
    walls[f"{prefix}_front_r"] = _box((side, thickness, height), (ox - side / 2, cy, 0.0, 0.0), color)
    walls[f"{prefix}_front_sill"] = _box((width, thickness, z0), (0.0, cy, 0.0, 0.0), color)
    walls[f"{prefix}_front_lintel"] = _box((width, thickness, height - z1), (0.0, cy, z1, 0.0), color)
    return walls


def tunnel_walls(half_width, y_range, z_range, wall, color, prefix="shroud"):
    """A square tunnel along y: floor, roof and two side walls.

    :param half_width: half-width of the clear bore in x.
    :param y_range: (y0, y1) the tunnel spans.
    :param z_range: (z0, z1) of the clear bore. Floor and roof sit outside it.
    :param wall: wall thickness.
    :returns: ``{name: placeholder spec}`` to merge into ``PLACEHOLDERS``.
    """
    y0, y1 = y_range
    z0, z1 = z_range
    depth, mid_y = y1 - y0, (y0 + y1) / 2
    span = (2 * half_width + 2 * wall, depth, wall)      # floor/roof footprint
    side = (wall, depth, z1 - z0 + wall)                 # side walls close the corner
    cx = half_width + wall / 2
    return {
        f"{prefix}_floor": _box(span, (0.0, mid_y, z0 - wall, 0.0), color),
        f"{prefix}_roof": _box(span, (0.0, mid_y, z1, 0.0), color),
        f"{prefix}_left": _box(side, (-cx, mid_y, z0 - wall, 0.0), color),
        f"{prefix}_right": _box(side, (cx, mid_y, z0 - wall, 0.0), color),
    }


# ------------------------------------------------- scene building
@dataclass
class Scene:
    robots: dict = field(default_factory=dict)
    props: dict = field(default_factory=dict)
    placeholders: dict = field(default_factory=dict)


def make_robot(spec):
    model = CUSTOM_ROBOTS.get(spec["model"]) or getattr(ir_support, spec["model"])
    robot = model()
    # Pre-multiply so models with a built-in base rotation (LinearUR3) keep it.
    robot.base = pose(*spec["base"]) * robot.base
    return robot


def make_placeholder(spec):
    x, y, z, yaw = spec["pose"]
    if spec["shape"] == "box":
        sx, sy, sz = spec["size"]
        return geometry.Cuboid(scale=[sx, sy, sz], pose=pose(x, y, z + sz / 2, yaw), color=spec["color"])
    radius, height = spec["size"]
    return geometry.Cylinder(radius=radius, length=height, pose=pose(x, y, z + height / 2, yaw), color=spec["color"])


def build_scene(layout, env=None):
    """Create all robots and objects in `layout`; add them to env if given."""
    scene = Scene()
    for name, spec in layout.ROBOTS.items():
        scene.robots[name] = make_robot(spec)
    for name, (part, at, color) in layout.PROPS.items():
        scene.props[name] = part_mesh(part, pose=pose(*at), color=color)
    for name, spec in layout.PLACEHOLDERS.items():
        scene.placeholders[name] = make_placeholder(spec)

    if env is not None:
        for robot in scene.robots.values():
            robot.add_to_env(env)
        for obj in (*scene.props.values(), *scene.placeholders.values()):
            env.add(obj)
    return scene


def check_reach(scene, layout):
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


# ------------------------------------------------- geometry dump
@lru_cache(maxsize=None)
def mesh_bounds(part):
    """(min, max) of a part mesh in its own frame, from the DAE vertex data.

    Worth reading rather than assuming: part origins are not all centred. The
    Tray has its origin at bottom-centre, so its pose z is the underside, while
    a PintCup's origin sits at its middle.
    """
    text = part_path(part).read_text(encoding="utf-8", errors="ignore")
    arrays = re.findall(
        r'<float_array[^>]*id="[^"]*[Pp]osition[^"]*"[^>]*>(.*?)</float_array>', text, re.S)
    pts = []
    for body in arrays:
        v = np.fromstring(body.replace("\n", " "), sep=" ")
        if v.size and v.size % 3 == 0:
            pts.append(v.reshape(-1, 3))
    if not pts:
        return None
    allpts = np.vstack(pts)
    return allpts.min(0), allpts.max(0)


def extents(layout, name):
    """World-space (min, max) of a named prop or placeholder in `layout`."""
    if name in layout.PROPS:
        part, at, _ = layout.PROPS[name]
        origin = np.array(at[:3], dtype=float)
        bounds = mesh_bounds(part)
        if bounds is None:
            return origin, origin
        return origin + bounds[0], origin + bounds[1]
    spec = layout.PLACEHOLDERS[name]
    x, y, z, _ = spec["pose"]
    if spec["shape"] == "box":
        sx, sy, sz = spec["size"]
    else:
        radius, sz = spec["size"]
        sx = sy = 2 * radius
    lo = np.array([x - sx / 2, y - sy / 2, z], dtype=float)
    return lo, lo + np.array([sx, sy, sz], dtype=float)


def describe(layout, stream=None):
    """Print where every object in `layout` actually ends up.

    Yaw is ignored, so a rotated object's figures are its unrotated extents.
    """
    out = stream or sys.stdout
    title = (layout.__doc__ or layout.__name__).strip().splitlines()[0]
    print(f"\n{title}", file=out)

    if layout.ROBOTS:
        print(f"\n  {'robot':12s} {'model':11s} base (x, y, z, yaw)", file=out)
        for name, spec in layout.ROBOTS.items():
            b = spec["base"]
            role = spec.get("role", "")
            print(f"  {name:12s} {spec['model']:11s} "
                  f"({b[0]:+.3f}, {b[1]:+.3f}, {b[2]:+.3f}, {b[3]:+.2f})  {role}", file=out)

    names = [*layout.PROPS, *layout.PLACEHOLDERS]
    print(f"\n  {'object':20s} {'kind':9s} {'x':^15s} {'y':^15s} {'z':^15s}", file=out)
    lows, highs = [], []
    for name in names:
        lo, hi = extents(layout, name)
        lows.append(lo)
        highs.append(hi)
        kind = layout.PROPS[name][0] if name in layout.PROPS else layout.PLACEHOLDERS[name]["shape"]
        print(f"  {name:20s} {kind:9.9s} "
              f"[{lo[0]:+.3f},{hi[0]:+.3f}] [{lo[1]:+.3f},{hi[1]:+.3f}] [{lo[2]:+.3f},{hi[2]:+.3f}]", file=out)

    if names:
        lo, hi = np.min(lows, axis=0), np.max(highs, axis=0)
        print(f"  {'-- scene bounds':20s} {'':9s} "
              f"[{lo[0]:+.3f},{hi[0]:+.3f}] [{lo[1]:+.3f},{hi[1]:+.3f}] [{lo[2]:+.3f},{hi[2]:+.3f}]", file=out)
    print(f"\n  {len(layout.ROBOTS)} robots, {len(layout.PROPS)} props, "
          f"{len(layout.PLACEHOLDERS)} placeholders\n", file=out)
