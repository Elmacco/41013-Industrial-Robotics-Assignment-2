"""
Ways to steam the milk.

A steam profile says where the wand tip sits inside the pitcher over the course
of a steam, as a function of progress u in [0, 1]. The robot holds the wand
still in the world and moves the pitcher around it, so milk_scene turns each
placement into a pitcher pose and then a joint configuration.

Every profile is built from the same barista vocabulary:

* stretch: tip just under the surface, drawing air in to make foam. The foam
  makes the surface rise, so the pitcher has to drop to keep the tip shallow.
* texture: tip deeper and off-centre with the pitcher tilted, so the steam
  rolls the milk into a whirlpool that folds the foam in.

To add a style, write a function u -> WandPlacement (two_phase covers most
drinks) and add it to PROFILES.
"""
from dataclasses import dataclass
from math import cos, pi, radians, sin
from typing import Callable

from barista.milk_layout import MILK_FILL


@dataclass(frozen=True)
class WandPlacement:
    """Wand tip in the pitcher frame (metres) plus the pitcher's tilt.

    x, y: tip offset from the pitcher axis (+x towards the spout).
    z:    tip height above the pitcher floor.
    tilt: roll of the pitcher about its spout axis (radians).
    """
    x: float
    y: float
    z: float
    tilt: float = 0.0


@dataclass(frozen=True)
class SteamProfile:
    description: str
    duration: float                          # seconds of sim time
    placement: Callable[[float], WandPlacement]


def _smoothstep(u):
    u = min(max(u, 0.0), 1.0)
    return u * u * (3.0 - 2.0 * u)


def _blend(a, b, w):
    return a + (b - a) * w


def two_phase(stretch_frac, growth, stretch_depth=0.005, texture_depth=0.02,
              offset=0.015, tilt_deg=8.0, blend=0.08):
    """Stretch for the first `stretch_frac` of the steam, then texture.

    :param stretch_frac: share of the steam spent stretching (0-1).
    :param growth: how much the milk grows from the foam, e.g. 0.3 = +30 %.
    :param stretch_depth: tip depth below the surface while stretching.
    :param texture_depth: tip depth below the surface while texturing.
    :param offset: sideways tip offset from the axis while texturing.
    :param tilt_deg: pitcher roll while texturing.
    :param blend: share of the steam spent easing from one phase to the next,
        so the arm never jumps between them.
    """
    def placement(u):
        # Surface climbs while stretching, then holds.
        surface = MILK_FILL * (1.0 + growth * _smoothstep(u / max(stretch_frac, 1e-6)))
        w = _smoothstep((u - stretch_frac) / blend)    # 0 stretching -> 1 texturing
        depth = _blend(stretch_depth, texture_depth, w)
        return WandPlacement(x=0.0, y=_blend(0.0, offset, w), z=surface - depth,
                             tilt=_blend(0.0, radians(tilt_deg), w))
    return placement


def orbit(radius=0.012, turns=3.0, depth=0.012, growth=0.15):
    """Tip circles the pitcher axis at constant depth: a hand-swirl style."""
    def placement(u):
        surface = MILK_FILL * (1.0 + growth * _smoothstep(u / 0.3))
        r = radius * _smoothstep(u / 0.1)               # ease out from the axis
        a = 2 * pi * turns * u
        return WandPlacement(x=r * cos(a), y=r * sin(a), z=surface - depth)
    return placement


PROFILES = {
    "latte": SteamProfile("short stretch, long whirlpool: silky microfoam",
                          duration=7.0, placement=two_phase(0.25, growth=0.25)),
    "cappuccino": SteamProfile("long stretch for thick, airy foam",
                               duration=7.0, placement=two_phase(0.55, growth=0.55, tilt_deg=5.0)),
    "flat_white": SteamProfile("barely any stretch, deep and thin foam",
                               duration=6.0, placement=two_phase(0.10, growth=0.10, texture_depth=0.025)),
    "swirl": SteamProfile("tip orbits the axis instead of a fixed whirlpool",
                          duration=7.0, placement=orbit()),
}
DEFAULT_PROFILE = "latte"
