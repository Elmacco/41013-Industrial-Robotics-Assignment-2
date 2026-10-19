"""
A glass cup that can be carried around with its contents.

The glass is translucent, so the liquid inside shows. Swift cannot resize a
shape once it is added, so the liquid is a stack of thin discs: the discs up
to the current level are shown, the rest are parked out of sight. Coffee fills
from the bottom, milk sits on top of it.
"""
import spatialgeometry as geometry
from spatialmath import SE3

GLASS = (0.85, 0.90, 0.95, 0.35)
COFFEE = (0.40, 0.24, 0.12, 1)
MILK = (0.97, 0.95, 0.90, 1)
HIDDEN = SE3(0.0, 0.0, -5.0)
SLICE = 0.01                    # disc thickness = level resolution


class Cup:
    def __init__(self, radius, height, T=SE3()):
        self.radius, self.height = radius, height
        self.glass = geometry.Cylinder(radius=radius, length=height, color=GLASS)
        n = int((height - 0.008) / SLICE)
        r = radius - 0.005
        # One coffee and one milk disc per slot; only one of each slot shows.
        self.coffee = [geometry.Cylinder(radius=r, length=SLICE * 1.05, color=COFFEE) for _ in range(n)]
        self.milk = [geometry.Cylinder(radius=r, length=SLICE * 1.05, color=MILK) for _ in range(n)]
        self.T = T                      # pose of the cup's floor centre
        self.espresso = 0.0             # coffee height above the floor
        self.level = 0.0                # total liquid height (coffee + milk)
        self.visible = True
        self.update()

    def shapes(self):
        return [self.glass, *self.coffee, *self.milk]

    def add_to_env(self, env):
        for s in self.shapes():
            env.add(s)

    def set(self, T=None, espresso=None, level=None, visible=None):
        if T is not None:
            self.T = T
        if espresso is not None:
            self.espresso = espresso
        if level is not None:
            self.level = level
        if visible is not None:
            self.visible = visible
        self.update()

    def update(self):
        if not self.visible:
            for s in self.shapes():
                s.T = HIDDEN.A
            return
        self.glass.T = (self.T * SE3.Tz(self.height / 2)).A
        top = max(self.level, self.espresso)
        for i, (c, m) in enumerate(zip(self.coffee, self.milk)):
            z0 = i * SLICE
            mid = (self.T * SE3.Tz(z0 + SLICE / 2 + 0.002)).A
            filled = z0 + SLICE / 2 < top
            is_coffee = z0 + SLICE / 2 < self.espresso
            c.T = mid if filled and is_coffee else HIDDEN.A
            m.T = mid if filled and not is_coffee else HIDDEN.A

    def spec(self):
        """Collision-check stand-in: a box spec at the cup's current pose."""
        x, y, z = self.T.t
        return dict(shape="cylinder", size=(self.radius, self.height), pose=(x, y, z, 0.0))
