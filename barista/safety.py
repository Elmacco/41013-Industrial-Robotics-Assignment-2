"""
Simulated safety sensors at the cell's maintenance doorway, and a member of
staff (SafetyPerson, in hi-vis) to set them off.

    area scanner    watches the floor in front of the doorway. Someone in it:
                    the robots run at REDUCED_SPEED, and go back to full
                    speed by themselves once they leave.
    light curtain   across the doorway. Broken: a protective stop through the
                    same latch as the e-stops (estop.py), held while broken.
    presence        someone inside the cell holds the stop too, because a
                    light curtain only sees what is in its plane, and would
                    clear again once a person is all the way through.

So walking in stops every robot, and they stay stopped until the person has
left and someone presses Reset (which rechecks the paths) then Resume.

The person walks on their own once sent (`send`), and update() reads the
sensors from where they are on every frame. The robots' programs never ask
the sensors anything: the signals arrive whenever the person gets there,
like a real sensor's would. This is a simulation, not a certified safety
system.
"""
from math import atan2, pi

import numpy as np
from ir_support_extra_parts import part_mesh
from spatialmath import SE3

from barista import full_layout as layout

REDUCED_SPEED = 0.5         # share of full speed while someone is near the doorway
WALK_SPEED = 1.0            # m/s
BODY_R = 0.25               # m: a person's footprint, as the sensors see it
MODEL_YAW = pi              # the SafetyPerson mesh faces -x: turn it to face the way it walks
CURTAIN = "light curtain at the maintenance door"
INSIDE = "person inside the cell"

_X, _Y = layout.DOOR_X, layout.DOOR_Y
SPOTS = {                   # where the person can be sent, on the floor
    "away": (_X + 1.75, _Y - 0.60),
    "door": (_X + 0.70, _Y),                  # in the scanner zone, short of the curtain
    "inside": (_X - 0.20, _Y),                # in the walk-round gap behind the glass
}
# How to get from one spot to another: always through the doorway's front.
ROUTES = {("away", "inside"): ["door", "inside"], ("inside", "away"): ["door", "away"]}


class Safety:
    def __init__(self, estop, env=None, log=print):
        self.estop = estop
        self.log = log
        self.spot = "away"
        self.pos = np.array(SPOTS["away"], float)
        self.heading = np.pi                  # facing the cell
        self.path = []                        # points still to walk to
        self.in_zone = self.in_curtain = self.inside = False
        self.person = part_mesh("SafetyPerson", pose=self._pose())
        if env is not None:
            env.add(self.person)

    # ---- the person
    def send(self, where):
        """Walk to SPOTS[where] (through the doorway if it means going in or out)."""
        hops = ROUTES.get((self.spot, where), [where])
        self.path = [np.array(SPOTS[h], float) for h in hops]
        self.spot = where
        self.log(f"  staff member walking to: {where}")

    def _pose(self):
        return SE3(*self.pos, 0.0) * SE3.Rz(self.heading + MODEL_YAW)

    # ---- every frame
    def update(self, dt):
        step = WALK_SPEED * dt
        while self.path and step > 0:
            d = self.path[0] - self.pos
            dist = np.linalg.norm(d)
            if dist > 1e-9:
                self.heading = atan2(d[1], d[0])
            if dist <= step:
                self.pos = self.path.pop(0)
                step -= dist
            else:
                self.pos = self.pos + d / dist * step
                step = 0
        self.person.T = self._pose().A
        self._sense()

    def _sense(self):
        x, y = self.pos
        across = abs(y - _Y)
        in_zone = _X < x <= _X + layout.SCAN_DEPTH and across < layout.SCAN_HALF_W
        in_curtain = abs(x - _X) < BODY_R and across < layout.DOOR_W / 2
        inside = x < _X                       # past the doorway
        if in_zone != self.in_zone:
            self.log("  area scanner: someone near the maintenance door - reduced speed" if in_zone
                     else "  area scanner: clear - full speed")
        self.in_zone = in_zone
        for source, now, before in ((CURTAIN, in_curtain, self.in_curtain), (INSIDE, inside, self.inside)):
            if now and not before:
                self.estop.press(source)
            elif before and not now:
                self.estop.release(source)
        self.in_curtain, self.inside = in_curtain, inside

    @property
    def speed(self):
        return REDUCED_SPEED if self.in_zone else 1.0

    def text(self):
        if self.inside:
            return "someone is inside the cell"
        if self.in_curtain:
            return "light curtain broken at the maintenance door"
        if self.in_zone:
            return f"someone near the maintenance door: reduced speed ({REDUCED_SPEED:.0%})"
        return ""
