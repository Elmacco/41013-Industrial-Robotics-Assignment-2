"""
Run several robots at the same time from their planned Segments.

Each robot (and each machine with timed effects, like the drawer) is a Lane
with a program: a list of moves, plus Wait / Signal markers for the points
where one has to wait for another ("cup under the group head", "pour done").

schedule() steps every lane forward one frame at a time and checks every pair
of robots against each other at every frame, using the same capsules as the
collision checker, so moving together is checked against where the others
really are rather than frozen at home. When two would come closer than the
margin, the lower-priority one pauses that frame and tries again on the next.
If robots block each other for good, it fails with a PlanningError naming them.

The result is a Timeline: one entry per frame saying which segment row each
lane shows. It is worked out once, before anything plays, like the plans.
"""
from dataclasses import dataclass, field

import numpy as np

from barista.collision import capsule_gap
from barista.planner import DT, PlanningError, Segment


@dataclass
class Wait:
    event: str


@dataclass
class Signal:
    event: str


@dataclass
class Step:
    seg: Segment
    order: int = 0            # which order (cup) this belongs to


@dataclass
class Lane:
    """One robot or machine and its program.

    capsules(seg, k) -> list of Capsules at that segment row, or None for a
    lane that is not a robot (it is then never collision checked).
    priority: higher keeps moving when two robots would clash.
    """
    name: str
    program: list
    home: Segment = None              # robots: a one-row segment at the home pose
    capsules: object = None
    priority: int = 0
    i: int = 0                        # program position
    k: int = 0                        # row within the current Step
    state: tuple = None               # (Step, row) the lane is showing
    waited: float = 0.0               # time spent waiting on events or paused
    paused: int = 0                   # frames paused to avoid another robot

    def __post_init__(self):
        if self.home is not None:
            self.state = (Step(self.home), 0)

    def current(self):
        return self.program[self.i] if self.i < len(self.program) else None


@dataclass
class Timeline:
    frames: list                      # per frame: {lane: (Step, row, active)}
    lanes: list
    min_gap: float = np.inf           # closest any two robots came
    min_gap_what: str = ""
    stats: dict = field(default_factory=dict)

    @property
    def duration(self):
        return len(self.frames) * DT


def _bounds(caps):
    pts = np.vstack([c.pts for c in caps])
    r = max(c.radius for c in caps)
    return pts.min(0) - r, pts.max(0) + r


def _pair_gaps(robots, pose, margin):
    """(gap, what, lane a, lane b) for every pair of robots close enough to matter."""
    out = []
    for x in range(len(robots)):
        for y in range(x + 1, len(robots)):
            a, b = robots[x], robots[y]
            (ca, (alo, ahi)), (cb, (blo, bhi)) = pose[a.name], pose[b.name]
            if np.any(alo - margin > bhi) or np.any(blo - margin > ahi):
                continue                      # bounding boxes far apart
            gap, what = min((capsule_gap(p, q), f"{a.name} {p.name} / {b.name} {q.name}")
                            for p in ca for q in cb)
            out.append((gap, what, a, b))
    return out


def schedule(lanes, margin=0.01, stall_limit=20.0):
    """Merge every lane's program into one Timeline (see module docstring)."""
    events = set()
    frames = []
    cache = {}
    min_gap, min_what = np.inf, ""
    robots = [ln for ln in lanes if ln.capsules is not None]
    stalled = 0

    def caps_of(lane, step, k):
        key = (lane.name, id(step.seg), k)
        if key not in cache:
            caps = lane.capsules(step.seg, k)
            cache[key] = (caps, _bounds(caps))
        return cache[key]

    while True:
        # Signals and satisfied waits take no time.
        for ln in lanes:
            while True:
                item = ln.current()
                if isinstance(item, Signal):
                    events.add(item.event)
                    ln.i += 1
                elif isinstance(item, Wait) and item.event in events:
                    ln.i += 1
                else:
                    break
        if all(ln.current() is None for ln in lanes):
            break

        proposals = {ln.name: (ln.current(), ln.k) for ln in lanes if isinstance(ln.current(), Step)}
        if not proposals:
            waiting = {ln.name: ln.current().event for ln in lanes if isinstance(ln.current(), Wait)}
            raise PlanningError(f"every lane is waiting and nothing will signal: {waiting}")

        # Pause the lower-priority robot of any clashing pair, until none clash.
        moving = set(proposals)
        last_clash = ""
        while True:
            pose = {ln.name: caps_of(ln, *(proposals[ln.name] if ln.name in moving else ln.state))
                    for ln in robots}
            gaps = _pair_gaps(robots, pose, margin)
            clash = next((g for g in gaps if g[0] - margin < 0.0
                          and (g[2].name in moving or g[3].name in moving)), None)
            if clash is None:
                break
            gap, last_clash, a, b = clash
            yielder = min((ln for ln in (a, b) if ln.name in moving), key=lambda ln: ln.priority)
            moving.discard(yielder.name)
            yielder.paused += 1
        for gap, what, a, b in gaps:
            if gap < min_gap:
                min_gap, min_what = gap, what

        if moving:
            stalled = 0
        else:
            stalled += 1
            if stalled * DT > stall_limit:
                raise PlanningError(f"robots block each other: {sorted(proposals)} cannot move "
                                    f"({last_clash})")

        frame = {}
        for ln in lanes:
            if ln.name in moving:
                step, k = proposals[ln.name]
                frame[ln.name] = (step, k, True)
                ln.state = (step, k)
                ln.k += 1
                if ln.k >= len(step.seg.qs):
                    ln.i += 1
                    ln.k = 0
            else:
                if ln.state is not None:
                    frame[ln.name] = (*ln.state, False)
                if ln.current() is not None:
                    ln.waited += DT
        frames.append(frame)

    stats = {ln.name: dict(waited=round(ln.waited, 1), paused=round(ln.paused * DT, 1)) for ln in lanes}
    return Timeline(frames, [ln.name for ln in lanes], min_gap, min_what, stats)
