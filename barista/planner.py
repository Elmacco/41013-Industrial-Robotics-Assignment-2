"""
Collision-checked motion planning for one arm, shared by every robot here.

A task is planned up front as a list of Segments (joint paths) and only then
played back, so a collision or an unreachable pose fails straight away with a
clear message instead of halfway through an animation.
"""
from dataclasses import dataclass, field
from math import ceil, pi, radians

import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

DT = 0.05
MAX_JOINT_STEP = radians(2)           # finest joint step a path is checked at


class PlanningError(RuntimeError):
    pass


@dataclass
class Segment:
    name: str
    qs: np.ndarray                         # (N, n) joint path; row 0 is where it starts
    holding: bool = False                  # held object travels with the gripper
    robot: str = ""                        # which robot moves (scene key)
    cup_level: np.ndarray = None           # per row milk height in the cup
    effects: dict = field(default_factory=dict)   # effect name -> per row on/off

    @property
    def duration(self):
        return (len(self.qs) - 1) * DT


class Planner:
    """Turns targets into collision-checked joint paths, one Segment at a time.

    * ``ik`` solves from the current joints first and, when asked, from a spread
      of other seeds too, and keeps the clearest collision-free answer.
    * ``free_move`` (only the end pose matters) tries the direct joint path to
      each IK answer, then via random clear configurations (the Lab 5
      primitive RRT) if every direct path is blocked.
    * ``follow`` / ``line_move`` are constrained Cartesian moves; they keep the
      arm's configuration and are rejected if any step collides.

    :param grasp: pose of the held object in the flange frame while gripped.
    :param held_T: where the held object stands while it is not gripped.
    :param name: scene key of the robot, stamped on every Segment.
    """

    def __init__(self, robot, q0, checker=None, grasp=SE3(), held_T=None, name="",
                 dt=DT, seed=1, tracker="ik"):
        self.robot = robot
        self.q = np.array(q0, dtype=float)
        self.dt = dt
        self.checker = checker
        self.grasp = grasp
        self.held_T = held_T
        self.name = name
        self.holding = False
        self.segments = []
        self.rng = np.random.default_rng(seed)
        self.tracker = tracker              # "ik" or "rmrc", for Cartesian moves
        self.manipulability = []            # (segment name, per-row values) from RMRC moves
        # A DH robot's ikine takes world targets; a URDF robot's solves in its
        # base frame (though its fkine does include the base).
        self._world_ik = isinstance(robot, rtb.DHRobot)

    # ---- helpers
    def _steps(self, duration):
        return max(int(round(duration / self.dt)), 1)

    def fk_held(self, q):
        return self.robot.fkine(q) * self.grasp

    def clearance(self, q):
        if self.checker is None:
            return np.inf, ""
        P = self.fk_held(q) if self.holding else self.held_T
        return self.checker.worst(q, self.holding, P)

    def collision(self, qs):
        if self.checker is None:
            return None
        return self.checker.first_collision(qs, self.holding, self.held_T, self.fk_held)

    def _unwrap(self, q, ref):
        """Every joint turns +-2 pi, so a solution may be the same pose a full
        turn away; take the turn nearest ref (if within the joint limits)."""
        near = q - 2 * pi * np.round((q - ref) / (2 * pi))
        lo, hi = self.robot.qlim
        return np.where((near >= lo) & (near <= hi), near, q)

    def _solve(self, T, seed, ref, restarts=100):
        target = T if self._world_ik else self.robot.base.inv() * T
        # seed=0 keeps the solver's random restarts repeatable from run to run.
        sol = self.robot.ikine_LM(target, q0=seed, joint_limits=True, slimit=restarts, seed=0)
        if not sol.success or np.linalg.norm(self.robot.fkine(sol.q).t - T.t) > 1e-3:
            return None
        return self._unwrap(np.array(sol.q), ref)

    def ik(self, T, seed, label, search=0, restarts=100):
        """IK for flange pose T. With search > 0, also try that many random
        seeds and return the clear solutions, clearest first. Every answer is
        taken at the joint turn nearest `seed`."""
        found = []
        for s in [seed] + [self.rng.uniform(-pi, pi, self.robot.n) for _ in range(search)]:
            q = self._solve(T, s, seed, restarts)
            if q is not None and all(np.max(np.abs(q - f)) > 1e-2 for f in found):
                found.append(q)
        if not found:
            raise PlanningError(f"{label}: no IK solution")
        if not search:
            return found[0]
        scored = sorted(((self.clearance(q)[0], i) for i, q in enumerate(found)), reverse=True)
        clear = [found[i] for c, i in scored if c >= 0.0]
        if not clear:
            c, i = scored[0]
            raise PlanningError(f"{label}: every IK solution collides "
                                f"(best {c * 1000:.0f} mm, {self.clearance(found[i])[1]})")
        return clear

    def _add(self, name, qs, **fx):
        qs = np.asarray(qs)
        hit = self.collision(qs)
        if hit is not None:
            k, what, c = hit
            raise PlanningError(f"{self.name + ': ' if self.name else ''}{name}: collision at step "
                                f"{k}/{len(qs) - 1}: {what} ({c * 1000:.0f} mm)")
        seg = Segment(name, qs, holding=self.holding, robot=self.name, **fx)
        self.segments.append(seg)
        self.q = seg.qs[-1]
        return seg

    def _joint_path(self, q0, q1, duration=0.0):
        n = max(self._steps(duration), int(ceil(np.max(np.abs(q1 - q0)) / MAX_JOINT_STEP)))
        return rtb.jtraj(q0, q1, n + 1).q

    # ---- gripper
    def grip(self, closed):
        if not closed:
            self.held_T = self.fk_held(self.q)
        self.holding = closed

    # ---- moves
    def free_move(self, name, target, duration, search=40, tries=200):
        """Joint-space move to a flange pose (SE3) or a joint vector."""
        goals = (self.ik(target, self.q, name, search) if isinstance(target, SE3)
                 else [np.asarray(target, float)])
        for q1 in goals:
            path = self._joint_path(self.q, q1, duration)
            if self.collision(path) is None:
                return self._add(name, path)
        # Primitive RRT (Lab 5, 3.3): random clear configuration as a via point.
        q1 = goals[0]
        for _ in range(tries):
            via = self.q + self.rng.uniform(-pi / 2, pi / 2, self.robot.n)
            if self.clearance(via)[0] < 0.0:
                continue
            a = self._joint_path(self.q, via, duration / 2)
            b = self._joint_path(via, q1, duration / 2)
            if self.collision(a) is None and self.collision(b) is None:
                return self._add(name, np.vstack([a, b[1:]]))
        raise PlanningError(f"{name}: no collision-free joint path found")

    def follow(self, name, flange_poses, max_jump=0.2, **fx):
        """Track sampled flange poses: by IK seeded from the last sample, or by
        RMRC (see rmrc_step) when the planner was made with tracker="rmrc".

        A big jump between samples means the solver flipped configuration (or
        the path passes near a singularity), so it is refused instead of
        letting the arm whip round.
        """
        qs, manip = [self.q], [self.robot.manipulability(self.q)]
        for i, T in enumerate(flange_poses):
            label = f"{name}, sample {i}"
            if self.tracker == "rmrc":
                q = self.rmrc_step(qs[-1], T, label)
            else:
                # Few restarts: a tracking step that fails from the last joints
                # should not be "rescued" by a random configuration elsewhere.
                q = self.ik(T, qs[-1], label, restarts=3)
            jump = np.max(np.abs(q - qs[-1]))
            if jump > max_jump:
                raise PlanningError(f"{label}: joint jump of {jump:.2f} rad")
            qs.append(q)
            manip.append(self.robot.manipulability(q))
        if self.tracker == "rmrc":
            self.manipulability.append((name, np.array(manip)))
        return self._add(name, qs, **fx)

    # ---- resolved motion rate control
    RMRC_W0 = 0.02            # manipulability below which damping starts
    RMRC_LAMBDA = 0.05        # damping at zero manipulability
    RMRC_ITER = 4             # corrective iterations per sample
    RMRC_W = np.diag([1.0, 1.0, 1.0, 0.3, 0.3, 0.3])   # position matters more than a slight tilt
    RMRC_POS_TOL = 0.006      # m: worst position error accepted (only reached for a few samples
    #                           right at the M0609 wrist singularity in the pour; elsewhere < 0.5 mm)
    RMRC_ROT_TOL = 0.03       # rad: worst orientation error accepted

    def rmrc_step(self, q, T, label):
        """One sample of resolved motion rate control (Labs 6 and 9).

        The flange velocity needed to reach the next pose in one step,
        xdot = [position error; orientation error] / dt, is turned into joint
        velocities and integrated over the step. The joint velocities solve a
        weighted, damped least-squares problem (Lab 9):

            qdot = (J^T W J + lambda^2 I)^-1 J^T W xdot

        W weights position above orientation, and lambda grows as the
        manipulability falls below RMRC_W0 (zero damping when well away). Near a
        singularity (for the M0609's wrist: joint 5 near zero, joints 4 and 6
        lined up) plain IK can flip joints 4 and 6 by half a turn between
        samples; damping keeps the joint rates bounded and the motion smooth,
        at the cost of a small, checked tracking error there.

        A few corrective iterations per sample (feeding the remaining error
        back) keep the path from drifting.
        """
        robot, dt = self.robot, self.dt
        target = T.A
        for _ in range(self.RMRC_ITER):
            cur = robot.fkine(q).A
            e_p = target[:3, 3] - cur[:3, 3]
            R_err = target[:3, :3] @ cur[:3, :3].T            # rotation still to go, world frame
            e_r = 0.5 * np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0],
                                  R_err[1, 0] - R_err[0, 1]])
            xdot = np.concatenate([e_p, e_r]) / dt
            J = robot.jacob0(q)
            w = np.sqrt(max(np.linalg.det(J @ J.T), 0.0))
            lam2 = (1.0 - (w / self.RMRC_W0) ** 2) * self.RMRC_LAMBDA ** 2 if w < self.RMRC_W0 else 0.0
            W = self.RMRC_W
            qdot = np.linalg.solve(J.T @ W @ J + lam2 * np.eye(6), J.T @ W @ xdot)
            q = q + qdot * dt
        lo, hi = robot.qlim
        if np.any(q < lo) or np.any(q > hi):
            raise PlanningError(f"{label}: RMRC would pass a joint limit")
        cur = robot.fkine(q).A
        pos_err = np.linalg.norm(target[:3, 3] - cur[:3, 3])
        rot_err = np.arccos(np.clip((np.trace(target[:3, :3] @ cur[:3, :3].T) - 1) / 2, -1, 1))
        if pos_err > self.RMRC_POS_TOL or rot_err > self.RMRC_ROT_TOL:
            raise PlanningError(f"{label}: RMRC tracking error {pos_err * 1000:.1f} mm, "
                                f"{np.degrees(rot_err):.1f} deg")
        return q

    def line_move(self, name, T_flange, duration, **fx):
        """Straight-line Cartesian move of the flange (orientation slerped)."""
        T0 = self.robot.fkine(self.q)
        return self.follow(name, list(rtb.ctraj(T0, T_flange, self._steps(duration)))[1:], **fx)

    def wait(self, name, duration, **fx):
        return self._add(name, np.repeat(self.q[None], self._steps(duration) + 1, axis=0), **fx)
