"""
Teach window for the Doosan M0609 at its milk station (tkinter, part of Python).

    python -m barista.doosan_teach

Opens Swift with the milk station and a teach window beside it:

* Joint sliders. The arm follows a slider at up to JOINT_SPEED, checked for
  collisions at every step, and stops short of anything it would hit.
* Cartesian jog: hold a button to move the flange along X, Y or Z (world or
  tool frame) or turn it about the tool's own axes. Each step is one RMRC
  step (Planner.rmrc_step, the same damped least squares the pour uses), so
  the arm slows down instead of flipping near the wrist singularity.
* Go to pose: type a flange pose; the planner solves IK, picks the clearest
  configuration and plans a collision-checked path (Planner.free_move).
* Live readout of the flange pose, manipulability and the tightest clearance.
* E-stop: stops all motion until reset.

Like the order window, it runs in the same loop as Swift (one update per
simulation step), so nothing needs a thread.
"""
import sys
import tkinter as tk
from math import radians
from pathlib import Path

import numpy as np
from spatialmath import SE3

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from barista import milk_layout as layout  # noqa: E402
from barista import milk_scene  # noqa: E402
from barista.milk_scene import GRASP, HOME_Q, flange, pitcher_home  # noqa: E402
from barista.planner import DT, Planner, PlanningError  # noqa: E402

JOINT_SPEED = radians(60)       # rad/s while following a slider
JOG_SPEED = 0.05                # m/s for a Cartesian jog
JOG_TURN = radians(15)          # rad/s for a rotational jog
FONT = ("Segoe UI", 10)

# Named poses for "Go to", as flange poses (or joint vectors).
PRESETS = {
    "Home": HOME_Q,
    "Dispenser": flange(pitcher_home()) * milk_scene.BACK_OFF,
    "Over the cup": flange(milk_scene.pour_pose(0.0)),
    "Steam wand": flange(milk_scene.at_wand(milk_scene.WandPlacement(0.0, 0.0, layout.PITCHER_H + 0.03))),
}


class TeachWindow:
    def __init__(self, scene, env):
        self.scene, self.env = scene, env
        self.robot = scene.robots["milk"]
        self.checker = milk_scene.make_checker(self.robot)
        self.q = np.array(HOME_Q, float)
        self.target_q = self.q.copy()       # where the sliders want the arm
        self.path = []                      # planned rows still to play
        self.jog = None                     # (axis 0-5, sign) while a jog button is held
        self.estop = False
        self.closed = False
        self._syncing = False

        root = self.root = tk.Tk()
        root.title("Doosan M0609 - teach")
        root.protocol("WM_DELETE_WINDOW", self.close)
        pad = dict(padx=8, pady=4)

        # ---- joints
        joints = tk.LabelFrame(root, text="Joints (deg)", font=FONT)
        joints.grid(row=0, column=0, rowspan=2, sticky="nsew", **pad)
        self.sliders = []
        for j, link in enumerate(self.robot.links):
            lo, hi = np.degrees(link.qlim)
            tk.Label(joints, text=f"J{j + 1}", font=FONT).grid(row=j, column=0)
            s = tk.Scale(joints, from_=lo, to=hi, resolution=0.5, orient="horizontal", length=260,
                         command=lambda v, j=j: self._slider(j, float(v)))
            s.grid(row=j, column=1)
            self.sliders.append(s)

        # ---- Cartesian jog
        jog = tk.LabelFrame(root, text="Cartesian jog (hold)", font=FONT)
        jog.grid(row=0, column=1, sticky="nsew", **pad)
        self.frame = tk.StringVar(value="world")
        tk.Radiobutton(jog, text="World", variable=self.frame, value="world").grid(row=0, column=0)
        tk.Radiobutton(jog, text="Tool", variable=self.frame, value="tool").grid(row=0, column=1)
        for axis, name in enumerate(("X", "Y", "Z", "Rx", "Ry", "Rz")):
            for col, sign in ((0, -1), (1, +1)):
                b = tk.Button(jog, text=f"{name} {'-' if sign < 0 else '+'}", width=7, font=FONT)
                b.grid(row=axis + 1, column=col, padx=2, pady=1)
                b.bind("<ButtonPress-1>", lambda _e, a=axis, s=sign: self._start_jog(a, s))
                b.bind("<ButtonRelease-1>", lambda _e: self._stop_jog())

        # ---- go to pose
        goto = tk.LabelFrame(root, text="Go to flange pose (mm, deg; world frame)", font=FONT)
        goto.grid(row=1, column=1, sticky="nsew", **pad)
        self.entries = []
        for i, name in enumerate(("x", "y", "z", "roll", "pitch", "yaw")):
            tk.Label(goto, text=name, font=FONT).grid(row=i // 3 * 2, column=i % 3)
            e = tk.Entry(goto, width=8)
            e.grid(row=i // 3 * 2 + 1, column=i % 3, padx=2)
            self.entries.append(e)
        tk.Button(goto, text="Copy current", command=self._copy_pose).grid(row=4, column=0, pady=4)
        tk.Button(goto, text="Plan and go", command=self._goto_entries).grid(row=4, column=1, columnspan=2)
        presets = tk.Frame(goto)
        presets.grid(row=5, column=0, columnspan=3)
        for i, name in enumerate(PRESETS):
            tk.Button(presets, text=name, width=11,
                      command=lambda n=name: self._goto(PRESETS[n], n)).grid(row=i // 2, column=i % 2, padx=2, pady=1)

        # ---- readout and e-stop
        info = tk.Frame(root)
        info.grid(row=2, column=0, columnspan=2, sticky="ew", **pad)
        self.readout = tk.Label(info, font=("Consolas", 10), justify="left", anchor="w")
        self.readout.pack(fill="x")
        self.status = tk.Label(info, font=FONT, anchor="w", fg="#1d4ed8")
        self.status.pack(fill="x")
        bar = tk.Frame(root)
        bar.grid(row=3, column=0, columnspan=2, sticky="ew", **pad)
        tk.Button(bar, text="E-STOP", font=("Segoe UI", 14, "bold"), bg="#dc2626", fg="white",
                  width=10, command=self._press_estop).pack(side="left")
        tk.Button(bar, text="Reset e-stop", command=self._reset_estop).pack(side="left", padx=10)

        self._apply(self.q)
        self._copy_pose()
        self.say("Ready.")

    # ---- helpers
    def say(self, text, colour="#1d4ed8"):
        self.status.configure(text=text, fg=colour)

    def planner(self):
        return Planner(self.robot, self.q, self.checker, grasp=GRASP, held_T=pitcher_home(),
                       name="M0609", tracker="rmrc")

    def clearance(self, q):
        return self.checker.worst(q, False, pitcher_home())

    def _apply(self, q):
        """Move the arm (and the gripper drawn on it) to q, and sync the sliders."""
        self.q = np.array(q, float)
        self.robot.q = self.q
        milk_scene.update_attached(self.scene, holding=False)
        self._syncing = True
        for s, v in zip(self.sliders, np.degrees(self.q)):
            s.set(v)
        self._syncing = False

    def _blocked(self, q):
        c, what = self.clearance(q)
        if c < 0:
            self.say(f"Stopped: that would hit ({what}, {c * 1000:.0f} mm)", "#b91c1c")
            return True
        return False

    # ---- inputs
    def _slider(self, j, deg):
        # Tk calls this later for values _apply set too: ignore those echoes
        # (within half the slider's 0.5 deg resolution of where the arm is).
        if self._syncing or abs(deg - np.degrees(self.q[j])) < 0.3:
            return
        if self.estop:
            self._apply(self.q)                 # snap back
            return
        self.path, self.jog = [], None
        self.target_q = self.q.copy()
        self.target_q[j] = radians(deg)
        self.say(f"J{j + 1} to {deg:.1f} deg")

    def _start_jog(self, axis, sign):
        if not self.estop:
            self.path = []
            self.target_q = self.q.copy()
            self.jog = (axis, sign)
            name = ("X", "Y", "Z", "Rx", "Ry", "Rz")[axis]
            self.say(f"Jogging {name}{'+' if sign > 0 else '-'} by RMRC ({self.frame.get()} frame)")

    def _stop_jog(self):
        self.jog = None
        self.target_q = self.q.copy()

    def _copy_pose(self):
        T = self.robot.fkine(self.q)
        vals = [*(T.t * 1000), *T.rpy(order="xyz", unit="deg")]
        for e, v in zip(self.entries, vals):
            e.delete(0, "end")
            e.insert(0, f"{v:.1f}")

    def _goto_entries(self):
        try:
            x, y, z, r, p, yw = (float(e.get()) for e in self.entries)
        except ValueError:
            self.say("Enter six numbers.", "#b91c1c")
            return
        self._goto(SE3(x / 1000, y / 1000, z / 1000) * SE3.RPY(r, p, yw, order="xyz", unit="deg"), "typed pose")

    def _goto(self, target, name):
        if self.estop:
            return
        self.say(f"Planning to {name} ...")
        self.root.update()
        try:
            seg = self.planner().free_move(name, target, 3.0)
        except PlanningError as e:
            self.say(f"Can't go there: {e}", "#b91c1c")
            return
        self.jog = None
        self.path = list(seg.qs[1:])
        self.target_q = seg.qs[-1].copy()
        self.say(f"Going to {name}: {len(seg.qs)} steps, IK + collision-checked path.")

    def _press_estop(self):
        self.estop = True
        self.path, self.jog = [], None
        self.target_q = self.q.copy()
        self.say("E-STOP: all motion stopped. Reset to carry on.", "#b91c1c")

    def _reset_estop(self):
        if self.estop:
            self.estop = False
            self.say("E-stop reset.")

    # ---- one simulation step
    def tick(self):
        if self.estop:
            pass
        elif self.path:
            self._apply(self.path.pop(0))
            if not self.path:
                self.say("Arrived.")
        elif self.jog is not None:
            self._jog_step(*self.jog)
            self.target_q = self.q.copy()       # stay put when the jog ends
        elif np.any(np.abs(self.target_q - self.q) > 1e-6):
            dq = np.clip(self.target_q - self.q, -JOINT_SPEED * DT, JOINT_SPEED * DT)
            if self._blocked(self.q + dq):
                self.target_q = self.q.copy()
                self._apply(self.q)
            else:
                self._apply(self.q + dq)
        self._readout()

    def _jog_step(self, axis, sign):
        T = self.robot.fkine(self.q)
        if axis < 3:
            step = np.zeros(3)
            step[axis] = sign * JOG_SPEED * DT
            target = T * SE3(*step) if self.frame.get() == "tool" else SE3(*step) * T
        else:
            turn = (SE3.Rx, SE3.Ry, SE3.Rz)[axis - 3](sign * JOG_TURN * DT)
            target = T * turn                   # about the tool's own axes
        try:
            q = self.planner().rmrc_step(self.q, target, "jog")
        except PlanningError as e:
            self.jog = None
            self.say(f"Jog stopped: {e}", "#b91c1c")
            return
        if self._blocked(q):
            self.jog = None
            return
        if axis < 3 and np.linalg.norm(self.robot.fkine(q).t - T.t) < 0.5 * JOG_SPEED * DT:
            self.jog = None
            self.say("Jog stopped: the arm can't go further that way (edge of reach / singular).", "#b91c1c")
            return
        self._apply(q)

    def _readout(self):
        T = self.robot.fkine(self.q)
        x, y, z = T.t * 1000
        r, p, yw = T.rpy(order="xyz", unit="deg")
        w = self.robot.manipulability(self.q)
        c, what = self.clearance(self.q)
        warn = "   <- near the wrist singularity" if w < Planner.RMRC_W0 else ""
        self.readout.configure(text=(
            f"flange  x {x:7.1f}  y {y:7.1f}  z {z:7.1f} mm   roll {r:6.1f}  pitch {p:6.1f}  yaw {yw:6.1f} deg\n"
            f"manipulability {w:.4f}{warn}\n"
            f"clearance {c * 1000:5.0f} mm beyond the 10 mm margin ({what})"))

    def close(self):
        self.closed = True
        self.root.destroy()

    def update(self):
        try:
            self.root.update()
        except tk.TclError:
            self.closed = True


def main():
    import swift

    env = swift.Swift()
    env.launch(realtime=True)
    scene = milk_scene.build_scene(env)
    env.set_camera_pose(list(layout.CAMERA["position"]), list(layout.CAMERA["look_at"]))
    win = TeachWindow(scene, env)
    while not win.closed:
        win.tick()
        env.step(DT)
        win.update()
    env.close()


if __name__ == "__main__":
    main()
