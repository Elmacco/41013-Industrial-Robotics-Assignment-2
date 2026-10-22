"""
Doosan M0609 viva demo: the model and each technique the milk station uses,
one stage at a time in Swift. Press ENTER to go on to the next stage.

    python -m barista.doosan_demo              # every stage
    python -m barista.doosan_demo --stage 5    # start at stage 5
    python -m barista.doosan_demo --no-plots   # skip the matplotlib windows

    1  the model      DH table, checked against Doosan's URDF; DH frames shown
                      while each joint turns
    2  IK             every arm configuration for one pose, scored by clearance
    3  RMRC           IK per sample fails in the pour (wrist singularity); RMRC
                      gets through. Plots manipulability, damping and tracking
                      error, and a jtraj move against a straight-line one
    4  avoidance      a bottle left in the way blocks every direct path; the
                      planner finds a detour through a random clear pose (RRT)
    5  visual servo   the cup is not quite where the UR3 should have left it;
                      a wrist camera centres on its rim (IBVS), and the task is
                      re-planned to pour where the camera found it

Each stage reuses the milk station's own code (milk_scene.py, planner.py,
collision.py); nothing here is a separate implementation for show.
"""
import argparse
import importlib.util
import sys
import time
from math import atan2, pi
from pathlib import Path

import numpy as np
import roboticstoolbox as rtb
import spatialgeometry as geometry
from spatialmath import SE3

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from barista import doosan_m0609 as m0609  # noqa: E402
from barista import milk_layout as layout  # noqa: E402
from barista import milk_scene  # noqa: E402
from barista.collision import Box  # noqa: E402
from barista.milk_scene import GRASP, HOME_Q, flange, pitcher_home, pour_pose  # noqa: E402
from barista.planner import DT, Planner, PlanningError  # noqa: E402

HIDDEN = SE3(0.0, 0.0, -5.0)
SHOW_PLOTS = True

# Stage 4: a 1 L milk bottle left on the bench between the wash station and the
# dispenser. Placed so every direct path to the dispenser hits it.
BOTTLE_AT, BOTTLE_HALF = (*layout.polar(70.0, 0.50), layout.TABLE_TOP_Z + 0.25), (0.06, 0.06, 0.25)

# Stage 5: an eye-in-hand camera on the side of the gripper, looking along the
# tool axis. 8 mm lens, 10 um pixels, 640 x 640 image.
CAM_ON_FLANGE = SE3(0.05, 0.0, 0.02)
CAM_F, CAM_RHO, CAM_PIX = 0.008, 10e-6, 640
VIEW_HEIGHT = 0.25          # goal: camera this far above the rim, looking straight down
START_HEIGHT = 0.32         # where it starts looking from (above where the cup should be)
CUP_ERROR = (0.03, -0.025)  # where the UR3 actually left the cup, from where it should be
IBVS_GAIN = 1.2             # 1/s: image error decays like exp(-gain t)
IBVS_TOL = 0.5              # pixels


def pause(msg="ENTER for the next stage"):
    input(f"\n  -- {msg} --")


def title(n, text):
    print(f"\n{'=' * 72}\n  {n}. {text}\n{'=' * 72}")


def show(fig_fn):
    if SHOW_PLOTS:
        import matplotlib.pyplot as plt
        fig_fn(plt)
        print("  (close the plot window to carry on)")
        plt.show()


class Demo:
    def __init__(self, env):
        self.env = env
        self.scene = milk_scene.build_scene(env)
        self.robot = self.scene.robots["milk"]
        env.set_camera_pose(list(layout.CAMERA["position"]), list(layout.CAMERA["look_at"]))
        # DH frame axes (stage 1), the bottle (stage 4), the camera and rim markers (stage 5).
        self.axes = [geometry.Axes(length=0.08, pose=HIDDEN) for _ in range(7)]
        self.bottle = geometry.Cuboid(scale=[2 * h for h in BOTTLE_HALF], pose=HIDDEN, color=(0.9, 0.2, 0.2, 1))
        self.camera = geometry.Cuboid(scale=[0.04, 0.03, 0.03], pose=HIDDEN, color=(0.1, 0.1, 0.1, 1))
        self.markers = [geometry.Sphere(radius=0.006, pose=HIDDEN, color=(1.0, 0.1, 0.1, 1)) for _ in range(4)]
        for shape in (*self.axes, self.bottle, self.camera, *self.markers):
            env.add(shape)
        self.reset()

    # ---- display helpers
    def reset(self):
        """Arm home, pitcher in its bay, a fresh cup where it should be."""
        self.robot.q = HOME_Q
        self.scene.pitcher_pose = pitcher_home()
        milk_scene.update_attached(self.scene, holding=False)
        milk_scene.set_effects(self.scene, {})
        self.scene.cup.set(T=SE3(*layout.CUP_AT, layout.TABLE_TOP_Z))
        milk_scene.reset_cup(self.scene)
        self.env.step(DT)

    def set_q(self, q, holding=False):
        self.robot.q = q
        milk_scene.update_attached(self.scene, holding)
        if self.camera.T[2, 3] > -1.0:                   # mounted: follows the flange
            self.camera.T = (self.robot.fkine(q) * CAM_ON_FLANGE).A
        if self.axes[0].T[2, 3] > -1.0:
            for ax, T in zip(self.axes, self.robot.fkine_all(q)):
                ax.T = T.A
        self.env.step(DT)

    def play(self, qs, holding=False):
        for q in qs:
            self.set_q(q, holding)

    def go(self, q, duration=2.0):
        """Joint move for display only (stage 1 sweeps)."""
        self.play(rtb.jtraj(self.robot.q, q, max(int(duration / DT), 2)).q)

    def planner(self, checker=None, tracker="rmrc"):
        checker = checker or milk_scene.make_checker(self.robot)
        return Planner(self.robot, self.robot.q, checker, grasp=GRASP, held_T=pitcher_home(),
                       name="M0609", tracker=tracker)

    # ---------------------------------------------------------------- 1
    def stage_model(self):
        title(1, "The model: Doosan M0609, standard DH")
        print("  joint      d (m)     a (m)   alpha   offset   limits (deg)")
        for i, link in enumerate(self.robot.links, 1):
            lo, hi = np.degrees(link.qlim)
            print(f"    {i}     {link.d:7.4f}   {link.a:7.4f}  {np.degrees(link.alpha):5.0f}   "
                  f"{np.degrees(link.offset):6.0f}    {lo:+5.0f} .. {hi:+.0f}")

        spec = importlib.util.spec_from_file_location("derive_dh", m0609.ROOT / "derive_dh.py")
        derive = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(derive)
        joints = derive.urdf_joints()
        rng = np.random.default_rng(0)
        worst = 0.0
        for _ in range(500):
            q = rng.uniform(-pi, pi, 6)
            mine = (self.robot.base.inv() * self.robot.fkine(q)).t
            worst = max(worst, np.linalg.norm(mine - derive.urdf_fk(q, joints)[-1][:3, 3]))
        print(f"\n  My DH model vs Doosan's URDF, 500 random poses: worst flange error {worst * 1000:.2f} mm")
        out = (self.robot.base.inv() * self.robot.fkine([0, pi / 2, 0, 0, 0, 0])).t
        print(f"  Reach (arm straight out): {np.hypot(out[0], out[1]):.3f} m   (datasheet: 0.900 m)")
        print("  Spherical wrist: d5 = a4 = a5 = 0, so joints 4-6 meet at one point")

        print("\n  Showing the DH frames (red x, green y, blue z) while each joint turns ...")
        for ax in self.axes:
            ax.T = SE3().A
        self.go(np.zeros(6), 2.0)
        for j in range(6):
            print(f"    joint {j + 1}")
            q = np.zeros(6)
            q[j] = pi / 4
            self.go(q, 1.2)
            q[j] = -pi / 4
            self.go(q, 1.6)
            self.go(np.zeros(6), 1.2)
        self.go(HOME_Q, 2.0)
        for ax in self.axes:
            ax.T = HIDDEN.A
        self.env.step(DT)

    # ---------------------------------------------------------------- 2
    def stage_ik(self):
        title(2, "Inverse kinematics: which of the arm's configurations to use")
        self.reset()
        pl = self.planner()
        target = flange(pitcher_home()) * milk_scene.BACK_OFF
        print("  Target: the gripper 8 cm back from the pitcher handle, in the dispenser bay.")
        print("  ikine_LM (Levenberg-Marquardt) from 40 random seeds, joint limits on:\n")
        found = []
        for seed in [HOME_Q] + [pl.rng.uniform(-pi, pi, 6) for _ in range(40)]:
            q = pl._solve(target, seed, HOME_Q)
            if q is not None and all(np.max(np.abs(q - f)) > 1e-2 for f in found):
                found.append(q)
        rows = []
        for q in found:
            c, what = pl.clearance(q)
            err = np.linalg.norm(self.robot.fkine(q).t - target.t)
            rows.append((c, what, err, q))
        rows.sort(key=lambda r: -r[0])
        print("   #   joints (deg)                                   clearance   tightest")
        for i, (c, what, err, q) in enumerate(rows, 1):
            verdict = "clear" if c >= 0 else "COLLIDES"
            print(f"  {i:2d}   {np.array2string(np.degrees(q), precision=0, suppress_small=True):44s}"
                  f" {c * 1000:6.0f} mm  {what:22s} {verdict}  (FK error {err * 1e3:.2f} mm)")
        print(f"\n  {len(rows)} distinct solutions, {sum(r[0] >= 0 for r in rows)} collision-free.")
        print("  Showing each one, worst first ...")
        for i, (c, what, _, q) in reversed(list(enumerate(rows, 1))):
            print(f"    solution {i}: {c * 1000:+.0f} mm ({what})")
            self.set_q(q)
            time.sleep(1.5)
        self.set_q(HOME_Q)
        print("\n  The planner takes the clearest one, then a collision-checked jtraj there:")
        seg = pl.free_move("approach dispenser", target, 3.0)
        self.play(seg.qs)
        print(f"    {len(seg.qs)} steps, every one checked: arm, gripper and pitcher vs the station and itself")
        time.sleep(1.0)
        self.set_q(HOME_Q)

    # ---------------------------------------------------------------- 3
    def stage_rmrc(self):
        title(3, "Trajectories and RMRC: the pour")
        self.reset()
        print("  Planning a full latte cycle (RMRC for every Cartesian move) ...")
        segs = milk_scene.plan_task(self.robot)
        pour = next(s for s in segs if s.name == "pour")
        us = np.linspace(0.0, 1.0, len(pour.qs))
        targets = [flange(pour_pose(u)) for u in us]

        print("\n  a) Same pour, solved by IK at every sample instead:")
        ik = Planner(self.robot, pour.qs[0], milk_scene.make_checker(self.robot), grasp=GRASP,
                     held_T=None, tracker="ik")
        ik.holding = True
        try:
            ik.follow("pour", targets[1:])
            print("     (got through this time)")
        except PlanningError as e:
            print(f"     refused: {e}")
            print("     joint 5 passes through 0, joints 4 and 6 line up, and IK flips them")

        robot = self.robot
        w = np.array([robot.manipulability(q) for q in pour.qs])
        lam = np.where(w < Planner.RMRC_W0, np.sqrt(np.clip(1 - (w / Planner.RMRC_W0) ** 2, 0, 1))
                       * Planner.RMRC_LAMBDA, 0.0)
        err = np.array([np.linalg.norm(robot.fkine(q).t - T.t) for q, T in zip(pour.qs, targets)]) * 1000
        q5 = np.degrees(pour.qs[:, 4])
        k = int(np.argmin(w))
        print("\n  b) RMRC, weighted damped least squares:  qdot = (J'WJ + l^2 I)^-1 J'W xdot")
        print(f"     manipulability {w.max():.3f} -> {w.min():.4f} at t = {k * DT:.2f} s (q5 = {q5[k]:+.1f} deg)")
        print(f"     damping on for {np.count_nonzero(lam) * DT:.2f} s; worst tracking error "
              f"{err.max():.1f} mm, median {np.median(err):.2f} mm")

        # The same start and end joined in joint space instead: where does the spout go?
        jt = rtb.jtraj(pour.qs[0], pour.qs[-1], len(pour.qs)).q
        lip = SE3(*layout.PITCHER_LIP)
        lip_rmrc = np.array([(robot.fkine(q) * GRASP * lip).t for q in pour.qs])
        lip_jt = np.array([(robot.fkine(q) * GRASP * lip).t for q in jt])

        def off_cup(P):
            return np.linalg.norm(P[:, :2] - layout.CUP_AT, axis=1).max() * 1000

        hit = ik.checker.first_collision(jt, True, None, lambda q: robot.fkine(q) * GRASP)
        print("\n  c) Why not jtraj straight from the pour's first pose to its last?")
        print(f"     spout up to {off_cup(lip_jt):.0f} mm from the cup's axis with jtraj, "
              f"{off_cup(lip_rmrc):.0f} mm with RMRC (rim radius {layout.CUP_R * 1000:.0f} mm)")
        if hit:
            print(f"     and jtraj collides at step {hit[0]}: {hit[1]}")
        print("     jtraj only fixes the two ends; a pour needs the path in between.")

        print("\n  Showing the pour ...")
        self.set_q(pour.qs[0], holding=True)
        milk_scene.play(self.scene, self.env, [pour], verbose=False)
        time.sleep(1.0)

        def plots(plt):
            t = np.arange(len(pour.qs)) * DT
            fig, ax = plt.subplots(4, 1, figsize=(8, 9), sharex=True)
            fig.suptitle("M0609 pour: RMRC through the wrist singularity")
            ax[0].plot(t, w)
            ax[0].axhline(Planner.RMRC_W0, ls="--", c="grey", label="damping starts")
            ax[0].set_ylabel("manipulability")
            ax[0].legend()
            ax[1].plot(t, q5)
            ax[1].axhline(0, ls="--", c="grey")
            ax[1].set_ylabel("q5 (deg)")
            ax[2].plot(t, lam)
            ax[2].set_ylabel("damping lambda")
            ax[3].plot(t, err)
            ax[3].set_ylabel("tracking error (mm)")
            ax[3].set_xlabel("time (s)")
            fig.tight_layout()
            fig2 = plt.figure(figsize=(6, 5))
            a3 = fig2.add_subplot(projection="3d")
            a3.plot(*lip_jt.T, label="jtraj (joint space)")
            a3.plot(*lip_rmrc.T, label="RMRC (Cartesian)")
            a3.plot([layout.CUP_AT[0]], [layout.CUP_AT[1]], [layout.TABLE_TOP_Z + layout.CUP_H], "ko",
                    label="cup centre, rim height")
            a3.set_title("Spout tip during the pour")
            a3.legend()
        show(plots)
        self.reset()

    # ---------------------------------------------------------------- 4
    def stage_avoid(self):
        title(4, "Collision avoidance: a bottle left on the bench")
        self.reset()
        T = np.eye(4)
        T[:3, 3] = BOTTLE_AT
        self.bottle.T = T
        self.env.step(DT)
        checker = milk_scene.make_checker(self.robot)
        checker.obstacles.append(Box("milk bottle", T, np.array(BOTTLE_HALF)))
        pl = self.planner(checker)
        target = flange(pitcher_home()) * milk_scene.BACK_OFF
        goals = pl.ik(target, pl.q, "approach", search=40)
        print(f"  Going to the dispenser again. {len(goals)} clear IK answers; the direct path to each:")
        for i, q in enumerate(goals, 1):
            hit = pl.collision(pl._joint_path(pl.q, q, 3.0))
            print(f"    {i}: " + (f"blocked at step {hit[0]}: {hit[1]}" if hit else "clear"))
        print("\n  So: random clear configurations as a via point (Lab 5's RRT step),")
        print("  each half of the path checked like any other ...")
        t0 = time.time()
        seg = pl.free_move("approach dispenser", target, 3.0)
        print(f"  found a detour in {time.time() - t0:.1f} s; tightest clearance on it "
              f"{min(pl.clearance(q)[0] for q in seg.qs) * 1000:.0f} mm beyond the 10 mm margin")
        self.play(seg.qs)
        time.sleep(1.0)
        self.set_q(HOME_Q)
        self.bottle.T = HIDDEN.A
        self.env.step(DT)

    # ---------------------------------------------------------------- 5
    def stage_servo(self):
        title(5, "Visual servoing: find the cup before pouring")
        from machinevisiontoolbox import CentralCamera

        self.reset()
        robot = self.robot
        cam = CentralCamera(f=CAM_F, rho=CAM_RHO, imagesize=CAM_PIX, name="wrist camera")
        nominal = np.array(layout.CUP_AT)
        true = nominal + CUP_ERROR

        def cup_frame(xy):
            return SE3(xy[0], xy[1], layout.TABLE_TOP_Z) * SE3.Rz(atan2(xy[1], xy[0]))

        def rim(T):
            """Four marks round the cup rim (3xN), the image features."""
            return np.array([(T * SE3(layout.CUP_R * np.cos(a), layout.CUP_R * np.sin(a), layout.CUP_H)).t
                             for a in np.arange(4) * pi / 2]).T

        # The goal image: rim seen from VIEW_HEIGHT straight above the cup. It is
        # the same wherever the cup is, so the goal is set once from the plan.
        view = SE3(0, 0, layout.CUP_H + VIEW_HEIGHT) * SE3.Rx(pi)       # camera in the cup frame
        p_goal = cam.project_point(rim(cup_frame(nominal)), pose=cup_frame(nominal) * view)

        self.scene.cup.set(T=cup_frame(true))
        P = rim(cup_frame(true))
        for m, p in zip(self.markers, P.T):
            m.T = SE3(*p).A
        self.camera.T = (robot.fkine(robot.q) * CAM_ON_FLANGE).A
        self.env.step(DT)

        lip = (pour_pose(1.0) * SE3(*layout.PITCHER_LIP)).t
        miss = np.linalg.norm(lip[:2] - true)
        print(f"  The UR3 left the cup {np.linalg.norm(CUP_ERROR) * 1000:.0f} mm from where the plan expects it.")
        print(f"  Pouring open loop, the spout would end {miss * 1000:.0f} mm from the cup's centre "
              f"(rim radius {layout.CUP_R * 1000:.0f} mm): milk on the bench.")

        checker = milk_scene.make_checker(robot, cup_at=tuple(true))
        pl = self.planner(checker)
        look = cup_frame(nominal) * SE3(0, 0, layout.CUP_H + START_HEIGHT) * SE3.Rx(pi)
        print("\n  Moving the wrist camera over where the cup should be ...")
        self.play(pl.free_move("look", look * CAM_ON_FLANGE.inv(), 2.5).qs)

        print("  IBVS:  v_cam = -gain * pinv(L) * (p - p*);  qdot from the camera's Jacobian (DLS)")
        print("  Depth Z for L is estimated from the rim's size in the image (the cup size is known).")
        q, hist, errs, depths = robot.q.copy(), [], [], []
        for k in range(400):
            Tf = robot.fkine(q)
            Tc = Tf * CAM_ON_FLANGE
            p = cam.project_point(P, pose=Tc)
            e = p - p_goal
            hist.append(p)
            errs.append(np.linalg.norm(e, axis=0).mean())
            if np.max(np.abs(e)) < IBVS_TOL:
                break
            Z = cam.f[0] * 2 * layout.CUP_R / (np.linalg.norm(p[:, 0] - p[:, 2]) * cam.rho[0])
            depths.append(Z)
            v = -IBVS_GAIN * np.linalg.pinv(cam.visjac_p(p, Z)) @ e.flatten(order="F")
            v_world = np.r_[Tc.R @ v[:3], Tc.R @ v[3:]]
            # Camera velocity from joint rates: the flange Jacobian shifted to the camera.
            r = Tc.t - Tf.t
            shift = np.block([[np.eye(3), -_skew(r)], [np.zeros((3, 3)), np.eye(3)]])
            J = shift @ robot.jacob0(q)
            qdot = np.linalg.solve(J.T @ J + 1e-4 * np.eye(6), J.T @ v_world)
            q = q + qdot * DT
            c, what = checker.worst(q, False, pitcher_home())
            if c < 0:
                raise PlanningError(f"visual servo: collision ({what})")
            self.set_q(q)
            if k % 20 == 0:
                print(f"    t = {k * DT:4.1f} s   feature error {errs[-1]:6.1f} px   Z = {Z:.3f} m")
        Tc = robot.fkine(q) * CAM_ON_FLANGE
        found = (Tc * view.inv()).t[:2]
        print(f"  converged in {k * DT:.1f} s; cup found at ({found[0]:.4f}, {found[1]:.4f}), "
              f"{np.linalg.norm(found - true) * 1000:.2f} mm from where it really is")

        def plots(plt):
            h = np.array(hist)
            fig, ax = plt.subplots(1, 2, figsize=(11, 5))
            for i in range(4):
                ax[0].plot(h[:, 0, i], h[:, 1, i])
                ax[0].plot(*h[0, :, i], "o", c="grey")
                ax[0].plot(*p_goal[:, i], "x", c="k", ms=10)
            ax[0].set_xlim(0, CAM_PIX)
            ax[0].set_ylim(CAM_PIX, 0)
            ax[0].set_aspect("equal")
            ax[0].set_title("Image plane: rim marks (o start, x goal)")
            ax[1].plot(np.arange(len(errs)) * DT, errs)
            ax[1].set_yscale("log")
            ax[1].set_xlabel("time (s)")
            ax[1].set_ylabel("mean feature error (px)")
            ax[1].set_title("IBVS convergence")
            fig.tight_layout()
        show(plots)

        print("\n  Re-planning the whole milk cycle to pour at the cup the camera found ...")
        self.set_q(HOME_Q)
        self.camera.T = HIDDEN.A
        segs = milk_scene.plan_task(robot, cup_at=tuple(found))
        print(f"  {len(segs)} segments, collision checked against the cup where it really is. Playing it:")
        milk_scene.play(self.scene, self.env, segs)
        for m in self.markers:
            m.T = HIDDEN.A
        self.env.step(DT)


def _skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


STAGES = ["stage_model", "stage_ik", "stage_rmrc", "stage_avoid", "stage_servo"]


def main(argv=None):
    global SHOW_PLOTS
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument("-s", "--stage", type=int, default=1, choices=range(1, len(STAGES) + 1),
                        help="stage to start at")
    parser.add_argument("--only", action="store_true", help="run just that one stage")
    parser.add_argument("--no-plots", action="store_true", help="skip the matplotlib windows")
    args = parser.parse_args(argv)
    SHOW_PLOTS = not args.no_plots

    import swift

    env = swift.Swift()
    env.launch(realtime=True)
    demo = Demo(env)
    todo = STAGES[args.stage - 1:args.stage] if args.only else STAGES[args.stage - 1:]
    pause("Swift is open. ENTER to start")
    for i, name in enumerate(todo):
        getattr(demo, name)()
        pause("ENTER for the next stage" if i < len(todo) - 1 else "Done. ENTER to close")
    env.close()


if __name__ == "__main__":
    main()
