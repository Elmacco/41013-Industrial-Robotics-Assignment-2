"""
Simulated rehearsal of ur3_demo.py.

Drives a simulated UR3 in Swift through the exact same POSE_A/POSE_B joint
targets used by ur3_demo.py (the real-robot ROS control script), animating
the transition between them so the motion can be checked visually for
self-collision / joint-limit issues before it is sent to the real robot.
"""
import swift
from roboticstoolbox import jtraj
from ir_support import UR3

# Joint order matches ur3_demo.py's JOINT_NAMES:
# [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
POSE_Initial = [0.0, -1.57, 0.0, 0.0, 0.0, 0.0]
POSE_A = [1.57, -1.57, -1.57, -1.57, 0.0, 0.0]
POSE_B = [0.0, -1.57, -1.57, -1.57, 0.0, 0.0]

TRAJ_STEPS = 50
STEP_TIME = 0.02  # secs per animation step, matches env.step() dt


def move_to(env, robot, pose, label):
    input(f"Press ENTER to move (sim) to {label}")
    for q in jtraj(robot.q, pose, TRAJ_STEPS).q:
        robot.q = q
        env.step(STEP_TIME)
    print(f"Reached {label}")


env = swift.Swift()
env.launch(realtime=True)

ur3 = UR3()
ur3.add_to_env(env)
env.set_camera_pose([1.5, 1.5, 1.0], [0, 0, 0.3])

move_to(env, ur3, POSE_A, "pose A")
move_to(env, ur3, POSE_B, "pose B")
move_to(env,ur3, POSE_Initial, "POSE_Initial")

input("Press ENTER to close")
env.close()
