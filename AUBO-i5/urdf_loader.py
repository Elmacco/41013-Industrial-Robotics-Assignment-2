from roboticstoolbox import Robot
from pathlib import Path
import numpy as np
import swift
import threading
import time

urdf_dir = Path(__file__).parent
# tld makes relative mesh paths in the URDF resolve against this folder
links, name, urdf_string, urdf_filepath = Robot.URDF_read("aubo_i5.urdf", tld=urdf_dir.as_posix())
robot = Robot(links, name=name, urdf_string=urdf_string, urdf_filepath=urdf_filepath)

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

robot.q = np.zeros(robot.n)

# input() blocks, so wait for ENTER on a background thread while the sim loops
stop = threading.Event()
threading.Thread(target=lambda: (input("Press ENTER to close\n"), stop.set()), daemon=True).start()

t0 = time.time()
try:
    while not stop.is_set():
        t = time.time() - t0
        robot.q = 0.5 * np.sin(t) * np.ones(robot.n)
        env.step(0.02)
except KeyboardInterrupt:
    pass

env.close()