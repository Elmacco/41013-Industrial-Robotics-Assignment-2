"""
AUBO i5 cobot, loaded from the URDF and meshes in ``AUBO-i5/``.

Wrapped as a class with ``add_to_env`` so ``build.make_robot`` can place it from
a layout like any ir_support robot (layout model name ``"AuboI5"``).

At q = 0 the arm stands straight up with the flange 1.008 m above the base.
"""
from pathlib import Path

import numpy as np
from roboticstoolbox import Robot

URDF_DIR = Path(__file__).resolve().parents[1] / "AUBO-i5"


class AuboI5(Robot):
    def __init__(self):
        links, name, urdf_string, urdf_filepath = Robot.URDF_read(
            "aubo_i5.urdf", tld=URDF_DIR.as_posix())
        super().__init__(links, name=name, manufacturer="AUBO Robotics",
                         urdf_string=urdf_string, urdf_filepath=urdf_filepath)
        self.q = np.zeros(self.n)

    def add_to_env(self, env):
        env.add(self)
