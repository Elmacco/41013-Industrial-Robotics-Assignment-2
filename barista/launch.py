"""Launch the barista cell in Swift."""
import sys
from pathlib import Path

import swift

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from barista import layout
from barista.scene import build_scene


def main():
    env = swift.Swift()
    env.launch(realtime=True)
    scene = build_scene(env)
    env.set_camera_pose(list(layout.CAMERA["position"]), list(layout.CAMERA["look_at"]))
    env.step(0.02)

    print("Barista cell loaded:")
    for name, spec in layout.ROBOTS.items():
        print(f"  {name:9s} {spec['model']:10s} {spec['role']}")
    input("Press ENTER to close")
    env.close()
    return scene


if __name__ == "__main__":
    main()
