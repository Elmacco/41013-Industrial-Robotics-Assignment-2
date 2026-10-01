"""Launch the barista cell in Swift.  Run from the repo root:  python -m barista.launch"""
import swift

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
