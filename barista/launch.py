"""Launch a barista cell scene in Swift.

    python -m barista.launch            # scene 1 (default)
    python -m barista.launch --scene 2  # scene 2
"""
import argparse
import importlib
import sys
from pathlib import Path

import swift

# Allow running this file directly (e.g. VS Code's Run button), not just via -m.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# scene name -> (scene module, layout module). Add a line per new scene.
SCENES = {
    "1": ("barista.scene", "barista.layout"),
    "2": ("barista.scene2", "barista.layout2"),
}
DEFAULT_SCENE = "2"


def load(name):
    """Import the scene and layout modules for a scene name."""
    scene_mod, layout_mod = SCENES[name]
    return importlib.import_module(scene_mod), importlib.import_module(layout_mod)


def main(scene_name=DEFAULT_SCENE, cycles=1):
    scene_module, layout = load(scene_name)

    env = swift.Swift()
    env.launch(realtime=True)
    scene = scene_module.build_scene(env)
    env.set_camera_pose(list(layout.CAMERA["position"]), list(layout.CAMERA["look_at"]))
    env.step(0.02)

    print(f"Scene {scene_name} loaded:")
    for name, spec in layout.ROBOTS.items():
        print(f"  {name:9s} {spec['model']:10s} {spec['role']}")
    print(f"  {len(scene.props)} props, {len(scene.placeholders)} placeholders")

    # Scenes with moving parts expose animate(scene, env, cycles=...).
    animate = getattr(scene_module, "animate", None)
    if animate is not None and cycles > 0:
        print(f"  running {cycles} animation cycle(s)")
        animate(scene, env, cycles=cycles)

    input("Press ENTER to close")
    env.close()
    return scene


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-s", "--scene", choices=sorted(SCENES), default=DEFAULT_SCENE,
                        help=f"which scene to launch (default: {DEFAULT_SCENE})")
    parser.add_argument("-c", "--cycles", type=int, default=1,
                        help="animation cycles to run, for scenes that have moving "
                             "parts; 0 for a static scene (default: 1)")
    return parser.parse_args(argv)


if __name__ == "__main__":
    args = parse_args()
    main(args.scene, args.cycles)
