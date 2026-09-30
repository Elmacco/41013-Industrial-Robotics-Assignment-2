# 41013 Industrial Robotics - Assignment 2

Assignment 2 for 41013 Industrial Robotics (UTS), Spring 2026.

## Environment setup

This subject requires **Python 3.10-3.12** (3.13+ is not supported by the
Robotics Toolbox / RVC3 stack used here). Do not use your default/global
Python install if it's a different version - create a dedicated virtual
environment instead so this subject never interferes with others.

1. Create the virtual environment (from this folder):

   ```powershell
   py -3.12 -m venv .venv
   ```

2. Select the `.venv` interpreter in VS Code (`Ctrl+Shift+P` -> *Python: Select Interpreter*).

3. Install the dependencies. Either pin to the exact versions used here:

   ```powershell
   .\.venv\Scripts\python.exe -m pip install -r requirements.txt
   ```

   or install the subject support package and let pip resolve:

   ```powershell
   .\.venv\Scripts\python.exe -m pip install ir-support-full
   ```

4. Apply Windows workaround patches:

   ```powershell
   .\.venv\Scripts\python.exe -m ir_support.doctor --patch
   ```

See the Canvas "Python Wiki - Toolbox and Library Installation" page for full details.

## Key packages

| Package | Purpose |
| --- | --- |
| `ir-support-full` | UTS subject support package (models, extra robots/parts) |
| `roboticstoolbox-python` | Peter Corke's Robotics Toolbox |
| `spatialmath-python`, `spatialgeometry` | Poses, transforms, collision primitives |
| `swift-sim` | Browser-based 3D robot simulator |
| `machinevision-toolbox-python`, `rvc3python` | Machine vision / RVC3 textbook support |
| `open3d`, `trimesh`, `pybullet` | Point clouds, meshes, physics |
| `pygame`, `keyboard` | Teach pendant / gamepad input |

`numpy` is held at `1.26.4` for Robotics Toolbox compatibility - do not upgrade it to 2.x.
