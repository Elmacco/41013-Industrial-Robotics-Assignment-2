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

## Team workflow (2 people)

`main` should always run. Nobody commits to it directly: all work goes through a
short-lived branch and a pull request (PR) that the other person reviews.

### Day-to-day

1. Start from an up-to-date `main`:

   ```powershell
   git switch main
   git pull
   git switch -c <your-name>/<short-task>     # e.g. alec/milk-robot-path
   ```

2. Work and commit in small steps:

   ```powershell
   git add <files>
   git commit -m "Add steam wand approach pose for milk robot"
   ```

3. Before pushing, bring in your partner's latest work and re-test:

   ```powershell
   git fetch
   git rebase origin/main
   .\.venv\Scripts\python.exe -m barista.scene      # reach check must still pass
   ```

4. Push and open a PR on GitHub:

   ```powershell
   git push -u origin <your-name>/<short-task>
   ```

5. The other person reviews: they pull the branch, run `python -m barista.launch` to
   look at it in Swift, then approve. Merge with **Squash and merge** and delete the branch.

### Avoiding conflicts

- **Split ownership.** One person drives the espresso robot (UR3), the other the milk
  robot (UR3e). Do the shared runner (LinearUR3) together or take turns.
- **`barista/layout.py` is shared.** Every position lives there, so keep layout
  changes in their own small PR and merge them quickly, separate from motion code.
- **Say what you're starting** (group chat or a GitHub issue) so two people don't edit
  the same file at once.
- Merge at least once a day. Long-lived branches are what cause painful conflicts.

### Don't commit

- `.venv/`. It is gitignored; each person creates their own (see *Environment setup*).
- New packages without telling your partner. If you add one, update `requirements.txt`
  in the same PR so the other person can re-run `pip install -r requirements.txt`.
- AI tools as authors. Do not credit Claude (or any other AI assistant) as an author
  or co-author anywhere: no `Co-Authored-By: Claude ...` trailers in commit messages,
  no "Generated with Claude Code" lines in PR descriptions, and no author credits in
  code or docs. Remove any such lines a tool adds before committing.
