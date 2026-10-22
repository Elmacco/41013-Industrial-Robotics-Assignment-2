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

## Scenes

| Command | What it shows |
| --- | --- |
| `python -m barista.full_scene` | **The full cell.** Opens Swift and an order window: order a drink, the robots make it, it comes out through the collection drawer |
| `python -m barista.full_scene --no-window` | The same, ordering from the Swift tablet buttons instead |
| `python -m barista.full_scene --check` | Plans every drink without Swift and reports any collision |
| `python -m barista.milk_scene` | The Doosan M0609 milk station on its own (model: [Doosan-M0609/README.md](Doosan-M0609/README.md); station: [barista/MILK_STATION.md](barista/MILK_STATION.md)) |
| `python -m barista.doosan_demo` | The M0609 one technique at a time, for the viva: model, IK, RMRC, collision avoidance, visual servoing |
| `python -m barista.doosan_teach` | Teach window for the M0609: joint sliders, Cartesian jog, go to pose, e-stop |
| `python -m barista.gamepad` | Checks the Xbox controller used as the physical e-stop: prints each button and the e-stop state |

### The full cell (`barista/full_scene.py`, `barista/full_layout.py`)

- **Enclosure:** 2 m clear walls on the floor round the table (no roof, so the
  camera has a clear view). Yellow and
  black safety tape on the floor just outside. The only opening is the slot the
  collection drawer slides through, and a shroud stops anyone reaching through it.
- **Ordering:** a desktop window opens next to Swift (`barista/kiosk_window.py`,
  tkinter, so nothing extra to install). It has:
  - a button per drink (latte, cappuccino, flat white, swirl latte, espresso)
    with its usual time;
  - a live queue: each order's number, status (queued, planning, being made,
    ready at the drawer, collected) and a countdown to when it can be collected;
  - what each robot is doing, the beacon colour, a banner showing the e-stop
    state, and E-STOP, Release, Reset and Resume buttons.

  Countdowns marked `~` are estimates for orders not yet started; once a batch
  is planned they come from its schedule (and stop while the e-stop is on).
  Closing the window quits. The Swift page keeps its own tablet buttons too,
  standing in for the kiosk outside the glass.
- **Planning ahead:** on start-up every drink is planned in the background
  (about two minutes the first time; you can order meanwhile). Plans are saved
  in your temp folder, named after a hash of the `barista/` code, so later runs
  load them instantly and only re-plan after the code changes. A new mix of
  drinks ordered together then takes about 2 s to schedule. `--check` never
  uses the saved plans.
- **E-stops** (`barista/estop.py`): one inside the cell (on the table corner),
  one outside (by the kiosk), one in the order window, and a physical one, **B**
  on a USB Xbox controller (`barista/gamepad.py`; `--no-gamepad` turns it off).
  The shared e-stop logic works like this:
  1. Any e-stop freezes every robot on the next frame.
  2. Releasing it (twisting the screen e-stop out, letting go of B) restarts
     nothing.
  3. **Reset** (Y) acknowledges the stop. It is refused while any e-stop is
     still pressed.
  4. A separate **Resume** (A) restarts. The robots carry on from the exact
     frame they stopped on, along paths that were collision checked before
     anything moved.

  Unplugging the controller counts as pressing the e-stop. The beacon on its
  mast shows green (ready), amber (making), red (e-stop) or blue (reset,
  waiting for Resume). Nothing waits in a loop while stopped: the normal frame
  loop just holds the frame. These are a simulation's buttons, not a certified
  safety system. `python -m barista.gamepad` checks the controller and prints
  its button numbers.
- **Rechecking before a restart:** the plans were checked against the cell as
  it was when they were made. Something left in the cell afterwards would not
  be in them, so Reset rechecks first.
  - **The test objects.** Buttons in the order window and in Swift leave a box
    in the UR3's path or a box in a clear corner. They stand in for what a
    light curtain or camera would report. Leaving one while the robots are
    moving trips the e-stop.
  - **What Reset checks.** It tests every robot, and whatever it is
    carrying, against anything left in the cell, at every remaining frame of
    the batch. If something is in the way, Reset is refused with the reason,
    for example *"the UR3's cup would hit the box in the UR3's path in 3.8 s
    - remove it, then Reset"*. The robots stay stopped until the cell is
    cleared.
  - **What is out of the way.** A box out of the way passes the check, and
    the batch carries on.
  - **New orders.** A new batch that would hit something left in the cell
    waits at the front of the queue, and starts once the cell is cleared.
  - **No re-planning.** The robots never re-plan round the obstacle mid-drink:
    with cups and the portafilter mid-move, staying stopped is the safe
    recovery.
- **Maintenance doorway** (`barista/safety.py`): staff get in through a
  doorway in the right-hand glass. It has no door. Instead:
  - an **area scanner** watches the floor in front of it, and anyone there
    slows every robot to **50 %** (same paths, shown at half rate) until they
    leave;
  - a **light curtain** across the doorway, and presence sensing inside the
    cell, stop every robot through the e-stop latch. The robots stay stopped
    until the person is out, then Reset (with the path recheck), then Resume.

  "Staff member" buttons in the order window and in Swift send the person (in
  hi-vis) to the door, into the cell, or away. They walk there themselves.
  The sensors read where they are every frame, so the signals arrive
  whenever the person gets there, not at set points in the robots' program.
- **Safety equipment and measures, by risk:**

  | Risk | What deals with it |
  | --- | --- |
  | Someone reaching a moving robot | Clear 2 m walls all round; the drawer slot fits only the tray, behind a shroud |
  | Staff coming to the cell | Floor tape marking the zone; area scanner (reduced speed); warning sign at the doorway |
  | Staff entering | Light curtain and presence sensing (protective stop); restart only by Reset then Resume once clear |
  | Something left in a robot's path | Recheck of the remaining paths before any restart; new batches wait until the cell is clear |
  | Anything else | Four e-stops (inside, outside, order window, Xbox controller B) |
  | Knowing the state from outside | Beacon on a mast (green, amber, red, blue) and banners on the screens |
  | Robots or carried objects colliding | Every path collision checked before anything moves; robot pairs checked at every frame |
  | Fire | Extinguisher by the maintenance doorway |

  This is a simulation for teaching, not a certified safety system.
- **One order:**
  1. The cup dispenser (a clear tube holding a stack of cups) drops a cup, and
     the UR3 puts it under the espresso machine's group head.
  2. The CS66 makes the shot:
     - takes the portafilter off its dock;
     - fills it at the grinder;
     - presses it in the tamping machine;
     - locks it into the group head with a twist, and the shot pours into the cup;
     - knocks the spent puck into the knock box;
     - rinses the portafilter at its wash station and puts it back on the dock.
  3. The UR3 carries the cup to the hand-off spot.
  4. The Doosan M0609 fills, steams and pours the milk.
  5. The UR3 takes the drink to the drawer.
  6. The drawer slides out to the customer.

  An espresso skips steps 3 and 4.
- **Robots work at the same time.** Each robot only waits where it really has to:
  - the CS66 grinds and tamps while the UR3 fetches the cup, and only waits for
    the cup before locking into the group head;
  - the M0609 steams the milk from the start and only waits for the cup to pour;
  - the CS66 cleans up, and the M0609 rinses and refills, while the drink moves on;
  - orders queued together are made together: the UR3 fetches the next cup
    before taking the last drink to the drawer.

  A latte takes about 81 s (164 s one robot at a time), an espresso about 63 s,
  and a latte, espresso and cappuccino together about 206 s (421 s). Orders
  placed while a batch is being made go into the next batch.
- **Collision checking:** each robot's moves are checked against the cell as
  they are planned. Then `barista/schedule.py` runs the robots together and
  checks every pair of robots against each other at every frame. If two would
  come within 10 mm, the lower-priority robot pauses until the way is clear; if
  they would block each other for good, planning fails and names them.
  `--check` schedules every drink and a three-drink batch, and reports the
  time, any pauses, and how close the robots came.
- **Espresso station layout:** every CS66 station sits 0.5 m from its base,
  set by angle in `CS66_ANGLE` in `full_layout.py`, with the portafilter
  handle always pointing back at the CS66. The CS66 never swings the
  portafilter through the UR3's side (`CS66_KEEP_CLEAR` in `full_scene.py`),
  which also stops its base joint winding past its limit.

## Real UR3

The real UR3e (with its OnRobot gripper) runs a short pick-and-place demo: it
picks a 5 cm block up off the bench, carries it to a second spot and puts it
down. `UR3/ur3_real.py` holds the demo as a `STEPS` list of joint targets and
gripper actions, and sends every move through `move_ur_joint_positions` from
the lab's control script, `UR3/new_ur_control_script.py` (exactly as given,
not modified). The gripper is driven
through `UR3/simple_gripper_control.py` (the `/onrobot/open` and
`/onrobot/close` services); the script stops if a gripper call fails.

| Command | What it does |
| --- | --- |
| `python UR3/jointstatesim.py` | Animates `STEPS` in Swift, with a gripper and the block, to rehearse before using the real robot |
| `python UR3/simple_gripper_control.py` | Opens then closes the real gripper, to check it responds |
| `python UR3/ur3_real.py --dry-run` | Prints every call it would make, without a robot |
| `python UR3/ur3_real.py` | Runs the demo on the real UR3e |
| `python UR3/pick_place.py` | Re-works out `STEPS` with the `ir_support` UR3e model after changing the block position, drop position or gripper length; paste its output over `STEPS` |

Before running it on the real arm, measure the gripper's flange-to-finger-pad
distance, set `TOOL_LEN` in `pick_place.py` and regenerate `STEPS`. Then put
the block where the sim shows it, keep the physical e-stop in reach, and
follow the lab's risk assessment.

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
   .\.venv\Scripts\python.exe -m barista.full_scene --check   # every drink must still plan
   ```

4. Push and open a PR on GitHub:

   ```powershell
   git push -u origin <your-name>/<short-task>
   ```

5. The other person reviews: they pull the branch, run `python -m barista.full_scene` to
   look at it in Swift, then approve. Merge with **Squash and merge** and delete the branch.

### Avoiding conflicts

- **Split ownership.** Each person owns their own robot: the milk robot (M0609),
  the espresso robot (CS66) and the runner (UR3).
- **`barista/full_layout.py` is shared.** Every position in the cell lives there, so
  keep layout changes in their own small PR and merge them quickly, separate from
  motion code.
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
