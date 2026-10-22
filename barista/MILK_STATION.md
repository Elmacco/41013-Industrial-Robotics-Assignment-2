# The milk station (Doosan M0609)

The milk side of the barista cell runs on the Doosan M0609 (the model and how
its DH table was derived: [Doosan-M0609/README.md](../Doosan-M0609/README.md)).
Each cycle it:

1. takes the full milk pitcher out of the milk dispenser,
2. steams the milk at the steam wand,
3. pours it into the cup,
4. rinses the pitcher upside down at the wash station,
5. puts it back in the dispenser, which refills it while the arm returns home.

All the station equipment is placeholder boxes and cylinders until real meshes are made.

This page covers the milk station on its own (`barista.milk_scene`). The full cell
(`barista.full_scene`) runs the same task, with the UR3 bringing the cup to the
hand-off spot and taking the finished drink to the collection drawer; see
*Scenes* in the main [README](../README.md).

## Running it

From the repo root, with the `.venv` set up (see the main [README](../README.md)):

```powershell
.\.venv\Scripts\python.exe -m barista.milk_scene                    # run one cycle in Swift (latte)
.\.venv\Scripts\python.exe -m barista.milk_scene --profile swirl    # pick a steam style
.\.venv\Scripts\python.exe -m barista.milk_scene --cycles 3         # run several cycles
.\.venv\Scripts\python.exe -m barista.milk_scene --list             # list the steam styles
.\.venv\Scripts\python.exe -m barista.milk_scene --check            # plan every style, no Swift
.\.venv\Scripts\python.exe -m barista.doosan_demo                   # each technique, one stage at a time
.\.venv\Scripts\python.exe -m barista.doosan_teach                  # teach window: sliders, jog, go to pose
.\.venv\Scripts\python.exe -m barista.milk_layout                   # print where every object sits
```

Run `--check` after any layout or motion change. It plans every steam style
without opening Swift and prints the tightest clearance in each segment, or
the step and parts where a plan fails.

## Files

| File | What it holds |
| --- | --- |
| `barista/doosan_m0609.py` | The robot: DH model drawn with Doosan's meshes |
| `barista/milk_layout.py` | Every position and size in the milk station, as plain data |
| `barista/steam_profiles.py` | The steam styles |
| `barista/milk_scene.py` | Task sequence and Swift playback |
| `barista/planner.py` | Motion planner shared by every robot |
| `barista/cup.py` | The glass cup and the coffee and milk inside it |
| `barista/collision.py` | Collision checker used by the planner |
| `barista/doosan_demo.py` | Viva demo: each technique on its own, with plots |
| `barista/doosan_teach.py` | Teach window |

## The layout

Top view (+X out from the robot, +Y to its left):

```
            milk dispenser (+100 deg)
                  [D]
                          wash (+50 deg)
                            [W]

         (M0609)  ----------------- [C]  cup (0 deg)
          origin       0.6 m

                            [S]
                          steamer (-50 deg)
```

The robot is bolted to the table at the world origin, 0.75 m up. The stations
sit on an arc 0.6 m out from the base (`STATION_R`), at these angles (`ANGLE`):

| Station | Angle | Notes |
| --- | --- | --- |
| Steamer | -50 deg | Tall body, wand hanging from an arm, drip tray |
| Cup | 0 deg | Translucent glass, so the milk level shows |
| Wash station | +50 deg | Low basin with an upward spray nozzle |
| Milk dispenser | +100 deg | Tall body, nozzle over a bay; the pitcher lives here between drinks |

Two reasons for this arrangement:

- **0.6 m out.** The arm's two main links add up to 0.78 m (0.411 + 0.368). With stations
  closer than that, the elbow folds up and the wrist dips into the table. At
  0.6 m the elbow stays between about 70 and 145 degrees.
- **Tall machines at the ends.** The cup and basin are low, so the full
  pitcher can be carried over them from the dispenser to the steamer without
  routing around anything.

Each machine is built facing the robot with `polar()` / `_facing()`, so moving
a station means changing one angle in `ANGLE`.

## Frames worth knowing

- **Pitcher frame:** origin at the centre of the pitcher's floor, +Z up its
  axis, +X out through the spout. The handle is on -X, facing the robot.
- **Gripper frame:** the robot flange, +Z pointing out of the tool. The
  gripper holds the handle with the flange `GRIP_DEPTH` (0.14 m) from the
  pitcher axis, `GRIP_HEIGHT` (0.06 m) up the pitcher. `GRASP` in
  `milk_scene.py` is the pitcher frame expressed in the flange frame.

Motion is planned in terms of where the pitcher should be, and
`flange(pitcher_pose)` converts that into the flange target for IK. The
pitcher's yaw always points its spout away from the base, so the gripper
always approaches from the robot's side.

## How a cycle is planned

`plan_task()` builds the whole cycle before anything moves, as a list of
`Segment`s (a joint path plus what the gripper is holding and which effects
are showing). `play()` then just steps through them in Swift. Planning first
means a collision or an unreachable pose fails straight away with a clear
message, instead of halfway through an animation.

The `Planner` offers these moves:

| Move | Used for | How |
| --- | --- | --- |
| `free_move` | Empty-handed moves (approach, home) | Joint-space path; only the end pose matters |
| `line_move` | Short approaches, lifts, dipping onto the wand | Straight line for the flange |
| `carry` | Carrying the pitcher between stations | Sweeps round the base on arcs, pitcher kept upright |
| `follow` | Steaming, pouring, turning over | Tracks a list of poses generated by a profile |
| `wait` | Rinsing, refilling | Holds still while an effect plays |

`line_move`, `carry` and `follow` track their poses by RMRC (damped least
squares, `Planner.rmrc_step`), starting from the previous sample's joints, so
the arm keeps one configuration throughout. A jump of more than 0.2 rad
between samples is refused. Why RMRC rather than IK at every sample: the pour
passes through the M0609's wrist singularity (see the Doosan README).

### Picking an IK solution

A 6-joint arm usually has up to 8 joint solutions for the same flange pose,
and some of them put the wrist through the table. `free_move` solves from the
current joints plus about 40 random starting points, discards every solution
that collides, and goes to the one with the most clearance. The Cartesian
moves that follow inherit that configuration.

Two details of the IK are handled inside `Planner._solve` (`planner.py`):

- **Checked answers.** Every answer is checked against the target with `fkine`.
- **Full turns.** Every joint can turn plus or minus 2 pi, so the solver may
  return the same pose a full turn away. Each answer is shifted to the turn
  nearest the starting joints.

## Collision checking

Every segment is checked at every step as it is planned (`collision.py`). If
any step collides, planning stops with the segment, the step and the two parts
involved, for example:

```
dip wand in: collision at step 13/29: gripper / steam_wand (-2 mm)
```

How it works:

- **Bodies.** Each arm link, the gripper and the held pitcher is a capsule: a
  line segment with a radius. The M0609's capsules are fixed in each link's DH
  frame and fitted to Doosan's meshes (see the Doosan README). This is
  the Lab 5 "check every link, every step" approach with thickness added. A
  plain line has no thickness, and can sit inside a box without crossing any
  face.
- **Obstacles.** Every placeholder in `milk_layout.PLACEHOLDERS` becomes a box
  (cylinders become their bounding box). Clearance is the distance from points
  along the capsule to the box, minus the capsule radius.
- **Margin.** The arm and gripper must stay 10 mm clear of everything. The
  pitcher only has to avoid touching, since it is set down on the dispenser
  grate.
- **Self-collision.** Capsule-to-capsule distance between links at least two
  joints apart (for example base vs forearm, upper arm vs wrist or pitcher).
- **The resting pitcher.** While it stands in the bay, the pitcher is an
  obstacle for the arm (except the gripper, which closes on it).
- **Allowed contact.** The pitcher may touch the steam wand, which goes inside
  it by design.

If a direct joint path is blocked, `free_move` falls back to random detour
points (Lab 5's basic RRT): it picks a random collision-free configuration and
tries the path through it. The station's own layout never needs this; stage 4
of `doosan_demo` leaves a bottle in the way to show it working.

## Steam styles

The wand never moves; the robot moves the pitcher around it. A style is a
function of progress `u` (0 to 1) that returns where the wand tip should sit
inside the pitcher (`WandPlacement`: x, y, height above the pitcher floor, and
the pitcher's tilt). `at_wand()` turns that into a pitcher pose.

| Style | What it does |
| --- | --- |
| `latte` | Short stretch, long whirlpool: silky microfoam |
| `cappuccino` | Long stretch for thick, airy foam |
| `flat_white` | Barely any stretch, deep and thin foam |
| `swirl` | The tip circles the pitcher axis instead of a fixed whirlpool |

The first three use `two_phase()`, which models how a barista steams:

- **Stretch:** the tip sits just under the surface to draw in air. The foam
  makes the milk rise, so the pitcher drops to keep the tip shallow.
- **Texture:** the tip goes deeper and off-centre with the pitcher tilted, so
  the milk spins into a whirlpool that folds the foam in.

The two phases blend over a short window so the arm never jumps between them.

To add a style, write a function `u -> WandPlacement` (or call `two_phase()`
with new numbers) and add it to `PROFILES` in `steam_profiles.py`. Then run
`--check` to make sure the arm can follow it without colliding.

## Pouring and rinsing

- **Pour.** The pitcher tips about its spout lip, which starts high over the
  near side of the cup and drops towards the coffee as the pitcher tips.
  `POUR_KEYS` sets the tilt and lip height over the pour. Milk flows once the
  tilt passes 30 degrees: a stream appears below the spout and the cup level
  rises.
- **Rinse.** Over the basin, the pitcher rolls upside down about the gripper
  axis (only the last wrist joint turns), lowers over the spray, rinses for
  `RINSE_TIME`, then rises and turns back upright.

## Common changes

| To change | Edit |
| --- | --- |
| Where a station is | `ANGLE` or `STATION_R` in `milk_layout.py` |
| Machine sizes | `PLACEHOLDERS` in `milk_layout.py` |
| The order of the cycle | `plan_task()` in `milk_scene.py` |
| Carry height | `CARRY_Z` in `milk_scene.py` |
| Pour shape or timing | `POUR_KEYS`, `POUR_DURATION` in `milk_scene.py` |
| Rinse and refill timing | `RINSE_TIME`, `REFILL_TIME` in `milk_scene.py` |
| Safety margin | `margin` in `CollisionChecker` (`collision.py`) |
| Link thickness | `M0609` in `collision.py` |

If `--check` fails after a change, the message names the segment and the two
parts that collided. The usual fixes:

- move the station,
- raise the carry height,
- add a waypoint to the `carry()` call in `plan_task()`.
