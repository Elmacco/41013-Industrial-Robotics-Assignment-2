# Doosan M0609 - my robot model

The Doosan M0609 is a 6-axis collaborative robot (6 kg payload, 900 mm reach).
In the barista cell it runs the milk station: it fills, steams and pours the
milk, then rinses the pitcher. Doosan supplies real barista robots of this
kind to cafes, which is why I picked it for this job.

It is not in the 41013 model packs or Peter Corke's Robotics Toolbox. The packs
do have a different Doosan, the A0509, from another series with different
dimensions.

| File | What it is |
| --- | --- |
| `barista/doosan_m0609.py` | The model: my DH table, joint limits and the meshes |
| `Doosan-M0609/derive_dh.py` | Derives the DH table from Doosan's joint geometry and checks it |
| `Doosan-M0609/m0609.urdf`, `meshes/` | Doosan's description files (BSD-3, see `LICENSE-doosan`), used for the visuals and the check |
| `barista/collision.py` (`M0609`) | Collision capsules fitted to the meshes |
| `barista/doosan_demo.py` | Viva demo: each technique on its own (see below) |
| `barista/doosan_teach.py` | Teach window: joint sliders, Cartesian jog, go to pose, e-stop |

## The DH table

Standard DH, metres and radians. `offset` is the joint angle at which my DH
frame lines up with Doosan's zero position, so the joint angles in the model
are the robot's own joint angles. At q = 0 the arm stands straight up.

| Joint | d | a | alpha | offset | Limits |
| --- | --- | --- | --- | --- | --- |
| 1 | 0.1345 | 0 | pi/2 | pi | +-360 deg |
| 2 | 0 | 0.411 | 0 | pi/2 | +-360 deg |
| 3 | 0.0062 | 0 | pi/2 | pi/2 | +-150 deg |
| 4 | 0.368 | 0 | pi/2 | -pi | +-360 deg |
| 5 | 0 | 0 | pi/2 | pi | +-360 deg |
| 6 | 0.121 | 0 | 0 | 0 | +-360 deg |

What the numbers mean:

- 0.1345 m: the shoulder is 134.5 mm above the mounting face.
- 0.411 m: the upper arm, from the shoulder to the elbow.
- 0.0062 m: a small 6.2 mm sideways step at the elbow, along the joint 3 axis.
- 0.368 m: the forearm, from the elbow to the wrist centre.
- 0.121 m: from the wrist centre to the tool flange.
- Joints 4, 5 and 6 meet at one point (d5 = 0, a4 = a5 = 0). This is a
  spherical wrist, which gives the arm a wrist singularity when joint 5 is at
  zero (see RMRC below).

With the arm straight out (joint 2 at 90 degrees) the flange is 0.900 m from
the base axis. That matches Doosan's published 900 mm reach.

## How I derived it

`python Doosan-M0609/derive_dh.py`

Doosan's URDF gives each joint as a fixed transform from the previous link
followed by a rotation about z. From it I took only where each joint axis is
at the zero pose. Then I built the DH frames with the common-normal procedure
from lectures:

1. z(i-1) is the axis of joint i.
2. x(i) lies along the common normal from z(i-1) to z(i). If the axes
   intersect, x(i) = z(i-1) x z(i). If they are parallel, the normal goes
   through the previous origin.
3. d(i) is the distance along z(i-1) to the normal. theta(i) is the angle from
   x(i-1) to x(i). a(i) is the length of the normal. alpha(i) is the angle
   from z(i-1) to z(i) about x(i).
4. The last frame goes at the flange, along the joint 6 axis.

The table above is that result, written with exact pi/2 values.

## How I know it is right

1. **Derived table vs Doosan's URDF:** over 2000 random joint poses, the flange
   position matches to 0.000000 mm.
2. **Hand-written table (exact pi/2) vs Doosan's URDF:** within 0.31 mm.
   Doosan's URDF rounds pi/2 to 1.571, which accounts for the difference.
3. **Reach:** 0.900 m, matching the datasheet.

## How it looks

Each Doosan mesh belongs to one link, so it is fixed to that link's DH frame by
a constant transform: (DH frame)^-1 x (Doosan's link frame), both taken at
q = 0. It stays constant because both frames move with the link. This gives
the real robot's shape and proportions instead of cylinders.

## Collision model

The M0609's bodies sit well off its joint axes. For example, the upper arm runs
18-21 cm out to the side along the shoulder axis. So capsules drawn between
the DH frame origins would miss most of the robot. Instead, each link gets
capsules fixed in its own DH frame, fitted to Doosan's meshes:

1. Take the link's mesh vertices (placed by the mesh file's own scene-graph
   transforms).
2. Find their main direction (principal axis).
3. Split the link into equal lengths along that axis: four for the upper arm,
   two for the forearm, one for each other link.
4. Give each piece a capsule whose radius encloses 99.5% of its vertices.

Every vertex ends up inside a capsule or within 3.2 mm of one. The planner then
keeps a further 10 mm of clearance on top of that.

## RMRC in the pour

The milk station's straight-line moves (pouring, carrying, steaming) use
resolved motion rate control instead of solving IK at every sample
(`Planner.rmrc_step` in `barista/planner.py`).

**Why it is needed.** Partway through the pour, joint 5 passes through zero,
so joints 4 and 6 line up: the wrist singularity. Manipulability falls from
about 0.02 to 0.001. Solving plain IK at each sample flips joints 4 and 6 by
a large amount between two samples: a 1.3 rad jump, which the planner refuses.

**What RMRC does.** For each sample, it computes the flange velocity needed to
reach the next pose. It turns that into joint velocities by weighted, damped
least squares:

    qdot = (J^T W J + lambda^2 I)^-1 J^T W xdot

- lambda grows only when the manipulability drops below 0.02.
- W weights position above orientation: the spout must stay over the cup,
  while a fraction of a degree of tilt does not matter.

**The cost.** Damping trades a little accuracy for smooth, bounded joint
speeds. Right at the singularity, the flange drifts up to 5 mm for 3 of the
pour's 100 samples (0.15 s). Every other RMRC move tracks within 0.5 mm.

## Changes to the milk station for this arm

The station was first laid out for an AUBO i5, which I dropped because it is
already in the subject's model packs. The M0609's wrist housing is much
bulkier (about 10 cm radius against 4.5 cm), so two things changed:

- **The dispenser bay is raised to 10 cm.** Gripping the pitcher in a bay at
  table height put the wrist into the table.
- **The pitcher is drawn out of the bay 7 cm instead of 12 cm.** That is just
  enough to clear the nozzle, without folding the elbow until the wrist meets
  the shoulder.

`python -m barista.milk_scene --check` and `python -m barista.full_scene --check`
plan every drink with it, with no collisions.

## The viva demo

`python -m barista.doosan_demo` runs the M0609 through each technique on its
own, pausing for ENTER between stages (`--stage N` starts at stage N, `--only`
runs just that one, `--no-plots` skips the plot windows). Every stage uses the
milk station's own code, not a copy made for the demo.

| Stage | Shows | What to look for |
| --- | --- | --- |
| 1 Model | DH table, check against Doosan's URDF, reach, DH frames on the arm while each joint turns | 0.32 mm worst error over 500 poses (the URDF's rounded pi/2); 0.900 m reach |
| 2 IK | `ikine_LM` from 41 seeds for one pose: all 8 configurations, each scored by clearance | 4 collide (upper arm in the table); the planner takes the clearest |
| 3 Trajectories and RMRC | The pour solved three ways | IK per sample is refused at sample 33 (a 1.30 rad flip of joints 4 and 6); `jtraj` from the pour's first pose to its last swings the spout 71 mm off the cup's axis and hits the cup; RMRC gets through. Plots manipulability, q5, damping and tracking error |
| 4 Collision avoidance | A bottle left between the wash station and the dispenser | All 4 direct paths are blocked; a random clear via point (Lab 5's RRT step) gives a detour |
| 5 Visual servoing | The cup 39 mm from where the plan expects it | See below |

### Visual servoing (stage 5)

**Why the milk robot needs it.** The UR3 puts the cup down at the hand-off
spot, and in a real cell it will not land exactly where the plan says. A cup
39 mm out means the pour lands 60 mm from the cup's centre, outside its 40 mm
rim, so the milk ends up on the bench.

**How.** A camera on the side of the gripper (`CentralCamera`, 8 mm lens,
640 x 640) looks along the tool axis. Its image features are four marks round
the cup rim. The goal image is the rim seen from 25 cm straight above the cup.
It is the same wherever the cup is, so it is set once from the plan. Each
step is image-based visual servoing (IBVS):

    v_cam = -gain * pinv(L(p, Z)) * (p - p*)

- `L` is the interaction matrix (`visjac_p`).
- The depth Z is estimated from how big the rim looks, since the cup's size
  is known.
- The camera velocity becomes joint rates through the flange Jacobian,
  shifted to the camera's position, using damped least squares.
- Every step is collision checked.

**Result.** The error halves about every 0.6 s (gain 1.2/s). It converges in
about 4 s and finds the cup to within 0.2 mm. The whole milk cycle is then
re-planned with `plan_task(cup_at=...)` to pour where the camera found the
cup.

In the full cell the UR3 places the cup exactly (it is a simulation), so the
cell itself does not servo. This stage shows what the milk station does when
the hand-off is off.

## Teach window

`python -m barista.doosan_teach` opens the milk station with a teach window:

- **Joint sliders.** The arm follows a slider at up to 60 deg/s and stops short
  of anything it would hit.
- **Cartesian jog.** Hold a button to move along X, Y or Z (world or tool frame)
  or to turn about the tool's own axes. Each step is one `rmrc_step`, so near
  the wrist singularity the arm slows down instead of flipping. A jog stops at
  the edge of reach.
- **Go to pose.** Type a pose or pick a preset (home, dispenser, over the cup,
  steam wand). The planner solves IK, takes the clearest configuration and
  plans a collision-checked path to it.
- **Readout.** The flange pose, the manipulability (with a warning near the
  singularity) and the tightest clearance.
- **E-stop and reset.**
