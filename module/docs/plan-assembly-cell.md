# Plan: the mailbox assembly cell

A rebuild, not an edit. The two-arm inspection cell is tagged `two-arm-inspection-cell`
and everything below replaces its layout and its round.

## What the cell does, as asked

A mailbox arrives on a fixture that rides a conveyor across the cell. Then, in order:

1. **Classify** what arrived, from a camera above the conveyor: good mailbox, damaged
   mailbox, or empty fixture.
2. **Inspect inside** - an arm puts a fisheye camera into the mailbox and takes a flash
   photograph, looking for paint defects and damage on the inside.
3. **Find the flag hole** - a RealSense on another arm detects the hole on the outside,
   by object detection.
4. **Insert the flag** - the same arm.
5. **Fit the door** - an arm takes a front door off a feeder rack and places it on the
   front end.
6. **Pin the door** - the same arm drives a pin through a hole.
7. **Check the back** - a photograph of the back, for paint.
8. **Check all round** - lift the mailbox in Z and rotate it under the overhead camera
   for paint.
9. **Route** - pass goes back in the fixture; fail goes to a repaint conveyor.

Four paint schemes. The flag and the door are NOT part of the mailbox asset any more:
they are separate parts the cell fits.

## The three decisions

### Three arms - and this time measured, not argued

A fable advisor reviewed the first version of this plan and took the three-arm reasoning
apart: it optimised for the simulator rather than the cell, and was wrong even on those
terms, because under physics-off a tool change is a weld swap that `surface_gripper.py`
already does, while the genuinely new mechanisms are the END EFFECTORS - and three arms
need every one of them anyway. The reviewer said a harness sweep covering every station
with margin would change their mind. So the sweep was built, and it does.

`probes/assembly/site.py` sweeps belt height, pedestal height, pedestal positions and
pedestal yaw, solving the belt's index positions for each arrangement and judging every
station on reach, pan and height. Over 15,498 arrangements that survive the physical
filters:

    THREE arms   worst margin -100  - the two FEEDER stations, at 38 and 40 degrees of pan
    TWO arms     worst margin -200  - the feeders worse, AND pin_insert at 34 degrees

Every station that rides the belt clears with +46 to +242 of margin under three arms,
including all three insertions. Under two arms the pin insertion itself goes outside the
pan bound. That is the deciding number: a feeder pick at 38 degrees is a comfort
complaint about a station that picks a door off a rack, and an insertion at 34 degrees is
a hole the arm has to find at an angle it does not like.

The two failures that remain are both fixed points the optimiser places rather than
stations the process dictates, and neither is an insertion. They are recorded rather than
hidden, and are the first thing to revisit if the cell is ever tight.

### What the sweep taught, which the plan had wrong

**The belt indexes the shell to each station.** This was the biggest error in the first
draft: it pinned the shell at one work position, and then the cycle's stations spanned
about 900 mm of x - the fisheye enters the front aperture at one end while the back cap is
photographed at the other - which no 30 degree pan cone covers. Every sweep failed on it.
A conveyor running the length of the cell exists precisely so the work comes to the arm,
and once the index positions are solved per operation the two inspection shots share ONE
flange pose: advance the shell by its own length and the back cap is where the front was.

**Pedestal yaw is a free variable and was missing.** Pan was being measured from world +X
for every arm, which is only right if every arm faces +X - true of the previous two-arm
cell and false here. A pedestal's facing costs nothing at install time.

**The asset decides which side each arm stands on.** The flag hole's axis and the pin
bore's axis are both (0,-1,0), so both are entered from +Y: an assemble arm on -Y reaches
across the shell for every insertion, which the sweep reported as 938 mm of reach. And the
handling arm takes the -Y wall, so the two arms never share a side.

**Tool lengths are design choices, so the sweep sizes them.** At a 250 mm fisheye stem
nothing cleared; at 190 it does, and the stem still puts its tip 70 mm inside the aperture
with the flange 120 mm clear of the shell's front.

### No tool changers
Count the end effectors the sequence needs: a fisheye camera with a flash, a gripper that
can hold a flag and a pin and a door, a RealSense, and a gripper big enough to lift and
rotate the whole mailbox. A RealSense rides alongside a gripper on the same wrist - that
is what eye-in-hand means - so it needs no arm of its own. That leaves three jobs that
cannot share one wrist: looking INSIDE the mailbox, assembling small parts onto it, and
handling the mailbox itself.

With two arms it is one tool change per cycle at least, probably two: the arm that puts a
camera inside cannot also be the arm that lifts the mailbox without swapping. With three
there are no changes at all.

**Decided on three, and the reason is development cost rather than cycle time.** A tool
changer is a mechanism to model in Isaac - attach, detach, the tool's own mass and
collision, a failure mode when it does not latch - and I have none of that. A third UR5e
on a pedestal is a thing this repo already builds, tests and teaches. The cycle-time
argument points the same way but less strongly: two changes at a plausible 8 s each is
16 s on a cycle the budget puts near 60, so it is 25% and not a disaster.

The honest cost: a third arm is real money and real floor space, and if this cell were
being quoted rather than simulated the two-arm-plus-changer version deserves a second
look. Recorded here so the choice is visible rather than assumed.

Jobs:
- **arm-a, inspection.** Fisheye camera with flash on the wrist. Inside shot, back shot.
- **arm-b, assembly.** Gripper plus RealSense on the wrist. Flag, door, pin.
- **arm-c, handling.** Vacuum or jaw gripper sized for the mailbox. Lift, rotate under
  the overhead camera, return to the fixture or move to repaint.

### Sensors
- **overhead-cam** - fixed above the conveyor, looking down. Does the classification in
  step 1 and the all-round paint check in step 8. Needs the whole fixture in frame.
- **arm-a wrist, fisheye with flash** - a wide field of view, because it is inside a
  172 mm wide box. A light on the same bracket; the inside of a closed mailbox has no
  light of its own.
- **arm-b wrist, RealSense** - RGB for detection now. Depth when the module grows it: the
  camera model reports no depth and no point cloud today, and a peer session is already
  asking for both for the Viam 103 course. The flag hole is a 2D detection problem, so
  this does not block.
- **Dropped**: the old fixed `inspect-cam` at the side. Nothing in the new sequence
  presents a face to a fixed side camera - the arms bring the camera to the work instead.

### Changes to the cell's shape
- **The conveyor runs the whole length of the cell**, narrower than the current 900 mm
  deck. Narrow because the fixture is what carries the mailbox and the fixture is the
  thing that has to fit, and because a wide deck puts its own centreline further from
  every arm - which is what the current cell's outfeeds were narrowed for.
- **A fixture that raises the mailbox 100 mm off the belt**, with lips that hold it. A
  box for now; a USD later. The lips matter for a reason beyond tidiness: every grasp and
  every insertion in this sequence is relative to the mailbox, so the mailbox not moving
  is what makes a taught program possible at all.
- **A feeder rack** holding front doors, and a supply of flags and pins.
- **A repaint conveyor** for failures.

## How it gets built

### Physics off first
Everything kinematic: props do not fall, grasps are welds, no shear limits, no settling.
The two-arm cell spent most of its development on physics - a 50 N shear limit that could
not hold the part, a cup that appeared to slide 213 mm and did not, drives that settled
0.7 degrees short - and none of that work was about whether the cell's geometry and
sequence are right. Physics comes back on once the sequence runs.

### Build in Python and Isaac, come back to Viam after
The slow loop in the last build was: change the fragment, push config, wait 25 s, restart
the module, wait 45 s for Isaac, run a probe. For layout and reachability work that is
most of a minute per question. Directly against the sim there is no push and no restart.

So: a standalone harness that opens the stage, places props, and asks about reach and
collision, with no viam-server in the loop. The Viam fragment is generated from the same
description the harness uses, so there is one source and no second place to keep in step.

### Arm position savers, in folders per arm and per task
`fragments/poses/<arm>/<task>.json`. A taught pose is a joint configuration with a name,
and the name says which arm and which task - so "where does arm-b go to pick a flag" is
answerable without reading the round. This is the lesson of `stations()` in the old cell:
a pose the round does not visit will pass every test and fail every run, and the fix is
for poses to be first-class and named rather than derived in three places.

### Constraints on moves, by default
The old round learned this twice. Left free the planner may reorient the tool however it
likes along the way, and for a 495 mm part on a 206 mm lever that is a flail - measured,
the same start and goal gave a part on the floor free and an intact hold constrained. An
insertion is worse: a flag going into a hole has an approach axis, and a wrist that flips
on the way breaks the part. So moves that hold anything get an orientation constraint, and
insertions get a linear one.

## Tasks, in dependency order

- **A1 Layout harness.** Stage, props, no viam-server. Answer reach and collision offline.
- **A2 The conveyor and the fixture.** Full-length narrow belt, fixture raising the
  mailbox 100 mm, lips. Mailbox arrives and stays put.
- **A3 Three arms sited.** Every station in every arm's comfortable envelope, checked by
  the harness before anything is taught.
- **A4 Overhead camera and classification.** Three classes: good, damaged, empty fixture.
- **A5 Inside inspection.** Fisheye plus flash on arm-a, into a 172 mm opening.
- **A6 Flag hole detection.** RealSense on arm-b, 2D detection, then the insertion.
- **A7 Door from the feeder, and the pin.**
- **A8 Back and all-round paint checks.**
- **A9 Routing.** Pass back to the fixture, fail to repaint.
- **A10 Physics back on**, one stage at a time.
- **A11 Teach, measure a cycle, film.**

## Waiting on the mailbox-usd session
- Flag and door removed from the asset, as separate parts.
- Four paint schemes as a variant set.
- A flag hole on the outside, as geometry a detector can find.
- A door aperture and a pin hole.
- Collision bounds published as attributes, as the current asset does - the layout reads
  them rather than guessing.
