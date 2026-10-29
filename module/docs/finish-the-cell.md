# Finishing the QC cell

What "finished" means here: a round runs end to end without a human in it — a mailbox
arrives on the belt, both arms inspect it, and it lands in the tray its defects say it
belongs in — and a test says so rather than a person watching a video.

## What a round is, precisely

1. **Pick.** arm-a takes the mailbox off the belt by a **flat side**, approaching from +y.
   The −y side is over the belt and the planner refuses it; that constraint is the
   layout's, not the gripper's.
2. **First check.** arm-a presents the faces it is not covering to `inspect-cam`: crown,
   base, door end, back end, and the free side. Five of six.
3. **Handoff, by set-down.** arm-a places the mailbox on a small fixture near y=0 and
   retreats; arm-b then picks it by the **opposite** flat side. The part has to change
   hands at all because arm-a **cannot reach the trays** by construction.

   An earlier draft had arm-b take it mid-air, and said in one line that arm-b takes "the
   side arm-a was holding" and in the next that arm-b presents "the face arm-a's cup was
   covering". Those cannot both be true, and the first makes the second check pointless.
   It is the *opposite* side.

   Mid-air is also the wrong mechanism. Each cup's attachment joint locks all six degrees
   of freedom, so while both cups are closed on one rigid body any drive mismatch between
   the arms is reported by PhysX as joint load — and load is what breaks the shear limit
   and drops parts. A set-down handoff has no moment where two welds fight, and no moment
   where both arms need to plan into the same space.

4. **Second check.** arm-b presents the face arm-a's cup was covering — the +y panel. This
   is the whole reason there are two arms and two checks.
5. **Place.** arm-b puts it in the good or bad tray according to the verdict.

The learner writes step 2 and 4's *judgement*. Everything else is the cell's job.

## Foundations to fix first

**F1. Station poses must be derived, not stored.** `probes/stations.json` is stale in three
independent ways: it encodes a tool-down grip (we grip the side now), an 80 mm cube (the
part is a 182×110×112 mailbox), and a cell with no scenery (there are 18 props). `arm-a
infeed` already fails IK because of it. It is the hand-copied-constants problem the earlier
review named, and the fix is the one it named: **one module that computes station poses
from the cell's geometry, which the fragment, the probes and the round all import**, with a
test that asserts the reach window and the handoff constraint still hold. Nothing else on
this list is trustworthy until this exists.

**F2. A visual gripper — done.** Landed before this review; visual-only, oriented along
the cup axis.

Originally specified as: The cup exists to the planner (a capsule in `world_state`) and to
physics (the attachment rig) but has no geometry, so the arm appears to pick things up with
nothing on the end of it. Visual-only prims under the flange: a body cylinder and a cup.
**Visual only** — collision is already handled, and a second collider on the tool would
fight the one the planner knows about.

## The round

**R1. Defects the cell can vary.** The part carries its possible defects as composite
children named `defect_*`; a world verb toggles each one's **visibility** at runtime, so a
round is repeatable with no respawn and no restart.

**At least one defect must sit on the +y panel** — the face arm-a's cup covers. Without it
the second check can never change a verdict, the two-arm story has no teeth, and a test
for "defective goes to the bad tray" passes on arm-a's check alone.

**Defect children must not collide.** Visibility in USD is a render attribute; PhysX keeps
the collider either way. An invisible "proud door" still protrudes, an invisible bent flag
still sits in arm-b's approach, and the suction raycast still hits both. `children` needs a
`collision: false` key, honoured where the collider is applied. Structural children still
collide — that distinction needs stating, because `qc-cell-plan.md` currently says
children carry no collision at all, and the code applies it to every one.

A dent also cannot be modelled by adding a child, which is a bump. Use a dark patch or a
small ding.

**R2. Ground truth, occlusion-aware.** Reading back the visibility flag the verb just wrote
is tautological: it says a defect exists, not that the camera could see it at any
presentation. Replicator's `bounding_box_2d_tight` annotator on `inspect-cam`, with
semantics on the defect children, answers the question actually being asked — and the same
mechanism answers "was every face presented". It needs no camera intrinsics, which also
retires that open finding. Set `extent` on every child or the tight boxes inflate.

A toggle followed immediately by a frame grab can return the pre-toggle render, so the verb
must not return until a render step has run.

**R3. The round driver.** `probes/round.py` — pick, present, handoff, present, place —
built on the motion service with the scenery and the held part in `world_state` throughout.
It stays a probe for now rather than becoming `qc:cell` verbs: the verbs are the course's
API and belong in the course module, and extracting them once the sequence is proven is
mechanical. Building them first would mean debugging the cell through an API that is also
being designed.

## Tests

The tests have to run the real thing. A round that passes in mock proves the wiring, not
the cell.

**T1. `test_cell_layout.py`** (no machine), written *with* F1 rather than after the round.
Every station inside the arm's reach with margin; the grip side clear of the belt; and the
handoff constraint asserted **at the flange, for every grip side the cell admits** — not at
the tray centre, which passes trivially. That distinction is not academic: with the cup on
the −y panel the good tray is about 940 mm from arm-a and therefore *reachable*. Only the
belt blocking the −y approach at the pick keeps the handoff mandatory, and the test has to
say which grip the layout relies on.

**T2. `test_round_e2e.py`** (needs the machine). One test per claim, so a failure names
itself:

* the part leaves the belt and stays held through the first presentation;
* the handoff transfers it — asserted on the **simulator**, not the grippers: after the
  set-down the part's pose tracks arm-b's flange and is independent of arm-a's;
* every face is presented to the camera at least once across the two checks, measured by
  the annotator rather than by face-normal arithmetic, which cannot see occlusion by the
  tool or the arm;
* the part ends **upright and at rest** inside the tray the verdict names — the trays are
  solid blocks with no walls, so a part knocked on its side is still "inside" in xy;
  orientation and two reads half a second apart are what make the claim mean something;
* a defective part ends in the bad tray and a clean one in the good tray — the same
  round run twice with different defects, because a cell that always says "bad" passes
  half of any weaker test.

**T3. Assert against the simulator, not against the cell's own claims.** Every check reads
`prop_poses`, not `is_holding_something`. The gripper reported a grip it did not have for
most of a day; a test that trusts it would have passed throughout.

**T4. Failure modes this project has actually hit, which the list above misses:**

* *swept off the belt before the pick* — the final approach drops the part from the
  obstacle set, so assert its pose is unchanged from spawn until `grab()` returns;
* *slip during a carry* — a z-only check misses rotation; assert the part's pose
  **relative to the flange** is constant across each carry;
* *the held part never entering `world_state`* — no probe has actually attached it as a
  `Transform` yet, despite the plan calling the mechanism proven. The round must build it
  and the test must refuse a carry without it;
* *a stale weld across a reset* — `reset` snaps the part back but does not open the cups,
  so a failed round leaves the next one starting with the part welded to arm-b. The
  fixture opens both grippers first;
* *`place_bad` timing out in sequence*, still unexplained from an earlier session, lands
  squarely on the place step.

Keep `grasp_reliability.py`'s *agreement* check as an assertion too — a gripper whose
claim stops matching reality should be named, not routed around silently.

**T5. Shape.** One test per claim over one sequential round would run five rounds and fail
all five identically on a bad pick. Run each round once in a session-scoped fixture that
returns a **trace** — poses, claims and defect state at every stage — and write the
assertions over the trace. The second defect set is a second fixture. Gate with a
`machine` marker skipped unless the API key is present; there is no pytest config or CI
workflow yet, so "runs in CI" is aspirational for T1 too until that exists.

## How this gets verified

* `probes/record_cell.py` after each milestone — the video has found what probes missed
  every time: floating arms, the cup on a curved crown, a part released in mid-air.
* A fable review of this plan, of the tests once written, and of the results once green.
* `probes/grasp_reliability.py` unchanged as the regression guard on the grip.

## Order

**F1 → R3 → T2 is the critical path.** T1 is F1's own test and is written with it. R1 and
R2 touch none of the motion work and can proceed alongside. F2 is done.

F1 also owns the camera: `fov_deg` and `target` were derived for an 80 mm cube and the
mailbox is 182 mm long, so both had to be re-derived. They since have been - 44 degrees,
aimed at the midpoint of the two presentation points - and `clipping_range_m` turned out
to matter more than either.

R3 is a package (`probes/qc/`), not another monolithic script: `stations.py` from F1 and a
`round.py` exposing `pick`, `present`, `handoff`, `place` that take their clients as
arguments, so the tests import it and the later extraction into `qc:cell` verbs really is
mechanical. `record_cell.py` is closures over module-level clients and nothing in it can be
imported.

Delete `.scratch/stations.json` when F1 lands — it is a byte-identical copy of
`probes/stations.json`, the hand-copied-constants problem committed twice.

## Known risks

| risk | what it looks like | |
|---|---|---|
| side grip leaves too few presentable faces | the two checks cannot cover six faces between them | count the faces in T2 rather than assume |
| handoff needs both arms in the same space | planner refuses, or they collide | it is the one moment both arms are close; expect to tune the handoff pose |
| defect marks too small for the camera | the oracle sees them, a detector never could | measure mark pixels at the model input as the old cell did |
| trays are 23 mm apart | a place into one clips the other | still unresolved from the earlier review |
| the part's planner box omits the flag | the flag reaches 30 mm past the declared half-width on the −y side — the side arm-b grips — and the carried-part transform inherits the same box | report the true bounds, or a box per child |


## What the handoff turned out to need

An earlier revision of this document declared the handoff blocked by its own geometry
and proposed redesigning it around an end grip. That was wrong, and the way it was wrong
is worth keeping.

The reasoning ran: both arms grip a ±y flat side, their bases are at y=∓380, so a shared
handoff point near y=0 puts each arm's flange 175 mm across the centreline on the far
side of the part. Measured, arm-a reached that position and stalled with the wrist about
160° from where it needed to be, at every roll offered it. The conclusion drawn was that
the position was reachable and the configuration was not.

Every measurement there was real. The conclusion was still wrong, because the same
symptom had a cause outside the geometry entirely - see below - and because the layout
never needed a shared point in the first place. Setting the part down **yawed half a
turn** swings the face arm-a is covering round to arm-a's own side and the free face
round to arm-b's, so each arm works entirely on its own side and neither crosses the
centreline. Both flanges sit 531 mm from their own base. The second check still sees the
face arm-a's cup covered, which is the whole point of the handoff.

Three things had to be right for it to run, and each had been wrong:

**The set-down must state where the PART goes, not where the flange goes.** `side_grip`
places the flange 25 mm below the part's centre, which is true of the way the part was
picked up and false once it has been turned: set down after a half turn the part hangs
*below* the flange. The arithmetic aimed 50 mm low and drove the part into the stand.
`flange_for` derives the flange from the hold `measure_hold` already takes, so the
handoff now reads as its own design - upright on the stand, yawed half a turn from the
way it came off the belt - and the trays get the same treatment for free.

**The orientation maths has to be the SDK's.** The conversions here were hand-rolled and
measured theta from the shortest arc taking +z onto the vector, which is not viam's
convention. They were exact inverses of each other and disagreed with the motion service
by a quarter or a half turn depending on the vector. That value declares the carried
part's box to the planner, so the planner had been clearing paths for a box the right
size and the wrong shape. `tests/test_orientation.py` asserts agreement with the SDK
rather than self-consistency, which is the distinction that hid it.

**The arm's drives were undamped.** This is the one that cost the most time. The UR5e
asset's authored gains are stiffness around 5e5 on every joint against damping
[21.7, 23.6, 234.5, 90.4, 3.5, 0.2] - four orders short of critical, and 0.2 on the
wrist roll. Drives that stiff and that undamped do not settle, they ring, and a joint
still ringing when its tolerance is checked cannot be told apart from a joint held by a
collision. Every "the wrist stalls short" in this project's notes was that. The claim
that "a loaded arm has far less wrist authority than an empty one" was inferred from it
and is not supported: a held part rolls 180° in 30° steps with zero error.

`drive_damping_ratio` sets damping as a fraction of each joint's own stiffness. At 0.01
an empty-arm park that had failed identically across four restarts succeeds first time.

The diagnostic worth remembering: the three wrist joints, which carry almost nothing,
were hitting their targets exact to a tenth of a degree while the gravity-loaded elbow
sat 34° below its. Distal joints exact and loaded joints wrong is a drive symptom. No
arrangement of props explains it, and I spent hours looking for one.

## Where the round stands

| step | state |
|---|---|
| pick by a flat side | works |
| lift clear of the belt | works, in joint space |
| first check | works, 5 of 5 faces |
| handoff by set-down, yawed half a turn | works |
| second check | works, 5 of 5 faces |
| place in either tray | works, upright and at rest |

The place took three goes and each one taught the same lesson in a new place.

The trays sat 23 mm apart, so placing in the good one put arm-b's wrist inside the bad
one and the planner refused every IK solution. The earlier review flagged that spacing
and it stayed open because a test written at the tray centre passes.

Separating them in y then failed differently: the bad tray's *approach* - 140 mm above a
place that was itself in reach - produced no IK solutions at all, and the reach test had
only checked the destination.

And moving it closer still failed at 837 mm, inside both the 915 mm budget and the
1016 mm positional bound, because out at y=1005 the tool has to point back along -y from
625 mm beyond arm-b's base and that orientation is not achievable there. A spherical
reach bound is honest about position and silent about orientation, which `layout.py`
has always said in a comment and no test enforced.

The trays now sit side by side in **x** at one y, so both places use the configuration
the cell has already demonstrated and the wrist - which always stands off in +y - is
clear of the other tray whatever x it is at. The tests assert the wrist box against the
other tray in every axis, the reach at the approach as well as the place, and that the
two trays share a y at all.

## The cell is taught, not planned

This is the change that made the motion consistent, and it should have been the design
from the start.

A QC cell runs one fixed sequence over and over. Asking the motion service to plan that
sequence again on every cycle asks a randomised algorithm for a fresh answer each time,
and it gives one: RDK seeds IK from wherever the arm happens to be and scores candidate
solutions by joint distance, so the same station is reached through a different arm
configuration on different runs. That is what "the movements are wonky" was. It is also
why the marginal poses failed on some cycles and not others, and why a week of
diagnosing individual stalls kept finding a different link fouling a different thing:
each layout change made the planner pick a different branch, so a clearance fitted to
one run was wrong for the next.

So the round is planned **once**, with every obstacle declared, and the joint paths the
planner produced are kept and replayed. `probes/teach_round.py` does the teaching and
writes `fragments/qc-cell-program.json`; everything else replays it.

Measured: five consecutive replays put the part within a millimetre of the same place,
with the same verdict, where planned runs had been failing about one cycle in two.

Three things make it work:

**Replay tracks the path; it does not step to it.** The old executor drove to each
waypoint and waited for the arm to settle within half a degree before issuing the next,
so the drive was always chasing a target a long way off. At the extended poses the arm
overshot and hunted - `moving=True` at up to 2.7 deg/s, a degree or two short,
indefinitely. Replay issues a target every 20 ms a step ahead of where the arm is, so
there is no large error to overshoot on, and only the final pose has to settle. The
planned path is executed the same way, which is why teaching works at all.

**The drive gains are tuned rather than inherited.** The asset ships stiffness around
5e5 against damping of 21, which does not settle. Setting damping to a fraction of that
stiffness - what this project did first - took it to 5741, and damping torque alone then
consumed the entire torque budget at a few degrees per second, so the arm crept and
stalled short. The masses are not the problem: measured, the seven links total 21.0 kg
against a real ur5e's 20.6, and the part is 0.45 kg. What the cell uses is stiffness
20000 and damping 300, which measures out at a holding error of 0.10 degrees at every
pose in the handoff descent, with the shoulder using 31-37 N.m.

The `drive_max_effort: 300` in the fragment is a request that does not take. Measured:
`set_max_efforts` accepts it without raising and the limits still read the asset's
authored [150, 150, 150, 28, 28, 28] afterwards, so every gain decision in this project
- including the damping-5741 overcorrection - was made against a ceiling that had never
moved, and the exception that would have said so went to a log this cell sends to
/dev/null. `arm.do_command({"command": "efforts"})` now reports the request, whether
anything accepted it, the measured torque, and the limit as read from both the
articulation and its controller, which disagree. None of it is the binding constraint at
37 N.m of 150, but tuning blind was.

**Park has to be the shape the arm works in, not a place in front of it.** The earlier
diagnosis here was a pan wrap, and it was wrong: pan at -155 and pan at +20 are not two
representatives of one solution, they are two different arm configurations, and the
route between them is real work rather than an unwinding. Measured against the live
cell, one pose at a time - arm-a's pick, its pick approach, and both handoff grips have
NO solution with the arm facing the station, only with pan swung to -155 and the
shoulder past -130, while every presentation pose solves either way. So the working half
of the round lives on one branch.

Park was on the other, and it cost three crossings a part, not one. 185 degrees getting
onto the working branch at the start and 217 coming off at the end - and, because the
forward-facing park led the planner into the pick with the elbow the other way up, and
from THAT shape the part's free side has no solution at all, another 270 flipping the
elbow back mid-presentation. The elbow flip was never forced: seeded from the handoff
shape the pick approach solves with the elbow down. Park is now the working shape with
the shoulder lifted 30 degrees, which measures out at a tool tip of 769 mm for arm-a and
870 for arm-b, clear of the belt at 584 and both outfeed decks at 400. `probes/
branch_check.py`, `probes/home_branch.py` and `probes/elbow_branch.py` are how the next
layout change gets judged; none of this was visible from the program alone.

**The wrist clamps are a trade, and each one was tested on its own.** `joint_limit_deg`
narrows every revolute joint to +-180 so that one pose has one solution, and that clamp is
also what forces the planner to return the FAR wrist solution when the near one falls
outside it. Measured per joint, widening one at a time:

  wrist_1 to +-270   breaks the free-face presentation
  wrist_2 to +-270   009:arm-a falls from 307 degrees to 192, nothing breaks - KEPT
  wrist_3 to +-270   breaks the pick, the one move planned under a linear constraint

Only one of the three was safe, and widening them together - which is what was tried
first - shows none of that: both runs failed the free-face check and the whole change got
reverted as "breaks it", with the joint that actually helped thrown out alongside.

So `017:arm-b` still takes a quarter turn as 270 degrees of wrist_3, and that is now a
priced decision rather than an unexamined default: 270 degrees once a round against a pick
that fails half the time. `probes/free_face_check.py` is how a pose's refusal gets read
directly instead of inferred from a missed presentation.

**Some poses cannot be held at all.** The same probe found arm-b's free-face pose reached
from park with the shoulder pinned at 150 of 150 N.m, three degrees short - a drive holding
a configuration it cannot quite reach. Asked from the last roll, which is where the round
actually asks from, it picks a different solution and arrives. So more wrist freedom is not
free: it lets the planner choose poses the arm has no torque for, and the effort limit
cannot be raised because `set_max_efforts` does not take.

**Each step is budgeted.** The wrap check that preceded this could only look at strides
inside a dense path, and the planner returns most moves as their two endpoints - 26 of
the round's 30 steps - so it had nothing to look at on the majority of the cell. What
replaced it budgets the whole step: a quarter turn of wrist plus a degree per five
millimetres the tool is carried, calibrated against the steps known to be sane. It fails
a 140 mm straight descent that spends 197 degrees of elbow and pan, which is the
complaint the user made about this cell in almost those words, and which passed every
check the cell had. `probes/program_report.py` prints the same table the test asserts.

**Steps are numbered, and the numbering is the contract.** Replay walks the round's
moves in order and looks each one up by number, so a round that makes a different
sequence of moves than the teaching run gets an error rather than the wrong path at the
wrong moment. The round is one sequence until the verdict and two after it, because the
part goes to a different conveyor, so steps after the decision are keyed by what was
decided and one program holds both endings.

Replay does no planning and no collision checking. That is safe for the reason a taught
robot program is safe - the path was validated against the full scene when it was taught
- and it is why `teach_round.py` is the only thing that plans, and why re-teaching is
the required step after the cell changes. The one motion that is NOT taught is park:
`home()` commands it as joints, so no planner ever checked the way in. It is safe on
narrower grounds - park is 30 degrees of one joint from a pose the planner accepted,
along an arc that lifts the tool away from everything - and those grounds have to be
rechecked if park moves. `tests/test_taught_program.py` checks the
program still fits the round without needing a machine.

## The part is the mailbox asset

The cell's part is the Skinnable mailbox USD, scaled to 0.3677 so it is 182.0 x 63.3 x
83.4 mm - the length the layout was already built around - and yawed -90 so its flat back
cap faces arm-a and its flag faces the camera. It handles BETTER than the cube it
replaced: 1936 degrees of joint travel against the cube's best 2033, because a 63 mm part
gripped on its back needs less reaching than a 110 mm one gripped across its side.

Nothing about the asset is written down. `probes/derive_mailbox.py` measures it in a
running scene and writes `fragments/mailbox-derived.json`: the scaled dimensions, the
offset from the prim's origin to the geometry's centre, where the cup should land, which
face each decal is on and how big it is, and the patch of wall a cup can seal on. It also
generates the obstacle bounds in the fragment, because the offline tests have no stage to
measure from. A cube stated its size in config; an asset states it in geometry, and the
honest way to consume that is to read it.

**Two things about gripping an ASSET rather than a box.** Both cost most of a day.

A suction cup cannot grip a link of somebody else's ARTICULATION. The gripper grips by
making a D6 joint to the rigid body its raycast hits, and physx will not join a link of an
articulation it is not part of - and does not say so, it just leaves the gripper Open,
which is what a cup that found nothing looks like. Measured by holding the rig, the pose,
the clearance and the probe fixed and changing only the part: a plain rigid body 3/3, the
mailbox as authored 0/4, the mailbox with `articulation_root: false` 4/4. The door still
hinges afterwards; it is an ordinary jointed pair rather than an articulation.

And a part's OUTER BOUND is not the surface a cup can seal on. This asset's +y bound is
its flag, 8.5 mm proud of the wall, so a standoff computed from the bound aims short of
the wall everywhere the flag is not and cannot seal on the flag where it is. The seal
patch is derived by measuring the hull against the flag and the rails, not read off the
asset's notes. A cube hid this for the cell's whole life, because a box's side is all seal.

**Choosing a defect and blinking one are different weights of operation.** Which mark the
part carries is a `paint_defect` variant, and switching a variant RECOMPOSES the prim,
which briefly invalidates the physics views - the arms then fail reading their own joint
velocities and recover immediately, leaving nothing behind. So the variant is chosen once
before the round moves, and the blink that proves the camera can see the mark is a
visibility change (`show_defects`), which is what the cube always did.

**What the swap was worth.** Six faults that a box had been hiding, every one of them live
before the asset arrived: `part_pose` reported the prim's origin where its callers wanted
the geometry's centre; the round carried on after `grab()` returned False, presenting five
faces of a part still on the belt; `prop_obstacles` silently skipped a usd prop with no
declared bounds, so the part vanished from the planner's view; `grasp_reliability` scored a
top-down grasp the round never makes; `side_grip` stood the cup flush against the face, so
the gripper's own raycast started inside the part; and the module's bounding-box cache
omitted `guide` purpose, so collision geometry read as absent.

## The door is a second KIND of check

The asset carries a mechanical fault as well as a painted one: `door_condition` overrides
the hinge's stiffness, damping and upper limit, so the defect is not something a camera
sees on a still part - it is how the door behaves when it is made to move. Tilting the
carried part drives it to exactly the limits the asset authors:

    condition  expected   at rest   tilted   +1.5s
    ok            True       0.0      2.6     2.6    holds shut
    stuck        False       0.0      2.6     2.6    holds shut
    binding      False       0.0     35.0    35.0    at its 35 degree stop
    loose        False       0.0    100.0   100.0    at its 100 degree limit

35.0 and 100.0 are the authored `physics:upperLimit` values, so the hinge works - and
stripping the articulation root, which the suction cup needed, did not break it.

**What a tilt can and cannot tell.** `ok` and `stuck` both read 2.6 degrees, and that is
correct behaviour rather than a failed measurement: a healthy door is a self-closing spring
hinge that holds shut under gravity. Telling those two apart needs the ~5 N pull on the
handle that the asset's own notes use, on a handle a few millimetres across at this scale.
So the tilt detects a door that WILL NOT HOLD SHUT - loose and binding, the faults that
matter for a mailbox in transit - and not one that will not open.

For the same reason nothing springs back within 1.5 s: `ok` never opened, so there was
nothing to return. The rate half of the measurement - which is what separates `ok` from
`loose`, since they differ only in whether the door comes back - only means anything once
the door has been opened against its spring. That is the handle pull again, and it is
separate work rather than something to fold in quietly.

The angle is read as the Door's rotation RELATIVE to the Body, so it does not matter which
way the arm is holding the part. Normalise the quaternion's sign first: a quaternion and
its negation are the same rotation and the backend returns either, which reads a shut door
as -360 degrees and a door on its 35 degree stop as -325.

## Still to do

The motion is finished and the round reaches its own verdict. What is not finished is
the *seat the learner sits in*, and a review found that the tests said otherwise in
several places. In priority order:

**1. The judge's inputs, now fixed, still need the loop around them.** `judge` takes
`List[Shot]` - stage, the face of the part towards the lens, the frame path - and is
`async`, so a learner can call a vision service from inside it. What is missing is an
offline iteration loop: the judge runs after all the motion, so every threshold tweak
costs a full round. Persist the trace as JSON beside the frames and expose
`judge_offline(trace_path)`, so the verdict tests can run over a saved round. Nothing
records round duration either, and a two-hour course wants to know.

**2. R2, occlusion-aware ground truth.** Still the right long-term answer, and still not
the next thing. The round now answers the narrower question - *could the camera see this
mark at all* - by photographing one presentation with the mark switched on and off and
comparing pixels, which needs no intrinsics and no Replicator. What that does not give
is a per-frame box, which is what a detector is scored against.

**3. The oracle is a stand-in and the tests know it.** `make_oracle` closes over the
simulator's visibility map. It establishes that a mark was switched on and that some
presentation turned its face towards the lens; it establishes nothing about pixels
except through the blink test. Any test asserting `found` or `presented` is testing the
oracle, not the cell, and should not be inherited by a learner's judge.

**4. Mock mode cannot run a round**, because `prop_poses` returns spawn positions there.
So there is no dry run without Isaac. Probably right to leave, but the course should say
so rather than let someone discover it.

## What the review caught that the tests did not

Worth keeping, because the pattern repeats: **a test that cannot fail is worse than no
test**, and this project has now shipped five.

* A yaw test that compared an arm to itself.
* An obstacle probe that counted "did not reach" as "the planner knew".
* A two-check test over world-axis labels, which are identical whether or not the
  handoff turns the part.
* Two layout tests that were arithmetic identities on `side_grip` - true for any cell,
  passing whether or not the handoff turned anything.
* A pixel test, added *in response to the review*, that passed on 446 scattered pixels
  of jpeg noise while both frames still contained the mark.

The last one is the instructive one. The check was right in principle and wrong in
execution, and it went green, and the only reason it was caught is that the number was
looked at instead of the colour of the dot. When a test asserts a threshold, print what
it actually measured at least once.

Three more, of a different kind - claims written as measurements:

* `apply_action` "not landing", refuted by the next run.
* CPU contention from another job, refuted once that job was gone.
* "A loaded arm has far less wrist authority", which survived in four docstrings after
  being retracted here, and was really the undamped drives.
