# Plan: finish the full-scale QC cell

The cell must run a **full-scale** Skinnable mailbox (495 x 172 x 227 mm, 2.95 kg) through a
realistic sequence on realistic scenery, at a believable speed. This is the ordered plan to
get there, with an acceptance test per task so "done" is measured rather than asserted.

## Where it stands now (measured, not assumed)

Working:
- The part is the real asset at scale 1.0 with its own masses (Body 2.5 kg, Door 0.45).
- In free space it falls straight down - no hinge explosion, no constraint problem.
  (`FREE t+0.2s [1600,-1600,1473] -> t+0.6s [1600,-1600,862]`, x and y hold.)
- The `-x` side is the right grip face on both halves of the question: 86 mm standoff, 481 mm
  of clear wall, nothing proud of it, and the arm REACHES it at approach and grip. The ends
  need 229-252 mm and cantilever the full 500 mm; `+x` has no IK at all.
- A suction cup cannot grip a link of a foreign ARTICULATION - 0/4 with the root, 4/4 with
  `articulation_root: false`, same pose and clearance. The root must stay stripped.
- Defects are the asset's `paint_defect` variants, with `mailbox:expectedPaintPass` as truth.
  Choosing one is a variant change (before the round moves); blinking one for the camera is a
  visibility change (`show_defects`), because a variant switch recomposes the prim and
  invalidates the physics views mid-round.
- The door hinge works: tilting drives it to its authored limits, `binding` 35.0, `loose`
  100.0.

Broken or unfinished:
- ~~The part will not stay on the infeed.~~ **Fixed, and the slab is gone.** The infeed is
  ConveyorBelt_A05 from the nucleus, and the part rests on it at 0.1 mm drift and 0.000 tilt
  across three resets. The deck is the asset's own roller bed at 769.3 mm - the 30-inch
  industry standard - where the slab was an invented 584. The rollers turned out not to be
  the ribbed hazard they looked like: a 495 mm mailbox spans about eight of them and sits
  dead flat.
- Ten layout tests fail on full-scale geometry: camera framing (part's edge 501 mm off axis
  against a 466 mm half-frame), presentation reach (39 deg against a 30 bound), station
  heights, the centre table, the outfeeds.
- Motion is capped at 20 deg/s because damping 300 at 30 deg/s demands 194 N.m of a 150 N.m
  joint, and `set_max_efforts` silently does not raise the ceiling.
- A tilt-based door check cannot separate `ok` from `stuck` - a healthy spring hinge holds
  shut under gravity by design. It needs an arm to actuate the door.
- The taught program is for the old small part and is invalid.

## What the user asked for

1. Mailbox at full scale (1.0).
2. The door opened realistically - not by tilting the part, which would damage it or pull it
   off the cup.
3. No cantilevered vacuum grip.
4. The part not passing so close to the conveyor (seen at 0:21 in the last video).
5. Faster arm motion.
6. The centre table becomes a FIXTURE. arm-a places the mailbox and STAYS ATTACHED while
   arm-b opens and closes the door. Then arm-a releases, arm-b regrasps, and arm-b places it
   on the correct conveyor.
7. Conveyors replaced with a real conveyor asset, and a BREAK BEAM sensor that stops the
   mailbox in the right spot for pickup.

## The round, as it will run

Written out because T4 onwards IS this sequence, and because the middle of it is the
part the user changed: the centre table becomes a fixture, and the part stays held while
the other arm works on it.

 1. **Reset.** Arms home, cups open, part re-staged upstream on the infeed, belt stopped,
    defect variants chosen. Variants are set BEFORE anything moves - a variant switch
    recomposes the prim and invalidates the physics views.
 2. **Feed.** The belt runs; the break beam at y -711 stops the part with its origin at
    y -952, repeatable to 0.1 mm. Nothing may pick before this returns.
 3. **Pick.** arm-a takes the part by its `-x` side face - 86 mm standoff, 481 mm of
    clear wall - and lifts it clear of the guide rails.
 4. **Present.** arm-a shows the faces the camera has to see. The learner's perception
    runs on these frames.
 5. **Seat.** arm-a places the part into the FIXTURE and KEEPS HOLDING it. The fixture
    locates it; arm-a is what holds it down.
 6. **Hand over.** arm-a releases and retracts. arm-b grips the part - on a SIDE face,
    not an end - and lifts it out of the fixture.
 7. **Place.** arm-b puts the part on the outfeed the verdict chooses.

The door is OUT of the round for now, at the user's call. The check that reads it passes
and is kept as `probes/door_release.py`; what is removed is an arm opening and closing it,
and the hook that was built for that is off the tool.

## Tasks, in dependency order

Each task states its acceptance test. A task is not done until its test passes on the live
cell. After each, the advisor reviews progress and names the next step.

### T1 - Infeed conveyor that actually holds a mailbox
Replace the flat `belt` slab with a composite: deck, two side guides, an end stop, on its
stand, sized for a 495 x 172 mm part with clearance either side. Guides are what a real
conveyor has and what a slab lacks.

**Accept:** the part spawns and settles within 5 mm of its configured pose, with tilt under
0.02, and stays there across three consecutive resets. A downward raycast confirms the deck
height the layout computes.

**PASSED** on ConveyorBelt_A05 - not on the slab, which is deleted. Drift 0.1 mm, tilt
0.000, unchanged over three resets; a downward raycast reads the deck at 769.3, which is
what the layout computes. Two things had to be true first:

- The conveyor's first boot came to rest tilted 18 degrees, its deck sloping from 207 mm
  to 759 mm end to end. I wrote `_make_static` to freeze the rigid bodies an asset ships
  with, and the next boot was clean - so I recorded that as the cause. It was not.
  Reporting `_make_static`'s findings through `status` showed **count 0**: A05 ships no
  rigid bodies at all, and freezing nothing cannot fix anything. The remaining difference
  between the two boots is that the first streamed 13 MB of asset from S3 while physics
  was starting. The asset is now mirrored locally, which removes the variable rather than
  explaining it - said plainly because an unexplained fault that stopped happening is not
  a fixed fault. `_make_static` is kept: it is a correct reading of `fixed`, and it is the
  thing that reported the truth.
- One box per usd prop would have swallowed the deck: a conveyor measured whole runs from
  the floor to its post tops, putting the resting part and the grip pose both *inside* the
  obstacle. `bounds_m` now takes a LIST - base, deck, two rails, two posts - so the space
  above the deck is open to plan through.

The 769 deck was not assumed to be reachable. `probes/pick_window.py` swept the part down
the centreline and asked the motion service at the approach and at the grip. That sweep was
run TWICE and only the second one counts: the first ran before the conveyor was in the
scene, so it measured the arm against the slab it was replacing and reported the window as
y -1000..-400. Live, with the conveyor as an obstacle and the overhang constraint applied:

       pick y   reach  on deck   approach  grip     blocked by
        -1000     861  yes       refused   -        belt.deck, belt.wall_l
         -980     846  yes       ok        ok       <== USABLE
         -940     819  yes       ok        ok       <== USABLE   (chosen)
         -900     792  yes       ok        ok       <== USABLE
         -890     785  OVERHANGS ok        ok

Two constraints close from opposite sides. Below -890 the part's leading edge passes the
last roller, and a beam cannot stop a part off the end of the deck. Upstream of -990 the
arm's upper link hits the deck and the near wall.

The first version of this sweep reported a 50 mm window, and it was wrong for a reason
worth keeping: it was blocked by `belt.base` - a box this repo INVENTED, floor to deck,
full length, on the reasoning that "the arm never works below the deck". That reasoning is
false here, since arm-a's shoulder sits about 215 mm below the deck, and the asset has no
collider there at all. Modelling what the asset actually has - rollers, two side walls,
two legs at the upstream end - widened the window to 80 mm. The planner names the geometry
it hits, and truncating that name to 28 characters hid this for a cycle.

**The beam stops the part at y = -940**, the middle of the window: reach 819 mm of 1016.7,
and the leading edge 100 mm inside the end of the deck, which is the room the stop has to
overshoot into.

### Conveyor asset: found, not substituted

The user asked for "a real conveyor usd". There is one - 47 of them, on the nucleus at
`.../Assets/Isaac/6.1/Isaac/Props/Conveyors/ConveyorBelt_A{01..49}.usd`, found by listing
the asset bucket over plain HTTPS (`omni.client` needs no simulation app, and the bucket
needs no `omni` at all). Each ships a 256x256 thumbnail, which is how the shortlist was
picked without booting isaac while the cell was up.

The set is a kit: curves, T-junctions, angled feeds, and a handful of straight standalone
units. Judged by eye, the straight ones that come complete with legs are A05, A08, A09,
A20, A29 and A49. Of those:

- **A08** - long straight roller conveyor, own legs, blue side frames standing PROUD of the
  roller deck, plus a control box. The proud side frames are guide rails, which is exactly
  what the slab lacked in T1, and the length is travel distance for the beam to stop the
  part in. First choice for the infeed.
- **A05** - the same unit, shorter. First choice for the two outfeeds.
- **A09 / A29** - flat belt decks, kinder to a part that rests on rails, but NO side guides.
  The fallback if the rollers turn out to be ribbed collision geometry the mailbox rocks on.

Unmeasured and load-bearing: whether the rollers are individual colliders (a ribbed surface
of point contacts under a mailbox that already rocks) or one convex deck, the deck height,
and the footprint. That is the first measurement of T2, not a guess to design around.

### T2 - Break beam that stops the part at the pick point
A `sensor` model in the module reporting the beam's state (clear/blocked) from a raycast
across the deck, plus conveyor motion that drives the part along the deck and stops it when
the beam breaks. The beam's position DEFINES the pick point.

**Accept:** the sensor reads clear with no part and blocked with one; the part released
up-stream travels and halts with its leading edge at the beam, within 10 mm, three times
running - and its x and yaw at the stop are held too, since the asset's own walls are
906 mm apart and hold neither.

**PASSED**, on a FLAT BELT (ConveyorBelt_A06), not the roller unit:

      leading edge spread 0.0 mm      yaw spread 0.00 deg      x spread 0.0 mm
      1149 mm travelled, beam tripped at 3.5 s
      beam clear with the part upstream: True
      beam blocked with the part across it: True
      and 6 feeds out of 6 from a full reset, landing within 0.1 mm each time

The roller conveyor had to go, and the reason is in the PART, not the conveyor. The
mailbox stands on `Rails_interior` - rails 7 mm proud of its underside, 430 mm long,
running along its length. A05's rollers sit on a 68.5 mm pitch with 10.6 mm of air
between them, so a rail drops into a gap and wedges. That is a part-on-rollers mismatch,
not something to tune:

      channel 194, 300 mm/s, default friction      2 of 6 feeds
      channel 194, 300 mm/s, UHMW guides           3 of 6
      channel 210, 300 mm/s, UHMW guides           5 of 6   (best; yaw -4.8..+0.4)
      channel 230, 300 mm/s, UHMW guides           5 of 6   (yaw -7.5..+4.5)
      channel 210, 150 mm/s, UHMW guides           3 of 6
      A06 flat belt                                6 of 6   (yaw 0.00, x identical)

Reversing to clear a jam did not help: the part re-wedged in the same place three times
running. A continuous surface has nowhere for a rail to fall, and a uniform surface
velocity produces no differential contact across the part, so nothing yaws it.

A06 is an elevated unit - its belt is authored at 1780 mm - so it is placed 1011 mm down
and its legs pass below the floor. The deck still lands on the 769.3 line the whole cell
is built around, so nothing else moved.

The part stops with its origin at y -952, inside the -980..-900 pick window. The stop is
taken inside the sim step that sees the beam break, not over gRPC: at 290 mm/s a round
trip alone would exceed the 10 mm acceptance.

Four things had to be got right, and three of them were wrong first:

- **The asset's walls do not guide anything.** 906 mm apart for a 172 mm part: the first
  run yawed 269 degrees and missed the beam entirely. Guide rails set to the product
  width are what a real line has.
- **A long narrow channel wedges.** A 495 mm part jams across a 186 mm channel at 1.6
  degrees of yaw, and the more channel it has the more chance it has to find that angle.
  Guides that stop short are worse still: the nose comes free, swings, and catches the
  end face. One continuous 194 mm channel that runs PAST the stop - so the nose never
  leaves it - is what holds.
- **The beam looked straight through a guide rail.** At 30 mm above the deck it was
  inside the rails' own 40 mm height, so it read blocked before the belt started and the
  conveyor stopped instantly. Three identical non-events scored a spread of 0.0 mm on
  every axis and the test reported PASS. It now refuses to pass a run in which the part
  did not travel.
- **`reset` does not re-stage a usd prop**, because a referenced asset is not in
  `world.scene`. Run 2 once began from where run 1 stalled, turning three independent
  runs into one drifting sequence. `place_prop` does it - and has to move EVERY rigid
  body under the prop as one, since writing the pose to the mailbox's shell alone
  stretches its door hinge across the cell and throws the part 20 metres.

### T3 - Pick from the stop point
Re-derive the pick pose from where the beam stops the part. Grip the `-x` face.

**Accept:** `probes/grasp_reliability.py` reports 3/3 carried, with the part rising the
commanded lift and surviving a sideways carry.

**PASSED, 5/5** - carried 5 of 5, claimed 5 of 5, disagreed 0 of 5, the part rising
249.6 mm and holding through a 150 mm sideways carry.

Two corrections were needed and neither was in the cell:

- The grip was built from `cell.pick()`, the nominal layout pose. On the roller belt the
  part's lateral rest varied by 12 mm and the cup stands off 6 mm, so a rail-hugging stop
  would have driven the cup INSIDE the part. It grips the measured pose now.
- The probe's own carry test compared the part's x against `gx - 120`, which is where the
  cup would be if the tool pointed -x from the flange. On the -x FACE the cup points the
  other way, at `gx + 120`, so the test looked for the part 330 mm from where a good
  carry leaves it and reported 0/5 "dropped in the carry" while the part was being
  carried the whole way. It measures the part's own displacement against the arm's now.

The lift also had to clear the guide rails, which stand 40 mm proud of the deck BETWEEN
arm-a and the part: a 20 mm rise left the part's underside below the rail and the first
move towards the arm would have dragged it across. Derived from the cell rather than
stated: 65 mm.

### T4 - Fixture in place of the centre table
A nest that locates the mailbox: deck plus two locating walls, at a height both arms can work
at. arm-a places the part into it and KEEPS holding.

**Accept:** the part sits in the fixture, the holding arm still attached
(`is_holding_something` true and the part's pose following it), part tilt under 0.02.

**PASSED**: seated at [512.2, 1.0, 713.5] against [520, 0, 713.5] - 7.8 mm - tilt 0.000,
arm-b holding. Three things were decided by measurement, and two of them overruled what
the request literally said.

**The arms' jobs are swapped, and the cell's geometry is what decided it.** The door is
at one END of the part, and the part's yaw is not free: `-x` is the only grippable face
(480 mm clear, nothing proud) because `+x` carries the flag AND both paint defects, and
neither arm has IK for `+x` anyway - both bases are at x=0, so gripping it means reaching
past the part and pointing the tool back at yourself. With the clean face on `-x` the
door necessarily faces `-y`, which is arm-a's side.

`fixture_window.py` found one pose, y 350, where arm-a could hold AND arm-b could reach
that door - the assignment as asked. It was retested there after the carried-box bug was
fixed, because the first attempt's failure was that bug. It still fails, for a different
and simpler reason: with the fixture at y 350 **arm-b cannot hold its own home pose**,
both shoulder joints pinned at 150 N.m (`j0: at -123.5 want -108.7, j1: at -159.5 want
-162.9`). The fixture is 30 mm from arm-b's base and its links have nowhere to rest. So
the fixture sits at y 0, where both arms are comfortable (arm-a 594 mm, arm-b 579), and there arm-b has
no IK for the door at all. **arm-a seats the part and lets go; arm-b takes it by the face
arm-a vacates and holds it; arm-a - whose side the door is on - opens and closes it.**
Functionally what was asked for, with the arm names the other way round.

**The nest has one wall, not two.** A wall on `+x` refused the seat outright - a 100%
violation between the carried part and the wall even at 35 mm of clearance - and `+x` is
the face the camera has to see anyway. The `+y` end wall stays. The locating clearance is
35 mm rather than 10: 10 mm sits inside the planner's own collision margin.

**The cup cannot hold the part through the carry on its derived limit.** See below.

### The vacuum: the cup is fine, and I was wrong about it

**Retracted.** An earlier version of this plan said the cup could not hold the part
through the carry on its derived limit, and that the cell needed `shear_force_limit:
2000`. That was wrong, and the mechanism it described was invented to fit one run each
way.

The real fault was in what the planner was TOLD the arm was carrying. `measure_hold`
declared the carried box with `cell.part.dims_mm` - a WORLD-aligned extent measured with
the part at its -90 yaw - as though those were the box's own dimensions, and then
oriented the box by the part's orientation, which carries that same yaw. The rotation was
applied twice: the planner believed a 495 mm mailbox lay ACROSS the cell rather than
along it.

With the box declared in the part's own frame, on the cup's DERIVED limit and no
override, three runs:

    tilt 0.004 / 0.001 / 0.000 deg      arm-b holding: True, True, True
    seated within 17-25 mm of nominal, inside the nest's 35 mm locating clearance

The part does shift slightly in the cup over the 950 mm carry - the offset moves about
34 mm along the cup's own x - but it holds, stays flat, and hands over cleanly. The
override is deleted.

The lesson is the one already on the list and not applied: a probe must travel the
round's code path, and a measurement of the wrong thing is worse than none. Two of T4's
three conclusions rested on this and had to be re-measured.

### T5 - the door check  (PASSED, then taken OUT of the round)

Removed from the round at the user's call, to get the rest of the cell finished. What was
learned stands, and the check still passes on demand:

      condition   at rest   staged    +0.5s    +2.0s    +4.0s   reading
      ok             0.0     76.5     29.9      0.0      0.0    closes
      stuck          0.0      5.0      0.7      0.0      0.0    will not open
      binding        0.0     35.0     35.0     35.0     35.0    jams part-open
      loose          0.0     86.0    100.0    100.0    100.0    hangs open

Four distinct readings for four conditions, with no arm force: the door is STAGED open
(`set_door`) and watched. The asset separates itself once released, which is what its
author built those four drives to do - `ok` makes 1.50 N.m of spring against gravity's
0.47, `stuck` has a 3 degree limit, `binding` has 83x the damping and stops at 35, and
`loose` has no spring at all.

`probes/door_release.py` runs it and `probes/record_release.py` films it. The hook is off
arm-a's tool; `_author_tool_hook` and `probes/door_jog.py` stay in the tree because the
joint-space jog is general - it holds a tool's orientation to 0.1 degrees where a pose
cannot - and will be wanted again.

### T6 - Release, regrasp, place
arm-a releases; arm-b grips the part and lifts it out of the fixture; arm-b places it on the
conveyor the verdict chooses.

**Accept:** a full round reaches `at_rest` on the correct conveyor, part upright, its
underside on the deck, across both endings.

**PASSED on the clean ending** (the defective ending is still to run). First complete round,
35 stages, planning every move:

    verdict   good, presented ['+x','+y','+z','-x','-y','-z'], missed []
    placed    [1199, 330, 462] against [1200, 330, 463.5]
    at rest   [1201, 329, 444] then [1201.2, 329.2, 443.5] a moment later

Four faults stood between the carry working and this, and all four were the same shape - a
number that was true of an older grip, still being used for the current one:

1. **The presentations' standoff** was half the part's LENGTH (367 mm), the end grip the
   cell abandoned, against the side grip's real 206.
2. **The fixture's position** was chosen against `fixture_grip`, a pose the round never
   visits, because `stations()` reported it as both the give and the take.
3. **Both arms used arm-a's standoff**, though arm-b holds the part by its end at 377 mm,
   which put arm-b's presentations 171 mm out over the infeed.
4. **The place's nominal hold** was a 175 mm side grip, so every place number was wrong -
   and wrong in the direction that mattered, because at 377 the flange sits UPSTREAM of the
   part and what has to clear the fixture is the part's far end, 624 mm behind the flange.

The lesson worth keeping is the one `stations()` was built for and did not finish: a pose
the round does not visit will pass every test and fail every run. Each of these four was
caught by running the round, not by the 146 offline tests.

**And the handoff's half turn is geometrically impossible here**, which no amount of
positioning could fix. arm-a holds the part by the face pointing world -x, so turning it
half a turn swings that face - and arm-a's flange with it - to the far side, leaving arm-a
reaching to x 668 with its tool pointing back at itself. Swept the approach height: no IK at
+160, +100, +60 or 0 mm. Unturned, all four solve. `fixture_window.py` had written the
reason in its own docstring long before: "neither arm has IK for +x; both bases sit at x=0".

### T7 - Outfeed conveyors sized for the part
Real conveyors, wide enough that a 495 mm mailbox rests fully on the deck rather than
overhanging, and placed so the carried part clears them on the way past.

**Accept:** the part rests fully within the deck's footprint; no carried pose comes within
50 mm of an outfeed, asserted as a layout test (this is the 0:21 complaint).

### T8 - Faster motion
Raise the arms' authored drive `maxForce` in the USD so the commanded speed is trackable,
then raise `joint_speed_deg_s`. The ceiling must be raised where it is actually read, since
`set_max_efforts` does not take.

**Accept:** `efforts` reports the new limit in force; at the new speed no joint's measured
effort exceeds 80% of its limit; a full round completes with no settle failures; the round is
at least 40% faster wall-clock than the 159 s baseline.

**The drive override is written, inert until configured.** `_author_drive_max_force` writes
each joint's angular drive ceiling to USD, where PhysX reads it, and records the joints it
wrote and what they held before so `efforts` can report it. It does nothing unless
`drive_max_effort` is set.

**Measured first, before changing any speed - and it reframes the task.** Per-stage timings
are now in every `Trace` and `run_round` prints a cycle summary. On the planning path the
round's time is not in the arm:

    approach_pick        29.9 s      planning, one move
    first_check_+x       40.5 s      planning, one move, orientation-constrained
    each later roll       8.5 s
    fed                   7.3 s      the conveyor, at 300 mm/s
    at_contact            1.1 s      the 35 mm straight descent
    after_grab            0.8 s
    after_lift            1.7 s      16 deg of shoulder, in joint space
    carry (measured separately, 4 moves)  18.7 s then 1.9, 1.9, 3.0

Every stage that is actually the arm moving costs 1-3 s. The 30-40 s stages are the motion
service planning, and the first plan of a leg costs 10-20x the ones after it. Raising
`joint_speed_deg_s` from 20 cannot touch that. **A cycle time for this cell has to be
measured on the REPLAY path, where the taught trajectory is driven and nothing is planned -
which is also what a real cell does.** So T8 is blocked on T10, not the other way round, and
the honest speed levers are, in order: replay rather than plan, fewer moves, then the drives.

**And the drives may need to be gentler, not stiffer.** See T9's slide, below: the rolls
break the cup's shear limit on single-step force spikes. That is the same failure the
coaxial check was disabled for, and it argues the arm is already too aggressive for a 2.5 kg
cantilevered load.

**Sim time vs wall time is not yet established.** Every number above is wall-clock on this
machine. A stated cycle time needs Isaac's real-time factor measured, or it is a benchmark
of this PC.

### T9 - Presentations and camera for a 495 mm part
Re-derive the presentation points, the camera standoff and its FOV so every face is in frame
and every station is inside the reach and comfort bounds.

**Accept:** all of `tests/test_cell_layout.py` passes, including framing and the 30 deg /
280 mm comfort bounds.

**It passed, and the cell still could not run the round.** Every layout test checked pan,
height and reach, and none checked whether the part FITTED or whether the pose was one the
round visits. Running the round end to end, once the carry held, found six faults - five
fixed, one open:

1. **The shear limit in use was a sixth of the derived one.** `sim_manager` restated the
   gripper's fallbacks as literals, and 50 N shadowed the 305.6 N `surface_gripper` derives
   from the cup diameter and the vacuum an EPick pulls. 50 N cannot carry a 2.5 kg part held
   by a side face. Fixed; retracts a6462a3.
2. **A part's size was a function of how it was held.** `measure_hold` derived the part's own
   dimensions from its live pose, so at the 16 deg shoulder tilt it measured 55 and 42 mm
   over and the planner refused all five presentations. Now a constant, derived once from the
   declared yaw, cross-checked against the asset's own collision bounds.
3. **The fixture was sited against a pose no arm visits.** 57d3f4f moved it to x 900 on
   `fixture_grip`, but the round sets the part down YAWED, which puts arm-a's flange 452 mm
   further out - 1178 mm of reach against a 1016 mm limit. `stations()` reported the unyawed
   grip as both the give and the take. Fixed; fixture at 460, window 440-480.
4. **The inspection station sat inside the infeed.** The part hung 102 mm into a deck that
   tops out at 769 mm; this worked only while the infeed was a 584 mm slab. Raised to 905 and
   moved to the camera axis, because two of the four rolls stand the part's 495 mm length
   VERTICAL and no height clears a belt with that.
5. **Presentations stood off by half the part's LENGTH** - 367 mm, the abandoned end grip -
   so the part never reached the point the camera is aimed at. Now the grip's own 206 mm.
6. **The rolls slide the part in the cup, 144 to 213 mm - OPEN.** It does not fall: the cup
   reports holding throughout and the part ends with the cup near the end of its panel, which
   is what free sliding under gravity looks like. The mechanism is almost certainly the one
   the coaxial check was disabled for - the plugin reads shear over ONE physics step with no
   averaging, a roll swings the part's far corner through a 200 mm arc, the spike breaks the
   single cup that locks the lateral axes, and `retry_interval_sec` re-grips it 0.5 s later
   at wherever it has slid to. Not yet confirmed per-cup: the gripper exposes `status` and
   `gripped` and no per-attachment diagnostics.

Two faults the round itself had, both the same shape - reaching a pose was treated as doing
the job:

- **`HOLDING` was defined, documented as mandatory, and passed to nothing.** Same ends, same
  goal: free, the move to the first presentation takes 20.9 s and leaves the part on the
  floor at [-639, 708, 114]; constrained, 45.5 s and the hold is intact.
- **A presentation of a part that is not there counted as a pass.** `shown` filled with five
  faces while the part lay at [-668, 500, 113]. The hold is now re-measured against the hold
  at the lift, and drift past 40 mm fails that face.

### T10 - Teach and verify
Teach both endings, then measure.

**Accept:** `probes/program_report.py` shows no step over its travel budget; the offline suite
passes except deliberate exceptions; `tests/test_round_e2e.py` passes; three consecutive
replays complete and land the part on the verdict's conveyor.

**In progress.** `--fresh` is required: building on the saved program replays its 30 stale
moves as a shared prefix, and those were taught against a cell whose fixture, camera and
outfeed have all since moved, so the replayed approach misses and the grip reports nothing
held. Teaching also needs more memory than a round does - every move planned with every
obstacle declared, up to six passes - and was OOM-killed at an 11 GiB cap; it runs at 13 G,
peaking 7.5.

**This is what T8 is blocked on, not the other way round.** See the cycle budget below.

### The cycle time, and why it is not an arm-speed question
`probes/qc/cycle.py` derives what one part should cost from the cell's own stations, against
speeds and dwells stated in the fragment's `cycle` block:

    arm-a loop   12.3 s
    arm-b loop   13.4 s
    serial       25.7 s   140 parts/hour
    pipelined    13.4 s   269 parts/hour

Serial is what the round does: arm-a cannot start the next part until arm-b has cleared the
fixture. Pipelined is what a line does, and then the cycle is the slower arm's loop rather
than the sum. **Overlapping the two arms nearly halves the cycle with the arms moving no
faster at all, which is a bigger lever than any realistic speed-up.**

The completing round measured 399.1 s, 15.5x the serial budget, and almost all of the gap
is planning: stages that are the arm moving cost 1-9 s, stages that are the motion service
searching cost 29-63. So the honest levers, in order, are replay rather than plan, then
pipelining, then fewer moves, and only then the drives.

Two limits on the number, stated in the config rather than hidden: 600 mm/s and
1500 mm/s^2 are under a UR5e's 1 m/s rating by choice, not by measurement; and the dwells
are estimates until someone reads the EPick manual's response times and measures the
camera's exposure plus whatever inference the course runs.

### T11 - Record and document
Video of a full round; update `docs/finish-the-cell.md` and the fragment's notes.

**Accept:** a video of a completed round is sent, and the docs state what is true.

## Rules for myself while doing this

- A recoverable environment failure is never a turn boundary. Cell down, guard expired,
  module dead - recover in the same turn, no status, no asking.
- Change ONE thing per measurement. Two changes at once cost most of a day on the scale and
  the articulation root.
- Build the instrument before the third hypothesis. `raycast`, `prim_poses` and an A/B each
  answered in one measurement what several guesses could not.
- A probe must travel the round's own code path, or it answers a question nobody asked.
- Never invalidate a whole measurement when only part of it is affected: discarding the
  measured collider depth because the part was yawed threw away a valid z and cost three
  cycles.
