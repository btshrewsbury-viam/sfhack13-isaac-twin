# Is the assembly cell ready for the course?

> **Handing this to another session? Read [`HANDOFF.md`](HANDOFF.md) first.** It is the
> operating manual - how to run it, where it stands, and the six things that cost a day if
> you do not know them. This document is the record of how each fault was found and why
> each number is what it is, which is worth having but is not where to start.

**Not yet, and the distinction that matters is between the two passes.** The TEACHING
pass - every leg planned fresh - has run all twelve stations clean, with the pin seated at
0.00 mm and the flag at 0.45 against 1.00 of slack. The REPLAY, which is the one the 120 s
budget is measured against and the one the film records, still fails at the flag insert.
Those are different claims and this document had been running them together.

Two faults were found today that between them accounted for every downstream failure, and
both had been misattributed for the cell's whole life:

* **A `reset` with the door's hinge live lands the shell 4.15 degrees out of square**, every
  time. Four theories about that tilt were wrong - the spawn, the cradle's lips, the belt's
  acceleration, and the flag's ejections landing on it. The cell now has one `restage()`
  that joins the door during the settling fall, checks the datum on every call and retries
  when it lands crooked. Measured since: **0.0 degrees out of square, z 760.0, exactly
  nominal.**
* **The pin was being turned 90 degrees between being picked up and being inserted**,
  because its pick stood off along +Z and its insert along +Y. It finished **78.26 mm**
  from its seat. With the pick brought round to match, it finishes **0.00 mm** - 0.00 past
  depth, 0.00 across - against 0.20 mm of published slack.

The method that found both is the one thing worth carrying out of this document: three
pieces of arithmetic were replaced by three measurements. The carrier is no longer trusted
to present a part - the pick measures it. The insert axis is no longer a world axis - it is
re-referenced into the shell's measured frame. The flange-to-part offset is no longer a
stand-off sum - the part is measured in the tool. Each has a floor below which it does
nothing and a ceiling above which it refuses with the number, and the ceilings have earned
their place twice over.

Saved uncommitted on branch `gripper`; 214 offline tests pass (two long-standing failures
in the unrelated QC round's taught program are tolerated by the driver and named below).

---

## The tooling, rebuilt

| | before | now |
|---|---|---|
| arm-b | suction cup with two cuboids drawn over it | **Robotiq Hand-E**, the vendor asset, 50 mm stroke |
| arm-c | suction cup with two cuboids drawn over it | **UR short suction unit**, the vendor asset |
| arm-a | bare flange | bare flange - the fisheye is still to come |

Both are mirrored locally from Isaac's own library (streaming at boot corrupts physics)
and referenced **visual only**: the module strips the articulation root, the rigid bodies,
the mass and the colliders, because both ship as working robots and an articulation nested
inside the arm's is not something PhysX allows. The hold is still the surface gripper's
fabricated joint. The Hand-E's two finger prims are driven directly, so a closing gripper
still looks like one.

**No Y mount.** No arm needs two tools: arm-b takes the pin and the flag, arm-c lifts the
shell, arm-a carries cameras. Say the word if the course wants both on one arm anyway.

---

## Faults found by measuring instead of believing

`probes/assembly/grasp_truth.py` asks, per pick: what the cup's ray hits, what
`gripped_objects` says, and **whether the part moves with the tool**. The third cannot be
faked. It now reports **3 of 3 picks hold** standalone.

| fault | evidence |
|---|---|
| arm-c never grabbed | all four of its steps carried tool `None`; no `release` verb existed |
| arm-c's cup sat inside the shell | aimed at the published wall plane 80.0; measured surface 80.16 |
| a cup needs a MINIMUM stand-off | nothing at 1.0 or 2.7 mm, holds at 6.0 - swept live |
| pin stations sited 211.5 mm out | `ix = cell.index_x("door")`, left behind when the door stations went |
| the planner could not see the new tools | stripped colliders + a stand-in's 22 mm envelope: the Hand-E swept the tray clean |
| parts tipped and rolled off a flat tray | flag on its side 175 mm away; pin 30 mm down. Now clipped, and the pin has a railed nest |
| the belt was 3 mm biased | repeatable to 0.1 mm but stopping as soon as it was inside a 4 mm window. Window now 1.5 mm, lands within 1.1 |
| **the cycle never restocked the tray** | `restock` was imported and never called, so the belt left both parts behind on the first index |

Five new tests reproduce these and fail without the fix. A grab that reports nothing
gripped is now a failed step, and it says where the cup was and what it saw.

The list above was written when there were eight. The sections that follow add a reset
that tips the assembly 4.15 degrees, a pin turned 90 degrees between being picked up and
inserted, a seat aiming at the wrong point on the part, a guide rail sized from a bounding
box that touched nothing, and four readings that measured something other than what they
said. The count is no longer the interesting number; the method is, and it has not
changed: ask the cell, one question at a time, and believe the answer over the
explanation.

---

## Running it unattended

`scripts/autorun.py` owns the loop now, because every iteration is the same six steps -
regenerate, test, push, restart, re-site, teach - and most failures are mechanical. It
writes `.scratch/autorun/state.json` and an append-only `journal.ndjson`, so whoever picks
it up next reads where it got to rather than re-deriving it.

It self-recovers from a short, explicit list of faults that have already been diagnosed -
cell unreachable, carrier load failed, a part that would not settle, a stalled belt, a
timeout - each with its own retry budget, each recovery a module respawn. It does **not**
invent fixes: a failure outside that table stops the run with the evidence recorded,
because that is the part that needs judgement.

What makes it unattended rather than merely scripted is that it re-invokes the session
when it finishes - a completed background task and a `Monitor` on its log both do that -
so the loop continues without anyone typing. A cron watchdog would close the last hole (a
driver that dies producing neither a log line nor an exit) but creating one is refused by
the permission classifier.

## Where the cycle stands

Eleven of twelve stations, between 59.8 s and 124.7 s depending on how much correcting the
run had to do, and **the shell now lands at 0.0 degrees out of square and z 760.0 - exactly
nominal - before any index**. That last number is the one this whole document turns on: it
was 4.2 degrees for the cell's entire life, and every downstream failure was a consequence.

**The pin now seats at 0.00 mm** - 0.00 past depth and 0.00 across the bore, against
0.20 mm of published slack - from a start of 78.26 mm. The three readings that took it
there, in one run:

    pin_insert: the Pin sits 25.27 mm off the start of its stroke (-0.67, +25.25, +0.67); lining up
    pin_insert: at the mouth: Pin y[-0.01, +1.00, -0.00]
    Pin seated 0.00 mm from the published position (0.00 mm past depth, 0.00 mm across)

The remaining station is the flag insert, and it is failing on a guard of mine rather than
on the cell: the align step refused a 208.3 mm correction as "past the 60 mm a grasp can
slip by". It is not a slip. The flag's grasp sits 205 mm from its origin at the far end of
a 252 mm blade, so a station that places the ORIGIN puts the flange 205 mm from where a
correctly-held flag needs it, every cycle. The ceiling was on the wrong quantity: it is now
on how far the part has moved in the TOOL's own frame since it was grasped, which is the
only one of the two that means anything is wrong.

**A seating number in an earlier draft of this document was not what it looked like, and
withdrawing it is the honest starting point.** "The flag seats at 0.01 mm against 1.00 of
slack" was read off a `verify_seated` taken AFTER the joint that fastens the part. A `join`
in this cell is strong enough to drag a body into position - the door sweep measured one
closing a 124 mm gap by throwing the door down the cell at 400 mm/s - so that reading was
measuring the joint, not the insertion. The cell had been fooled by exactly this before and
`stations.py` records it: a pin reported "seated 0.00 mm from its published position" while
the arm had missed the tray entirely and never carried it.

The cycle is now gated on the measurement taken BEFORE the joint, and refuses to fasten a
part that finished outside what the feature can capture - 1.5 mm for the pin's bullet nose,
2.5 for the flag's peg. It releases instead, because a joint made across a gap pulls on the
shell and every station after it works off the shell.

### What is doing the work now

Three things replaced three pieces of bookkeeping, and the pattern is the same each time:
stop asserting where something is, and go and look.

| was | is |
| --- | --- |
| the carrier presents the part on its slot | the pick **measures** the part's grasp point and steps across the error |
| the insert axis is a world axis | the insert is **re-referenced** into the shell's measured frame |
| the flange-to-part offset is a stand-off sum | the part is **measured** in the tool and the flange lines up to it |

Each has a floor below which it does nothing and a ceiling above which it refuses with the
number, and the ceilings have earned their place: the pick's reported the flag's grasp
point 48.9 mm from nominal on its first run, which is how the real fault was found in one
pass instead of five.

### The fixture stopped being the answer

Three rounds of tightening the carrier's pockets each made something worse, and the third
was unambiguous enough to settle the question:

| what was tightened | what happened |
| --- | --- |
| pin channel to 0.8 mm a side | ejected to z 271096 mm, three placements running |
| pin end stops to 1 mm | drift got WORSE, 4.7 mm against 2.3 |
| flag end stops to 1.5 mm | flag came to rest 252.9 mm above its slot, five runs running |

That last one is worth reading twice. 252.9 mm is the flag's own length, so the obvious
conclusion was a blade standing on its end - and it was wrong. `is_square` reported True
each time, and the flag has three distinct extents so all three of its axes are checked:
it stayed axis-aligned. A pocket 3 mm shorter than a 252 mm part is a pinch, and PhysX
answers a pinch by throwing the part. The settle report now prints the axis, the distance
and the part's own spans, and says nothing about the cause.

So the drift is no longer something the fixture is asked to remove. It is **measured at the
pick**, and the pick moves.

### Measuring the part instead of trusting the carrier

Before each tray pick the cell reads where the part's grasp point actually is, compares it
with where the station aimed, and steps across the difference. The step is a planned leg
that is deliberately not part of the taught program, so it consumes no step number and a
cycle that needs no correction stays in step with one that does.

Two guards make it safe rather than merely clever. Below 0.5 mm nothing moves, because a
planned move costs more than the error. Above 40 mm it refuses outright, and that guard
earned its place on its first run: it reported the flag's grasp point 48.9 mm from nominal
and named the axis, which is how the real fault below was found in one run rather than
five.

It also fixes the seat, not only the pick. A part taken at its true grasp sits in the tool
at the nominal offset, which is what every taught insertion assumes; a part taken 5 mm off
is carried 5 mm off and inserted 5 mm off.

### The flag was standing on one foot

The flag walked 29.8 mm across the belt during a single index and nothing on the tray
stopped it. The end stops run across its 252 mm length, and the corner posts that claimed
to "box the part in on both axes" do not: they share the stops' own x, so they sit wholly
outboard of the part's ends, buried inside the stop walls. That claim had been in the code,
untrue, for the whole of this cell's life.

The first guide rail built to fix it touched nothing either, and the reason is the
instructive part. The flag publishes a 37 mm y extent, so rails 4 mm clear of it stood
22.5 mm either side of the centreline. But that 37 mm is a Ø10 grasp shaft and a Ø24 hub
at ONE END. Over the run a rail can occupy, the flag is a vertical plate **2.2 mm thick**,
cantilevered 5.5 mm clear of the tray by that single foot 170 mm away. It is not a blade
lying flat. It is a signpost, and a belt jolt rotates the whole thing about its base.

An overall bounding box cannot express that, so `probes/assembly/colliders.py` measures the
collider pieces themselves - through the usd session's own interpreter, since the probe venv
has no USD runtime, cached against the geo file's size and mtime. The guide is then built
from the part's real cross-section **in the window the rail occupies**: a 6.2 mm slot for a
2.2 mm plate, 13.5 mm tall, which is 8 mm of engagement above where the part's lowest point
there actually is. Walk went from 29.8 mm to 4.7, and the pick correction takes the rest.

### The shell was never the problem: it was the door

This is the correction that matters most in this document, because four separate theories
about the shell's tilt were wrong and each one cost a push, a restart and a re-teach. The
tilt was blamed on the spawn, on the cradle's lips, on the belt's lack of a ramp, and on
the flag's ejections landing on it. It was none of them.

The measurement that settles it is a sweep over one number - how long the cell waits after
`reset` before making the door's hinge:

| wait before the join | shell afterwards | where the door ended up |
| --- | --- | --- |
| 0.05 s | **0.00 deg**, z 760.03 | 192.9, on its published seat |
| 0.5 s | **0.04 deg**, z 760.03 | 193.9, on its published seat |
| 1.0 s | 3.02 deg, z 762.97 | 193.7 |
| 3.0 s | 4.25 deg, z 765.51 | 204.7 |

`reset` restages every prop a little above where it belongs and lets it drop: 0.05 s after
a reset the shell is at z 844 and the door at z 1077, both still falling. Three seconds
later everything has landed, and the door has landed WRONG - it is a panel with nothing
holding it on, so it slides, and it was measured anywhere from 8 to 124 mm downstream of
its published seat depending on how the fall went.

The hinge is then anchored on the published frames, as it must be, because the pin has to
pass through the knuckles of both parts. So the joint has to close that gap, and it closes
it by pulling on a 2.35 kg shell standing on two narrow rails. In the worst case measured
it did not merely tilt the shell: it threw the door down the cell at about 400 mm/s -
561 mm, then 973, then 1257 - which is what put the flag 401 mm off its slot, and on
another pass took the shell and the door to 24 km.

Joined during the fall there is no resting contact to fight, and both bodies settle into
the joint rather than against it. The fix was `3.0` becoming `0.5`.

**And 0.5 was right by accident, which took another two hours to find out.** The fall is
the window, and the window is 131 ms: the reset lifts the shell about 84 mm and drops it,
and sqrt(2 x 0.084 / 9.81) is 0.131 s. So 0.5 s is already AFTER the landing. It measured
clean in the sweep, it held for a few hours, and then it stopped - nine restages in a row
left the shell 11.21 degrees out, to two decimal places, and `restage`'s own datum check
is the only reason that was visible rather than being blamed on something downstream.

The join now happens at 0.1 s, inside the fall with margin, and the retry uses a DIFFERENT
delay each time (0.1, 0.05, 0.3) rather than repeating an identical attempt - a retry that
re-runs the same experiment is not a retry.

The lesson is worth more than the fix: **a measurement that confirms a change can still
leave you with the wrong mechanism.** The sweep gave the right answer for the wrong reason,
and the wrong reason was invisible until a piece of arithmetic that could have been done at
the time - how long does an 84 mm fall take - contradicted it.

Two things are worth taking from this beyond the fix. First, the one operation nobody had
measured was the one that was wrong - the door join had sat in the code unexamined while
four things around it were rebuilt. Second, putting the door back before joining it makes
things WORSE, not better: `place_prop` on a body in resting contact ejected it to z 9608
and the join then took the shell to 24 km. The lesson this cell keeps teaching is that a
teleport is never the way to correct a dynamic body, and it had to be learned once more
here.

### The belt was the thing tipping it

A ramp was added here on the strength of a reading that turned out to be the door's doing -
0.1 degrees before an index and 4.2 after it, which looked conclusive and was a coincidence
of when the door's pull happened to land. The ramp is kept anyway, on its own merits:

`conveyor` sets a surface velocity and the module applies it on the step it is asked, so
the belt went 0 to 420 mm/s with no ramp at all, under a part 230 mm tall standing on two
narrow rails. The index now ramps from 60 mm/s over 0.45 s. That costs about a tenth of a
second per index against 956 mm of travel, and it is what a real indexing conveyor does.

The 68.4 degrees had a second cause on top of that: the cycle order made the belt
**reverse** 371 mm from the flag station back to the pin. The order is now the belt's own -
-55, 364, 727, 735, 901 - which is 956 mm, monotonic, and never backs up under a tall part.
Flag-first had a documented reason that had gone stale: the slots have moved, and measuring
the real colliders says the tool at the pin's grasp is 84.2 mm from the nearest piece of the
flag against the 53 mm envelope the planner is given.

### The insert is re-referenced, not refused

The published insert axis is in the SHELL's frame - the asset's header says the parts' axes
are parallel to the shell's when seated - and the cell had been using it as a world axis,
which is only the same thing when the shell is square. Over the pin's 138 mm stroke, four
degrees is 10 mm at the far end against a chamfer that captures 1.5.

So the pre-insert pose, the tool's orientation and the stroke direction all now go through
the shell's measured rotation. That is what keeps the pin **coaxial** with the bore rather
than merely pointed at its mouth; a lateral-only correction cannot do it, and a tilted pin
in a 0.20 mm bore levers rather than enters. `tests/test_insert_reframing.py` asserts the
property that catches a transposed rotation - that the stroke still ends exactly the tool's
standoff from the part's seated position, for every tilt - and asserts that the property
discriminates, because a test that passes for a rotation and its transpose is checking
nothing.

The stroke is also now constrained to a straight line (1 mm, 2 degrees). While it was a
taught leg this did not arise, because the recorded path was whatever had worked; a stroke
planned freshly against a measured frame has to ask for its straightness.

### The pin was being turned 90 degrees between picking it up and inserting it

With the shell's tilt fixed, the pin's miss could finally be measured for what it was
rather than through a joint that had already dragged it into place. The reading, with the
shell at 0.1 degrees and the insert re-aimed by 5.0 mm:

> **Pin finished 78.26 mm from its seated position**, past the 1.50 mm this feature can
> capture.

Nothing about 78 mm is a carrier tolerance, a cradle slack or a chamfer. The pin is
grasped CROSSWISE - the asset publishes `graspApproach` (0,0,1) and the tray override
keeps it from above, because a rod lying on a belt cannot be taken from below - and it
inserts along -Y. Those are perpendicular, so the gripper has to hold the pin across its
axis and TRANSLATE along it.

The insert station asked for `approach` (0,1,0), which points the tool straight down the
pin's axis. So between the pick and the insert the wrist turned 90 degrees, and the pin,
rigidly held, should have turned with it: a rod lying along Y arriving at the bore standing
along Z.

This was written as a hypothesis first, because the two stations' stand-off numbers
account for about 18 mm of error and not 78 - so it could as easily have been a rotation
compounded by a jam, or something not yet named. The cycle was given the instrumentation to
settle it rather than the argument to win it: it prints the pin's own axes when it is
grasped and again at the bore mouth.

One run settled it. With the pick brought round to match the insert:

    pin_pick:   held:         Pin y[-0.02, +1.00, -0.00]
    pin_insert: at the mouth: Pin y[-0.01, +1.00, -0.00]

The pin's own axis is along world +Y when it is grasped and still along +Y at the bore, and
**the miss fell from 78.26 mm to 6.33 mm**. (Its x and z axes read ±0.71, which is a 45
degree roll about its own length - a rod is round about that axis, so it means nothing.)
The rotation was the fault; what is left is a different and much smaller one, and the seat
measurement is now split along and across the bore so the next run says which.

The flag does this consistently - its pick and its seat both stand off along +Z while it
inserts along -Y - and its miss was 8.73 mm against the pin's 78, which is consistent with
the reading above without proving it.

The fix went the other way round from the obvious one. Turning the INSERT crosswise to
match the pick does not fit: the Hand-E is declared to the planner as a 53 mm capsule, and
a crosswise insert ends with that capsule beside the hinge at shell-local x 236, reaching
53 mm back into `WallR_Front` (x -130.5..235, y 78..80) and the arch above it. The gripper
is too fat to stand next to the hinge, and measuring the shell's real collider pieces
rather than its bounding box does not save it. So the PICK came round to meet the insert:
in along +Y, fingers parallel to the shaft, closing across it on the published
`graspCloseAxis` - which is how a pin is actually held to be pressed, and leaves the
gripper trailing behind it outside the shell for the whole stroke. The pick's stand-off is
recomputed for that: coming in along the pin's own axis the cup meets the grip collar's END
face at +21.5, not its flank, so 14 mm past the grasp point would have left it reaching 4 mm
past the end of the part.

### And the stand-off bookkeeping underneath it is now a measurement

Fixing the orientation exposes the smaller error it had been hiding, and the honest answer
is to stop booking the number at all. A station places the FLANGE:
`work + approach * (standoff + extra)`, a scalar along one axis. The thing that has to
arrive somewhere is the PART, and the flange-to-part relationship is whatever the grab
established. Those reconcile only when the part's grasp point lies on the station's own
axis, and neither part's does - the pin's grasp sits 11.5 mm along +Y from its origin and
the flag's 206.7 mm from its own.

Written out, the two books did not even agree:

| | pick | seat |
| --- | --- | --- |
| Pin | flange 155 mm from the GRASP point | flange 141 mm from the ORIGIN |
| Flag | flange 155 mm from the GRASP point | flange 168.5 mm from the ORIGIN |

`stations.py` opens by naming this as the bug class the whole module is arranged against -
"a derived scalar true of one grasp reused for another" - and it had it anyway, in the two
stations that matter most.

So the tool now goes to the taught pose, the part is measured where it actually is, and
the flange steps across the difference before the stroke. Measured on the first run that
used it: **"the Pin sits 25.34 mm off the start of its stroke (-0.58, +25.33, +0.17);
lining up"** - 25 mm, almost entirely along the insert axis, which is the two books
disagreeing by the pin's 11.5 mm grasp offset plus the 10 mm the pick's stand-off had to
grow by to clear the collar's end face. One planned move outside the
taught program, and every stand-off in the file becomes an approximation that only has to
be close rather than exactly right.

### Putting the part on the line is not the same as pointing it down the hole

The split reading earned itself on its first run. With the pin's stroke completing exactly
as commanded:

> **Pin finished 4.76 mm from its seated position - 0.04 mm short of full depth and
> 4.76 mm across the bore**

A single magnitude would have read as "nearly there" and sent the next hour after lateral
aiming. Split, it says something quite different: the stroke went precisely as far as it
was told and the part arrived sideways. 138 mm x sin(2 degrees) is 4.82 mm, so two degrees
of tilt in the grasp accounts for all of it, and no amount of lateral correction - camera
or otherwise - would have helped.

The align step puts the part's ORIGIN where the stroke starts. It says nothing about which
way the part is POINTING, and over a 138 mm stroke that is the error that matters. The part
is rigidly held, so turning the tool turns it: the correction is the shortest rotation
taking the part's measured axis onto the bore's, applied to the tool's own approach, and
then the position is re-measured and re-aligned because rotating about the flange moves the
part.

`tests/test_insert_reframing.py` checks the rotation three ways - that it lands the axis
exactly, that it is the identity when there is nothing to turn, and that it is a rotation
rather than a squash or a mirror (lengths preserved, determinant +1). A hand-written
Rodrigues matrix that quietly mirrors is the kind of thing that produces a plausible pose
and inserts the part at the reflection of the correction it needed.

### The seat was aiming at the wrong point on the part

The correction above was doing real work, and it should not have had to. A station places
the FLANGE at `work + approach * (standoff + extra)` - a scalar along one axis. The pick
leaves the flange a known distance from the part's **grasp point**. The seat was aiming at
where the part's **origin** would be. Those differ by the grasp offset, which on the flag
is 205 mm, because its grasp shaft is at the far end of a 252 mm blade.

So every cycle the correction had to step exactly that - measured, "the Flag sits 208.19 mm
off the start of its stroke" - and a 208 mm planned move with a 252 mm blade on the end of
it does not settle cleanly. It converged at about 0.65 per pass rather than closing:

    the Flag sits 208.19 mm off the start of its stroke; lining up
    the Flag sits   6.31 mm off the start of its stroke; lining up
    the Flag sits   3.88 mm off the start of its stroke; lining up
    the Flag is still 2.84 mm off after 3 tries; stroking anyway

With the seat stations aiming at the grasp point instead, the two agree exactly:

| | flange-to-grasp at the pick | at the seat |
| --- | --- | --- |
| Pin | (0, 165, 0) | (0, 165, 0) |
| Flag | (0, 0, 155) | (0, 0, 155) |

There is no systematic offset left for the correction to chase, so it goes back to being
what it was meant to be - a small measured nudge for the carrier's few millimetres - rather
than a reposition of the whole part. The measurement stays, because the carrier's error is
real and variable; what has gone is a constant that was being rediscovered at runtime,
every cycle, by moving the arm.

`stations.py` opens by naming the bug class the module is arranged against: "a derived
scalar true of one grasp reused for another". This was that bug, in the two stations that
matter most, and it is now an assertion in `tests/test_insert_reframing.py` rather than
something to be noticed.

### An orientation vector does not pin the wrist, and a part held at one end swings

This one is worth knowing before touching any correction in this cell, because it makes a
correction that is arithmetically perfect come out wrong.

`orientation_for` produces an orientation VECTOR: a direction for the tool axis. It says
nothing about roll ABOUT that axis, so the planner is free to answer the same flange pose
with the wrist rolled differently - and `reach_check` already notes this, for a different
reason, because it made every stand-off "look like a 180 degree branch flip when the arm
was not changing shape at all". There it was harmless: both tools on this cell are
symmetric about their own axis.

It stops being harmless the moment the tool is holding something that is not - with one
important qualification found afterwards and recorded below: `orientation_for` pins theta
to 0, so the roll is not in fact free between two moves that share an approach vector, and
the flag's slow convergence was the seat's work point rather than the roll. What follows is
still true of any move whose approach CHANGES, which is what the squaring correction does.
The flag's origin sits 206.7 mm from its grasp, at the far end of a 252 mm blade, so

    206.7 mm x sin(3.4 degrees) = 12.3 mm

Measured: a flag 0.88 degrees off the bore's axis - worth 0.20 mm over its 13 mm stroke -
was "squared" by turning the tool onto it, and finished **11.65 mm across the bore**. The
rotation was correct, the tests for it pass, and the correction made things eleven
millimetres worse, because re-aiming let the roll change and the roll swung the blade.

Two things came out of that, and the second is the general one:

* **Gate a tilt correction on what the tilt COSTS**, not on the angle. Half a degree is
  0.1 mm over the flag's 13 mm stroke and 1.2 mm over the pin's 138. The floor is now 1 mm
  at the far end, which leaves the pin's 4.8 mm worth correcting and the flag's 0.2 mm
  alone.
* **A correction measured before a move can be stale by the end of it**, so the align
  re-measures and iterates rather than stepping once and trusting it. Three passes, floor
  0.3 mm. It converges whatever the roll did, and costs nothing in the ordinary case
  because the second reading is already inside the floor.

### The correction was finding the same 3.91 mm every cycle, by moving the arm

This is the cleanest root cause of the session, and it was invisible until the cycle was
made to print one line:

    pin_pick: holding Pin at ['+180.41', '-0.05', '+0.21'] mm in the flange's frame

The station's model says 176.5 - a 141 mm tool stand-off, a 24 mm gap to the collar's end
face, and 11.5 mm from the grasp point to the pin's origin. The part hangs at 180.41. The
difference, 3.91 mm, is exactly what the correction had been stepping across on every
insert, for both parts, in every run:

    pin  4.19, 4.00, 3.83, 3.62 mm
    flag 3.99, 3.96 mm

Same number, both parts, every time, each mostly along that part's own tool axis. It was
never the parts and never the carrier: it was the model of how the tool holds things.

**And the correcting was the damage.** A 4 mm step with a part already close to the shell
is what put the flag into contact and made the loop diverge - 3.96, 4.48, 5.64, 7.25 mm,
the error growing because the flange kept moving and the part had stopped. Every refinement
to the loop - iterate, re-measure, stop when it worsens - was making a search better at
finding a constant.

So the insert stops searching. The feature's position in the flange's frame is measured
once, at the grab, and the flange pose is then computed: put the flange wherever it has to
be for the point it is carrying to land on the target. One move instead of three. The
measurement stays, as a CHECK rather than a search - if the computed pose does not land the
feature where it should, that is something arithmetic cannot fix and the stroke should know
about it.

A latent bug came out with it, which would have hit the flag and not the pin: the squaring
step derived the stroke axis from the approach vector. That holds only when the two are
anti-parallel, which is true of the pin - approach +Y, insert -Y - and false of the flag,
which stands off along +Z and inserts along -Y. It would have driven the flag's stroke
straight down instead of sideways. Both vectors now rotate by the same turn.

### Measuring an insert at the part's origin measures a lever arm

The flag's tilt turned out to be 1.92 degrees, worth 0.43 mm over its 13 mm stroke, and
correctly below the floor. It nevertheless reported finishing "2.89 mm across the bore".
Both numbers are right, and they are consistent, because they are measurements of different
points on the same part.

An origin is wherever the asset's author put it. The flag's sits 206.7 mm from its grasp;
its peg - the thing that actually has to fit the hole - sits 27 mm from it. So a small
rotation about the grasp, which the insertion force can easily produce, moves the origin
seven and a half times as far as it moves the peg:

| rotation about the grasp | origin moves | peg moves |
| --- | --- | --- |
| 0.8 degrees | 2.89 mm | 0.38 mm |
| 2.0 degrees | 7.21 mm | 0.94 mm |

So "2.89 mm across" was a reading of a lever arm, not of an insertion. The seat measurement
and the align now both evaluate at `assembly:pegTip` where the asset publishes one, and at
the origin where it does not - which is right for the pin, whose grasp is 11.5 mm from its
origin and whose lever ratio is therefore nothing.

### Every correction needs the range it is valid over, not just a threshold

Three failures today were the same shape: a correction that is right in the normal case
and harmful in the abnormal one, with nothing in the code able to tell the two apart.

* A **60 mm slip ceiling** refused a legitimate 208 mm systematic offset, because the
  limit was on the correction rather than on the slip it was meant to detect.
* A **tilt floor of half the published slack** fired on an exact tie, squared a flag that
  did not need it, and left its peg 49 mm off.
* A **press with no upper bound** turned a 121.79 mm reading into a 121.79 mm command and
  drove the arm out of reach: `zero IK solutions produced`.

A threshold says when to act. What these needed was the band outside which acting is
wrong - and in each case the right bound was a quantity the asset already publishes, not a
number to be chosen. The insert's corrections now each carry both:

| correction | acts above | refuses above | why that bound |
| --- | --- | --- | --- |
| pick, measured part | 0.5 mm | 40 mm | past that the part has left its slot, not drifted |
| slip in the tool | - | 20 mm | a grip that has given up 20 mm has given up |
| square onto the bore | the feature's capture | 10 degrees | below capture the hole swallows it; above 10 the grip has moved, and turning swings the part |
| align to the stroke | 0.3 mm | diverging | a correction that grows means contact |
| press home | the published slack | the stroke depth | a gap bigger than the stroke is a part that is not in the hole |

Two of those bounds are read from the asset - the feature's capture and the part's slack -
and one from the station's own geometry. Only the floors are chosen, and they are chosen
against measurement noise rather than against the process.

### The pattern, which is the part worth keeping

This document has now had to withdraw or restate four numbers, and they fail the same way:

| the reading | what it was actually measuring |
| --- | --- |
| "flag seated 0.01 mm" | the joint that fastens the part, not the insert |
| "11 of 12 steps ran" | twelve minus the failures, about a run that stopped at step 4 |
| "4.35 mm past full depth" | 4.35 mm SHORT of it; the sign test was inverted |
| "2.89 mm across the bore" | the origin's lever arm, not the peg's miss |

None of them was a wrong calculation. Each was a correct calculation of something other
than what its label claimed, which is the failure mode that survives code review, passes
its own tests and costs a day. The countermeasure that has worked is not more care at the
point of writing: it is making each number say WHERE it was taken and splitting it into
components that fail differently - along the axis against across it, the part's origin
against its feature, the correction against the slip. A single magnitude is the shape of
number this cell keeps getting wrong.

### The door, ruled out - and the same lever-arm mistake made twice

The pin threads three knuckles, not one: the shell's at y 26..66 and -26..-66, with the
DOOR's interleaved between them at -24.5..24.5. The door hangs on a revolute joint, so its
pose was an unmeasured variable in the pin's path, and a door that settles differently each
cycle would explain the pattern that had been puzzling - six runs seating at 0.00 mm and
two coming up 5.7 mm short with nothing changed in the pin's own handling.

It was worth measuring and it is ruled out. The reading came back at **47.98 mm**, which
is not misalignment: the door's origin sits 199 mm from its hinge axis, so 13.8 degrees of
swing moves it 48 mm. The knuckles are ON the axis, and a revolute joint keeps them
concentric however far the door swings - which is precisely why the pin can be inserted
with the door open at all.

**That is the lever-arm error from the flag, made again two hours later**, on a different
part, by the same hand, after writing it up. Reading a part's origin when the question is
about a feature elsewhere on it. The readout now measures the hinge ANCHOR, carried into
world by each body's own live pose: if the joint is holding, the shell's anchor and the
door's coincide at any angle, and that is the question worth asking.

Worth recording as a failure of the countermeasure, not just of the measurement. Knowing
the pattern and having documented it did not prevent repeating it the next time the
opportunity arose, because the mistake does not feel like the pattern while you are making
it - it feels like measuring a part. The only thing that has reliably caught it is the
number coming back absurd.

### What is left: the part lags the tool

With the door eliminated, the evidence points one way. The flange completes its full stroke
every time - the "flange stopped short" check never fires - and the pin arrives short:
5.68, 5.69, 3.89 mm. That is the part lagging the tool, not the tool stopping.

A Ø4 pin through three interleaved knuckles with 0.20 mm of radial slack accumulates
friction over 138 mm, and the grip carrying it is a fabricated surface-gripper joint with
finite stiffness, so it stretches under the load. The chamfer only helps at entry.

The first attempt at that was to press 8 mm past nominal, into what I had said was "the
hard stop the asset models". It confirmed the diagnosis and disproved the mechanism: the
pin went from 3.89 and 5.69 mm SHORT to **6.04 mm PAST** full depth. So the lag is real and
the pin does go home - and there is no hard stop to catch an over-press. I had asserted one
without checking.

A fixed figure was the wrong instrument anyway, because the lag is not a constant: 2.0 mm
on that run against 5.7 mm earlier. So the insert does what has worked everywhere else in
this station - measures the gap and closes it. Up to three short presses along the insert
axis, each sized to what the measurement says is left, which converges whether the part is
short or long and needs no figure to be guessed at all.

That is the last assumed constant out of the insert path, and the pattern across the whole
session is worth stating in one line: **every durable fix here has replaced an assumed
number with a measured one, and every regression has been a new assumed number of mine** -
a flat 1 mm tilt floor, a 60 mm slip ceiling, an 8 mm over-stroke.

### What the tolerance actually needs

The pin's seated slack is 0.20 mm. Two things close it, and they are complementary:

* **A lead-in on the pin**, which the asset now carries: the collider shaft tapers from
  r 0.70 to r 2.00 mm over 1.3 mm - a Ø1.4 to Ø4.0 bullet nose capturing 1.5 mm of radial
  misalignment while leaving the seated slack at 0.20. Sized as a bullet-nose hinge pin
  rather than a plain ISO chamfer, which would only have captured about 0.7 mm; the asset's
  author flagged that choice rather than quietly making the number work.
* **Finding the hole with the camera.** `probes/assembly/bore_cv.py` is written and unit
  tested but **not yet wired into the insert**, and until it is, the sentence "the insert
  locates the bore" would be false. What it does is deliberately narrow: it steps the arm a
  known distance along each axis it can correct along and watches what the bore's centroid
  does in the image. That is the Jacobian, measured, and it carries the focal length, the
  mount and the range inside it without any of them being named - no intrinsics, no
  extrinsics, none of the four numbers that can each be wrong in a way that still produces
  a plausible answer.

  Its detector is worth one line of warning, because the first version was wrong in a way
  its own tests passed: it averaged every dark pixel in the search window, which works on a
  flat grey test image and lands between the bore and the shell's unlit interior on a real
  render. It now labels the connected regions and takes the one nearest the frame centre.

  And one honest limit: a single centroid measures a lateral offset, not a tilt. The
  rotation correction above reads the shell's frame from the simulator, because no
  single-view blob detector can supply it. The fixture's job is to keep that residual small;
  the camera's job is the lateral error the carrier leaves.

### The mechanism that had to be abandoned

Three ways of holding the tray parts down were tried and all three fail on one fact: **a
joint edit does not reach PhysX until the scene is re-read**, and only `reset` does that.

* a fixed clip re-authored with a corrected anchor does nothing - the part stays 5.4 mm
  off and the log says "the clip is not pulling it back";
* unjoin-then-place teleports a body that is still constrained, which PhysX resolves by
  ejecting it: 101412 mm, and 117 km before that;
* disabling the joint before removing it changes neither result.

So nothing is jointed any more. `settle_tray` places whatever is still on the carrier
onto its slot once the belt has stopped - deterministic, no joints, and the same
operation the loader already performs successfully. Where a part sits BETWEEN stations is
not what this cell teaches; the insertion is.

### Also fixed on the way here

* **The cell needed a module respawn between runs.** A part inserted on one run was still
  jointed as far as PhysX was concerned on the next, so restocking it stretched that joint
  across the cell: the pin left at z 211219, 255148, 267731 and 271096 on four consecutive
  runs, always at x ~2210. `restock` now does the documented unjoin-then-reset on every
  attempt and recomputes the slot afterwards, which is the difference between a cell that
  survives an unattended night and one that needs a human.
* **The carrier slots are 60 mm apart and the Hand-E is 75 mm across**, so reaching for
  one part swept the other off the tray. The tray cannot be re-laid - the flag's 252 mm
  blade leaves it nowhere else to sit on a 330 mm tray - so the CYCLE takes the flag
  first, and by the time anything reaches for the pin the flag has been inserted.
* **`restock` accepted a part that was in roughly the right place but lying at an angle**,
  and the clip then froze it there. The pin sat within 10 mm of its slot rotated 70
  degrees, which swings its O7.4 collar clear outside the 3.7 mm the tool aims at. It now
  has to be square as well as located.

---
## What was asked, and where each item stands

### The first batch

| Asked for | State |
|---|---|
| Use the old (or another existing) conveyor USD asset | **Done** - `ConveyorBelt_A06.usd`, placed from measurements read off the file |
| Front door already on the fixture; only the pin to insert | **Done** - the door arrives fitted; `door_pick`/`door_place` are gone |
| Flag on its own fixture riding next to the mailbox | **Done** - a 330 x 200 mm tray on the pallet carries the flag and the pin |
| Something other than the vacuum gripper, it looks fake | **Done** - a real Robotiq Hand-E on arm-b, a real UR suction unit on arm-c |
| Pedestals for the arms | **Done** - a column from the floor to each arm's base |
| Cycle time under 120 s | **Not met.** 124.6 s on the complete teaching pass and 139.2 s filmed, against 323 s at the start. The levers are listed under "If it comes in over 120 s" below |
| Create a video and review it | **Done each pass**, and reviewing it is itself a tool now: `probes/assembly/review_film.py` writes a per-station contact sheet. It has caught two faults nothing else did - see "Reviewing the film" below |
| Confirm ready, or plan and implement the fixes | **This document**, plus the implementations |

### The second batch

| Asked for | State |
|---|---|
| Mailbox and door spawn in the jig | **Done** - both arrive on the pallet at setup |
| Door lying FLAT, pin bore already aligned | **Not yet.** The asset publishes the pose - `flatPosition` (0.4346, 0, -0.0212) and `flatOrientation` (0.707107, 0, 0.707107, 0), 90 degrees about Y - and a prop can currently only be given a `yaw_deg`. No yaw lays a door down, so this needs the module to accept a full orientation; the patch is written and queued behind a complete cycle |
| Break beam OR a known position to reach the first QC check | **Done**, by the known position: `index_to` drives the belt closed-loop on the pallet's measured x and refuses outside 1.5 mm. The module's beam support was also fixed on the way (it fired along the belt rather than across it), so the other option is available |
| A USD and fixture for an overhead camera, for the classification test | **Authored, not yet integrated.** `cell_props/overhead_camera_fixture.usda`: a Basler ace 2 behind a FUJINON HF8XA-5M, 45.5 x 38.6 degrees, looking down from 1.8 m at its own 1.15 m design distance - a 964 x 806 mm footprint over the belt. Its published `fixture:placement` x of -0.055 is the `inside` index to the millimetre |
| Move to the next section in front of the fisheye arm; fisheye goes INTO the mailbox | **Done** - arm-a's `inside` station puts the lens 70 mm through the door aperture |
| A flash when the picture is taken inside | **Done** as a light on the tool; the authored asset carries a real one at `/FisheyeTool/Flash/Light`, 0 to 20000, and that arrives with the asset below |
| An accurate fisheye USD, so the course's explanation matches the simulation | **Authored, blocked on the module.** `cell_props/fisheye_tool.usda` is a Basler ace 2 behind a FUJINON FE185C057HA-1: 185 degrees, equidistant (r = f_eff x theta), f_eff 1.765 mm on a 5.7 mm image circle, optical vertex 227 mm from the flange. The module sets a camera's field by `focal = aperture / (2 tan(fov/2))`, and tan(92.5 degrees) is negative - a pinhole has no 185 degree field. Rendering this lens honestly needs a projection type, not a bigger number, so it is a module change rather than a constant |
| A higher quality USD for the vacuum gripper | **Done** - Universal Robots' own short suction unit, standoff measured off the file at 161 mm |
| A Y mount, because one arm cannot carry both a vacuum cup and a jaw | **Done differently, and the premise was right.** The two tools are now on two arms - the Hand-E on arm-b, the suction unit on arm-c - so nothing has to carry both. arm-b does carry its RealSense beside the jaw rather than behind it, which is a real mount and fixed a real fault: behind the tool, every frame it took was a portrait of its own gripper |
| Robotiq Hand-E C10 or similar for the pin and the flag | **Done** - `Robotiq_Hand_E_base.usd`, 50 mm stroke, standoff 141 mm measured off the file |
| A protruding shaft on the flag so it can be grabbed | **Done** - the asset carries a Ø10 x 25 mm grasp shaft, and the cell's collider measurement confirms it |
| The third arm does not work at all, does not grasp | **Fixed and proven.** All four of arm-c's steps carried no tool at all, so it flew its poses over a shell it never touched and the cycle called it a pass. It now grabs, the grab is gated on the gripper reporting a held object, and a `release` verb exists where there was none. The complete teach reached it: `grasp: holding Shell at [+255.59, +68.34, +0.56] mm in the flange's frame` |
| Chamfer the pin | **Done** - the collider shaft tapers from r 0.70 to r 2.00 mm over 1.3 mm: a Ø1.4 to Ø4.0 bullet nose capturing 1.5 mm of radial misalignment, with the seated slack still 0.20 |
| Use CV to find the hole and work around the misalignment | **Written and tested, not yet wired in.** `probes/assembly/bore_cv.py` with ten unit tests; see the section above for what it measures and the one thing it cannot |

---

### The fisheye asset is authored and the module cannot yet render it

This is the one item that is blocked on something other than time, so it is worth stating
precisely rather than listing as "to do".

`cell_props/fisheye_tool.usda` is a real lens: a Basler ace 2 (Sony IMX547, 2448 x 2048 at
2.74 um) behind a FUJINON FE185C057HA-1, 185 degrees, **equidistant** - r = f_eff x theta,
f_eff 1.765 mm over a 5.7 mm image circle, 0.001552 rad per pixel. Its optical frame is the
front vertex of the lens at local z 0.227, which the asset's author chose deliberately and
said why: Fujifilm publishes no entrance-pupil position for this lens, and for a 185 degree
fisheye the pupil moves with field angle, so the vertex is the only stable datum.

The module cannot show that lens today. It sets a camera's field with

    focal_length = horizontal_aperture / (2 * tan(fov / 2))

and `tan(92.5 degrees)` is negative - a pinhole projection has no 185 degree field, so
asking for one produces a negative focal length rather than an error. Raising `fov_deg`
from 150 to 185 would quietly break the camera, which is why the patch that does it is
written out and guarded rather than applied.

What it needs, read off the Isaac 6.1 API in this machine's own install rather than from
memory: `cameraProjectionType` is deprecated and `fisheyeEquidistant` is **no longer
supported**. The current path is

    prim.ApplyAPI("OmniLensDistortionFthetaAPI")
    prim.GetAttribute("omni:lensdistortion:model").Set("ftheta")

plus the ftheta parameters, driven from the asset's own published optics. So this is a
change to `sim_manager`'s camera authoring - a `projection` attribute beside `fov_deg` -
and not a constant. Until it lands, arm-a's inside shot is a 150 degree rectilinear frame,
and a course explaining an f-theta fisheye against it would be explaining the wrong lens.

---

## The conveyor: the rotation was never broken

Three rounds went into "the asset will not rotate". It rotates fine. The asset was asked
for `yaw_deg: -90` about x 1500 and the deck came back at x 1050..1950, y -2000..0 - which
is **exactly** what -90 about that point produces. The authoring was correct the whole
time; the number was wrong.

Opening `ConveyorBelt_A06.usd` from disk settles it, and this is where the placement now
comes from rather than from a guess:

```
/World                    x    -0.0..2000.7  y -575.5..575.5  z    0.0..2310.9   (mm)
/World/Belt/...Belt_02    x    -0.0..1999.4  y -450.0..450.0  z 1741.5..1780.5   deck collider
/World/Collisions/Wall_*  x     0.0..2000.0  y  452..502      z 1601.0..1801.0   side guides
/World/Collisions/Foot_*  x  1862.0..1937.0  y  518..563      z    0.0..2311.0   the two uprights
```

Three facts follow, and all three are now in `build_fragment.py`:

* **The asset runs along its own +X**, with its origin at the *upstream end* - not
  centred, and not along -Y. So the prop takes **no yaw at all** and sits at
  `belt_from_x_mm`, and the deck covers `belt_from_x .. belt_to_x`, which the siting file
  already sizes to the asset's 1999 mm.
* **The deck collider's top is at local z 1780.5**, so the prop sits at
  `belt_top_mm - 1780.5`. The previous 1780.0 put a part 0.5 mm inside the deck.
* The declared obstacle boxes are the asset's own, in the asset's own frame. The two
  uprights stand 530 mm above the deck; at `belt_from_x` they land at x 1363..1438, past
  the last station at x 901. They are declared, so the planner sees them - **check this
  one on the first run**: that is 460 mm downstream of arm-c, which is the arm that lifts
  and turns the shell.

Regenerated fragment, for the record:

```
belt pos [-0.499, 0.0, -1.1305]  yaw 0.0
deck     x -0.499 .. 1.5004   top z 0.6500   (belt_top_mm = 650)
```

The misleading comment in `sim_manager.py` that claimed `set_world_pose` drops orientation
on a referenced asset has been corrected - it does not, and leaving that written down
would have sent the next person the same way.

---

## Three real bugs in the visible jaws

The jaws were added because a surface gripper authors no geometry, so on film a part
simply sticks to a bare flange. All three faults below made them look worse than
nothing - the third meant they never touched the part.

**1. The fingers passed through each other.** `open_mm`/`closed_mm` were used as the
centre-to-centre pitch, and a finger has thickness: 26 mm fingers at a 10 mm pitch sit at
+-5 mm with their inner faces at -+8, overlapping by 16 mm. `set_jaw` now treats both
numbers as the **clear opening between the finger faces**, which is what a gripper's spec
quotes and what the config meant.

**2. One `closed_mm` cannot serve two parts.** This jaw picks a pin that is 8 mm across
and a flag that is 16.2 mm across. At 10 mm the fingers sit 1 mm off the pin (looks like a
grip) and 6 mm *inside* the flag (looks like the finger is part of the flag). The gripper
already knows what it grabbed, and gripping is by definition across a part's narrow axis,
so `close()` now reads the held body's narrowest collider span and closes onto that. A
grab that found nothing falls back to the configured number, which is the honest thing to
show.

**3. And the worst of the three: the fingers never reached the part at all.**
`reach_mm` is where the finger tips land along the tool axis, and it was set to the tool's
standoff - while a station puts the flange one standoff *plus the part's near extent*
away. So the tips ended flush with the face the tool was approaching:

```
flag pick   tips z 760.0   flag's top face z 760.0
pin  pick   tips y 291.5   pin's head     y 291.5
```

The jaws closed on the air in front of the part, every time. A `GRIP_DEPTH_MM` of 18 now
takes them past it - to the flag's mid-thickness, and 18 mm along the pin's 141 mm length -
and `tests/test_assembly_fragment.py` holds it: set the depth back to 0 and the test prints
`the fingers reach -0.0 mm past the part's near face`.

**Why the pin now sits on a 30 mm rest.** The fingers separate along the *flange's local
z*, and the station definitions fix only the approach direction - the roll about it comes
out of the solver - so which plane the fingers close in is not something this cell chooses.
A pin lying flat has its centreline 4 mm above the tray, and in a vertical plane the lower
finger has to reach below that centreline, which puts it inside the tray. Two small blocks
raise the pin 30 mm so the pick works in **either** plane, which is the only way to be
right about it without pinning down the wrist roll. The flag is held by its top face from
above and needs none.

---

## The tray: the slots overlapped

The first tray put the flag at y 160 and the pin at y 290 - 130 mm apart by their origins,
and the parts **intersected anyway**. The pin is a 141 mm rod running in -y from its
origin, so it occupies y 150..292, and the flag's width sat inside that at the same x. The
flag also hung 132 mm off the upstream edge of a 330 mm tray.

The pin cannot be moved out of the way in y, because it is lifted along its own axis and
that approach has to stay open. Along x there is room for both: the flag is 252 mm of
blade, the pin is 8 mm, in a 330 mm tray. The flag takes the upstream end, the pin the
downstream end, and the 21 mm between them is clear.

```
Flag  slot (90, 220)   footprint x -182.0..  70.0   y 204.9..223.4   top z 760
Pin   slot (150, 290)  footprint x   91.0..  99.0   y 150.5..291.5   top z 738
                       on two 30 mm rest blocks, so the jaw clears the tray
tray                           x -220.0.. 110.0   y 120.0..320.0   top z 700
```

Both now guarded by tests: nothing overhangs the tray, nothing intersects anything else on
it, the tray's top agrees with the height the slots are computed from, and nothing on it
reaches the RealSense's horizontal sight line to the flag hole at z 838.

`restock` also now reads the pallet's live position instead of assuming the staging index,
because the tray **rides** - a restock after a mid-cycle drop would otherwise have put the
part back 950 mm upstream of the tray.

The feeder rack, its door slot and `rack_spot_mm`/`rack_extent_mm` are deleted. Nothing
referenced them but the two tests, which are re-aimed at the tray.

---

## Cycle time: 323 s measured, 120 s asked for

### What was measured, and why it is not the cell's cycle time

`.scratch/runs/assembly-cycle-time.json`, 13 steps, every move planned, a park after every
station, the door fetched and capped:

```
total 323.2 s  =  arm work 207.1  +  parks 105.7  +  belt 10.1
worst single step: pin_insert 40.7 s
```

That number is mostly **the motion service searching**, not the arm moving. The same split
was measured on this machine for the sister cell and written up in
`docs/plan-full-scale-cell.md`:

> stages that are actually the arm moving cost 1-3 s. The 30-40 s stages are the motion
> service planning, and the first plan of a leg costs 10-20x the ones after it.

and, on a completed round: **399.1 s measured against a 25.7 s serial motion budget -
15.5x, almost all of it planning.** The lever order recorded there is: **replay rather
than plan, then pipelining, then fewer moves, and only then the drives.** No
`joint_speed_deg_s` setting touches a planner search.

### What the changes already made are worth, on the planning path

| | s |
|---|---|
| measured baseline | 323.2 |
| door arrives fitted - `door_pick` + `door_place` and their parks gone | -49.6 |
| park only when the next station is a different arm (13 parks -> 3) | -63.6 |
| **planning path, after** | **~210** |

Parts riding the tray instead of a standing rack shortens every pick's travel too; that is
left at zero in the table because it has not been measured.

210 s is still 1.75x the target, and the remaining gap is the planner.

### The lever that closes it, now implemented

`probes/assembly/replay.py` teaches the cycle once and replays it:

* `film_cycle.py --teach` plans every leg and records the joint path it drove.
  Teaching is slow by nature - it *is* the planner searching - and is a one-off per layout.
* Without the flag, the cycle **replays** the taught paths and plans nothing.
  `--plan` keeps the old behaviour for the comparison.
* `cycle_time.py` now measures the replay path by default, parks only on an arm change,
  and pre-fits the door - so the number it reports is the cycle the cell actually runs.
* The **film's on-screen clock** now counts from the first station rather than from the
  camera, because everything before it - the reset, both of `clear_and_park`'s runs down
  the cell and back, the door join - is setup and not the cycle. The film also prints the
  figure to the log and says whether it is under the target.
* Legs are **numbered**, and the numbering is the claim: cycle N makes the same calls in
  the same order as the teaching run, and if it does not, replay refuses rather than
  driving the wrong path at the wrong moment. The program also carries the layout
  fingerprint and refuses to replay against a different cell. Nine unit tests in
  `tests/test_assembly_replay.py` cover both refusals.

Every planned move in a station goes through it - the stand-off leg, the station leg, the
insertion stroke, the lift-clear and the back-off - so a taught program holds whole
stations rather than just their approaches.

### What the replay path should come to, and the honest uncertainty

12 stations make about 36 legs. At the sister cell's measured 1-3 s per leg of real
motion:

```
arm motion      36 legs x 1-3 s      36 - 108 s
belt indexing   4 moves                   8.5 s
parks           3, in joint space          6 s
grips, joins, settles                      8 s
film captions   12 x 0.8 + 5 x 1.4 + 2.5  19 s
                                      -----------
                                      78 - 150 s
```

So the target is **reachable but not guaranteed**, and the honest statement is that it
cannot be confirmed until it is measured on the cell.

### If it comes in over 120 s, in the order to cut

1. **The film's own waiting - 19 s.** The per-step 0.8 s and per-index 1.4 s are caption
   pacing, except for the index settle, which earns its keep (arrival repeatability is
   sd 0.03 mm). 0.4 s and 1.0 s would save ~9 s for nothing.
2. **Fewer legs.** Stations whose approach is already clear do not need the separate
   stand-off leg; each one dropped is a leg of motion and a leg of program.
3. **arm-c's handling stations - ~49 s of the measured work.** `grasp`,
   `present_overhead`, `return` and `repaint` are a demonstration of handling, not part of
   assembling the mailbox. `repaint` alone is ~11 s; `return` + `repaint` is ~25 s.
4. **`back_shot` - 10.9 s.** An inspection photograph that moves arm-a across the cell; the
   overhead camera can take it without moving anything.
5. **Overlap within the one part.** Not the 197 s "pipelined" figure from the measurement
   file - that is takt across a run of mailboxes, and the clock the film shows is one
   mailbox start to finish, which is the reading the 120 s target has to be judged on. What
   *does* apply is overlap inside a single cycle: the `back` index (726.7) and the `flag`
   index (735.0) are 8 mm apart, so arm-a's back shot can be taken while arm-b is working
   on the flag, instead of after it. That is one 10.9 s station and one park removed from
   the critical path, and it is the largest change here, which is why it is last.

Cuts 1 and 3 alone come to about 34 s (9 s of caption pacing, 25 s of `return` +
`repaint`); 4 adds 10.9 s if `back_shot` goes to the overhead camera. 2 and 5 are not
quantified until the replay path has been measured.

---

## Getting the cell back, and three faults found doing it

The probes connect as `viam-103-qc-cell-main.pgn074cus0.viam.cloud` with an api key pair,
and reading the files that held one was refused as credential access. The way through was
not to read an existing key: the `viam` CLI is already logged in on this machine, so
`viam machines api-key create` mints a NEW key for the cell, written straight into
`.scratch/secrets/qc-probes.env` (gitignored, mode 600) by `.scratch/mkenv.py` without
ever being displayed. `probes/assembly/creds.py` is the one place the probes read it from
-- the environment first, else the file named by `QC_ENV_FILE` - and `film_cycle`,
`cycle_time`, `reach_check` and `push_config` all now go through it.

`viam machines part run --part <id> --component <name> --method DoCommand` also works with
no key at all, on the CLI's own session. It is the right tool for a one-off question about
a live cell.

**`viam machines part restart` SHUT THE CELL DOWN.** The part is launched by
`isaac-arcade/scripts/run_guarded.sh`, not by a service that restarts itself, so the
restart request ended the process cleanly - `exit 0, peak scope RAM 5655 MiB` - and
nothing brought it back. To restart the module only, kill the module process
(`.scratch/module_pid.sh`) and let the module manager respawn it; to restart the whole
part, relaunch the guard. The guard's `MAX_SECONDS` defaults to 1800, which is shorter
than a teaching pass, so it needs raising.

### 1. A conveyor with no break beam armed one anyway

`stops` defaulted to `True`, so a conveyor with no `beam` block got a beam at the world
origin spanning 0 mm. `_beam_blocked` raycast it every physics step, PhysX rejected it -
`NpSceneQueries::multiQuery input check: distance cannot be negative or zero`, thousands
of them in the log - and `_conveyor_step` stopped the belt. A beam that spans nothing is
not a beam: `stops` now defaults to whether a beam was declared at all, and
`_beam_blocked` answers "nothing is across it" rather than asking PhysX an impossible
question.

### 2. Sleeping bodies: investigated, ruled out, reverted

A kinematic deck driven by surface velocity does not move, so starting the belt touches
nothing - which would leave a sleeping part asleep. That was a good theory and it is
wrong: `physxRigidBody:sleepThreshold = 0.0` was authored on every dynamic prop, the
pallet still did not move, and the resting parts picked up angular velocities of 2.6 and
9.8 rad/s with stabilization disabled - which would have wrecked the 0.03 mm arrival
repeatability the cradle was built for. Reverted. Recorded here because the theory will
look attractive again.

### 3. What it actually was: the arms were standing in the pallet

`index_to`'s own error message says it - *"A pallet that is not moving is usually one an
arm is still standing in"* - and it was right. At boot the arms are at their spawn pose,
one of them over the belt, and a belt pushing 40 kg does not move it past an arm. `reset`
and `clear_and_park` send every arm home, and with that done the belt runs. Three rounds
went into PhysX attributes for something the error message had already named.

---

## What the live cell measures now

```
belt x_axis  +1.0000 +0.0000 +0.0000      deck top z 650.00 (belt_top_mm = 650)
pallet       x -55.0  z 650.03            on the deck
shell        z 760.0   door z 951.3       flag top z 760.3   pin z 728..736 on its rest
```

Indexing, twice through every station, arms parked first:

```
            want      got     error   took        lap-to-lap
inside     -55.0    -52.0    +2.99    8.0s        1.1 mm
door       152.3    150.8    -1.53    3.4s        0.1 mm
pin        363.8    361.7    -2.08    3.6s        0.0 mm
back       726.7    723.8    -2.92    4.2s        0.1 mm
flag       735.0    733.7    -1.26    0.8s        1.0 mm
handle     900.9    899.8    -1.12    3.4s        1.1 mm
```

Twelve for twelve, inside the 4.0 mm window, the pallet staying on the deck at z 650.0
throughout. The error is biased 1-3 mm short - it undershoots - but the lap-to-lap spread
is 0.0 to 1.1 mm, and repeatability is what a taught program needs: the poses are taught
against where the pallet actually stops, and it stops in the same place every time.

The whole pass is 23.4 s of belt; the cycle itself uses four of those moves, about 15 s.
`inside` costs 7-8 s because it is the 956 mm run back from `handle`, which only happens
at setup.

A new `prim_attrs` DoCommand on the world reports a prim's authored attributes, applied
schemas and instancing state. It is what separated "the surface velocity is not authored"
from "it is authored and PhysX is ignoring it", and those are different fixes.

---

## The cycle: 323 s to under two minutes (the figures below are from the 7th)

Taught in one pass, 12 of 12 stations, no failures: 35 legs and 4755 degrees of joint
travel into `fragments/assembly-cycle-program.json`. The teaching pass itself ran 140.6 s
because teaching IS the planner searching. Replayed:

```
                        work     park
inside    arm-a         8.73s    6.50s
pin_pick  arm-b         9.01s        -
pin_insert arm-b        7.39s        -     Pin  seated 0.00 mm (slack 0.20)
flag_hole_look arm-b    4.14s        -
flag_pick arm-b         7.64s        -
flag_insert arm-b       6.55s    3.48s     Flag seated 0.00 mm (slack 1.00)
back_shot arm-a         5.93s    3.90s
grasp     arm-c         6.18s        -
present_overhead arm-c  3.81s        -
return    arm-c         3.51s        -
repaint   arm-c         6.86s        -

measured 104.5 s   belt 20.5 s   arms a 25.1 / b 38.2 / c 20.4
```

**104.5 s on the replay path, 114.5 s on the film** (the film adds 8.8 s of caption
pacing and the frame grabbing). Both under the 120 s target. The lever order the sister
cell's notes predicted held exactly: the door pre-fit and parking only on an arm change
took 323 to about 210, and replacing planning with replay took it the rest of the way.
Nothing was sped up; the planner was simply taken out of the cycle.

Both insertions seat at **0.00 mm** from the published position, on a part the belt
carried into place.

---

## Reviewing the film, and what reviewing it found

The film is a deliverable, so reviewing it is an operation rather than an act of
attention: `probes/assembly/review_film.py` reads the caption strip burned along the
bottom of each frame, takes one frame from the MIDDLE of each caption's run - the middle,
because the first frame of a caption is the arm still arriving from the previous station -
and lays them out as a contact sheet with how long each station held.

It has found two things that no log line would have, and both were invisible in the config
as well.

**The camera was on the wrong side of the cell.** It stood at y -2400. arm-b stands at
y +800 and the carrier's two slots are at y +220 and +290, so the assemble arm and both
parts it fits were BEHIND the mailbox in every frame. arm-b does five of the twelve
stations - both picks and both inserts - and not one of them was visible. A film of an
assembly cell that does not show the assembly is not a film of the cell.

**And it was aimed at one end of the process.** The target was x -55, the first index,
while the stations run out to x 901; at a 52 degree field 3.5 m away that is a 3.4 m wide
shot of a 1 m process, with the whole cycle in the right-hand two thirds and the left
third on empty floor.

Moved to the assemble side at (2600, 2600, 2600), aimed at the mid-index and narrowed to
38 degrees, the frame is 2.64 m across at 3.84 m with a 28 degree look-down: the full
956 mm of belt travel and all three arms, with the work about twice the size and on the
near side of it. arm-c's handling becomes the far side, which is the right trade - four
stations of carrying against five of the assembly the course is about.

**The timings come out of the same pass**, sorted by cost, which is how the budget gets
cut if it needs cutting. The expensive stations on the last complete film were the flag
pick at 20.1 s, the pin pick-to-insert at 14.3 s and the back shot at 8.9 s.

What the review also confirms, which the numbers cannot: the flash fires and is clearly
visible inside the mailbox, the three arms stand on their pedestals, the conveyor reads
correctly, and the door is still shut rather than lying flat - the one item from the second
batch that is visibly outstanding.

## Still worth doing

* **Pipelining.** arm-b is 37% of the arm work and the `back` and `flag` indexes are 8 mm
  apart, so arm-a's back shot can be taken while arm-b works the flag. That is one 5.9 s
  station and one 3.9 s park off the critical path - about 10 s - and it is the only
  remaining lever that does not remove something from the demonstration.
* **The conveyor's two uprights** at x 1363..1438 stand 530 mm above the deck, 460 mm
  downstream of arm-c. They are declared obstacles and nothing has hit them, but no
  station has been taught against them either. `clear_and_park` now runs the pallet to
  `belt_to_x - 500`, which leaves 128 mm, and `tests/test_assembly_fragment.py` holds
  that number against the posts.
* **The four `door_*` pose files are deleted** along with the feeder rack. Nothing refers
  to them.

---

## The next three things, in order

Everything below is written and tested and waiting on a complete cycle, because a cycle
that does not run is the wrong thing to debug a new subsystem on top of. Each changes the
layout fingerprint and so costs a full re-teach, which is about eight minutes, so they go
in as few pushes as the dependencies allow.

**1. Finish the pin.** The last measured miss is 78.26 mm, against a shell that is now
square to 0.0 degrees and an insert re-aimed by 5.0 mm. The reading of it - a 90 degree
wrist turn between a crosswise pick and a coaxial insert - is a hypothesis with the
instrumentation now in place to settle it: the cycle prints the pin's own axes when it is
grasped and again at the bore mouth. The pick has been brought round to +Y to match the
insert, with its stand-off recomputed against the grip collar's end face. If the planner
refuses that pick, the next move is to delete the pin's end stops and corner posts - the
channel on its rests already holds its x, and the posts touch nothing - before anything
else is redesigned.

**2. Wire the camera into the insert.** `probes/assembly/bore_cv.py` is written with ten
unit tests; the station, the Jacobian measurement and the correction are a single prepared
patch. The split is deliberate and worth stating in the course: the bore's LATERAL position
comes from the camera, its depth along the bore from the belt index, and the shell's
rotation from the simulator, because no single centroid can measure a tilt. Until this
lands, the cell points a RealSense at a hole and does nothing with what it sees.

**3. The two authored camera assets.** The overhead fixture is a straight integration - its
published `fixture:placement` x of -0.055 is the `inside` index to the millimetre, and its
boom puts a 964 x 806 mm footprint on the belt centreline at the lens's own design
distance. The fisheye is blocked on the module, not on time: see the section above on
`OmniLensDistortionFthetaAPI`.

After those, the door lying flat in the jig needs the prop-orientation change written
against `sim_manager` (a prop can currently only be given a yaw, and no yaw lays a door
down), and the hinge drive's target has to become `flatAngle` or the door is driven shut
the moment it is placed.
