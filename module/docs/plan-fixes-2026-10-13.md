# Fix plan: cell review of 2026-10-13

Brandon reviewed the two-shell film (`/home/shrews/viam/viam-isaac-sim/.scratch/vm/runs/assembly-cycle.mp4`,
frames pulled to `/home/shrews/viam/viam-isaac-sim/.scratch/review1/`). Each item: what was seen,
what is measured before control changes, the fix, the gate. A Fable reviewer checks this
plan and each gate's results.

Done means (CLAUDE.md): a film where a watcher sees the parts go in and nothing collide.

## Items

| # | Brandon's item | Seen on film | Owner |
|---|---|---|---|
| 1 | Fixture flanges too tall; tunnel cameras can't see the sides | The lips are `LIP_HEIGHT_MM` 70 above the shell floor and hide the lower walls in `tunnel-left/right` | cell |
| 2 | Cameras must be modelled, visible, and it must be clear what each one is for | Only the tunnel cameras (gantry asset) and the fisheye tool have bodies. `overhead-cam`, `pick-cam`, `hinge-cam`, `flag-cam`, `grasp-cam` and `realsense-cam` float invisibly, and insets carry no camera name | usd session + cell |
| 3 | Arms collide with the environment; arm-b hits the queued pins going to the pin insert | Not measured yet: the tool models are visual-only, so nothing reports contact | cell |
| 4 | Pin must go in further and sit flush (a finger protrusion that pushes it home is allowed) | The head stops proud of the top knuckle, by an amount not yet measured | cell, maybe usd |
| 5 | Is the wrist camera ever seen? | Its feed is inset at two looks, unlabelled; its body is invisible (same as 2) | cell |
| 6 | Flag in the wrong spot at sim start, bounces | The first frames of the film show it; the cause is not measured yet (fragment spawn vs restock spawn, or the boot drop) | cell |
| 7 | Flag flipped; it must hang down | Seated, the flag points UP (1:40 frames). It is gripped around its shaft and the hole is round, so the flag's angle about the shaft is set only by how it lies in the nest | cell |
| 8 | Reject arm motion is wild; the cup passes through the shell; after the drop the arm blocks the spur | arm-c (base 900,-800) picks the -Y wall at x 735 and places at (1520,-920), *outboard of its own base*, so the arm wraps round itself and the shell yaws 90 deg in the air. After release it stays over the spur | cell |
| 9 | Conveyor legs too tall; they block the reject conveyor | The A06 asset's downstream uprights stand ~0.5 m above the deck at every 2 m segment; one stands where the spur leaves the line | cell |
| R4 | (viam-88) `realsense-cam` `get_properties` intrinsics empty | Course pages 4 and 6 are blocked | cell |

## Fixes

**1. Low flanges.** Lips 70 → 25 mm. The 20 mm locators stay; they do the datuming. The tall
lips were a lead-in for an arm setting the shell down, and nothing sets it down any more
(the shell is restaged, and arm-c no longer returns it). Measure first: shell yaw and drift
over a cycle with low lips (`checkpoint.py` datum prints). *Gate:* tunnel-left/right frames
show the wall down to the rails. The shell holds within its present 0.2 mm / 0.1 deg.

**2 + 5. Every camera a real object, labelled by role.**
- usd session builds two bodies, each with a published lens frame (optical axis, mount
  face):
  - an industrial camera on a short bracket;
  - a RealSense-style D4xx body for arm-b's wrist.
- Module: a camera attribute `body` (usd path plus lens-frame prim) references the body
  so that its lens sits on the camera prim. It is visual only.
- Fixed cameras get a post or arm to the floor or gantry, so none floats.
- Film:
  - every inset gets a label bar: camera name, the arm or fixture it hangs from, and the
    task (e.g. "realsense-cam · arm-b wrist · finds the hinge bore");
  - the active camera's frustum is drawn faintly in the overview while its inset is up
    (module command `show_frustum`, thin cylinders, no collision).
- Film-only detail cameras (`pick-cam`, `hinge-cam`, `flag-cam`) are labelled "(film)",
  so learners don't take them for cell sensors.

*Gate:*
- Every camera body is visible in an overview frame.
- Each inset is labelled.
- realsense-cam and fisheye-cam are visibly riding their wrists.

**3. Collision audit before any motion fix.** Measure first. A module command `audit` turns
on a per-step PhysX overlap query for:
- every arm link's collider;
- a box proxy around each tool's visual meshes (the EZU, the suction unit, the fisheye
  stem).

It logs, per sim time, each overlap against a non-arm prim (link, other prim, depth). Run
it over a full film cycle. Then fix what it finds:
- missing planner obstacles (e.g. the queued pins' bounds, the tool geometry radius);
- or linear-move legs that bypass the planner.

*Gate:* the audit is empty over a full cycle. arm-b's path from the pin pick to the
insert clears the queue on film.

**4. Pin flush.** Measure first, in one pass at the seat:
- head face vs the top knuckle face, along and across;
- jaw pad positions vs the knuckle and the shell;
- what stops further travel.

Then choose, cheapest first:
- (a) release at the measured stop, back off 15 mm, close the jaw, and press the head
  flush with the closed finger tips. No asset change.
- (b) Brandon's pusher protrusion on the EZU fingers. This changes both picks, so only if
  (a) shows the closed jaw cannot reach the head.
- (c) let the seating joint finish the last millimetres. The film would show it as a
  snap, so it is the last resort.

*Gate:* the head is flush within 0.5 mm, seen on hinge-cam.

**6. Flag start.** Measure: flag pose from boot through the first 3 s (feed_probe).
Likely, the fragment's spawn differs from restock's re-place, or the boot drop ejects it.
Fix it so the boot spawn equals the restock spawn, kinematic until first restock if needed.
*Gate:* the first film frames show the flag still in its nest.

**7. Flag hangs down.** The feed yaw rotates 180 deg about the vertical shaft. The nest
chute must take the flag yawed; check its strip and stop geometry, and mirror it if not.
The `flag_q` queue gets the same orientation. The flag-hole look and the insert are
unchanged (round hole). No usd change is needed unless the flag is not symmetric about
its plate plane (to be checked on the asset). *Gate:* the seated flag points down on
flag-cam.

**8. Reject transfer, redesigned (spec change, flagged):**
- arm-c takes the shell from the TOP: cup on the crown apex, tool pointing down;
- straight up 150 mm;
- a swing that is mostly j1 (pick and place at the same radius and height from arm-c's
  base);
- straight down onto the spur;
- release, straight up, then park clear of the spur.

The spur is re-sited so the place is a j1 swing of about -90 deg from the pick. The
shell's long axis then lands along the spur's run, so it leaves lengthwise. arm-c may move
inboard (y -650) to bring the pick within reach. `site.py` checks the reach for every
station with tools in. The `grasp` checkpoint for the course moves with it; tell viam-88.

*Gate:*
- arm-c's joint path is monotonic in j1, with no wrist flips (printed per leg).
- The audit is empty.
- The shell rides off the end of the spur on film.

**9. Legs.** Module prop option `hide_parts` (deactivate named sub-prims of a referenced
usd) removes the A06 uprights on every main-belt segment and the spur. Matching boxes are
dropped from `CONVEYOR_BOUNDS_M`. The deck's own short legs stay. *Gate:* overview frame,
nothing standing in the spur's path.

**R4.** `get_properties` returns the pinhole intrinsics the camera renders with
(fx = fy = 931.5, cx 640, cy 360 for 1280x720 at 69 deg). Report the values to viam-88.

## Updates

- **4 (measured):** the pin reaches the published seat to 0.01 mm. What stands proud is the
  asset's Ø8 head plus 20 mm collar (+33.5 mm above the origin). Per the usd session, the
  stock EZU tips close to a Ø8.66 hole, so they can neither grip the Ø3.4 shaft nor press a
  head, and anything pressing has to stay within r 4.5 mm of the axis for the last 18 mm.
  Chosen: **option A**, Brandon's finger inserts.
  - 4 mm inserts on each finger grip the shaft and press the head.
  - The pin gets a Ø8 × 1.5 head and no collar.
  - The flag collar grows to 36 mm.
  - Magazine lifter travel becomes 40 mm.
  - The 0.6 mm insert clearance is visual only (the tools have no colliders).
  Options (a) to (c) above are dropped.
- **7 (measured):** the flag is not symmetric about its shaft (the arm runs 180 mm rearward),
  so a feeder yaw would swing it forward over the door. Fix: the usd session mirrors the
  plate's height (it hangs below the arm). No feeder change.
- **9:** the uprights are part of the body mesh, not a sub-prim. Done as
  `assets/conveyor/ConveyorBelt_A06_lowposts.usda`, a sublayer over the vendor file:
  - `holeIndices` on the 5344 faces above deck height;
  - the post colliders clamped to z 1.80;
  - `CONVEYOR_BOUNDS_M` posts clamped to match.

## After the Fable review (2026-10-13)

- **8, redesigned:** keep the SIDE grasp, which is taught and works.
  - The carry is one j1 swing of -180 deg, round by +X, away from arm-a.
  - The spur runs parallel to the line at y -1600, starting x 415. It is sited at the grasp
    mirrored through arm-c's base, (1065, -1600).
  - The final approach onto the wall is a straight line (the audit caught the cup going
    through the arch).
  - After release: off, up 150 mm, home, and only then the spur runs (300 mm/s × 4.5 s).
- **Camera bodies:**
  - grasp-cam and the three film cameras are on posts: the head and post assets, snapped
    to their tilt and height steps, each post slid so the lens aims exactly at its target.
  - The film cameras stand behind arm-b at y ≈ 1.0–1.3 m, with 12–22 deg lenses.
  - overhead-cam is on the existing overhead fixture.
  - realsense-cam wears the D435 body (`prim_name` lets it share arm-b's flange with the
    EZU), and the planner carries a box for it.
  - Frustums dropped.
- **6:** recording starts after `restock_all`. The second shell's restage is a cut, with
  the feeders restocked off camera.
- **Audit:**
  - every step;
  - held pin and flag included;
  - a monotonic clock, so restages don't rewind it;
  - pedestals ignored.
- **One deploy, one teach:** lips, low posts, the spur, camera bodies, the usd pin, flag and
  insert batch, then the audit film. Layout-hash changes force a full teach, so nothing
  ships piecemeal.

## Order

All on the VM, one pass each where possible, measurement batched.

| Step | What |
|---|---|
| 0 | Send the camera-body request to the usd session. |
| 1 | Run the audit (3), the pin-seat print (4) and the flag-start print (6) in ONE instrumented film pass. Do this before any control change. |
| 2 | Cell-only changes, one deploy: lips (1), hide legs (9), flag yaw (7), flag start (6), R4. |
| 3 | Reject redesign (8): re-site, then teach, then audit. |
| 4 | Pin flush (4), option per step 1's numbers. |
| 5 | Fix what the audit found (3). |
| 6 | Camera bodies, labels and frustums (2, 5), once the usd session delivers. |
| 7 | Full teach, film and contact sheet. The Fable reviewer looks; then report to Brandon. |

## Asks of Brandon (only if they bite)

- The top pick for the reject is a sequence change (8). The plan proceeds on it and says so
  in the report.
- Whether the film-only cameras should exist at all, or the insets should use only the
  cell's real cameras. The plan keeps them, labelled "(film)".
