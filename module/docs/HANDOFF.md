# Handoff: the Viam 103 assembly cell

Read this first. `cell-ready-for-course.md` is the detailed record of how each fault was
found and why each number is what it is; this is the operating manual and the current
state. Everything here was true at 2026-10-14 16:50 UTC (VM clock).

## What it is

An Isaac Sim cell that assembles a mailbox, for Viam 103 (*Introduction to Computer
Vision*). Three UR5e arms on pedestals around a conveyor:

* **arm-a** - a fisheye (185 degree f-theta, `fisheye_tool.usda`) on a stem. Photographs
  the interior through the door aperture, and the back cap.
* **arm-b** - a Schunk EZU 35 three-finger jaw (`ezu_35`, mirrored into
  `isaac-package-shelf/assets/mirror`) with the usd session's 4 mm pin inserts on its
  fingers (`ezu35_pin_inserts.usda`), and a RealSense D435 body (`wrist_d4xx.usda`) toed in
  25.3 deg beside it. Takes the flush pin by its shaft from the magazine and drives it the
  full depth (Brandon chose this over release-and-press, which was unreliable); takes the
  flag by its 36 mm collar from its nest. The flag's plate hangs DOWN when seated.
* **arm-c** - a Universal Robots suction unit. Handles REJECTS only: takes a failed shell
  by its -Y wall at x 735 (a straight approach), lifts it, swings j1 -180 round by +X, sets
  it on the **reject spur** (a conveyor parallel to the line at y -1600, sited at the grasp
  mirrored through arm-c's base), goes home, then the spur carries it off.

The line: **vision tunnel** (x -1000, `tunnel-top/left/right`; it does the infeed check
that `overhead-cam` used to) -> the fisheye at x -55 -> pin (x 320) -> flag (x 735) -> a good shell rides off; a reject goes to
the spur. Fixture lips are 25 mm (the tunnel sees the walls to the rails). The conveyor is
`assets/conveyor/ConveyorBelt_A06_lowposts.usda`: the vendor A06 with its uprights cut at
deck height (holeIndices + clamped colliders).

**Every camera is a modelled object** (Brandon, 2026-10-13): `grasp-cam` hangs from a
post-and-boom fixture (`overhead_camera_fixture.usda`; it looks straight DOWN on the reject
stop; `overhead-cam` and its stand are gone); `hinge-cam` and `flag-cam` (film-only) sit on posts
(`camera_post.usda` + `cell_camera_head.usda`, tilt and height snapped to the variants by
`build_fragment.mounted_camera`). Film insets show the camera doing each task, labelled
name / mount / job. Every camera reports intrinsics in `get_properties` (no
distortion_parameters: viam-server's client rejects non-Brown-Conrady models). The flag
waits KINEMATIC in its nest; any gripper closing within 300 mm frees it
(`free_kinematic_near`).

**Collision audit:** `film_cycle.py --audit` (world command `audit`) logs every arm/tool/
carried-part mesh overlapping a collider, with a step timeline. Run it as its own replay
(`.scratch/vmtools/film_only.sh` films, then audits at 2 Hz): at 10 Hz it slows the sim
until taught paths time out. Accepted hits: the pin in its bores, the inserts against the
door plate and front wall at full insert depth, the EZU housing ~13 mm into the belt guide
during the insert (raise `fixture_lift_mm` to clear it - a full re-teach), and a
flag-vs-magazine artifact.

## Running it

```bash
cd /home/shrews/viam/viam-isaac-sim
nohup .venv/bin/python scripts/autorun.py --max-attempts 30 \
    > .scratch/autorun/driver.log 2>&1 &
./scripts/status.sh                       # one line, human or machine readable
./scripts/status.sh "what I am doing"     # leave a note in the same line
```

The driver owns the loop: regenerate the fragment, run the offline suite, push the config,
respawn the module, site, teach, measure the cycle time, film. It writes
`.scratch/autorun/{driver.log,journal.ndjson,STATUS}` and the per-phase logs under
`.scratch/runs/`.

**It blocks rather than exits.** A station failure or an offline-test failure parks the
loop, holds the cell, and watches every `.py` under `probes/assembly`, `src/isaac_module`
and `scripts`. The moment any of them changes it starts a fresh attempt. That is the
intended way to work on it: edit, save, and the loop picks it up. It recovered from every
fault class it met over a seven-hour run.

**`touch .scratch/autorun/HOLD`** pauses it between attempts while you edit - an attempt
reads the generator, the stations and the cycle as it starts, and an edit landing between
two of those reads gives you a half-written cell whose failure looks exactly like a cell
fault. Remove the file to continue; it expires after 30 minutes so a forgotten hold cannot
silently end the night.

### On the GCP VM (long runs)

`scripts/cloud_vm/`: `deploy.sh` (rsync this tree, isaac-arcade, the usd assets and the
secrets; it never sends `fragments/poses` or the taught program - those are the VM's own
output), `run_long.sh` (starts the driver there), `fetch.sh` (pulls films, logs and the
VM's `fragments/` into `.scratch/vm/`; adopt with `rsync -a .scratch/vm/fragments/
fragments/`), `stop_cell.sh`, `stop_vm.sh`/`start_vm.sh`, `vm_status.sh`. The VM is
`viam103-cell` (us-central1-a, L4) and runs its own Viam machine, `viam-103-qc-cell-vm`;
its part id, FQDN and key live in `.scratch/secrets/vm/`. The driver exits after FILMED.
`gcloud` needs a live login for start/stop; plain ssh works without it.

### Traps in running it

* **`viam machines part restart` SHUTS THE CELL DOWN.** It is launched by
  `run_guarded.sh`, not by a service. To restart the module only, kill the module pid; to
  restart the part, relaunch the guard. `MAX_SECONDS` defaults to 1800, which is far too
  short - launch with 21600.
* **`pgrep -f` matches its own shell.** Match `argv[1]` through `/proc/$pid/cmdline`
  instead. A bare `exit 144` is the tell that you killed yourself.
* Never use `/tmp` on this machine. Scratch goes in `.scratch/`.

## Where it stands

| | state |
| --- | --- |
| film | `/home/shrews/viam/viam-isaac-sim/.scratch/vm/runs/assembly-cycle.mp4` (2026-10-14 20:04 VM): two shells, every station, **no failed station**, clock **105.7 sim s, under 120**; arm-a's inside shot alongside arm-b's pin pick; reject pick aimed by grasp-cam's point cloud |
| scene | Isaac `Simple_Warehouse/full_warehouse.usd` around the cell (yaw -90, aisle along the belt; roof, lamps and its ground plane off; `obstacle: false`); a fresh shell rides 900 mm into the tunnel on camera (`fixture.INFEED_RIDE_MM`); no overhead camera (infeed check is at the tunnel, Brandon via viam-88) |
| pin | Ø8 x 12 grip KNOB on the head (Brandon), held through the full stroke; seated 0.00-0.05 mm (an audit pass saw 5 mm across, inside `PIN_CAPTURE_ACROSS_MM` 6) |
| flag | Ø8 x 12 collar, plate hangs down; waits kinematic 0.5 mm over its nest; held 9.8 mm deeper than designed (`FLAG_HELD_DEEPER_MM`, a constant of its grasp, re-measured); seated 0.01-0.03 mm |
| grip | jaw attach point 160 mm (`stations.JAW_ATTACH_MM`): at 134 the attach THREW the parts (pin 40 mm, flag 50 deg); grip centre per part from `assembly:ezuTcpOffset` (186.5) |
| reject | one j1 swing to the y -1600 spur; cup visibly on the wall (`CUP_GAP_MM` 8 + suction model `mount_mm` 7: the gripper will not seal on a face it touches) |
| fixture | lift 120 (was 100): the jaw's envelope met the belt guide at the pin stand-off |
| timings | SIM seconds (`status` `sim_clock`, never rewinds). Arms and belt are timed in sim seconds too (arm trajectories used to run off the wall clock - and ~2.5x below their configured speed). `cycle_time.py` 98.0 s idle (117.7 s wall); 87.9 s with four learner cameras streaming (309 s wall, 0.29x); film clock 105.7 s. The ~10 s spread with load is the probe scripts' own wall-clock pauses (ride-off, settles) - not yet sim-timed. Recorded, not gated |
| sim speed | 1.0x real time idle, capped (`max_rtf` 1.0); `rendering_dt` 1/30 with physics 1/60 (measured: physics 2 ms a step, a rendered frame 22-31 ms, each live 720p camera ~15 ms more, the warehouse ~0). Filming with the overview + insets live: ~0.5x, so the film plays slower than real time; its clock is sim seconds. `overview-cam` renders on demand like every camera |
| belt | indexing is decided IN THE MODULE every physics step (world command `index`: launch ramp, brake to creep, cut within 0.75 mm, creep back if it coasts); `fixture.index_to` only starts and watches it. Stops land within 0.9 mm and repeat to 0.01 mm at any sim rate (the old client poll stopped 27.7 mm out at 1.0x) |
| parks | each arm WAITS on its first station's own wrist branch (`probes/assembly/waits.py`: the stand-off's joints, pulled back and lifted out of the belt sweep with only j2-j4 moving); between two of its own stations an arm only steps out of the sweep (`reach_check.step_out`). Taught travel 5425 -> 3264 deg. After a VM teach changes the parks, ADOPT the VM's fragments before the next deploy or the homes go stale |
| reject pick | AIMED BY grasp-cam's POINT CLOUD (Brandon, 2026-10-14): the camera has `depth`, `get_point_cloud` returns a binary PCD (metres, camera frame, rgb), and `probes/assembly/shell_locate.py` fits the shell's crown and wall tops for x, y, yaw (`TOP_CENTRE_BIAS_MM` -2.45, a measured constant). On film: 0.2 mm across from the sim's truth, printed every pick. The fit band starts 140 mm above the shell origin because the cup, already on the wall when it looks, reaches +122 |
| sim loop | drains only the work queued when each drain began, then steps: drained until empty, four camera streams starved the sim to 0.01x (2026-10-14) |
| grasp-cam frame | ov (0, 0, -1, th 180), re-measured from the rendered prim (image right = world +X along the belt, image down = world -Y); th -90 was a quarter turn off. A depth camera names its image "color" (detections-to-segments requires it). `.scratch/vmtools/cam_frames.py` checks every fixed camera's frame against its render |
| offline tests | 215 pass; the 2 long-standing taught-program failures are tolerated |

Nothing is committed. Everything is on branch `gripper`; the VM's taught poses and
program are pulled back into `fragments/`.

## How the insert works, because it is the hard part

Every number the insert uses is measured rather than assumed, and each correction has a
floor it acts above and a ceiling it refuses above. In order:

1. **Pick** - the part's grasp point is measured and the tool steps across the error
   (acts above 0.5 mm, refuses above 40 mm, where the part has left its slot).
2. **Re-reference** - the whole insert is transformed into the shell's measured frame, so
   the pose, the tool's orientation and the stroke direction all rotate together. This is
   what keeps the pin COAXIAL with the bore rather than merely pointed at its mouth.
3. **Place** - the flange pose is computed from the hold measured at the grab, so the part
   lands on the target. Not searched for: see "the 3.91 mm" below.
4. **Square** - if the part's tilt would cost more at the far end of the stroke than the
   feature can capture, the tool turns onto the bore's axis and the part is placed again.
   Refuses above 10 degrees.
5. **Nudge** - up to three measured corrections if the computed pose did not land it,
   stopping if the error grows (which means contact).
6. **Stroke** - straight-line constrained, 1 mm and 2 degrees.
7. **Press** - up to three short presses sized to the measured shortfall, stopping if the
   part starts going ACROSS the bore instead of into it.
8. **Measure, THEN fasten.** The seat is measured before the joint is made, because a
   `join` here is strong enough to drag a part into place and hide a failed insert.

## The six things that will cost you a day if you do not know them

1. **A `reset` with a joint live lands the assembly out of square.** 0.00 degrees if the
   door's hinge is joined during the post-reset fall, 4.15 degrees every time if the joint
   is live across the reset. Use `fixture.restage()` - it is the cell's only reset, it
   unjoins everything, resets, re-joins the door inside the fall, and CHECKS the datum.
   The fall is 131 ms; the join delay ladder is (0.1, 0.05, 0.3) and the retry uses a
   different one each time, because a retry that re-runs the same experiment is not a retry.
2. **Teleporting a dynamic body in resting contact ejects it.** Measured: 435 mm, 985,
   1548, 2588, 3122. After a reset the props are ALREADY staged and stable, so
   `restock_all` looks first and leaves them alone. `restock` (singular) is the old
   teleporting path and is marked legacy.
3. **A joint edit does not reach PhysX until the scene is re-read**, and only `reset` does
   that. Removing the prim is not enough; nor is `physics:jointEnabled = False`.
4. **The pick and the insert must stand off along the SAME axis**, or the wrist turns the
   held part between them. The pin was 78.26 mm out from exactly this.
5. **An overall bounding box is the wrong thing to build a fixture from.** The flag
   publishes a 37 mm width that is a grasp shaft and a hub at one end; over the rest it is
   a 2.2 mm plate. `probes/assembly/colliders.py` measures the real pieces, through the
   usd session's own interpreter, cached against the geo file's mtime.
6. **Measure at the feature, not the part's origin.** An origin is wherever the asset's
   author put it. The flag's is 206.7 mm from its grasp and 27 from its peg, so a small
   rotation moves it seven times further than it moves the thing that has to fit. This
   mistake was made twice in one evening, on two different parts, after being written up.

## The 3.91 mm, which is the best story in here

The correction loop was stepping the same 4 mm on every insert, on both parts, in every
run - 4.19, 4.00, 3.83, 3.62 on the pin; 3.99, 3.96 on the flag. Every refinement to the
loop made it better at finding that constant. It was in the model of how the tool holds
things: the station assumed 176.5 mm from the flange and the part hangs at 180.41.

**If a correction loop produces the same magnitude every run, it is not a loop working -
it is a constant nobody has looked up.** The insert now measures the held part's offset in
the flange's frame once, at the grab, and computes the pose.

## Tools worth knowing about

| | |
| --- | --- |
| `probes/assembly/review_film.py` | turns a cycle film into a contact sheet, one frame per station with its caption and duration, sorted by cost. It has caught two faults invisible in both the logs and the config - including that the camera was on the far side from the assemble arm, so five of twelve stations were occluded in every frame |
| `probes/assembly/colliders.py` | the real collider pieces of a part, measured not transcribed |
| `probes/assembly/what_holds_shell.py` | fires rays at the shell from every direction a cradle could touch it and says what each one hits. Run it when the datum is wrong and you do not know why |
| `probes/assembly/grasp_truth.py` | whether a grab actually holds: raycast, `gripped_objects`, and whether the part MOVES with the tool - the third cannot be faked |
| `probes/assembly/bore_cv.py` | the camera bore-finder. Written and unit tested, not wired in - see `queued/` |

## Releases and branches (2026-10-20)

The course machine (the gate, `docs/RUNBOOK-gate.md`) runs registry module
`viam-education:isaac-sim` pinned exactly in fragment `c5310da2`; rollbacks for each repin
are `.scratch/runs/c5310da2-rollback-0NN.json`. Every version since 0.1.7 is the 0.1.5-line
cell (EZU 35, original lighting) plus:

| version | branch | adds |
| --- | --- | --- |
| 0.1.8 | lens-018 | fisheye-cam `lens` DoCommand |
| 0.1.9 | lens-019 | renamed `get_lens_properties` (`lens` kept as an alias) |
| 0.1.10 | sandbox-0110 | runs in the viam app's Simulate sandbox: a GPU without nvidia-smi; Isaac pre-installed in `/opt/isaac-sim-venv` |
| 0.1.11 | sandbox-0111 | models build on viam-sdk 0.85 (the sandbox venv ships it). Proved in a fresh sandbox: 22/22 ready in 4 min 24 s, page 10 750 in 187 s, renders equal the gate's |
| 0.1.12 | egp-0112 | the EGP 40 on arm-b (relight and matte belt built but OFF: 0.1.11 look, `build_fragment --relight` turns it on); other arms planned as link-mesh capsules, not world boxes (fixes a 1-in-51 arm-c/arm-a refusal); IsMoving survives physics-view rebuilds; boot phase log lines. Soak flat, page 10 750 |
| 0.1.13 | c-0113 | 0.1.12 (relight off by default, `build_fragment --relight`) + option C merged (side grasp-cam on Sam's mount) + Sam's light warehouse stand-in v2 (usd c156605) in place of Simple_Warehouse. On the course path: page 10 750 with every course model incl. `mailbox-shell-side`, grasps 5/5 held (wall point across 0.0 mm, along +20..38 mm from the model's box bias). A cold sandbox boots in ~78 s with the stand-in (0.1.11: 258 s) |
| 0.1.14 | boot-0114 | 0.1.13 + the sandbox boot cuts switched on in the module entry (`VIAM_ISAAC_HUB=off`: no Omniverse hub retries; `VIAM_ISAAC_KIT_CACHE=seed`: the package's 57 MB driver-independent Kit cache copied no-clobber into the Isaac venv's). As 0.1.14-rc1 on a cold sandbox (Khizar, 2026-10-21): ALL READY 75.6 s, page 10 with the course models 750 in 196.9 s |
| 0.1.15 | relit-0115 (169f5e1) | 0.1.14 + the relight ON by default (dome 3000, six high-bay disk lamps) + the matte belt + Sam's D435 bracket for the EGP 40 (`realsense-cam` tool variant `mount: egp40`). Package sha256 e5edd357. Proved on VM3 (0daebda6, local module): M4 4/4, 24 inserts seated (pin 1.3-1.9 mm, flag 0.2-0.6); M5 3/3, 12 grasps held; `mailbox-shell-side` rescore along +13.5..16.4, across 0.0, height -0.3 mm; arm-b's only contacts are its jaw fingers (none on the bracket). Shipped together with course-tools' inside model 20261022-135923 (Felix's relit retrain; Khizar M2 4/4). Gate proof: 36 resources, page 10 oracle 750 in 188 s. Rollback `c5310da2-rollback-0114.json` |
| 0.1.16 | settle-0116 (e022189) | 0.1.15 + camera reads settle after any scene change (SETTLE/WAKE 90 frames; the cell service marks every non-status verb) + the held pin padded 2 mm across its length, not 10 (queued pins sit on a 20 mm pitch: the 10 mm pad left ~2 mm and failed IK "held-pin vs pin_q1" on page 2) + `reset` rides the shell out only when an arm is caught at a station (`--stage`/`--batch 1` show one body) + the infeed oracle says `empty_fixture` (cell truth and `--stage empty` keep "empty"). Package sha256 b57dc6fa. Ships lockstep with course staging b17504b5 (infeed 20261022-210001, inside 20261022-210048, paint 20261022-194128): old course + 0.1.16, or new course + 0.1.15, scores 500 on page 10. Proved on VM3: 20x `--batch 1` 0 IK errors; M4 4/4, 24 seated; page 10 learner path 750 in 216 s and oracle 750. Gate proof: 36 resources, page 10 oracle 750 in 211 s (the gate part still pins the old course models). Rollback `c5310da2-rollback-0115.json` |
| **0.1.22 (live since 2026-10-28 12:44Z)** | off-sim (4ee6e36) | 0.1.21 + reads moved off the sim thread (a snapshot published every step: clock, arm joints/IsMoving/pose, gripper status, camera frames on request; a write barrier; latest-wins joint targets; one JPEG encode per frame; CONTROL card reads capped at one refresh per 5 s per camera), P3b (a stage straight after a reset keeps its restock), frozen `--stage` (captured station states in `probes/assembly/stage_states`, written back in one sim-thread callback; pin stop by pre-pick teleport; re-capture them with `cap_states.py` whenever poses, fragment or assets change), render-only camera settle (`VIAM_ISAAC_RENDER_SETTLE=0` turns it off), and end_verb marking a scene change only when the verb wrote to the sim. Sha bc95d644 (= rc.3). `--stage` good 20 to 6.4 s, flag 93 to 10 s, pin 70 to 37 s. Gate: oracle 750 236 s, learner 750 252 s, 22 pinned, 0 IK. Rollback `c5310da2-rollback-0122.json` |
| 0.1.21 (live 2026-10-27 07:53Z to 2026-10-28 12:44Z) | ctrl-0121-a (a3698bf) | 0.1.20 + a cap on display (CONTROL card) reads: an unpinned read that has a frame never waits for the renderer, at most 4 wait (the rest get "camera busy"), and reuse is re-checked after a wait. No change with CONTROL closed. Sha 3f254969 (= rc.2). All cards open still fails (app p50 132 s); the course's close-CONTROL rule stands. Gate: oracle 750 238 s, learner 750 253 s, 22 pinned, 0 IK; Khizar sandbox 750 254/249 s. Rollback `c5310da2-rollback-0120.json` |
| 0.1.20 (live 2026-10-26 22:39Z to 2026-10-27 07:53Z) | ctrl-0120 (bdc0d66) | 0.1.19 + CONTROL cards no longer degrade course camera frames: a camera switching on/off while another settles changes that frame (a 170-render tunnel-top good read 0.63-0.84 with cards waking, 0.90-0.96 with the same cameras steadily on). Course code PINS what it reads (set_rendering on; lockstep with course viam-103 6a5bedfc: the starter pins every station camera, pin-before-release; pasted scripts pin around reads). Unpinned (card) reads serve the last frame while a verb runs or a pin is held; otherwise reuse if nothing moved, else wake in a FIFO settle window; no camera switches during a pinned settle (sleeps deferred); reads re-settle after a display switch (tagged "(pinned, after a display switch)"). Pins lapse 300 s after the last pin/pinned read/verb; a reset clears pins. Two stamps: cell verbs re-settle course cameras (as 0.1.19); arm/gripper/world moves only bar reuse. Also viam-sdk 0.80 remove_resource KeyError guard (a removed service no longer wedges module rebuilds). Sha 8015a431 (= rc.2). VM3 all-cards: learner 750 in 243 s steady state (closed 245 s), oracle 750 223 s, infeed good 0.94-0.97, classify.py about 7 s; first run within about 40 s of opening CONTROL is slower and read infeed 0.812. Gate: 36 resources, oracle 750 235 s, learner 750 255 s, exactly 11 "(pinned)" settles per run. Rollback `c5310da2-rollback-0119.json` |
| 0.1.19 (2026-10-26 18:22Z) | scene-0119 (a198365) | 0.1.18 + `scene-cloud`, a point-cloud camera for the app's 3D viewer: single-view overview by default (presets infeed/inside/pin_flag/grasp/paint via `set_view`), WORLD frame in METRES (a PCD is metres; viam multiplies by 1000), cropped to the cell, voxel 15 mm, cap 40k (Brandon's pick, ~640 KB), cache 2 s, depth-only settle (10 renders, no converge, no settle line); its frame is the world origin and it is left out of the layout fingerprint (still 9dd032ca). Plus CONFIGURE folders: `ui_folder` on every fragment resource (Arms, Cameras, References, Simulation). Sha 22fcd264 (= rc.2). VM3: learner path 240-243 s with nobody pulling, 248 s at a 20 s poll, 268-274 s at 1 Hz; 11 settle lines every run. The viewer fetches about once per load (refresh is opt-in); learners set point size 0.015. Gate: 36 resources, page 10 oracle 750 in 226 s, learner path 750 in 246 s; gate course pins moved to the course set 2026-10-26. Rollback `c5310da2-rollback-0118.json` |
| 0.1.18 (2026-10-23 22:18Z) | fix-0118 (92bf9f1) | 0.1.17 + Sam's interior ribs (usd main 2079cb1: shell.usda + geo/shell.usdc in the git-ignored cell-assets) + the hybrid camera settle: after the 150-render wake / 90-render scene-change minimums a read that waited renders on in steps of 5 until d(F0,F1) <= 1.08 x d(F1,F2) (the camera's own noise floor; 1-in-8 subsample, cap 600), also after `set_rendering` on; every converge logs "settled after N renders" (env: VIAM_ISAAC_WAKE_FRAMES, _SETTLE_FRAMES, _CONVERGE_RATIO, _SETTLE_MAX_FRAMES). Fragment diff = the version only. Sha d7c7f41d. Course lockstep: staging fb03548b (inside 20261023-182836, page 5 ribs figure). Proved on VM3: page 10 learner 750 (237 s) + reference 750; 20x `--batch 1` all at the 0.1.17 outcomes, untraced 114-116 s (traced 140-146 s: the tracer's own reads now converge); M4 4/4; M2 inside 32/32; 0 IK; ~490 converges at 160-210 renders, 0 "still settling". Khizar's sandbox rc.4: 750 in 252 s. Gate: 35 resources, page 10 reference 750 in 231 s; the gate's learner path scores 125 because its own services pin the 2026-10-19 models (course pins differ; a gate config edit, Brandon's word). Rollback `c5310da2-rollback-0117.json` |
| 0.1.17 (2026-10-23 17:45Z) | fix-0117 (0cd3b8e) | Brandon's names: inspect-arm / insert-arm / reject-arm, insert-gripper / reject-gripper, insert-cam (was realsense-cam), the oracles as `<learner service>-reference` (shell-segments-reference); hinge-cam removed; usd prims and pedestals follow (/World/inspect_arm). No aliases. The layout fingerprint hashes names, so it was re-stamped (1d2cd0ef -> 9dd032ca) after proving only names and hinge-cam changed. Also: the oracles default to a camera and hand back any camera's image (the CONTROL card's nil pointer); camera settling counts the step loop's monotonic sim clock (world.reset() zeroes current_time: 42 "not settled" warnings on cand3, 0 on cand4); the pallet wears Sam's fixture look C-v2 (usd 0830923) through a composite prop's `visual_usd` + `hide_colliders`, colliders unchanged. Sha d4b17138. Ships lockstep with course staging b352c0b9: old course + 0.1.17 fails on every renamed name. Proved on VM3: 20x `--batch 1` = unedited baseline, 0 IK; M4 4/4 24 seated; M3 PASS; infeed on C-v2 60/60 (Felix); page 10 learner 750 + reference 750. Gate: 34 resources, page 10 reference 750 in 207 s (the gate's own realsense-cam-detections needs a config edit). Rollback `c5310da2-rollback-0116.json` |
| 0.1.12-boot1/2/3 (tests) | boot-0112 | sandbox boot cuts behind module env flags: `VIAM_ISAAC_HUB=off`, `VIAM_ISAAC_KIT_ROOT=bundled` (a warm Kit cache through `--portable-root`); the warehouse off by config |

**Option C, a side-facing grasp-cam (branch `side-grasp-cam`, prototype, not released).**
grasp-cam moves to Sam's side mount (viam-103-cell-assets `side-grasp-cam` b69e64e: lens
(1200, −800, 700) mm, 10° up, 34° off square, fov 40), clear of every arm move including the
moves between taught legs and the park moves; the cell reads it before arm-c leaves home.
On the cell it works: the wall fit is +2.1 mm, 3 of 3 held at 0.7–0.8 mm, page 10 750,
batch 12 2450. On the COURSE path it fails: detections-to-segments has no depth cut, so even
a perfect detector box keeps the saddle face in front of the wall, and page 8's lowest-y
rule lands 22.7 mm off (band 5). "The y most points share" would pass (0.1 mm across). The
rule, Felix's `mailbox-shell-side` retrain and page 8's text are Brandon's to decide. The
course side is viam-training `viam-103/side-grasp-cam` e76523e8. The report is
`.scratch/design/grasp-side-camera-options.md` on that branch.

## What is queued

`queued/` holds five one-shot scripts that edit the repo in place, each asserting the text
it replaces so a stale one refuses rather than half-applying. `queued/README.md` has the
table, the order, and the honest caveat about whether the CV correction is load-bearing or
decorative now that the insert is placed from a measured hold.

## Traps found 2026-10-09..12

* **The planner does not know what the arm is holding.** `reach_check.declared(...,
  motion=, grip=)` attaches the held part's live box to the arm's end frame and drops its
  world obstacle; parts fitted to a held shell ride with it. A release plans with the part
  dropped but not attached.
* **The flag's pick and insert reach on opposite wrist branches** unless the shaft waits
  at (720, 540): `.scratch/checks/branch_probe.py` maps it. The carry is then a joint move.
* **The simulated grip froze a part's lean**: the pin is stood square on its lifted seat
  as the jaws close. (The old flag's re-grip is gone - it dropped the new flag.)
* **A tool's colliders inside instanced prims survive a strip** unless de-instanced first.
* **`prop_poses` of a non-rigid prim used to make it a falling rigid body** - guarded now.
* **Background parks and planning do not mix**: the motion service re-commands every arm
  in its plan. Parks run in the background only on replay.
* **Import a service's SDK client before connecting**, or `from_robot` cannot find it.

## Traps found 2026-10-12 (the new line)

* **Toggling a conveyor's surface velocity, joints made anywhere, or a variant switch
  re-read the physics scene.** A light part resting within ~1 mm of something is thrown
  (the flag, 140 mm up). Hence the flag's 5 mm nest drop and its re-seat before the pick.
* **A shell variant switch kills every arm's physics view** ("Failed to get DOF
  positions") until the next reset: set the paint, then restage.
* **A USD prop's declared boxes did not turn with its yaw** - fixed in the module.
* **Every camera rendered every frame.** On demand now; `render: always` opts out.
* **A static collider moved by its transform does not push**: the magazine lifter is a
  `kinematic_parts` body.
* **The guard's 6 h limit** ends the cell mid-run; relaunch with `run_long.sh`.

## Open questions, in the order I would take them

1. **Cycle-time levers (Brandon's call):** belt creep near each stop (`fixture.CREEP_FROM_MM`
   90 at 60 mm/s; ~15-20 s wall), parks near each arm's first station (~5-8 s), the bore
   look on the insert's roll (~2-3 s). The infeed ride-in adds ~6-8 s.
2. **The course's cell service (R1)** per `viam-103/learner-contract.md` (classify-first then
   paint at the tunnel; grasp band on the -Y wall, z 788..916 now). `stage` should use the
   tight 1.5 mm stop, not the tunnel's 25 mm.
3. **R5** depth and point clouds (grasp-cam's cloud holds the roof only).
4. Release-and-press (`PIN_RELEASE_AND_PRESS`) is off and superseded by the knob.
5. The tunnel stop cannot creep into 1.5 mm (a forward creep there does not advance the
   pallet; unexplained) - hence its 25 mm tolerance.
