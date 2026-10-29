# Fix plan: Brandon's second review, 2026-10-13 (evening)

Film reviewed: `/home/shrews/viam/viam-isaac-sim/.scratch/vm/runs/assembly-cycle.mp4` (20:49 VM, clean,
cycle 198.7 s). Brandon: keep grasp-cam (it serves course page 8). A Fable reviewer checks
this plan, the progress and the result.

| # | Brandon | What is measured / known | Fix |
|---|---|---|---|
| 1 | mailbox should start before the tunnel and be SEEN moving into it | restage spawns the pallet AT the tunnel stop (`start_index_x`) | spawn at an infeed point 900 mm upstream (x -1900); the first CYCLE step indexes it into the tunnel with the camera on; the second shell (restocked during the cut) rides in the same way |
| 2 | the RealSense collides with the door during the pin insert | audit: `arm-b Visuals -> door/Collision/Plate` at the insert. With roll -165 the D435 body sits on the door side; at full depth its lens is ~y 50, inside the shell's y extent, so only "up" is free | live probe: pose arm-b at the insert and step j6, read the D435 body's bounds and the door's; choose the roll that puts the camera above the axis and clear of door and shell; set `PIN_INSERT_ROLL_DEG`; verify with the audit |
| 3 | the flag's shaft (collar) is too long; much shorter, still grippable | 36 mm collar from the insert batch | usd session asked for the shortest collar the EZU+inserts can grip (12–15 mm target); integrate their numbers (graspPosition, pad length); re-teach |
| 4 | the vacuum cup does not touch the mailbox | `CUP_GAP_MM` 14: the flange stands 175 mm off the wall, so the cup face stops ~14 mm short; the attach ray (20 mm) bridges it | `CUP_GAP_MM` -> ~1 mm (cup face on the wall); the straight-line approach and the planner's tool-less final leg already allow contact; verify the hold and a film frame |
| 5 | strange trajectories; merge/constrain consecutive moves | taught program, joint travel per leg: flag pick->insert `square` 230 deg + `via` 255 deg (sum 1226 deg over joints) - pick and insert taught on branches 240 deg apart; arm-a park->inside 269 deg; arm-b bore_look->insert 152+184; arm-c park->grasp 173; flag_hole_look via 139 (sum 501) | (a) teach flag_pick ON the insert's branch (reach's `_from_insert_branch`, which timed out before - retry with a longer planner deadline) so the carry is one short joint move; (b) parks near each arm's first station but OUTSIDE the belt sweep and the boom posts (planner checks with the sweep box), so park->first move is short; (c) bore look on the insert's branch and roll, then a straight line in; (d) the two short legs at each station (`at`, `off`) already straight. Gate: `tests/test_taught_program.py` travel test passes for those legs, cycle time drops |
| 6 | (keep grasp-cam) | - | no change |
| 7 | a warehouse background | Isaac's `Simple_Warehouse/full_warehouse.usd` mirrored (320 MB); a clear central aisle ~11 m wide x 50 m runs along the warehouse's Y between rack blocks | reference it as a fixed usd prop yawed -90 so the aisle runs along the belt, centred on the cell (aisle centre x -10.5 -> our y 0); our ground plane made `matte` (shadows only) so the warehouse floor shows; check RTF and the overview frame |

Order: 7 and 1 and 4 (geometry/config, one deploy) + 2's probe -> 3 when the usd session
delivers -> 5 -> one teach -> film -> 2 Hz audit replay -> Fable review of the result.

## Progress

- **2 (done in code):** j6 swept live at the pin insert (`.scratch/vmtools/rs_clock.py`).
  - At roll -165 the D435 sat at 21 deg in the shell's x-z plane, over the door.
  - j6 +45 puts it at 66 deg: about 85 mm above the door and 35 mm ahead of the shell front.
  - `PIN_INSERT_ROLL_DEG` = 150.
- **1 (done in code):** `INFEED_RIDE_MM` 900. The pallet spawns at x -1900, and the first index carries it into the tunnel.
- **3 (usd delivered):**
  - Flag collar is 12 mm, gripped by the inserts' outer 11 mm.
  - `ezuTcpOffset` is now per part (flag 186.5, pin 175).
  - `stations.jaw_extra_mm(part)` reads it.
- **4 (done in code):**
  - The surface gripper won't seal on a face it touches (it needs about 6 mm, measured).
  - So `CUP_GAP_MM` is 8, and the suction MODEL is mounted 7 mm further out (`SUCTION.mount_mm`). The cup reads as touching, 1 mm off.
  - The planner envelopes are lengthened to 168/172.
- **7 (done in code):**
  - `full_warehouse.usd` yawed -90 at (0, -10.5); the aisle runs along the belt.
  - Our ground is matte.
- **Pin (Brandon's choice):** a Ø10 × 12 mm grip KNOB on the pin head, which the jaw holds through the full stroke.
  - The usd session showed that gripping the shaft through the stroke puts the EZU fingers about 12 mm into the shell at the seat.
  - Waiting on the usd build.
