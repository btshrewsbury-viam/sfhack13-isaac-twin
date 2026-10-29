# Fix plan: cell review of 2026-10-12 (rev 2, after independent review)

Brandon reviewed the latest film (`/home/shrews/viam/viam-isaac-sim/.scratch/vm/runs/assembly-cycle.mp4`)
and listed what is wrong. This plan answers every item, says what was MEASURED behind each
answer, and orders the work so each fix is verified on the cell before the next builds on
it. Rev 2 folds in an independent review (Fable) that checked every claim against the code.

**Brandon's decisions, 2026-10-12:** flag hole stays on the +Y wall, moved near the door;
arm-b inserts both pin and flag; flag arrives lying flat with its shaft UP and is picked
like the pin; arm-c lifts and presents two stops (crown up, then rolled so the +Y wall faces
the overhead paint camera); the back-cap shot is dropped; learners locate the bore and the
flag hole from arm-b's wrist camera (`realsense-cam`); full-cycle time stays at 169 s for now.

## Findings (measured, not inferred)

| # | Observation | Finding | Evidence |
|---|---|---|---|
| F1 | arm-a's tool hits the mailbox and itself; fisheye faces the wrong way; arm-b's gripper looks backwards | **Both tool models are mounted backwards.** The mount rotation is built about X x axis, which sends the asset's axis to -X, into the wrist. The suction unit (axis already +X) is unaffected. Camera prims are placed separately and DO point +X, so images were right and only the models are reversed | `sim_manager.py` `_author_tool_asset` (~l.1810); pxr replay: (0,0,1)->(-1,0,0), (0,1,0)->(-1,0,0); `fisheye_tool.usda` lens at +Z 0.227 confirms axis (0,0,1) is the right declaration |
| F2 | tools pass through the shell | **No tool is in the planner.** `gripper.get_geometries` does publish a capsule, but grip-b, grip-c, fisheye-cam and realsense-cam have no `frame`, so the frame system never sees it; `camera.get_geometries` returns nothing | `gripper.py:134-160`, `camera.py:103`, fragment |
| F3 | only one pin | One pin prop, re-spawned between cycles | `build_fragment` props, `fixture.restock_all` |
| F4 | pin should seat flush | Seated, the 3 mm head stands 3 mm proud of the top knuckle (y 69 vs entry 66), a 20 mm grip collar stands above it, and the shaft tip is 3 mm past the bottom knuckle (138 mm shaft, 132 mm bore) | `assembly_frames.json` |
| F5 | flag not pick-up-able; shaft should be on its rotation axis, inserted like the pin | Today's grasp shaft is along the flag's local Z, perpendicular to its -Y insert: the root of the wrist roll, the opposite branches, the swivel and the widened capture | flag collider pieces; `flagHole:axis` |
| F6 | flag hole on the wrong side / should be near the door | Hole at shell x -135 on +Y: the back half; the door hinges at x +236 | `flagHole:entry` |
| F7 | light for the fisheye | **The tool asset already has a ring flash** (housing, diffuser, a DiskLight at z 0.2105, `flash:onIntensity` 20000) - mounted backwards with the tool, and **dark** because the module never drives `flash:*`. The module adds its own sphere lamp instead | `fisheye_tool.usda:22-25, 330-344, 393-403`; fragment `lighting.lamps` |
| F8 | what does arm-c do | Lifts the shell 90 mm and sets it back; nothing turns it | `cell.py:127` handle_lift_mm 90; `film_cycle.CYCLE` |
| F9 | homes | Each arm parks up and back over its own base; parks cost 32.6 s of the serial VM cycle | `assembly-cycle-time.json` |
| F10 | wrist camera | `realsense-cam` rides arm-b's wrist beside the jaw; there is a look before the flag insert only; it has no `frame`, so learners cannot transform its detections into world | fragment; `learner-contract.md:447-461` |
| F11 | rear shot | Not needed by the sequence any more | Brandon |

## Plan

Every step ends with a **gate**: a measurement on the cell AND a contact sheet
(`probes/assembly/review_film.py`) that a human looks at. Nothing builds on an ungated step.

### Day 1, in parallel

**A. One batched request to the mailbox usd session (sent first; their work is the long pole):**
1. **Flag (F5):** pivot shaft coaxial with the insert axis, with a grip collar like the pin's;
   published `graspApproach` along the insert axis; a feeder rest attitude **lying flat,
   shaft up**.
2. **Flag hole (F6):** +Y wall, moved forward near the door hinge, clear of the pin insert's
   working space; new `flagHole:*` frames.
3. **Pin flush (F4):** head bearing on the top knuckle face, shaft length = bore length, grip
   collar cut to the shortest the EZU finger pads hold (they give the number).
4. **Pin magazine (F3):** a vertical-pin track with an escapement lifter that pushes the end
   pin straight up clear of its neighbours; published lifter travel and pick frame.

**B. Cell fixes that do not depend on the new assets, one VM run each:**
5. **Tool mount (F1):** rotation = `Gf.Rotation(axis, +X)`; an offline test that replays the
   mount and asserts the fisheye asset's own `/Camera` prim lands at (+0.227, 0, 0) and the
   EZU's axis on +X. *Gate:* frames of arm-a at `inside` (stem into the door aperture) and
   arm-b at a pick (fingers around the collar), no tool inside an arm link.
6. **Flash (F7):** drive the tool's own `/FisheyeTool/Flash/Light` to `flash:onIntensity`;
   drop the module's sphere lamp. *Gate:* a fisheye frame of the interior, lit.
7. **Tools in the planner (F2):** give grip-b, grip-c, fisheye-cam and realsense-cam
   `frame`s (parent: their arm) so the frame system carries their geometry; add the
   fisheye's own envelope; and **open the shell's aperture in the obstacle model** (its
   planner box is solid today, and the fisheye is meant to be 70 mm inside it). *Gate:*
   reach_check sites every station with tools in; contact sheet shows no tool through a
   part.
8. **Drop the back shot (F11).**
9. **arm-c presents (F8):** stop 1 crown up to `paint-cam` (as lifted), stop 2 rolled about
   the shell's long axis so the +Y wall faces up; then set it back. *Gate:* paint-cam frames
   of the crown and the +Y wall; shell back within 2 deg.
10. **arm-a and arm-c homes (F9):** each park = its first station's stand-off LIFTED clear of
    the belt sweep (arm-a's `inside` stand-off sits in the pallet's path), checked with tools
    in the planner. *Gate:* park time drops; no collision on film.

### After the usd session delivers

11. **Pin feeder line (F3):** the queue is kinematic (decorative pins in the track) with ONE
    dynamic pin at the lifter - pins in resting contact cannot be teleported (HANDOFF trap 2);
    the lifter raises it; restock advances the visible queue.
12. **Flag feeder (F5):** flags lying flat, shaft up, on the angled belt (a kinematic queue
    behind one live flag, as for pins); picked and inserted on the pin's code path. Then
    REMOVE the flag-only workarounds (slot strips, 30 degree roll, branch reversal, joint
    carry, re-grip, widened seat capture).
13. **arm-b sequence and home:** pin insert's retreat ends over the flag pick in its
    orientation; arm-b's park = the pin pick's stand-off lifted.
14. **Wrist-camera looks (F10):** `pin_bore_look` and `flag_hole_look` framed on the NEW hole
    so each feature is centred on `realsense-cam`; tell the course session (viam-88) to
    point pages 6-7 at `realsense-cam`. *Gate:* `realsense-cam` in the frame system, saved
    frames of bore and hole, contract updated.

### Acceptance

A full taught cycle and film on the VM, plus the checkpoint probe for every step, with a
contact sheet per station reviewed before reporting done: no tool or arm through any part on
any frame; pin head flush; flag in the hole near the door, shaft on its axis; arm-c's two
stops visible; interior lit; wrist-camera frames of bore and hole saved. (Full-cycle time is
recorded, not gated - Brandon, 2026-10-12.)

## Risks

* With tools in the planner some stations may need re-siting; the shell aperture must be
  opened in the obstacle model first or `inside` can never site.
* The new flag hole near the hinge shares space with the pin insert; checked when the usd
  session publishes the frames.
* `docs/HANDOFF.md` says the shell is "turned"; it is only lifted until step 9 lands.
