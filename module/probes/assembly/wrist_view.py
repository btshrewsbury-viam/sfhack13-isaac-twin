"""Grab each vision task's own camera frame, from the pose that task is taught at.

The four inspections the cell owes - overhead classification, the interior under flash,
the flag hole, the all-round paint check - each want a DIFFERENT camera at a different
pose, and three of those cameras ride a wrist. Whether a detector can do its job is
settled by looking at its actual input, not by the station passing a reach check: a
flange pose 320 mm off the hole is reachable whether or not the hole is in frame.

This drives to each taught pose, takes that task's frame, and lays them out side by side.
"""
from __future__ import annotations

import asyncio
import io
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.build_fragment import FRAGMENT, chosen              # noqa: E402
from assembly.film_cycle import clear_of, park_if_adrift, taught  # noqa: E402
from assembly.fixture import CARRIED as SADDLE, indexed_mm as _moved  # noqa: E402
from assembly.reach_check import declared, layout_hash, orientation_for  # noqa: E402
from viam.proto.common import Pose, PoseInFrame                   # noqa: E402
from viam.services.motion import MotionClient                     # noqa: E402
from PIL import Image, ImageDraw                                  # noqa: E402
from viam.components.arm import Arm, JointPositions               # noqa: E402
from viam.components.camera import Camera                         # noqa: E402
from viam.components.generic import Generic                       # noqa: E402
from viam.robot.client import RobotClient                         # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
# task -> (belt index, arm, taught pose, camera, what the detector is being asked)
SHOTS = [
    ("classify", "inside", None, None, "overhead-cam",
     "overhead: good mailbox / damaged / empty fixture"),
    ("interior", "inside", "arm-a", "inside", "fisheye-cam",
     "fisheye + flash, inside the front aperture: interior paint"),
    ("flag-hole", "flag", "arm-b", "flag_hole_look", "realsense-cam",
     "RealSense, 320 mm off the left wall: find the 8 mm flag hole"),
    ("back-paint", "back", "arm-a", "back_shot", "fisheye-cam",
     "fisheye, angled up at the back cap: paint"),
    ("all-round", "handle", "arm-c", "present_overhead", "paint-cam",
     "overhead at the turn station, shell lifted clear: paint, all faces"),
]


RETRACT_MM = 220.0


async def retract_and_park(motion, arms, homes, arm, blob, state) -> None:
    """Back the tool out along its own axis, then park - and do it BEFORE the belt moves.

    Two faults, one ordering. The tool at `inside` stands 70 mm through a 160 x 216 mm
    aperture, and a park is a joint-space command: nothing about it says "come out the way
    you went in". And the park itself is a long way away - inside to park is 190 degrees on
    the base and 246 on the shoulder - so an arm that leaves late sweeps across the cell
    just as the belt has delivered the next part into it. Measured, arm-a stalled against
    the shell with 377, 475 and 559 N.m on its first three joints, and would only park
    once the part had been moved to the far end of the belt.

    So: retract along the approach vector the station was taught with, which is a planned
    Cartesian move and the only move that is guaranteed to clear the way in; park; and only
    then index the belt.
    """
    if blob is None:
        return
    # Leave by the TAUGHT stand-off if there is one: it is a configuration the planner has
    # already reached from this station, which a pose computed live is not.
    back = clear_of(arm, blob["task"], blob["layout"])
    if back is not None:
        f, a = back["flange_mm"], back["approach"]
        try:
            await motion.move(
                component_name=arm,
                destination=PoseInFrame(reference_frame="world", pose=Pose(
                    x=f[0], y=f[1], z=f[2], **orientation_for(a))),
                world_state=state)
            await arms[arm].move_to_joint_positions(
                JointPositions(values=homes[arm]), timeout=180)
            return
        except Exception as exc:                                   # noqa: BLE001
            print(f"    clear {arm}: {str(exc)[:140]}", flush=True)
    appr = blob["approach"]
    here = await arms[arm].get_end_position()
    # A LADDER, shortest-sufficient rather than one distance. 220 mm back along the tool
    # axis is outside the arm's workspace at some of these poses and the solver says so
    # ("zero IK solutions"), which left the tool exactly where it was - inside the part.
    # Any retract beats none, so take the longest one that solves.
    for distance in (220.0, 160.0, 110.0, 70.0):
        back = Pose(x=here.x + appr[0] * distance,
                    y=here.y + appr[1] * distance,
                    z=here.z + appr[2] * distance,
                    o_x=here.o_x, o_y=here.o_y, o_z=here.o_z, theta=here.theta)
        try:
            await motion.move(component_name=arm,
                              destination=PoseInFrame(reference_frame="world", pose=back),
                              world_state=state)
            break
        except Exception as exc:                                   # noqa: BLE001
            last = str(exc)[:160]
    else:
        print(f"    retract {arm}: no solution at any distance: {last}", flush=True)
    try:
        await arms[arm].move_to_joint_positions(
            JointPositions(values=homes[arm]), timeout=180)
    except Exception as exc:                                       # noqa: BLE001
        print(f"    park {arm}: {str(exc)[:160]}", flush=True)


async def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1
               else ".scratch/runs/vision-stations.png").resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    cams = {n: Camera.from_robot(robot, n)
            for n in ("overhead-cam", "paint-cam", "fisheye-cam", "realsense-cam")}
    cell, stamp = chosen(), layout_hash()
    frag = json.loads(FRAGMENT.read_text())
    homes = {c["name"]: c["attributes"]["home_joints_deg"]
             for c in frag["components"] if c.get("type") == "arm"}

    motion = MotionClient.from_robot(robot, "builtin")
    panels, held, held_blob = [], None, None
    for key, index, arm, task, cam, caption in SHOTS:
        # Clear the arm that was working, THEN index the belt, THEN send the next one in.
        # `moving=held`: the arm being retracted is the one that must not be an obstacle
        # to itself, and `arm` at this point is the NEXT station's arm.
        await retract_and_park(motion, arms, homes, held, held_blob,
                               await declared(world, moving=held))
        held, held_blob = None, None
        at = cell.index_x(index)
        for name in SADDLE:
            await world.do_command({"command": "place_prop", "name": name,
                                    "position_mm": _moved(cell, name, at)})
        if arm:
            await park_if_adrift(arms[arm], arm)
            blob = taught(arm, task, stamp)
            # PLAN the transit; do not jump to the taught joints.
            #
            # A taught pose records where the arm ended up, not how it got there, and the
            # route between two of them in joint space is whatever interpolation gives -
            # which here is a 190 degree base swing through the part. arm-a reached this
            # exact configuration under the planner during teaching and stalled on the way
            # to it at 533 N.m replaying it. The joints are the record; the planner is the
            # route.
            state = await declared(world, moving=arm)
            # In via the stand-off, then the station itself: the last leg of the approach
            # is along the tool axis, which is the only direction that is certainly clear.
            via = clear_of(arm, task, stamp)
            for step in ([via] if via else []) + [blob]:
                f, a = step["flange_mm"], step["approach"]
                await motion.move(
                    component_name=arm,
                    destination=PoseInFrame(reference_frame="world", pose=Pose(
                        x=f[0], y=f[1], z=f[2], **orientation_for(a))),
                    world_state=state)
            held, held_blob = arm, blob
        await asyncio.sleep(1.5)
        images, _ = await cams[cam].get_images()
        frame = Image.open(io.BytesIO(images[0].data)).convert("RGB")
        panels.append((key, cam, caption, frame))
        print(f"  {key:<11} {cam:<14} {frame.size[0]}x{frame.size[1]}", flush=True)

    # One row, each panel scaled to a common height with its caption under it.
    H, PAD, BAR = 340, 12, 52
    scaled = [(k, c, cap, f.resize((max(1, round(f.size[0] * H / f.size[1])), H)))
              for k, c, cap, f in panels]
    W = sum(f.size[0] for *_, f in scaled) + PAD * (len(scaled) + 1)
    sheet = Image.new("RGB", (W, H + BAR + PAD * 2), (18, 18, 20))
    draw = ImageDraw.Draw(sheet)
    x = PAD
    for key, cam, caption, frame in scaled:
        sheet.paste(frame, (x, PAD))
        draw.text((x + 4, PAD + H + 6), f"{key}  [{cam}]", fill=(235, 235, 240))
        draw.text((x + 4, PAD + H + 22), caption, fill=(150, 152, 160))
        x += frame.size[0] + PAD
    await retract_and_park(motion, arms, homes, held, held_blob,
                           await declared(world, moving=held))
    sheet.save(out)
    print(f"\n  wrote {out}")
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
