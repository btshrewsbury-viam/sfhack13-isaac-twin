"""One real insertion, with physics on for the part being inserted.

The fable review's top-ranked concern, quoted: "Physics-off is right for A1-A3 reach
questions only. Prototype one insertion with physics on first (one arm, one mailbox, flag,
hole, no viam-server) before siting anything."

The flag is the right one to try first. It publishes 1.00 mm of radial slack against the
pin's 0.20, its pre-insertion pose is a pure 10 mm translation along (0,-1,0), and it is
light enough that a suction cup on its blade is a plausible grasp rather than a hopeful
one. Door and pin stay kinematic until this works.

What the run has to show, and what it measures:
  - the cup actually takes the flag, rather than the flag staying on the rack
  - the flag travels WITH the flange, rather than being re-parented and teleported
  - the peg enters the hole, rather than the blade landing on the wall beside it
  - the flag stays seated once the cup lets go
"""
from __future__ import annotations

import asyncio
import io
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.arms import clear_and_park                          # noqa: E402
from assembly.asset import insertions, part                       # noqa: E402
from assembly.build_fragment import chosen                        # noqa: E402
from assembly.film_cycle import clear_of, taught                  # noqa: E402
from assembly.fixture import CARRIED, indexed_mm, restock         # noqa: E402
from assembly.reach_check import declared, layout_hash, orientation_for  # noqa: E402
from assembly.stations import stations_for                        # noqa: E402
from PIL import Image, ImageDraw                                  # noqa: E402
from viam.components.arm import Arm                               # noqa: E402
from viam.components.camera import Camera                         # noqa: E402
from viam.components.generic import Generic                       # noqa: E402
from viam.components.gripper import Gripper                       # noqa: E402
from viam.proto.common import Pose, PoseInFrame                   # noqa: E402
from viam.robot.client import RobotClient                         # noqa: E402
from viam.services.motion import MotionClient                     # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
FPS = 12.0
ARM, TOOL = "arm-b", "grip-b"
# How far the flag may be from where it should be and still count. The hole is 8 mm across
# with the peg 1.00 mm smaller in radius, so anything past a couple of millimetres is a
# miss, not a tolerance.
SEATED_TOL_MM = 3.0


async def flag_mm(world) -> list:
    r = (await world.do_command({"command": "prim_poses",
                                 "paths": ["/World/flag"]}))["prims"]
    return list(r["/World/flag"]["position_mm"])


async def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1
               else ".scratch/runs/flag-insertion.mp4").resolve()
    frames = out.parent / "_flagframes"
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    frames.mkdir(parents=True, exist_ok=True)

    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    motion = MotionClient.from_robot(robot, "builtin")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    grip = Gripper.from_robot(robot, TOOL)
    camera = Camera.from_robot(robot, "overview-cam")
    cell, stamp = chosen(), layout_hash()
    sts = {s.task: s for s in stations_for(cell)}

    caption = {"text": "one insertion, with the flag under physics"}
    started, count, filming = time.perf_counter(), {"n": 0}, True

    async def film():
        while filming:
            try:
                images, _ = await camera.get_images()
                frame = Image.open(io.BytesIO(images[0].data)).convert("RGB")
            except Exception:                                      # noqa: BLE001
                await asyncio.sleep(0.1)
                continue
            elapsed = time.perf_counter() - started
            draw = ImageDraw.Draw(frame)
            draw.rectangle([0, frame.height - 34, frame.width, frame.height],
                           fill=(0, 0, 0))
            draw.text((14, frame.height - 25),
                      f"{int(elapsed // 60)}:{elapsed % 60:04.1f}   {caption['text']}",
                      fill=(255, 255, 255))
            frame.save(frames / f"f{count['n']:05d}.jpg", quality=88)
            count["n"] += 1
            await asyncio.sleep(1.0 / FPS)

    async def go(task: str, via: bool = True) -> None:
        state = await declared(world, moving=ARM)
        steps = []
        if via:
            c = clear_of(ARM, task, stamp)
            if c:
                steps.append(c)
        steps.append(taught(ARM, task, stamp))
        for step in steps:
            f, a = step["flange_mm"], step["approach"]
            await motion.move(
                component_name=ARM,
                destination=PoseInFrame(reference_frame="world", pose=Pose(
                    x=f[0], y=f[1], z=f[2], **orientation_for(a))),
                world_state=state)

    report = {}
    task = asyncio.create_task(film())
    try:
        await clear_and_park(world, arms, cell)
        # Restock the feeder. The flag is DYNAMIC now, so where it is at the start of a run
        # is wherever the last run left it - and the last run left it on the floor. A pass
        # that assumes its own starting conditions measures the previous pass.
        await world.do_command({"command": "unjoin", "name": "flag_seat"})
        await restock(world, cell, "Flag", "flag")
        for prop in CARRIED:
            await world.do_command({
                "command": "place_prop", "name": prop,
                "position_mm": indexed_mm(cell, prop, cell.index_x("flag"))})
        await asyncio.sleep(1.5)
        report["flag_on_rack_mm"] = await flag_mm(world)

        caption["text"] = "arm-b comes down onto the flag's blade"
        await go("flag_pick")
        await asyncio.sleep(0.8)

        caption["text"] = "suction"
        await grip.grab()
        await asyncio.sleep(1.0)
        report["flag_after_grab_mm"] = await flag_mm(world)

        caption["text"] = "lifting clear of the rack"
        c = clear_of(ARM, "flag_pick", stamp)
        f, a = c["flange_mm"], c["approach"]
        await motion.move(component_name=ARM,
                          destination=PoseInFrame(reference_frame="world", pose=Pose(
                              x=f[0], y=f[1], z=f[2], **orientation_for(a))),
                          world_state=await declared(world, moving=ARM))
        await asyncio.sleep(0.8)
        lifted = await flag_mm(world)
        report["flag_lifted_mm"] = lifted
        report["held"] = lifted[2] - report["flag_on_rack_mm"][2] > 40.0

        caption["text"] = "across to the mailbox, standing off the flag hole"
        await go("flag_insert")
        await asyncio.sleep(1.0)
        report["flag_pre_mm"] = await flag_mm(world)

        # The seat: a pure translation along the part's own insert axis, 10 mm, planned
        # from the pose the arm is already in rather than replayed from a taught one.
        ins = insertions()["Flag"]
        st = sts["flag_insert"]
        axis = part("Flag").at("assembly:insertAxis")
        axis0 = [float(c) for c in axis]
        _, oy0, oz0 = cell.shell_origin_mm
        depth = float(part("Flag").at("assembly:insertDepth")) * 1000.0
        f = st.flange_mm()
        # The RADIAL error is the number that decides this, not the distance to the pose.
        #
        # At the pre-insertion pose the peg's tip is exactly AT the hole's mouth - the
        # asset's own numbers put it there, with no lead-in - so error along the insert
        # axis costs nothing and error across it is the whole budget. The flag publishes
        # 1.00 mm of radial slack. Measuring the two separately is the difference between
        # "2 mm out" and "0.9 mm of a 1.0 mm allowance used before the peg even enters".
        want_pre = [cell.index_x("flag") + ins.pre_mm[0], oy0 + ins.pre_mm[1],
                    oz0 + ins.pre_mm[2]]
        got = report["flag_pre_mm"]
        delta = [got[i] - want_pre[i] for i in range(3)]
        along = sum(delta[i] * axis0[i] for i in range(3))
        radial = math.sqrt(max(sum(d * d for d in delta) - along * along, 0.0))
        report["pre_error_along_mm"] = round(along, 2)
        report["pre_error_radial_mm"] = round(radial, 2)
        report["radial_slack_mm"] = round(ins.clearance_mm, 2)

        # The published depth EXACTLY, and no overshoot.
        #
        # The asset was rebuilt after the first run of this probe and the depths now mean
        # separate things: `insertDepth` is a 3 mm approach clearance plus the engagement,
        # and the grommet carries colliders that stop the part when the hub meets its face.
        # Driving past that is no longer travelling through empty space, it is pressing on
        # a hard stop - which is what the 3 mm overshoot was doing to the old geometry,
        # storing energy that threw the flag back out when the cup released.
        seat = [f[i] + axis[i] * depth for i in range(3)]
        caption["text"] = (f"seating: {depth:.0f} mm along {tuple(axis)} "
                           f"({ins.approach_mm:.0f} clearance + {ins.engage_mm:.0f} engage)")
        await motion.move(component_name=ARM,
                          destination=PoseInFrame(reference_frame="world", pose=Pose(
                              x=seat[0], y=seat[1], z=seat[2],
                              **orientation_for(st.approach))),
                          world_state=await declared(world, moving=ARM))
        await asyncio.sleep(1.2)
        report["flag_seated_mm"] = await flag_mm(world)

        # The JOINT, before the cup lets go.
        #
        # This is the asset's own answer to why the first run failed: geometry alone never
        # held the flag, and `assembly:seatJoint` says a snap or rivet does. Made while the
        # part is still held, so it is fastened in the pose the arm put it in rather than
        # in whatever pose it falls into between release and fastening.
        caption["text"] = f"{ins.joint} joint at the published frames"
        # The asset states both local frames: the anchor in the shell's frame, and the
        # anchor minus the seated position in the part's. Using those rather than the live
        # poses means the joint pulls the flag square instead of freezing the grasp's
        # error in - which, measured with the live version, dragged it 8 mm back out.
        report["joint"] = await world.do_command({
            "command": "join", "name": "flag_seat",
            "body0": "/World/shell", "body1": "/World/flag",
            "kind": ins.joint,
            "local0_mm": list(ins.joint_anchor_mm),
            "local1_mm": [ins.joint_anchor_mm[i] - ins.seated_mm[i] for i in range(3)],
            "axis": list(ins.joint_axis_mm), "limits_deg": list(ins.joint_limits_deg)})
        await asyncio.sleep(1.0)

        caption["text"] = "release"
        await grip.open()
        await asyncio.sleep(2.0)
        report["flag_released_mm"] = await flag_mm(world)

        caption["text"] = "arm-b backs off; does the flag stay?"
        f, a = (clear_of(ARM, "flag_insert", stamp)["flange_mm"],
                clear_of(ARM, "flag_insert", stamp)["approach"])
        await motion.move(component_name=ARM,
                          destination=PoseInFrame(reference_frame="world", pose=Pose(
                              x=f[0], y=f[1], z=f[2], **orientation_for(a))),
                          world_state=await declared(world, moving=ARM))
        await asyncio.sleep(3.0)
        report["flag_final_mm"] = await flag_mm(world)

        # Where it SHOULD be: the published seated position, in the shell's placed frame.
        want = [cell.index_x("flag") + ins.seated_mm[0],
                oy0 + ins.seated_mm[1], oz0 + ins.seated_mm[2]]
        report["flag_wanted_mm"] = [round(v, 2) for v in want]
        err = math.dist(report["flag_final_mm"], want)
        report["error_mm"] = round(err, 2)
        report["seated"] = err <= SEATED_TOL_MM
        caption["text"] = ("seated, " if report["seated"] else "NOT seated, ") + \
                          f"{err:.1f} mm from the published seated position"
        await asyncio.sleep(2.5)
    finally:
        filming = False
        await asyncio.sleep(0.3)
        task.cancel()
        for k, v in report.items():
            print(f"  {k:22s} "
                  f"{[round(c, 2) for c in v] if isinstance(v, list) else v}")
        await robot.close()

    took = time.perf_counter() - started
    rate = count["n"] / max(took, 0.001)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{rate:.3f}",
                    "-i", str(frames / "f%05d.jpg"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "20", str(out)], check=True)
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    print(f"\nwrote {out} ({out.stat().st_size / 1e6:.1f} MB, {count['n']} frames "
          f"over {took:.0f}s)")
    return 0 if report.get("seated") else 1


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
