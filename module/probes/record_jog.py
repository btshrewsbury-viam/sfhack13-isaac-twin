"""Film the door attempt as it now stands, including the part it still gets wrong.

Three things are worth seeing: the hook arriving over the handle under JOINT control with
its tab held to a tenth of a degree, the press on the handle's top face, and the mailbox
being shoved instead of the hinge turning - which is the fixture failing to react the
force, not the arm failing to apply it.
"""
import asyncio, math, os, subprocess, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from qc.layout import Cell, part_origin_offset_mm
from qc.round import (Clients, lifted, move, part_pose, props_of, world_state,
                      PART_LABEL, STRAIGHT)
from door_actuate import hinge_mm
from door_check import door_angle
from door_hook import aim, finger_offset
from door_jog import (HANDLE_PROUD_MM, HANDLE_TOP_ABOVE_HINGE_MM, jog_to)
from record_cell import Recorder
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
OUT = Path(sys.argv[1] if len(sys.argv) > 1
           else ".scratch/runs/door-hook.mp4").resolve()


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    cell = Cell.from_fragment()
    arm = c.arms["arm-a"]
    frames = OUT.parent / "jog-frames"
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    frames.mkdir(parents=True, exist_ok=True)
    recorder = Recorder(Camera.from_robot(robot, "door-cam"), frames)
    rate = 10.0
    try:
        await c.world.do_command({"command": "reset"})
        await asyncio.sleep(3.0)
        for g in c.grippers.values():
            await g.open()
        centre = cell.resting_on(cell.fixture)
        off = part_origin_offset_mm()
        origin = [centre[i] - off[i] for i in range(3)]
        await c.world.do_command({"command": "place_prop", "name": "part",
                                  "position_mm": [origin[0], origin[1],
                                                  origin[2] + 5.0]})
        await asyncio.sleep(2.0)
        task = asyncio.create_task(recorder.run())

        async def say(text, hold=1.3):
            recorder.caption = text
            await asyncio.sleep(hold)

        await say("mailbox seated in the fixture, door shut")
        seated = await part_pose(c)
        props = await props_of(c)
        take = cell.side_grip(seated["position_mm"], cell.GRIP_SIDE)
        await say("arm-b takes the -x face and holds it")
        await move(c, "arm-b", lifted(take, 60.0), world_state(props, cell=cell))
        await move(c, "arm-b", take,
                   world_state(props, cell=cell, ignore=[PART_LABEL]), STRAIGHT)
        await c.grippers["arm-b"].grab()
        await asyncio.sleep(1.0)

        hinge = await hinge_mm(c.world)
        props = await props_of(c)
        approach = world_state(props, cell=cell)
        start = (await part_pose(c))["position_mm"]
        reach = (cell.hook or {}).get("reach_mm", 150.0)
        above = [hinge[0], hinge[1] - HANDLE_PROUD_MM / 2.0,
                 hinge[2] + HANDLE_TOP_ABOVE_HINGE_MM + 6.0]
        park = (above[0] - reach - 60.0, above[1], above[2] + 40.0)
        await say("arm-a picks the wrist roll that points its hook's tab downward")
        pose, _ = await aim(c, approach, park, [0.0, 0.0, -1.0])
        offset, _, _ = await finger_offset(c)
        hold = np.array(offset, dtype=float)
        hold /= (np.linalg.norm(hold) or 1.0)

        await say("from here it is driven by JOINTS, so the roll cannot flip")
        await jog_to(c, arm, above, hold)
        offset, _, _ = await finger_offset(c)
        turned = math.degrees(math.acos(max(-1.0, min(1.0, float(
            np.dot(np.array(offset) / np.linalg.norm(offset), hold))))))
        await say(f"tab over the handle, held to {turned:.1f} deg", hold=1.8)

        for step in range(1, 7):
            angle = await door_angle(c.world)
            a = math.radians(angle)
            top = [above[0],
                   hinge[1] - HANDLE_PROUD_MM * math.cos(a)
                   - HANDLE_TOP_ABOVE_HINGE_MM * math.sin(a),
                   hinge[2] + HANDLE_TOP_ABOVE_HINGE_MM * math.cos(a)
                   - HANDLE_PROUD_MM * math.sin(a)]
            recorder.caption = f"pressing the handle's top face ({step})"
            await jog_to(c, arm, [top[0], top[1] + HANDLE_PROUD_MM / 2.0,
                                  top[2] - 3.0 * step], hold, tol_mm=3.0, rounds=3)
            now = (await part_pose(c))["position_mm"]
            drift = max(abs(now[i] - start[i]) for i in range(3))
            await say(f"door {await door_angle(c.world):.1f} deg, "
                      f"mailbox shifted {drift:.0f} mm")
            if drift > 35.0:
                await say("the fixture is not holding it: the part moves, "
                          "not the hinge", hold=2.5)
                break
        recorder.stop()
        await task
        rate = recorder.count / max(recorder.elapsed(), 0.001)
        print(f"captured {recorder.count} frames = {rate:.1f} fps")
    finally:
        recorder.stop()
        await robot.close()
    subprocess.run([
        "ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{rate:.3f}",
        "-i", str(frames / "f%05d.jpg"),
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", str(OUT),
    ], check=True)
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    print(f"wrote {OUT} ({OUT.stat().st_size / 1e6:.1f} MB)")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
