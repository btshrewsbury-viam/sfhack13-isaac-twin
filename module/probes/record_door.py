"""Film the door problem, so it can be watched rather than described.

The cell seats the mailbox in the fixture, arm-b holds it, and arm-a brings its cup down
onto the door panel and pulls. What the video shows is the fault: the cup lands visibly
on the door, the gripper reports it has hold of `/World/part/Body` - the SHELL - and the
pull drags the whole mailbox instead of swinging the door, which never leaves 0 degrees.
"""
import asyncio, io, os, subprocess, sys, time
from pathlib import Path
from PIL import Image, ImageDraw
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, part_origin_offset_mm
from qc.round import (Clients, flange_of, lifted, move, part_pose, props_of,
                      world_state, PART_LABEL, STRAIGHT)
from door_actuate import hinge_mm, on_panel
from door_check import door_angle
from record_cell import Recorder
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
OUT = Path(sys.argv[1] if len(sys.argv) > 1
           else ".scratch/runs/door-issue.mp4").resolve()
CAMERA = os.environ.get("QC_CAM", "overview-cam")


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    cell = Cell.from_fragment()
    frames = OUT.parent / "door-frames"
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    frames.mkdir(parents=True, exist_ok=True)
    recorder = Recorder(Camera.from_robot(robot, CAMERA), frames)
    rate = 10.0
    try:
        await c.world.do_command({"command": "reset"})
        await asyncio.sleep(3.0)
        for g in c.grippers.values():
            await g.open()
        centre = cell.resting_on(cell.fixture)
        offset = part_origin_offset_mm()
        origin = [centre[i] - offset[i] for i in range(3)]
        await c.world.do_command({"command": "place_prop", "name": "part",
                                  "position_mm": [origin[0], origin[1],
                                                  origin[2] + 5.0]})
        await asyncio.sleep(2.0)
        task = asyncio.create_task(recorder.run())

        async def say(text):
            recorder.caption = text
            await asyncio.sleep(1.2)

        await say("mailbox seated in the fixture")
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
        state = world_state(props, cell=cell, ignore=[PART_LABEL])
        start = (await part_pose(c))["position_mm"]
        await say(f"door shut at {await door_angle(c.world):.1f} deg")
        await move(c, "arm-a", on_panel(hinge, 0.0, cell.tool_mm, 30.0), approach)
        await say("arm-a brings its cup onto the door panel")
        hinge = await hinge_mm(c.world)
        await move(c, "arm-a", on_panel(hinge, 0.0, cell.tool_mm, 6.0), state, STRAIGHT)
        flange = await flange_of(c, "arm-a")
        cup = [flange[0], flange[1] + cell.tool_mm, flange[2]]
        look = (await c.world.do_command({"command": "raycast", "from_mm": cup,
                                          "along": [0, 1, 0],
                                          "distance_mm": 40.0}))["ray"]
        await say(f"a ray from the cup sees {str(look.get('body')).split('/')[-1]}"
                  f" at {look.get('distance_mm')} mm")
        await c.grippers["arm-a"].grab()
        await asyncio.sleep(0.6)
        got = await c.grippers["arm-a"].is_holding_something()
        grabbed = [str(g).split("/")[-1] for g in (got.meta.get("gripped") or [])]
        await say(f"but the gripper has hold of: {grabbed or 'nothing'}")
        for deg in (15.0, 30.0):
            recorder.caption = f"pulling to {deg:.0f} deg"
            try:
                await move(c, "arm-a", on_panel(hinge, deg, cell.tool_mm, 6.0), state)
            except Exception:  # noqa: BLE001 - the drag is the point, not a surprise
                pass
            now = (await part_pose(c))["position_mm"]
            drift = max(abs(now[i] - start[i]) for i in range(3))
            await say(f"door {await door_angle(c.world):.1f} deg   "
                      f"mailbox dragged {drift:.0f} mm")
        await say("the door never moves; the shell is what is being pulled")
        recorder.stop()
        await task
        rate = recorder.count / max(recorder.elapsed(), 0.001)
        print(f"captured {recorder.count} frames = {rate:.1f} fps "
              f"({recorder.failures} camera errors)")
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
