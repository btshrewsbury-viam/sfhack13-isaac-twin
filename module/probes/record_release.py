"""Film the door check: stage each condition open, let go, and watch what it does.

Four drives, four behaviours, no arm force. The camera looks straight at the door end, so
the difference is visible rather than only measurable.
"""
import asyncio, io, os, subprocess, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell, part_origin_offset_mm
from qc.round import Clients
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
           else ".scratch/runs/door-check.mp4").resolve()
CONDITIONS = (
    ("ok", "a good door: its spring should shut it"),
    ("stuck", "stuck: it should not open at all"),
    ("binding", "binding: it should stop part-open"),
    ("loose", "loose: no spring, it should hang open"),
)
STAGE_DEG = 80.0


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    cell = Cell.from_fragment()
    frames = OUT.parent / "rel-frames"
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    frames.mkdir(parents=True, exist_ok=True)
    recorder = Recorder(Camera.from_robot(robot, "door-cam"), frames)
    rate = 10.0
    task = None
    try:
        for condition, blurb in CONDITIONS:
            # Variant BEFORE the reset: it recomposes the prim and invalidates the
            # physics views, and the reset is what rebuilds them.
            await c.world.do_command({"command": "set_variants", "prop": "part",
                                      "variants": {"door_condition": condition}})
            await c.world.do_command({"command": "reset"})
            await asyncio.sleep(3.0)
            centre = cell.resting_on(cell.fixture)
            off = part_origin_offset_mm()
            origin = [centre[i] - off[i] for i in range(3)]
            await c.world.do_command({"command": "place_prop", "name": "part",
                                      "position_mm": [origin[0], origin[1],
                                                      origin[2] + 5.0]})
            await asyncio.sleep(2.0)
            if task is None:
                task = asyncio.create_task(recorder.run())
            recorder.caption = f"{condition} - {blurb}"
            await asyncio.sleep(2.0)
            recorder.caption = f"{condition}: staging the door open"
            await c.world.do_command({"command": "set_door", "degrees": STAGE_DEG})
            await asyncio.sleep(0.3)
            recorder.caption = f"{condition}: let go at {await door_angle(c.world):.0f} deg"
            for _ in range(9):
                await asyncio.sleep(0.6)
                recorder.caption = (f"{condition}: door "
                                    f"{await door_angle(c.world):5.1f} deg")
            settled = await door_angle(c.world)
            reading = ("will not open" if settled < 10.0 and STAGE_DEG > 10 and
                       (await door_angle(c.world)) < 10.0 else "")
            if settled > 70.0:
                reading = "HANGS OPEN"
            elif settled > 20.0:
                reading = "JAMS PART-OPEN"
            elif reading == "":
                reading = "CLOSES"
            recorder.caption = f"{condition}: settled {settled:.1f} deg  ->  {reading}"
            await asyncio.sleep(3.0)
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
