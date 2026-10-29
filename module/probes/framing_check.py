"""Hold the part at the inspection point and photograph it from both cameras.

Kept because the question it answers - "can the inspection camera actually see the
part" - went unasked for the whole of this project and the answer was no. Every frame
had the part rendering as a pure black silhouette, because no clipping range was set and
usd's default near plane is a metre while the part is presented 780 mm away. Comparing
the two cameras is what separated "the camera cannot see it" from "the light is wrong".
"""
import asyncio, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from qc.layout import Cell
from qc.round import (Clients, Trace, props_of, part_pose, park, pick, world_state,
                      move, unwind_wrist)
from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.components.generic import Generic
from viam.components.gripper import Gripper
from viam.robot.client import RobotClient
from viam.services.motion import MotionClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
OUT = Path("/tmp/claude-1000/-home-shrews-viam/0b32452a-7b31-46f9-97cf-c46f2dff3285/scratchpad")


async def main():
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    c = Clients(motion=MotionClient.from_robot(robot, "builtin"),
                world=Generic.from_robot(robot, "sim-world"),
                arms={n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b")},
                grippers={"arm-a": Gripper.from_robot(robot, "suction-a"),
                          "arm-b": Gripper.from_robot(robot, "suction-b")})
    cams = {"inspect": Camera.from_robot(robot, "inspect-cam"),
            "overview": Camera.from_robot(robot, "overview-cam")}
    cell = Cell.from_fragment()
    for g in c.grippers.values():
        await g.open()
    await c.world.do_command({"command": "reset"})
    await asyncio.sleep(2.0)
    props = await props_of(c)
    await park(c, "arm-b", props)
    trace = Trace()
    held = await pick(c, cell, props, trace)
    face, pose = cell.present_poses("arm-a")[0]
    await unwind_wrist(c, "arm-a")
    await move(c, "arm-a", pose, world_state(props, held=held))
    print(f"  part now at {[round(v) for v in (await part_pose(c))['position_mm']]}")
    for name, cam in cams.items():
        images, _ = await cam.get_images()
        path = OUT / f"lighting-{name}.png"
        path.write_bytes(bytes(images[0].data))
        print(f"    wrote {path}")
    await robot.close()

asyncio.run(main())
