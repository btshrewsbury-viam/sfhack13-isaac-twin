"""Photograph the jaw at each pick, open and closed, so the grip can be LOOKED AT.

The cycle film answers whether the cell runs. It does not answer whether the gripper
looks like it is holding something, because at the overview camera's distance a 14 mm
finger is three pixels and the moment of the grab is one frame in a thousand. This drives
each pick on its own, stops there, and takes the picture - open, then closed - which is
the only way the "it looks fake" question gets an honest answer.
"""
from __future__ import annotations

import asyncio
import io
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.build_fragment import chosen                         # noqa: E402
from assembly.creds import options, FQDN                           # noqa: E402
from assembly.arms import clear_and_park, home as arm_home         # noqa: E402
from assembly.fixture import index_to, restock                     # noqa: E402
from assembly.reach_check import POSES, layout_hash                # noqa: E402
from PIL import Image                                              # noqa: E402
from viam.components.arm import Arm, JointPositions                # noqa: E402
from viam.components.camera import Camera                          # noqa: E402
from viam.components.generic import Generic                        # noqa: E402
from viam.components.gripper import Gripper                        # noqa: E402
from viam.robot.client import RobotClient                          # noqa: E402

OUT = Path(__file__).resolve().parents[2] / ".scratch" / "frames" / "grip"
# Which picks to photograph, and which index the belt has to be at for each.
PICKS = (("arm-b", "pin_pick", "pin", "grip-b"),
         ("arm-b", "flag_pick", "flag", "grip-b"),
         ("arm-c", "grasp", "handle", "grip-c"))


async def shot(cams, name: str) -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    for label, cam in cams.items():
        try:
            images, _ = await cam.get_images()
            Image.open(io.BytesIO(images[0].data)).convert("RGB").save(
                OUT / f"{name}.{label}.jpg", quality=94)
        except Exception as exc:                                   # noqa: BLE001
            print(f"    {label}: {str(exc)[:100]}", flush=True)


async def main() -> int:
    robot = await RobotClient.at_address(FQDN, options())
    world = Generic.from_robot(robot, "sim-world")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    grips = {"grip-b": Gripper.from_robot(robot, "grip-b"),
             "grip-c": Gripper.from_robot(robot, "grip-c")}
    cams = {"overview": Camera.from_robot(robot, "overview-cam"),
            "overhead": Camera.from_robot(robot, "overhead-cam")}
    cell, stamp = chosen(), layout_hash()
    print(f"  layout {stamp}", flush=True)

    # UNJOIN FIRST, then reset. A previous cycle leaves the flag and the pin fastened to
    # the shell, and removing a joint prim does not take effect until physx re-reads the
    # scene - which `reset` is what makes happen. Restocking a part that is still jointed
    # teleports a constrained body across the cell: measured here as the flag arriving at
    # x 2171, y -228, z 3.4, which is the floor beyond the end of the conveyor.
    for which in ("flag", "door", "pin"):
        await world.do_command({"command": "unjoin", "name": f"{which}_seat"})
    await world.do_command({"command": "reset"})
    await asyncio.sleep(3.0)
    await clear_and_park(world, arms, cell)
    for part_name, prop in (("Flag", "flag"), ("Pin", "pin")):
        await restock(world, cell, part_name, prop)

    for arm, task, index, grip in PICKS:
        blob = json.loads((POSES / arm / f"{task}.json").read_text())
        if blob["layout"] != stamp:
            print(f"  {task}: taught against {blob['layout']}, skipping", flush=True)
            continue
        await index_to(world, cell, cell.index_x(index))
        await asyncio.sleep(1.0)
        # Stand off first, so the approach is the taught one and not a dive.
        clear = POSES / arm / f"{task}.clear.json"
        if clear.exists():
            await arms[arm].move_to_joint_positions(
                JointPositions(values=json.loads(clear.read_text())["joints_deg"]),
                timeout=180)
        await grips[grip].open()
        await arms[arm].move_to_joint_positions(
            JointPositions(values=blob["joints_deg"]), timeout=180)
        await asyncio.sleep(1.0)
        await shot(cams, f"{task}.open")
        await grips[grip].grab()
        await asyncio.sleep(1.5)
        held = await grips[grip].do_command({"command": "status"})
        await shot(cams, f"{task}.closed")
        print(f"  {task}: {held}", flush=True)
        await grips[grip].open()
        await asyncio.sleep(0.8)
        await arms[arm].move_to_joint_positions(
            JointPositions(values=arm_home(arm)), timeout=180)

    print(f"\n  wrote {OUT}")
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
