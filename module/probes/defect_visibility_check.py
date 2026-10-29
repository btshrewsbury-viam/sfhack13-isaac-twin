"""Does hiding a defect actually change what the camera renders?

Toggling visibility sets a usd attribute, and reading that attribute back tells you only
that the write landed - not that the renderer has drawn it. Measured: two render steps
were not enough, and a caller that grabbed a frame immediately got the pre-toggle render
every time while the flag read back correctly.

Reports the noise floor between two frames in the same state alongside the difference
between states, because they were the same magnitude the first time this was run and the
"difference" was the part still settling.
"""
import asyncio, io, os, sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
import numpy as np
from PIL import Image
from viam.components.camera import Camera
from viam.components.generic import Generic
from viam.robot.client import RobotClient

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")


async def grab(cam):
    images, _ = await cam.get_images()
    return np.asarray(Image.open(io.BytesIO(bytes(images[0].data))).convert("RGB")).astype(int)


async def main():
    robot = await RobotClient.at_address(FQDN, RobotClient.Options.with_api_key(
        api_key=os.environ["VIAM_API_KEY"], api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    cam = Camera.from_robot(robot, "overview-cam")
    await world.do_command({"command": "reset"})
    await asyncio.sleep(8.0)  # let the part settle on the belt first

    print("  the part is on the belt; toggling defect_overspray and watching overview-cam")
    shots = {}
    for label, wanted in (("on", True), ("off", False), ("on again", True)):
        reply = await world.do_command(
            {"command": "set_defects", "prop": "part", "defects": {"defect_overspray": wanted}})
        await asyncio.sleep(1.5)
        first = await grab(cam)
        await asyncio.sleep(1.5)
        later = await grab(cam)
        shots[label] = (first, later)
        noise = (np.abs(first - later).max(axis=2) > 12).sum()
        print(f"    asked {str(wanted):<5} -> usd says "
              f"{reply['defects']['defect_overspray']}; two settled frames in this "
              f"state differ in {noise} px (max {int(np.abs(first - later).max())})")
    on, off = shots["on"][1], shots["off"][1]
    changed = (np.abs(on - off).max(axis=2) > 12)
    ys, xs = np.nonzero(changed)
    print(f"    ON vs OFF (both settled): {changed.sum()} px differ, max delta "
          f"{int(np.abs(on - off).max())}")
    if changed.sum():
        print(f"      bounding box x {xs.min()}-{xs.max()} y {ys.min()}-{ys.max()}")
    await robot.close()

asyncio.run(main())
