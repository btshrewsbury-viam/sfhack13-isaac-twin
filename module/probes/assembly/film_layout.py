"""Film the assembly cell, indexing the shell through every station.

What there is to see at this stage is the LAYOUT: a conveyor the length of the cell, a
saddle that raises the mailbox 100 mm and holds it, three arms sited by the sweep, and the
belt stepping the shell to each arm in turn. The arms do not move yet - nothing is taught -
so the film is of the cell and the indexing, which is what has been built.
"""
from __future__ import annotations

import asyncio
import io
import os
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.build_fragment import chosen                       # noqa: E402
# Kept as a name here because two probes already import it from this module; the function
# itself moved to `fixture`, beside the saddle whose geometry it reads.
from assembly.fixture import CARRIED, indexed_mm as _moved        # noqa: E402,F401
from assembly.stations import THREE, stations_for                # noqa: E402
from PIL import Image, ImageDraw                                 # noqa: E402
from viam.components.camera import Camera                        # noqa: E402
from viam.components.generic import Generic                      # noqa: E402
from viam.robot.client import RobotClient                        # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
FPS = 12.0
# The order the cycle visits them, with what happens there.
STEPS = [
    ("inside", "classify overhead, then the fisheye goes in the front"),
    ("flag", "RealSense finds the flag hole, arm-b inserts the flag"),
    ("door", "arm-b caps the door on the front end"),
    ("pin", "arm-b drives the hinge pin, +Y to -Y"),
    ("back", "the back cap is photographed for paint"),
    ("handle", "arm-c lifts and turns it under the overhead camera"),
]


async def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1
               else ".scratch/runs/assembly-layout.mp4").resolve()
    frames = out.parent / "_asmframes"
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    frames.mkdir(parents=True, exist_ok=True)

    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    camera = Camera.from_robot(robot, "overview-cam")
    cell = chosen()

    caption = {"text": "the assembly cell"}
    started, count, filming = time.perf_counter(), {"n": 0}, True

    async def film():
        while filming:
            try:
                images, _ = await camera.get_images()
                frame = Image.open(io.BytesIO(images[0].data)).convert("RGB")
            except Exception:  # noqa: BLE001
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

    task = asyncio.create_task(film())
    try:
        caption["text"] = "three arms, a full-length belt, the shell in its saddle"
        await asyncio.sleep(3.0)
        for step, what in STEPS:
            x = cell.index_x(step)
            caption["text"] = f"index to {step} (x {x:.0f}) - {what}"
            # The saddle and the shell move together: the saddle IS the pallet.
            for name in CARRIED:
                await world.do_command({
                    "command": "place_prop", "name": name,
                    "position_mm": _moved(cell, name, x), "yaw_deg": 0.0})
            await asyncio.sleep(2.2)
            print(f"  {step:<8} x {x:8.1f}   {what}", flush=True)
        caption["text"] = "every station the sweep sited, reachable by its own arm"
        await asyncio.sleep(3.0)
    finally:
        filming = False
        await asyncio.sleep(0.3)
        task.cancel()
        await robot.close()

    took = time.perf_counter() - started
    rate = count["n"] / max(took, 0.001)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", f"{rate:.3f}",
                    "-i", str(frames / "f%05d.jpg"), "-c:v", "libx264",
                    "-pix_fmt", "yuv420p", "-crf", "20", str(out)], check=True)
    subprocess.run(["rm", "-rf", str(frames)], check=False)
    print(f"\nwrote {out} ({out.stat().st_size / 1e6:.1f} MB, {count['n']} frames "
          f"over {took:.0f}s)")
    return 0



if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
