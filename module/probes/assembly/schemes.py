"""The four paint schemes, under the classification camera and under the flag-hole camera.

Two questions at once. The overhead classifier has to tell a good mailbox from a damaged one
from an empty fixture ACROSS all four schemes, so the first thing to see is what the four
look like from the camera that has to do it. And the flag-hole detector failed outright on
`classic_black` - gloss paint returns almost nothing diffusely, so the hole is invisible
whatever the lamp does - which makes "is this a property of the paint or of the station?"
the question that decides whether the station needs redesigning or just a different part.

Variant switching recomposes the prim and invalidates every physics view in the scene,
INCLUDING the arms', so it happens before `reset` and before anything moves. Getting that
order wrong is how a previous build produced "random" degree-of-freedom failures.
"""
from __future__ import annotations

import asyncio
import io
import json
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly.arms import clear_and_park                          # noqa: E402
from assembly.asset import part                                   # noqa: E402
from assembly.build_fragment import FRAGMENT, chosen              # noqa: E402
from assembly.film_cycle import clear_of, taught                  # noqa: E402
from assembly.fixture import CARRIED, indexed_mm                  # noqa: E402
from assembly.reach_check import declared, layout_hash, orientation_for  # noqa: E402
from PIL import Image, ImageDraw                                  # noqa: E402
from viam.components.arm import Arm                               # noqa: E402
from viam.components.camera import Camera                         # noqa: E402
from viam.components.generic import Generic                       # noqa: E402
from viam.proto.common import Pose, PoseInFrame                   # noqa: E402
from viam.robot.client import RobotClient                         # noqa: E402
from viam.services.motion import MotionClient                     # noqa: E402

FQDN = os.environ.get("QC_FQDN", "viam-103-qc-cell-main.pgn074cus0.viam.cloud")
# gloss | satin | matte. The shell's paint is very nearly a pure mirror in gloss, which is
# what made the flag hole invisible in colour whatever the lamp did: on-axis the lamp's own
# reflection sits exactly where the hole is, off-axis nothing comes back at all. The asset
# now carries a `finish` variant set, so the question "is that the paint or the station?"
# is one variant away from an answer.
FINISH = os.environ.get("SHELL_FINISH", "gloss")


async def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1
               else f".scratch/runs/paint-schemes-{FINISH}.png").resolve()
    out.parent.mkdir(parents=True, exist_ok=True)
    robot = await RobotClient.at_address(
        FQDN, RobotClient.Options.with_api_key(
            api_key=os.environ["VIAM_API_KEY"],
            api_key_id=os.environ["VIAM_API_KEY_ID"]))
    world = Generic.from_robot(robot, "sim-world")
    motion = MotionClient.from_robot(robot, "builtin")
    arms = {n: Arm.from_robot(robot, n) for n in ("arm-a", "arm-b", "arm-c")}
    over = Camera.from_robot(robot, "overhead-cam")
    eye = Camera.from_robot(robot, "realsense-cam")
    cell, stamp = chosen(), layout_hash()
    schemes = part("Shell").variant_sets["scheme"]

    rows = []
    for scheme in schemes:
        # Variant FIRST, then reset, then move. In that order, every time.
        await world.do_command({"command": "set_variants", "prop": "shell",
                                "variants": {"scheme": scheme, "finish": FINISH}})
        await world.do_command({"command": "reset"})
        await asyncio.sleep(2.0)
        await clear_and_park(world, arms, cell)

        shots = []
        for index, arm, task, cam in (("inside", None, None, over),
                                      ("flag", "arm-b", "flag_hole_look", eye)):  # noqa
            for prop in CARRIED:
                await world.do_command({
                    "command": "place_prop", "name": prop,
                    "position_mm": indexed_mm(cell, prop, cell.index_x(index))})
            await asyncio.sleep(1.0)
            if arm:
                # Same two-attempt rule the teaching pass uses: the planner's answer
                # depends on where the arm started, so a second try gets a known start
                # rather than another roll of the same dice. One scheme failing must not
                # cost the other three - the sheet is written from whatever came back.
                for attempt in range(2):
                    try:
                        state = await declared(world, moving=arm)
                        via = clear_of(arm, task, stamp)
                        for step in ([via] if via else []) + [taught(arm, task, stamp)]:
                            f, a = step["flange_mm"], step["approach"]
                            await motion.move(
                                component_name=arm,
                                destination=PoseInFrame(
                                    reference_frame="world", pose=Pose(
                                        x=f[0], y=f[1], z=f[2], **orientation_for(a))),
                                world_state=state)
                        break
                    except Exception as exc:                        # noqa: BLE001
                        print(f"    {scheme} {task} ({attempt}): {str(exc)[:130]}",
                              flush=True)
                        if attempt == 0:
                            await clear_and_park(world, arms, cell)
                            for prop in CARRIED:
                                await world.do_command({
                                    "command": "place_prop", "name": prop,
                                    "position_mm": indexed_mm(
                                        cell, prop, cell.index_x(index))})
                            await asyncio.sleep(1.0)
            await asyncio.sleep(1.2)
            images, _ = await cam.get_images()
            shots.append(Image.open(io.BytesIO(images[0].data)).convert("RGB"))
        rows.append((scheme, shots))
        print(f"  {scheme:<16} overhead + flag-hole", flush=True)
        await clear_and_park(world, arms, cell)

    # Two columns - what the classifier sees, what the hole detector sees - one row each.
    H, PAD, BAR = 230, 10, 20
    w = round(rows[0][1][0].size[0] * H / rows[0][1][0].size[1])
    W = w * 2 + PAD * 3
    sheet = Image.new("RGB", (W, (H + BAR) * len(rows) + PAD), (18, 18, 20))
    draw = ImageDraw.Draw(sheet)
    for i, (scheme, shots) in enumerate(rows):
        y = PAD + i * (H + BAR)
        for j, img in enumerate(shots):
            sheet.paste(img.resize((w, H)), (PAD + j * (w + PAD), y))
        draw.text((PAD + 4, y + H + 4),
                  f"{scheme} / {FINISH}      left: overhead classifier     "
                  f"right: RealSense on the flag-hole wall", fill=(225, 226, 232))
    sheet.save(out)
    print(f"\n  wrote {out}")
    await robot.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(asyncio.run(main()))
